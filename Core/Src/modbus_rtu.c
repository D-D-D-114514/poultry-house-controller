#include "modbus_rtu.h"
#include "project_config.h"
#include "task_scheduler.h"
#include <string.h>

/*====================================================================*/
/* Modbus RTU Slave, 双串口                                           */
/*====================================================================*/

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static uint16_t s_regs[MODBUS_REG_COUNT];

typedef struct {
    uint8_t  buf[MODBUS_RX_BUF_SIZE];
    uint8_t  len;
    uint32_t last_byte_tick;
    uint8_t  ready;
} RxBuffer_t;

static RxBuffer_t s_rx1;
static RxBuffer_t s_rx2;
static uint8_t s_slave_id = 1;

/* 持久接收字节缓冲区 */
static uint8_t s_rx_byte1;
static uint8_t s_rx_byte2;

/*====================================================================*/
/* CRC16-Modbus                                                        */
/*====================================================================*/

static uint16_t crc16_modbus(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/*====================================================================*/
/* 发送响应                                                            */
/*====================================================================*/

static void send_response(UART_HandleTypeDef *huart, uint8_t *resp, uint16_t len)
{
    HAL_UART_Transmit(huart, resp, len, 100);
}

/*====================================================================*/
/* 处理 Modbus 帧                                                     */
/*====================================================================*/

static void process_frame(UART_HandleTypeDef *huart, uint8_t *data, uint8_t len)
{
    uint8_t resp[256];
    uint16_t resp_len = 0;

    if (len < 4) return;
    /* 对齐旧程序: 仅响应本机地址, 不响应广播地址 0 */
    if (data[0] != s_slave_id) return;

    uint16_t crc = crc16_modbus(data, len - 2);
    uint16_t recv_crc = (uint16_t)data[len-1] << 8 | data[len-2];
    if (crc != recv_crc) return;

    uint8_t func = data[1];

    switch (func) {
    case 0x03: {
        /* 对齐旧程序: 0x03 帧不足 8 字节静默丢弃 */
        if (len < 8) return;

        uint16_t start = (uint16_t)data[2] << 8 | data[3];
        uint16_t count = (uint16_t)data[4] << 8 | data[5];

        /* 对齐旧程序的校验顺序与异常码:
           数量非法(0或>100) -> 异常码 0x03;  地址越界 -> 异常码 0x02 */
        if (count == 0 || count > 100) {
            resp[0] = s_slave_id;
            resp[1] = func | 0x80;
            resp[2] = 0x03;
            resp_len = 3;
            break;
        }
        if (start + count > MODBUS_REG_COUNT) {
            resp[0] = s_slave_id;
            resp[1] = func | 0x80;
            resp[2] = 0x02;
            resp_len = 3;
            break;
        }

        resp[0] = s_slave_id;
        resp[1] = func;
        resp[2] = count * 2;
        for (uint16_t i = 0; i < count; i++) {
            resp[3 + i*2]     = (s_regs[start + i] >> 8) & 0xFF;
            resp[3 + i*2 + 1] = s_regs[start + i] & 0xFF;
        }
        resp_len = 3 + count * 2;
        break;
    }

    case 0x06: {
        /* 旧程序不支持 0x06, 此处保留用于组态王反向写控制(阀门等)。
           主机若只读取(0x03)则永远不会触发, 不影响与旧主机的兼容性。 */
        uint16_t addr = (uint16_t)data[2] << 8 | data[3];
        uint16_t value = (uint16_t)data[4] << 8 | data[5];

        if (addr >= MODBUS_REG_COUNT) {
            resp[0] = s_slave_id;
            resp[1] = func | 0x80;
            resp[2] = 0x02;
            resp_len = 3;
            break;
        }

        s_regs[addr] = value;
        memcpy(resp, data, len);
        resp_len = len;
        break;
    }

    default:
        resp[0] = s_slave_id;
        resp[1] = func | 0x80;
        resp[2] = 0x01;
        resp_len = 3;
        break;
    }

    if (resp_len > 0) {
        uint16_t resp_crc = crc16_modbus(resp, resp_len);
        resp[resp_len++] = resp_crc & 0xFF;
        resp[resp_len++] = (resp_crc >> 8) & 0xFF;
        send_response(huart, resp, resp_len);
    }
}

/*====================================================================*/
/* 帧完成检测: 字节间隔 > 15ms 判定帧结束                               */
/*====================================================================*/

static void check_frame_complete(RxBuffer_t *rx, UART_HandleTypeDef *huart)
{
    if (rx->len == 0) return;

    uint32_t now = TaskScheduler_GetTick();
    uint32_t gap = now - rx->last_byte_tick;

    if (gap >= 15 && !rx->ready) {
        rx->ready = 1;
    }

    if (rx->ready && rx->len > 0) {
        /* ====== 临时诊断: 回显[原始帧][字节数][CRC结果], 诊断后删除 ======
           发 01 03 00 00 00 01 84 0A 后, 回显末尾两字节判读:
             ..08 59  -> 收到8字节, CRC正确(Y) -> 问题在别处
             ..08 4E  -> 收到8字节, CRC错误(N) -> CRC问题
             ..07 4E  -> 只收到7字节 -> 接收丢字节 */
        {
            uint8_t dbg[80];
            uint8_t n = 0;
            uint8_t i;
            for (i = 0; i < rx->len && n < 70; i++) dbg[n++] = rx->buf[i];
            dbg[n++] = rx->len;   /* 收到的字节数 */
            if (rx->len >= 4) {
                uint16_t c = crc16_modbus(rx->buf, rx->len - 2);
                uint16_t r = (uint16_t)rx->buf[rx->len-1] << 8 | rx->buf[rx->len-2];
                dbg[n++] = (c == r) ? 0x59 : 0x4E;   /* 'Y' / 'N' */
            } else {
                dbg[n++] = 0x3F;  /* '?' 帧太短 */
            }
            (void)dbg; (void)n;   /* 诊断回显已关闭 */
        }

        process_frame(huart, rx->buf, rx->len);
        rx->len = 0;
        rx->ready = 0;
    }
}

/*====================================================================*/
/* 接收字节回调 (从 HAL_UART_RxCpltCallback 调用)                      */
/*====================================================================*/

void MODBUS_UART_RxCallback(UART_HandleTypeDef *huart)
{
    RxBuffer_t *rx;
    uint8_t received_byte;

    if (huart->Instance == USART1) {
        rx = &s_rx1;
        received_byte = s_rx_byte1;
    } else if (huart->Instance == USART2) {
        rx = &s_rx2;
        received_byte = s_rx_byte2;
    } else {
        return;
    }

    if (rx->len < MODBUS_RX_BUF_SIZE) {
        rx->buf[rx->len++] = received_byte;
    }
    rx->last_byte_tick = TaskScheduler_GetTick();

    /* 重新启动中断接收 */
    if (huart->Instance == USART1)
        HAL_UART_Receive_IT(huart, &s_rx_byte1, 1);
    else
        HAL_UART_Receive_IT(huart, &s_rx_byte2, 1);
}

/*====================================================================*/
/* 错误恢复 (从 HAL_UART_ErrorCallback 调用)                           */
/*                                                                     */
/* ORE 等错误发生后, HAL 不会走 RxCpltCallback, 接收链停止重启会导致   */
/* 串口彻底变聋。此处丢弃当前半截帧并强制重启单字节中断接收, 救活链路。 */
/*====================================================================*/

void MODBUS_UART_ErrorRestart(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        s_rx1.len = 0;
        s_rx1.ready = 0;
        HAL_UART_Receive_IT(huart, &s_rx_byte1, 1);
    } else if (huart->Instance == USART2) {
        s_rx2.len = 0;
        s_rx2.ready = 0;
        HAL_UART_Receive_IT(huart, &s_rx_byte2, 1);
    }
}

/*====================================================================*/
/* API                                                                 */
/*====================================================================*/

void MODBUS_Init(UART_HandleTypeDef *huart1_p, UART_HandleTypeDef *huart2_p,
                 uint8_t slave_id)
{
    (void)huart1_p;
    (void)huart2_p;
    s_slave_id = slave_id;

    memset(s_regs, 0, sizeof(s_regs));
    memset(&s_rx1, 0, sizeof(s_rx1));
    memset(&s_rx2, 0, sizeof(s_rx2));

    /* 启动中断接收 */
    HAL_UART_Receive_IT(&huart1, &s_rx_byte1, 1);
    HAL_UART_Receive_IT(&huart2, &s_rx_byte2, 1);
}

void MODBUS_Process(void)
{
    check_frame_complete(&s_rx1, &huart1);
    check_frame_complete(&s_rx2, &huart2);
}

void MODBUS_SetRegister(uint16_t reg_idx, uint16_t value)
{
    if (reg_idx < MODBUS_REG_COUNT)
        s_regs[reg_idx] = value;
}

uint16_t MODBUS_GetRegister(uint16_t reg_idx)
{
    if (reg_idx < MODBUS_REG_COUNT)
        return s_regs[reg_idx];
    return 0;
}
