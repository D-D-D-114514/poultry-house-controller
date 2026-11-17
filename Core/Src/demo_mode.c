/*
 * demo_mode.c - 整机功能演示 (直接驱动 Modbus 寄存器 + 实物执行器)
 *
 * 设计要点:
 *  1) 演示模式下 main.c 不注册 Task_ModbusServer, 由本模块每 tick 直接写
 *     全部 35 个 Modbus 寄存器(MODBUS_SetRegister), 保证组态王读到的画面
 *     与实物执行器(LED/风扇/阀)完全一致、连贯无残留。
 *  2) 不读真实传感器/virtual_env, 全部用本文件内部模拟量驱动。
 *  3) 非阻塞: 调度器每 10ms 调一次 DEMO_Run(), 纯状态机, 无 HAL_Delay。
 *  4) 场景由 S1 按键循环切换; 每个场景"演示一遍后停在正常值"。
 *
 * 报警采用闩锁(latch)实现回滞: 越过触发阈值置位, 越过解除阈值复位,
 * 与组态王变量表要求的"触发点/解除点不同"一致。
 *
 * tick: 10ms 一次, 100 tick = 1 秒。
 */

#include "main.h"
#include "project_config.h"
#include "modbus_rtu.h"
#include "ssd1306.h"
#include "stepper.h"
#include "keypad.h"

#define TPS              100u     /* ticks per second */

/*====================================================================*/
/* 演示阈值 (用需求给的值, 独立于生产 project_config 阈值)              */
/*  温湿度均 ×10                                                       */
/*====================================================================*/
#define D_TEMP_NORMAL    240      /* 24.0C */
#define D_TEMP_HI_PEAK   300      /* 30.0C 高温峰值 */
#define D_TEMP_HI_ON     290      /* >29.0C 触发高温 */
#define D_TEMP_HI_OFF    250      /* <25.0C 解除高温 */
#define D_TEMP_LO_PEAK   170      /* 17.0C 低温谷值 */
#define D_TEMP_LO_ON     210      /* <21.0C 触发低温 */
#define D_TEMP_LO_OFF    230      /* >23.0C 解除低温 */

#define D_HUMI_NORMAL    500      /* 50.0% */
#define D_HUMI_HI_PEAK   700      /* 70.0% 高湿峰值 */
#define D_HUMI_HI_ON     650      /* >65.0% 触发高湿 */
#define D_HUMI_HI_OFF    550      /* <55.0% 解除高湿 */
#define D_HUMI_LO_PEAK   350      /* 35.0% 低湿谷值 */
#define D_HUMI_LO_ON     460      /* <46.0% 触发低湿 */
#define D_HUMI_LO_OFF    480      /* >48.0% 解除低湿 */

#define D_LIGHT_NORMAL   40       /* 正常光照值 */
#define D_LIGHT_DARK_VAL 20       /* 光照场景谷值 */
#define D_LIGHT_RECOVER  50       /* 开灯后回升值 */
#define D_LIGHT_DARK_TH  30       /* <30 判为暗 */

/* 阶段时长 (tick) */
#define T_RISE           300u     /* 3s 快速变化 */
#define T_HOLD           300u     /* 3s 峰值停留 */
#define T_FALL           600u     /* 6s 逐渐恢复 */
#define T_WATER_ON       1000u    /* 低水位供水 10s */
#define T_FAULT_WARN     2000u    /* 供水故障: 20s 后报警 */
#define T_FAULT_END      3000u    /* 供水故障: 30s 后全停 */

/* 低料位配送时序 (tick) */
#define T_FEED_TRAVEL1   1000u    /* 去程 10s (反转到位1) */
#define T_FEED_DWELL     1500u    /* 到位1停留至 15s */
#define T_FEED_TRAVEL2   2500u    /* 回程到 25s (正转回位2) */

/* 配送车故障时序 (tick): 料位低3, 15s 后配送车故障报警灯亮(保持) */
#define T_DELIV_FAULT_WARN  1500u

/* 供料故障时序 (tick):
   0~10s   反转去程(POS2未到位)
   10~20s  到位1(POS1)停, 20s 起供料故障报警灯亮
   20~30s  正转回程
   >30s    到位2(POS2), 电机停 */
#define T_FF_TRAVEL1     1000u    /* 反转去程 10s */
#define T_FF_DWELL       2000u    /* 到位1停留至 20s(报警触发) */
#define T_FF_TRAVEL2     3000u    /* 正转回程到 30s */

/* 电机演示频率: 实测 <=600Hz 不失步, 取 400Hz 免软启动 (见 stepper-stall) */
#define DEMO_MOTOR_HZ    400u

/*====================================================================*/
/* 场景定义                                                            */
/*====================================================================*/
typedef enum {
    SC_NORMAL = 0,
    SC_TEMP_HIGH,
    SC_TEMP_LOW,
    SC_HUMI_HIGH,
    SC_HUMI_LOW,
    SC_WATER1,
    SC_WATER2,
    SC_WATER3,
    SC_WATER_FAULT,
    SC_CLEANING,
    SC_LIGHT,
    SC_FEED1,
    SC_DELIVERY_FAULT,
    SC_FEED_FAULT
} SceneType_t;

static const SceneType_t s_sequence[] = {
    SC_NORMAL,
    SC_TEMP_HIGH,
    SC_TEMP_LOW,
    SC_HUMI_HIGH,
    SC_HUMI_LOW,
    SC_WATER1,
    SC_WATER2,
    SC_WATER3,
    SC_WATER_FAULT,
    SC_CLEANING,
    SC_LIGHT,
    SC_FEED1,
    SC_DELIVERY_FAULT,
    SC_FEED_FAULT
};
#define SEQ_COUNT (sizeof(s_sequence) / sizeof(s_sequence[0]))

/*====================================================================*/
/* 寄存器镜像 (每 tick 由场景填充, 再统一写出)                          */
/*====================================================================*/
typedef struct {
    int16_t  temp_x10;       /* reg0 */
    int16_t  humi_x10;       /* reg1 */
    uint8_t  light_dark;     /* reg2  0=亮 1=暗 */
    uint8_t  water_low[3];   /* reg3-5 */
    uint8_t  water_valve[3]; /* reg6-8 */
    uint8_t  water_warn;     /* reg9 */
    uint8_t  feed_low[3];    /* reg10-12 */
    uint8_t  motor_x;        /* reg13 */
    uint8_t  motor_y;        /* reg14 运行/停止 */
    uint8_t  motor_y_dir;    /* reg22 方向: 0=停 1=正转 2=反转 */
    uint8_t  pos[3];         /* reg15-17 */
    uint8_t  deliv_warn;     /* reg19 */
    uint8_t  feed_warn;      /* reg20 */
    uint8_t  vent;           /* reg21 */
    uint8_t  humi_fan;       /* reg23 */
    uint8_t  humidifier;     /* reg24 */
    uint8_t  humi_h;         /* reg25 */
    uint8_t  humi_l;         /* reg26 */
    uint8_t  temp_fan;       /* reg28 */
    uint8_t  temp_light;     /* reg29 */
    uint8_t  temp_h;         /* reg30 */
    uint8_t  temp_l;         /* reg31 */
    uint8_t  light_on;       /* reg32 */
    uint16_t light_val;      /* reg33 LIGHT_VALUE 模拟量 20~50 */
    uint8_t  clean;          /* 无寄存器, 仅实物 */
} RegImage_t;

/*====================================================================*/
/* 状态变量                                                            */
/*====================================================================*/
static uint8_t  s_seq_idx   = 0;
static uint32_t s_tick      = 0;     /* 进入当前场景后的 10ms 计数 */
static uint8_t  s_disp_div  = 0;
static uint8_t  s_latch     = 0;     /* 报警闩锁(温/湿/光场景复用) */
static uint8_t  s_zmotor_on = 0;     /* Z电机(配送)运行标志 */
static KeyCode_t s_last_key = KEY_NONE;

static SceneType_t cur_scene(void) { return s_sequence[s_seq_idx]; }

/*====================================================================*/
/* 舵机 (TIM1_CH1, 比较值=脉宽us): 2000=开窗, 0=松力                    */
/*====================================================================*/
extern TIM_HandleTypeDef htim1;
static void set_servo_pulse(uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}

/* 线性插值 a->b, 经过 span tick; t>=span 返回 b */
static int16_t lerp16(int16_t a, int16_t b, uint32_t t, uint32_t span)
{
    if (t >= span) return b;
    return (int16_t)(a + (int32_t)(b - a) * (int32_t)t / (int32_t)span);
}

/*====================================================================*/
/* Z 电机(配送车)启停 (演示用, 400Hz 免软启动)                         */
/*====================================================================*/
static void z_motor_set(uint8_t on, uint8_t forward)
{
    if (on) {
        if (!s_zmotor_on) {
            STEPPER_Start(&g_z_motor, forward, 0);
            STEPPER_SetFreq(&g_z_motor, DEMO_MOTOR_HZ);
            s_zmotor_on = 1;
        } else {
            STEPPER_SetDirection(&g_z_motor, forward);
        }
    } else if (s_zmotor_on) {
        STEPPER_Stop(&g_z_motor);
        s_zmotor_on = 0;
    }
}

/*====================================================================*/
/* 进入新场景: 复位计时/闩锁/电机/舵机                                  */
/*====================================================================*/
static void enter_scene(void)
{
    s_tick  = 0;
    s_latch = 0;
    z_motor_set(0, 1);
    set_servo_pulse(0);
}

/*====================================================================*/
/* 把镜像写入全部 Modbus 寄存器 (40001~40034 -> 索引0~33)              */
/*====================================================================*/
static void push_registers(const RegImage_t *r)
{
    MODBUS_SetRegister(0,  (uint16_t)r->temp_x10);
    MODBUS_SetRegister(1,  (uint16_t)r->humi_x10);
    MODBUS_SetRegister(2,  r->light_dark);
    MODBUS_SetRegister(3,  r->water_low[0]);
    MODBUS_SetRegister(4,  r->water_low[1]);
    MODBUS_SetRegister(5,  r->water_low[2]);
    MODBUS_SetRegister(6,  r->water_valve[0]);
    MODBUS_SetRegister(7,  r->water_valve[1]);
    MODBUS_SetRegister(8,  r->water_valve[2]);
    MODBUS_SetRegister(9,  r->water_warn);
    MODBUS_SetRegister(10, r->feed_low[0]);
    MODBUS_SetRegister(11, r->feed_low[1]);
    MODBUS_SetRegister(12, r->feed_low[2]);
    MODBUS_SetRegister(13, r->motor_x);
    MODBUS_SetRegister(14, r->motor_y);
    MODBUS_SetRegister(15, r->pos[0]);
    MODBUS_SetRegister(16, r->pos[1]);
    MODBUS_SetRegister(17, r->pos[2]);
    MODBUS_SetRegister(18, 0);              /* HOPPER 废除 */
    MODBUS_SetRegister(19, r->deliv_warn);
    MODBUS_SetRegister(20, r->feed_warn);
    MODBUS_SetRegister(21, r->vent);
    MODBUS_SetRegister(22, r->motor_y_dir); /* Y电机方向 0停/1正/2反 */
    MODBUS_SetRegister(23, r->humi_fan);
    MODBUS_SetRegister(24, r->humidifier);
    MODBUS_SetRegister(25, r->humi_h);
    MODBUS_SetRegister(26, r->humi_l);
    MODBUS_SetRegister(27, 0);              /* 预留 */
    MODBUS_SetRegister(28, r->temp_fan);
    MODBUS_SetRegister(29, r->temp_light);
    MODBUS_SetRegister(30, r->temp_h);
    MODBUS_SetRegister(31, r->temp_l);
    MODBUS_SetRegister(32, r->light_on);
    MODBUS_SetRegister(33, r->light_val);   /* LIGHT_VALUE 模拟量 */
    MODBUS_SetRegister(34, 0);              /* 清洗(超35寄存器范围, 置0) */
}

/* 按布尔值开/关一个执行器 GPIO */
#define ACT_SET(name, on) do { if (on) ACT_ON(name); else ACT_OFF(name); } while (0)

/*====================================================================*/
/* 把镜像同步到实物执行器 GPIO (让现场 LED/风扇/阀 跟随演示)            */
/*====================================================================*/
static void push_actuators(const RegImage_t *r)
{
    ACT_SET(COOLING_FAN,    r->temp_fan);
    ACT_SET(TEMP_HIGH_WARN, r->temp_h);
    ACT_SET(INCANDESCENT,   r->temp_light);
    ACT_SET(TEMP_LOW_WARN,  r->temp_l);

    ACT_SET(VENT_FAN,       r->humi_fan);
    ACT_SET(HUMI_HIGH_WARN, r->humi_h);
    ACT_SET(HUMIDIFIER,     r->humidifier);
    ACT_SET(HUMI_LOW_WARN,  r->humi_l);

    ACT_SET(VALVE1,         r->water_valve[0]);
    ACT_SET(VALVE2,         r->water_valve[1]);
    ACT_SET(VALVE3,         r->water_valve[2]);
    ACT_SET(WATER_LED1,     r->water_low[0]);
    ACT_SET(WATER_LED2,     r->water_low[1]);
    ACT_SET(WATER_LED3,     r->water_low[2]);
    ACT_SET(WATER_FAULT,    r->water_warn);

    ACT_SET(FEED_LED1,      r->feed_low[0]);
    ACT_SET(FEED_LED2,      r->feed_low[1]);
    ACT_SET(FEED_LED3,      r->feed_low[2]);
    ACT_SET(FEED_FAULT,     r->feed_warn);
    ACT_SET(DELIVERY_FAULT, r->deliv_warn);

    ACT_SET(FLUORESCENT,    r->light_on);
    ACT_SET(CLEAN_VALVE,    r->clean);

    set_servo_pulse(r->vent ? 2000 : 0);
}

/*====================================================================*/
/* 各场景: 根据 s_tick 计算镜像                                        */
/*====================================================================*/
static void scene_normal(RegImage_t *r)
{
    /* 正常: 温24/湿50/光40, 配送车到位传感器2(POS2)激活, 其余全0 */
    r->temp_x10  = D_TEMP_NORMAL;
    r->humi_x10  = D_HUMI_NORMAL;
    r->light_val = D_LIGHT_NORMAL;
    r->pos[1]    = 1;     /* POS2 激活 */
}

static void scene_temp_high(RegImage_t *r)
{
    int16_t t;
    if (s_tick < T_RISE)
        t = lerp16(D_TEMP_NORMAL, D_TEMP_HI_PEAK, s_tick, T_RISE);
    else if (s_tick < T_RISE + T_HOLD)
        t = D_TEMP_HI_PEAK + (((s_tick / 25) & 1) ? 3 : -3);   /* 峰值微波动 */
    else
        t = lerp16(D_TEMP_HI_PEAK, D_TEMP_NORMAL, s_tick - T_RISE - T_HOLD, T_FALL);

    if (t > D_TEMP_HI_ON)  s_latch = 1;
    if (t < D_TEMP_HI_OFF) s_latch = 0;

    r->temp_x10  = t;
    r->humi_x10  = D_HUMI_NORMAL;
    r->light_val = D_LIGHT_NORMAL;
    r->pos[1]    = 1;
    r->temp_h    = s_latch;
    r->temp_fan  = s_latch;
}

static void scene_temp_low(RegImage_t *r)
{
    int16_t t;
    if (s_tick < T_RISE)
        t = lerp16(D_TEMP_NORMAL, D_TEMP_LO_PEAK, s_tick, T_RISE);
    else if (s_tick < T_RISE + T_HOLD)
        t = D_TEMP_LO_PEAK + (((s_tick / 25) & 1) ? 3 : -3);
    else
        t = lerp16(D_TEMP_LO_PEAK, D_TEMP_NORMAL, s_tick - T_RISE - T_HOLD, T_FALL);

    if (t < D_TEMP_LO_ON)  s_latch = 1;
    if (t > D_TEMP_LO_OFF) s_latch = 0;

    r->temp_x10    = t;
    r->humi_x10    = D_HUMI_NORMAL;
    r->light_val   = D_LIGHT_NORMAL;
    r->pos[1]      = 1;
    r->temp_l      = s_latch;
    r->temp_light  = s_latch;   /* 白炽灯 */
}

static void scene_humi_high(RegImage_t *r)
{
    int16_t h;
    if (s_tick < T_RISE)
        h = lerp16(D_HUMI_NORMAL, D_HUMI_HI_PEAK, s_tick, T_RISE);
    else if (s_tick < T_RISE + T_HOLD)
        h = D_HUMI_HI_PEAK + (((s_tick / 25) & 1) ? 3 : -3);
    else
        h = lerp16(D_HUMI_HI_PEAK, D_HUMI_NORMAL, s_tick - T_RISE - T_HOLD, T_FALL);

    if (h > D_HUMI_HI_ON)  s_latch = 1;
    if (h < D_HUMI_HI_OFF) s_latch = 0;

    r->temp_x10  = D_TEMP_NORMAL;
    r->humi_x10  = h;
    r->light_val = D_LIGHT_NORMAL;
    r->pos[1]    = 1;
    r->humi_h    = s_latch;
    r->humi_fan  = s_latch;   /* 主动通风风扇 */
    r->vent      = s_latch;   /* 通风窗(舵机) */
}

static void scene_humi_low(RegImage_t *r)
{
    int16_t h;
    if (s_tick < T_RISE)
        h = lerp16(D_HUMI_NORMAL, D_HUMI_LO_PEAK, s_tick, T_RISE);
    else if (s_tick < T_RISE + T_HOLD)
        h = D_HUMI_LO_PEAK + (((s_tick / 25) & 1) ? 3 : -3);
    else
        h = lerp16(D_HUMI_LO_PEAK, D_HUMI_NORMAL, s_tick - T_RISE - T_HOLD, T_FALL);

    if (h < D_HUMI_LO_ON)  s_latch = 1;
    if (h > D_HUMI_LO_OFF) s_latch = 0;

    r->temp_x10    = D_TEMP_NORMAL;
    r->humi_x10    = h;
    r->light_val   = D_LIGHT_NORMAL;
    r->pos[1]      = 1;
    r->humi_l      = s_latch;
    r->humidifier  = s_latch;   /* 加湿器 */
}

/* 低水位 i (0/1/2): 供水 10s 后全停 */
static void scene_water(RegImage_t *r, uint8_t i)
{
    if (s_tick < T_WATER_ON) {
        r->water_low[i]   = 1;
        r->water_valve[i] = 1;
    }
}

static void scene_water_fault(RegImage_t *r)
{
    /* 低水位1触发供水; 20s 后供水故障报警; 30s 后全停 */
    if (s_tick < T_FAULT_END) {
        r->water_low[0]   = 1;
        r->water_valve[0] = 1;
        if (s_tick >= T_FAULT_WARN)
            r->water_warn = 1;
    }
}

static void scene_cleaning(RegImage_t *r)
{
    /* 清洁指示 + 清洁电磁阀 (无对应寄存器, 仅实物) */
    r->temp_x10  = D_TEMP_NORMAL;
    r->humi_x10  = D_HUMI_NORMAL;
    r->light_val = D_LIGHT_NORMAL;
    r->pos[1]    = 1;
    r->clean     = 1;
}

static void scene_light(RegImage_t *r)
{
    uint16_t v;
    /* 光照值: 40 -> 20 (变暗开灯) -> 50 */
    if (s_tick < T_RISE)
        v = (uint16_t)lerp16(D_LIGHT_NORMAL, D_LIGHT_DARK_VAL, s_tick, T_RISE);
    else
        v = (uint16_t)lerp16(D_LIGHT_DARK_VAL, D_LIGHT_RECOVER, s_tick - T_RISE, T_FALL);

    if (v < D_LIGHT_DARK_TH) s_latch = 1;   /* 暗 -> 开灯(锁存) */

    r->temp_x10   = D_TEMP_NORMAL;
    r->humi_x10   = D_HUMI_NORMAL;
    r->pos[1]     = 1;
    r->light_val  = v;
    r->light_dark = (v < D_LIGHT_DARK_TH) ? 1 : 0;
    r->light_on   = s_latch;   /* 日光灯: 一旦变暗点亮并保持 */
}

static void scene_feed1(RegImage_t *r)
{
    /* 低料位1配送时序:
       0~10s: 料位低1+料位灯, Y(Z)电机反转去程, POS全不到位
       10~15s: 到位1(POS1), 电机停, 料位仍低
       15~25s: 料位恢复, Y电机正转回程, 不到位
       >25s:   到位2(POS2)正常, 电机停 */
    r->temp_x10  = D_TEMP_NORMAL;
    r->humi_x10  = D_HUMI_NORMAL;
    r->light_val = D_LIGHT_NORMAL;

    if (s_tick < T_FEED_TRAVEL1) {
        r->feed_low[0] = 1;
        r->motor_y     = 1;          /* 反转去程 */
        r->motor_y_dir = 2;          /* 2=反转 */
        z_motor_set(1, 0);
    } else if (s_tick < T_FEED_DWELL) {
        r->feed_low[0] = 1;
        r->pos[0]      = 1;          /* 到位1 */
        r->motor_y     = 0;
        z_motor_set(0, 0);
    } else if (s_tick < T_FEED_TRAVEL2) {
        /* 料位恢复, 正转回程 */
        r->motor_y     = 1;
        r->motor_y_dir = 1;          /* 1=正转 */
        z_motor_set(1, 1);
    } else {
        r->pos[1]      = 1;          /* 回到位2(正常) */
        r->motor_y     = 0;
        z_motor_set(0, 1);
    }
}

static void scene_delivery_fault(RegImage_t *r)
{
    /* 配送车故障: 料位低3 + 料位灯3, Y电机无动作,
       15s 后配送车故障报警灯亮并保持 */
    r->temp_x10  = D_TEMP_NORMAL;
    r->humi_x10  = D_HUMI_NORMAL;
    r->light_val = D_LIGHT_NORMAL;
    r->pos[1]    = 1;                /* 配送车停在正常位2 */

    r->feed_low[2] = 1;              /* 料位低3 (驱动 FEED_LED3) */
    /* motor_y 保持 0 (无动作) */
    if (s_tick >= T_DELIV_FAULT_WARN)
        r->deliv_warn = 1;           /* 配送车故障报警灯 */
}

static void scene_feed_fault(RegImage_t *r)
{
    /* 供料故障:
       0~10s   料位低1+料位灯, Y电机反转去程, POS2未到位
       10~20s  到位1(POS1)停; 20s 起供料故障报警灯亮
       20~30s  Y电机正转回程
       >30s    到位2(POS2), 电机停 */
    r->temp_x10  = D_TEMP_NORMAL;
    r->humi_x10  = D_HUMI_NORMAL;
    r->light_val = D_LIGHT_NORMAL;

    r->feed_low[0] = 1;              /* 料位低1 全程保持 */

    if (s_tick < T_FF_TRAVEL1) {
        r->motor_y = 1;              /* 反转去程 */
        r->motor_y_dir = 2;          /* 2=反转 */
        z_motor_set(1, 0);
    } else if (s_tick < T_FF_DWELL) {
        r->pos[0]  = 1;              /* 到位1 */
        r->motor_y = 0;
        z_motor_set(0, 0);
    } else if (s_tick < T_FF_TRAVEL2) {
        r->pos[0]    = 1;            /* 仍在位1, 报警已触发 */
        r->feed_warn = 1;            /* 供料故障报警灯 */
        r->motor_y   = 1;            /* 正转回程 */
        r->motor_y_dir = 1;          /* 1=正转 */
        z_motor_set(1, 1);
    } else {
        r->feed_warn = 1;            /* 故障保持(料位仍低) */
        r->pos[1]    = 1;            /* 回到位2 */
        r->motor_y   = 0;
        z_motor_set(0, 1);
    }
}

/*====================================================================*/
/* OLED (降频, 不用 snprintf, 只显示场景名)                            */
/*====================================================================*/
static const char *scene_name(SceneType_t sc)
{
    switch (sc) {
    case SC_NORMAL:      return "NORMAL";
    case SC_TEMP_HIGH:   return "HI TEMP";
    case SC_TEMP_LOW:    return "LO TEMP";
    case SC_HUMI_HIGH:   return "HI HUMI";
    case SC_HUMI_LOW:    return "LO HUMI";
    case SC_WATER1:      return "LO WATER 1";
    case SC_WATER2:      return "LO WATER 2";
    case SC_WATER3:      return "LO WATER 3";
    case SC_WATER_FAULT: return "WATER FAULT";
    case SC_CLEANING:    return "CLEANING";
    case SC_LIGHT:       return "LIGHT/NIGHT";
    case SC_FEED1:       return "LO FEED 1";
    case SC_DELIVERY_FAULT: return "DELIVERY FAULT";
    case SC_FEED_FAULT:     return "FEED FAULT";
    default:             return "DEMO";
    }
}

static void draw_oled(void)
{
    if (++s_disp_div < 25) return;   /* 250ms 刷一次 */
    s_disp_div = 0;

    SSD1306_Clear();
    SSD1306_DrawString(0, 0,  (char *)scene_name(cur_scene()));
    SSD1306_DrawString(0, 24, "S1: NEXT SCENE");
    SSD1306_Display();
}

/*====================================================================*/
/* 当前场景 -> 镜像                                                    */
/*====================================================================*/
static void build_image(RegImage_t *r)
{
    /* 全部清零(默认正常态的非激活值), 再按场景填充 */
    RegImage_t zero = {0};
    *r = zero;
    r->temp_x10  = D_TEMP_NORMAL;
    r->humi_x10  = D_HUMI_NORMAL;
    r->light_val = D_LIGHT_NORMAL;

    switch (cur_scene()) {
    case SC_NORMAL:      scene_normal(r);       break;
    case SC_TEMP_HIGH:   scene_temp_high(r);    break;
    case SC_TEMP_LOW:    scene_temp_low(r);     break;
    case SC_HUMI_HIGH:   scene_humi_high(r);    break;
    case SC_HUMI_LOW:    scene_humi_low(r);     break;
    case SC_WATER1:      scene_water(r, 0);     break;
    case SC_WATER2:      scene_water(r, 1);     break;
    case SC_WATER3:      scene_water(r, 2);     break;
    case SC_WATER_FAULT: scene_water_fault(r);  break;
    case SC_CLEANING:    scene_cleaning(r);     break;
    case SC_LIGHT:       scene_light(r);        break;
    case SC_FEED1:       scene_feed1(r);        break;
    case SC_DELIVERY_FAULT: scene_delivery_fault(r); break;
    case SC_FEED_FAULT:     scene_feed_fault(r);     break;
    default: break;
    }
}

/*====================================================================*/
/* API                                                                 */
/*====================================================================*/
void DEMO_Init(void)
{
    s_seq_idx   = 0;
    s_tick      = 0;
    s_disp_div  = 0;
    s_latch     = 0;
    s_zmotor_on = 0;
    s_last_key  = KEY_NONE;
    enter_scene();
}

void DEMO_Run(void)
{
    RegImage_t img;

    build_image(&img);
    push_registers(&img);
    push_actuators(&img);

    /* S1 上升沿 -> 下一场景 */
    KeyCode_t key = KEYPAD_Scan();
    if (key == KEY_S1 && s_last_key != KEY_S1) {
        s_seq_idx = (uint8_t)((s_seq_idx + 1) % SEQ_COUNT);
        enter_scene();
        s_disp_div = 24;   /* 立即刷新 OLED */
    }
    s_last_key = key;

    s_tick++;
    draw_oled();
}
