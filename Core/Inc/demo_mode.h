#ifndef DEMO_MODE_H
#define DEMO_MODE_H

/*
 * demo_mode.h - 整机功能演示 (自包含 / 不依赖生产逻辑)
 *
 * 目的: 上电后自动循环演示各子系统(温度/湿度/水位/料位配送/光照),
 *       用模块内部的"虚拟变量"驱动真实执行器(风扇/灯/电磁阀 LED)+ OLED,
 *       不读取任何真实传感器, 不调用 auto_control / manual_control / modbus,
 *       因此相对主程序完全独立。
 *
 * 非阻塞: 纯状态机, 由调度器每 10ms 调用一次 DEMO_Run(), 内部不使用 HAL_Delay,
 *         不会卡死主循环。OLED 刷新已降频, 避免 I2C 累积阻塞。
 *
 * 启用方式: 在 main.c 把 DEMO_MODE 置 1 (与 STEPPER_TEST_MODE 同理, 互斥)。
 */

void DEMO_Init(void);
void DEMO_Run(void);

#endif /* DEMO_MODE_H */
