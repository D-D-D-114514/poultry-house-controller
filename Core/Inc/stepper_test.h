#ifndef STEPPER_TEST_H
#define STEPPER_TEST_H

/*
 * stepper_test.h - 步进电机自动扫频测试
 *
 * 上电后从低频逐级升到高频, 每档停留约 3 秒, OLED 显示当前频率/ARR/状态。
 * 用于观察电机从哪个频率开始失步(假堵转), 不依赖按键, 全自动循环。
 * 由调度器每 10ms 调用 STEPPER_RunTest()。
 */

void STEPPER_RunTest(void);

#endif /* STEPPER_TEST_H */
