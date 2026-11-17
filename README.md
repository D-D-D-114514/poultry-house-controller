# 智能鸡舍环境控制终端（STM32F103）

> 一台 STM32F103RCT6 的课程设计：**Modbus RTU 从机 + 12 键 ADC 电阻分压输入 + HX711 称重 + 双轴步进定量投料 + OLED 菜单**，手动/自动双模式，内置虚拟环境与演示状态机，不接硬件也能跑通整条流程。

---

## 一、它解决什么问题

小型养殖房需要「定时定量投料 + 环境告警」，但现场只有一台 PLC 或一台便宜单片机。这套固件把常见的 HMI（按键 + OLED）、称重、执行机构（步进、舵机、电磁阀）和上位机通信（Modbus）全部塞进一颗 F103，并且：

- **不阻塞**：10 ms 时基的任务调度，替代 `HAL_Delay` 式阻塞延时
- **不失控**：手动模式优先，10 s 无操作自动回到自动模式
- **不依赖硬件**：虚拟环境 + 虚拟 RTC + 21 步演示状态机，答辩与回归测试不必接实物

## 二、系统结构

```
        ┌─────────────── STM32F103RCT6 ───────────────┐
PC/组态王 │ USART1/2 9600 8N1 ◄──► Modbus RTU 从机      │
         │                    (TIM2 4ms 超时断帧)      │
         │                                              │
         │  PC4 ── ADC 电阻分压网络 ── 12 按键            │
         │  I2C ── SSD1306 OLED（分级菜单）              │
         │  GPIO ── HX711 24 位称重（去皮标定）           │
         │  TIM ── TB6600 X/Y 双轴步进（2 kHz 脉冲）      │
         │  GPIO/PWM ── SG90 通风窗、2 路电磁阀、加湿阀    │
         └──────────────────────────────────────────────┘
```

## 三、关键技术点

| 问题 | 做法 |
|---|---|
| 串口帧边界不确定 | TIM2 4 ms 超时模拟 Modbus RTU 的 3.5 字符空闲，收到字节即重置定时器 |
| IO 不够用 | 12 个按键只用 1 路 ADC：电阻分压网络，每个键对应唯一 ADC 区间，省 11 个 IO |
| 主循环被延时阻塞 | 自建 10 ms 时基调度器，采集/控制/显示/通信/报警五类任务按周期轮转 |
| 手动与自动打架 | `g_manual_active` 让自动逻辑直接返回；手动 10 s 无操作自动释放 |
| 没有硬件也要演示 | 虚拟环境/虚拟 RTC + 21 步状态机（`demo_mode.c`），`#define DEMO_MODE 1` 即可切换 |

## 四、仓库结构

```
Core/Src/       应用代码
  task_scheduler.c   10 ms 调度器
  keypad.c           ADC 12 键识别
  modbus_rtu.c       Modbus RTU 从机（收帧/CRC/应答）
  modbus_data.c      35 个保持寄存器的映射
  hx711.c            24 位称重与去皮
  stepper.c          X/Y 双轴步进
  auto_control.c     自动控制策略
  manual_control.c   手动模式
  alarm_monitor.c    告警监控
  demo_mode.c        21 步演示状态机
  virtual_env.c/.h   虚拟传感器
  virtual_rtc.c/.h   虚拟时钟
Core/Inc/       对应头文件
Drivers/        STM32 HAL + CMSIS（ST 官方）
cmake/          GCC/Clang 工具链文件
```

## 五、构建

```bash
cmake --preset Release
cmake --build build/Release
# 产物 build/Release/test5.elf → ST-Link 烧录
```

## 六、文档

- `手动模式控制逻辑说明书.md` — 模式切换、按键映射表、ADC 区间
- `演示模式流程说明书.md` — 21 步演示流程与状态机设计
- `配电电器清单及逻辑清单..md` — IO 分配总表、外围器件清单
- `ADC按键电路设计图.md` — 单 ADC 多键电路原理与实测值
- `PROJECT_ARCHITECTURE_FOR_AI.md` — 架构速览（给协作者/后续维护用）

---
作者：韩昊峻 · 河池学院 电气工程及其自动化 · 2027 届

## 许可证

本项目采用 [MIT License](LICENSE)。

仓库内含 ST 官方代码：`Drivers/CMSIS/`、`Drivers/STM32F1xx_HAL_Driver/`，
版权归 STMicroelectronics，按其自带许可证（BSD-3-Clause）分发，原始 `LICENSE.txt` 已保留。
