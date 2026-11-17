# STM32 验收程序 (智能控制终端) - 项目架构与实现解析

这是一份专为 AI 助手（及后续开发者）编写的项目理解指南。本项目是一个基于 STM32F10x 系列微控制器的嵌入式应用，结合了多种传感器数据采集、执行器控制以及标准的 Modbus RTU 工业通讯协议。根据业务模块推测，这大概率是一个**智能饲喂/环境监测系统**（包含水循环、喂食、温湿度控制）。

## 1. 软硬件开发环境
*   **MCU**: STM32F103 系列 (本项目配置了 `STM32F103C8` 及 `STM32F103RC` 的目标)
*   **固件库**: 意法半导体标准外设库 (STM32F10x Standard Peripheral Library, 见 `Library/` 目录)
*   **开发 IDE**: Keil MDK 5 (`Project.uvprojx`)
*   **主要通讯机制**: Modbus RTU 从机（双路并发）

## 2. 目录结构概览
*   `Start/`: 启动文件和内核级代码 (如 `startup_stm32f10x_*.s`, `core_cm3.c`, `system_stm32f10x.c`)
*   `Library/`: STM32 标准外设驱动库 (GPIO, USART, TIM, I2C, SPI 等)
*   `User/`: 用户应用入口和中断管理 (`main.c`, `stm32f10x_it.c`)
*   `Hardware/`: **核心层**，包含所有传感器驱动、执行器控制、通讯协议和具体业务逻辑模块。

## 3. 核心功能模块解析 (Hardware 目录)

项目在架构上可以分为三层：**外设驱动层**、**业务逻辑层** 和 **通讯协议层**。

### 3.1 通讯协议层 (Modbus RTU 双路从机)
项目实现了一个解耦度高且稳健的 Modbus RTU Slave。
*   **底层硬件**: `usart1.c` (PA9/TX, PA10/RX) 和 `usart2.c` (PA2/TX, PA3/RX) 分别负责与两路组态王/上位机通讯。
*   **断帧机制**: `tim2.c` 配置了 4ms 超时中断。利用 Modbus 标准的“3.5个字符空闲时间断帧”原理，每当串口收到一字节数据就重置 TIM2；若连续 4ms 未收到数据，TIM2 溢出并判定一帧结束。
*   **协议解析**: `modbus_slave.c`。
    *   内置双缓冲区 (`rx_buf1`, `rx_buf2`) 互不干扰。
    *   暴露了 `holding_regs[35]` 全局共享数组，映射到 Modbus 的 `40001 ~ 40035` 地址。
    *   主要支持 `0x03` (读保持寄存器) 指令，并在 `main` 循环中需要轮询 `Modbus_Poll()`。

### 3.2 外设驱动层 (Sensors & Actuators)
包含了该控制系统所需的所有底层元器件控制逻辑：
*   **环境传感器**: `dht22.c/h` (单总线温湿度传感器)
*   **称重传感器**: `hx711.c/h` (24位 ADC，用于重量检测)
*   **人机交互**: 
    *   显示：`OLED.c/h` (I2C/SPI OLED 屏幕底层) 和 `oled_display.c/h` (上层 UI 显示逻辑)
    *   输入：`adc_key.c/h` (通过 ADC 实现的矩阵或多段按键)、`Key.c/h`、`button.c/h`
*   **电机与执行机构**:
    *   `stepper.c/h`: 步进电机驱动
    *   `sg90.c/h`: SG90 舵机控制 (基于 PWM)
    *   `Motor.c/h`, `dianji.c/h`: 直流电机/水泵或继电器控制
    *   `Pid.c/h`: PID 控制算法（可能用于恒温或精准定位控制）

### 3.3 业务逻辑层 (Business Logic)
在底层驱动之上，项目封装了三大核心业务场景：
*   **`water.c/h`**: 饮水/水位控制逻辑。
*   **`feed.c/h`**: 喂食控制逻辑 (结合 HX711 称重和 舵机/步进电机 实现精准下料)。
*   **`climate.c/h`**: 环境温湿度控制 (读取 DHT22 并执行风扇/加热棒控制)。

## 4. 给 AI 的后续开发/维护提示

1.  **中断服务函数 (IRQ) 的位置警告**:
    `USART1_IRQHandler`、`USART2_IRQHandler` 和 `TIM2_IRQHandler` 已经被硬编码在各自的驱动文件 (`usart1.c`, `usart2.c`, `tim2.c`) 中。**请勿**在 `stm32f10x_it.c` 中再次生成或编写同名中断函数，否则会引起 "Multiple definition" 链接错误。
2.  **数据流范式**:
    该系统的数据中心是 Modbus 的 `holding_regs`。业务逻辑层 (water, feed, climate) 应负责定时读取传感器 (如 DHT22, HX711)，并将数据通过 `Modbus_SetReg()` 更新到寄存器中；上位机只需通过串口读取对应的 Modbus 寄存器即可获取系统状态。同样，上位机下发的命令也应由主循环解析寄存器的变化来驱动硬件动作。
3.  **移植性**:
    底层硬件高度模块化。如需将通讯模块移植到新项目，只需提取 `modbus_slave`, `usart1/2`, `tim2` 即可，这 4 个模块构成了完整的双路 Modbus 通讯环。

---
*Created automatically for AI context ingestion and developer onboarding.*