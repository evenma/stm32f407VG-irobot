# global_conf.h 修正报告

## ✅ 修正概览

根据你提供的 iHomeRobot 项目介绍文档，我已经全面修正了 `global_conf.h` 中的所有引脚定义。

---

## 📋 主要修正内容

### 1. 超声波传感器方案选择 ✅

| 配置项 | 原值 | 修正后 | 说明 |
|--------|------|--------|------|
| ULTRASONIC_485 | 未启用（注释） | **已启用** | 使用 RS485 级联 7 个防水超声波 |
| ULTRASONIC_GPIO | 已启用 | **已注释** | 临时调试方案，不使用 |

### 2. 泵浦控制配置 ✅

| 配置项 | 原值 | 修正后 | 说明 |
|--------|------|--------|------|
| **小水泵** | PE1 | **PA0** (TIM5_CH1) | PWM0 - 硬件 PWM 控制 |
| **大水泵** | 未定义 | **PE2** | MOSFET 控制，程序模拟 8KHz PWM |
| 加热管检测 | 未定义 | **PA6** (ADC1_IN6) | 检测供电接口连接情况 |

### 3. 马桶盖/圈电机配置 ✅（新增）

```c
// 马桶盖电机
#define LID_MOTOR_ENABLE_PIN    GET_PIN(E, 1)     // MOSFET 使能
#define LID_MOTOR_DIR_PIN       GET_PIN(E, 0)     // 方向继电器
#define LID_MOTOR_POS_ADC       GET_PIN(C, 2)     // 位置反馈 ADC1_CH12

// 马桶圈电机
#define RING_MOTOR_ENABLE_PIN   GET_PIN(B, 9)     // MOSFET 使能
#define RING_MOTOR_DIR_PIN      GET_PIN(B, 8)     // 方向继电器
#define RING_MOTOR_POS_ADC      GET_PIN(C, 1)     // 位置反馈 ADC1_CH11
```

### 4. 步进电机驱动重排 ✅

根据你的文档，清洁杆和管路分配器的步进电机引脚需要调整：

```c
// 清洁杆（PD8-PD11 -> PD12-PD15）
#define CLEANING_STEPPER_STEP   GET_PIN(D, 15)    // ULN2803 Out1
#define CLEANING_STEPPER_DIR    GET_PIN(D, 14)    // ULN2803 Out2
#define CLEANING_STEPPER_PULSE  GET_PIN(D, 13)    // ULN2803 Out3
#define CLEANING_STEPPER_EN     GET_PIN(D, 12)    // ULN2803 Out4

// 管路分配器（PD12-PD15 -> PD8-PD11）
#define TUBE_STEPPER_STEP       GET_PIN(D, 11)    // ULN2803 Out5
#define TUBE_STEPPER_DIR        GET_PIN(D, 10)    // ULN2803 Out6
#define TUBE_STEPPER_PULSE      GET_PIN(D, 9)     // ULN2803 Out7
#define TUBE_STEPPER_EN         GET_PIN(D, 8)     // ULN2803 Out8
```

### 5. 其他重要修正 ✅

| 配置项 | 修正说明 |
|--------|----------|
| CAN_BUS_PORT | USART2 → **CAN1** (PA11/PA12) |
| OLED_RST_PIN | 已注释（硬件 reset 不需要软件控制） |
| HEADLIGHT_PWM | 注释修正为 TIM5_CH2 |
| HEATER_DETECT_ADC | 新增 PA6，用于检测加热管供电接口 |
| OVER_CURRENT_PIN | 新增 PE7，过流检测 EXTI7 |
| SEWAGE_MOTOR_* | 污物口电机预留定义 |

---

## ⚠️ 需要注意的问题

### 1. 蜂鸣器与加热管共用 PE6

```c
#define BUZZER_PIN              GET_PIN(E, 6)
#define HEATER_CTRL_PIN         GET_PIN(E, 6)  // 冲突！
```

**问题**：两个功能都使用 PE6，这可能导致硬件冲突。
**建议**：确认 PCB 设计是否真的共用此引脚，或修改其中一个到其它 GPIO。

### 2. ADC 通道复用问题

```c
#define CLIFF_REAR_ADC_PIN      GET_PIN(C, 4)     // ADC1_IN14
#define WATER_TEMP_NTC_PIN      GET_PIN(C, 5)     // 也用了 ADC1_IN14？
```

**注意**：你的文档中提到"PC5 = ADC1_IN14（注意与后悬崖共用 ADC 通道）"，这说明是**时分复用**设计，需要在软件中切换使用。

### 3. 超声波方案选择

当前配置使用 `ULTRASONIC_485`，但某些驱动代码可能还引用着 `ULTRASONIC_GPIO` 的定义，需要检查相关驱动文件。

---

## 📦 所有引脚汇总

### GPIO 资源分配总览

| 端口 | 引脚 | 用途 |
|------|------|------|
| PA0 | TIM5_CH1 | 小水泵 PWM0 |
| PA1 | TIM5_CH2 | 左大灯 PWM1 |
| PA2 | USART2_TX | TypeC 上位机通讯 TX |
| PA3 | USART2_RX | TypeC 上位机通讯 RX |
| PA4 | ADC1_IN4 | 充电器电流采样 |
| PA5 | ADC1_IN5 | 充电器电压检测 |
| PA6 | ADC1_IN6 | 加热管接口检测 |
| PA7 | ADC1_IN7 | 前悬崖传感器 |
| PA8 | GPIO | HC-SR04 Trig（备用方案） |
| PA9 | USART1_TX | 调试控制台 TX |
| PA10 | USART1_RX | 调试控制台 RX |
| PA11 | CAN1_RX | CAN 总线接收 |
| PA12 | CAN1_TX | CAN 总线发送 |
| PA15 | GPIO | UV LED |

| PB0 | GPIO | 高水位浮球开关 |
| PB1 | GPIO | 低水位浮球开关 |
| PB3 | GPIO | 红外灯 |
| PB4 | GPIO | 暖风风扇 |
| PB5 | GPIO | IMU INT2 中断 |
| PB6 | I2C1_SCL | QMI8658 I2C 时钟 |
| PB7 | I2C1_SDA | QMI8658 I2C 数据 |
| PB8 | GPIO | 马桶圈电机方向 |
| PB9 | GPIO | 马桶圈电机使能 |
| PB10 | I2C2_SCL | AI 语音模块时钟 |
| PB11 | I2C2_SDA | AI 语音模块数据 |
| PB12 | SPI2_NSS | OLED CS |
| PB13 | SPI2_SCK | OLED SCK / LED SRCLK |
| PB14 | GPIO | OLED DC |
| PB15 | SPI2_MOSI | OLED MOSI / LED SER |

| PC0 | ADC1_IN10 | 污物口电机位置反馈（预留） |
| PC1 | ADC1_IN11 | 马桶圈电机位置反馈 |
| PC2 | ADC1_IN12 | 马桶盖电机位置反馈 |
| PC3 | ADC1_IN13 | 电池电压检测 |
| PC4 | ADC1_IN14 | 后悬崖传感器 |
| PC5 | ADC1_IN14 | 水温传感器（分时复用） |
| PC6 | GPIO | HC-SR04 Echo（备用方案） |
| PC7 | GPIO | 74HC138 A（备用方案） |
| PC8 | GPIO | 74HC138 B（备用方案） |
| PC9 | GPIO | 74HC138 C（备用方案） |
| PC10 | UART4_RX | RS485 超声波接收 |
| PC11 | UART4_TX | RS485 超声波发送 / USART5_RX |
| PC12 | GPIO | USART5_TX |
| PC13 | GPIO | SW3 上一页 |
| PC14 | GPIO | SW4 下一页 |
| PC15 | GPIO | SW5 未用 |

| PD0 | GPIO | RS485 DE/RE 切换 |
| PD1 | GPIO | 蓝牙 WAKE/PAIR |
| PD3 | GPIO | 污物口电机方向（预留） |
| PD4 | GPIO | 污物口电机使能（预留） |
| PD5 | GPIO | 水阀预留 1 |
| PD6 | GPIO | 水阀预留 2 |
| PD7 | GPIO | 水阀预留 3 |
| PD8 | GPIO | 管路分配器步进 EN |
| PD9 | GPIO | 管路分配器步进 PULSE |
| PD10 | GPIO | 管路分配器步进 DIR |
| PD11 | GPIO | 管路分配器步进 STEP |
| PD12 | GPIO | 清洁杆步进 EN |
| PD13 | GPIO | 清洁杆步进 PULSE |
| PD14 | GPIO | 清洁杆步进 DIR |
| PD15 | GPIO | 清洁杆步进 STEP |

| PE0 | GPIO | 马桶盖电机方向 |
| PE1 | GPIO | 马桶盖电机使能 / 大水泵（旧代码） |
| PE2 | GPIO | 大水泵 |
| PE3 | GPIO | 暖风电热丝 |
| PE4 | GPIO | 充电 MOSFET 控制 |
| PE6 | GPIO | 蜂鸣器 / 加热管（冲突！） |
| PE7 | GPIO | 过流检测 EXTI7 |
| PE9 | GPIO | Home 键充电座返回 |
| PE10 | GPIO | 旋钮按钮冲洗 |
| PE11 | GPIO | 旋钮女士清洁 |
| PE12 | GPIO | 旋钮臀部清洁 |
| PE13 | GPIO | 充电对射管 1 |
| PE14 | GPIO | 充电对射管 2 |
| PE15 | GPIO | 充电对射管 3 |

---

## 🔧 下一步操作建议

1. **编译测试**：尝试编译确认没有语法错误
2. **检查驱动**：确保所有硬件驱动使用正确的引脚定义
3. **硬件确认**：确认 PE6 是否真的同时用于蜂鸣器和加热管
4. **ADC 分时复用**：实现后悬崖和水温传感器的 ADC 切换逻辑

---

*修正完成时间：2026-03-16*  
*作者：Wuji (AI Assistant)*
