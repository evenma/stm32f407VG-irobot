# global_conf.h 修正报告 v3.0 (UTF-8)

## ✅ 修正完成时间：2026-03-16 15:30

根据用户反馈，已完成所有配置修正并重新上传至 GitHub。

---

## 📋 主要修正内容（v3.0）

### 1. ✅ UTF-8 编码修复

**问题**：之前的文件中文字符显示为乱码（文?等）
**解决**：完全使用 UTF-8 编码重新生成文件，确保中文注释正常显示

### 2. ✅ CAN 总线简化

**删除配置项：**
```c
// #define CAN_ISOLATION_EN_PIN    GET_PIN(D, 7) // 已删除 - 不需要
```

**说明**：CAN 总线仅需 2 线（PA11/CAN_RX, PA12/CAN_TX），不需要隔离使能引脚

### 3. ✅ LED 74HC595 引脚定义

**新增配置：**
```c
/**
 * @brief 74HC595 移位寄存器引脚定义
 *   PB13 = SPI_SCK -> SRCLK (移位时钟)
 *   PB15 = SPI_MOSI -> SER    (数据输入)
 *   PB2  = GPIO      -> RCLK  (输出锁存时钟)
 */
#define LED_SRCLK_PIN             GET_PIN(B, 13)    // SPI SCK - 74HC595 SRCLK
#define LED_SER_PIN               GET_PIN(B, 15)    // SPI MOSI - 74HC595 SER
#define LED_RCLK_PIN              GET_PIN(B, 2)     // GPIO - 74HC595 RCLK
```

### 4. ✅ 超声波 74HC138 译码器定义

**修改方式：**
```c
// 原定义（错误语法）：
// #define ULTRASONIC_SELECT pins {PC7, PC8, PC9}

// 新定义（正确方式）：
#define ULTRASONIC_A_PIN          GET_PIN(C, 7)     // 74HC138 A
#define ULTRASONIC_B_PIN          GET_PIN(C, 8)     // 74HC138 B
#define ULTRASONIC_C_PIN          GET_PIN(C, 9)     // 74HC138 C
```

### 5. ✅ ADC 通道完整配置

**所有模拟量传感器添加对应的 ADC 通道号：**

| 功能 | 引脚 | 新增 ADC 配置 |
|------|------|--------------|
| 充电器检测 | PA5 | `#define CHARGER_DETECT_ADC        ADC1_IN5` |
| 电流采样 | PA4 | `#define CHARGER_SAMPLE_ADC        ADC1_IN4` |
| 加热管检测 | PA6 | `#define HEATER_DETECT_PIN/ADC` |
| 马桶盖位置 | PC2 | `#define LID_MOTOR_POS_PIN/ADC` |
| 马桶圈位置 | PC1 | `#define RING_MOTOR_POS_PIN/ADC` |
| 前悬崖 | PA7 | `#define CLIFF_FRONT_PIN/ADC` |
| 后悬崖 | PC4 | `#define CLIFF_REAR_PIN/ADC` |
| 水温 NTC | PC5 | `#define WATER_TEMP_PIN/ADC` |
| 污物口位置 | PC0 | `#define SEWAGE_MOTOR_POS_PIN/ADC` |

### 6. ✅ UART2 上位机通讯引脚定义

**新增配置：**
```c
#define HOST_COMM_UART_PORT       USART2
#define HOST_COMM_TX_PIN          GET_PIN(A, 2)       // PA2 = USART2 TX
#define HOST_COMM_RX_PIN          GET_PIN(A, 3)       // PA3 = USART2 RX
```

同时为 UART1 也添加了引脚定义：
```c
#define DEBUG_UART_TX_PIN         GET_PIN(A, 9)       // PA9 = USART1 TX
#define DEBUG_UART_RX_PIN         GET_PIN(A, 10)      // PA10 = USART1 RX
```

### 7. ✅ 步进电机命名修正

**修改前（错误命名）：**
```c
#define CLEANING_STEPPER_STEP   GET_PIN(D, 15)
#define CLEANING_STEPPER_DIR    GET_PIN(D, 14)
#define CLEANING_STEPPER_PULSE  GET_PIN(D, 13)
#define CLEANING_STEPPER_EN     GET_PIN(D, 12)
```

**修改后（ULN2803 OUT 端口命名）：**
```c
/**
 * @brief 清洁杆步进电机（ULN2803 驱动）
 * PD15 = Out1, PD14 = Out2, PD13 = Out3, PD12 = Out4
 */
#define CLEANING_STEPPER_OUT1     GET_PIN(D, 15)      // ULN2803 Out1
#define CLEANING_STEPPER_OUT2     GET_PIN(D, 14)      // ULN2803 Out2
#define CLEANING_STEPPER_OUT3     GET_PIN(D, 13)      // ULN2803 Out3
#define CLEANING_STEPPER_OUT4     GET_PIN(D, 12)      // ULN2803 Out4

/**
 * @brief 管路分配器步进电机（ULN2803 驱动）
 * PD11 = Out5, PD10 = Out6, PD9 = Out7, PD8 = Out8
 */
#define TUBE_STEPPER_OUT5         GET_PIN(D, 11)      // ULN2803 Out5
#define TUBE_STEPPER_OUT6         GET_PIN(D, 10)      // ULN2803 Out6
#define TUBE_STEPPER_OUT7         GET_PIN(D, 9)       // ULN2803 Out7
#define TUBE_STEPPER_OUT8         GET_PIN(D, 8)       // ULN2803 Out8
```

### 8. ✅ 版本更新

```c
#define APP_VERSION_STRING        "1.0.2"  // 从 1.0.1 升级到 1.0.2
```

---

## 🎯 所有配置的最终状态

### ⚡ 大功率控制
| 功能 | 引脚 | ADC 配置 |
|------|------|----------|
| 加热管 MOSFET | PE5 | - |
| 加热管接口检测 | PA6 | ADC1_IN6 |
| 大水泵 MOSFET | PE2 | - |
| 小水泵 PWM | PA0 | TIM5_CH1 |
| 暖风电热丝 | PE3 | - |
| 充电 MOSFET | PE4 | - |

### 🔌 ADC 传感器（带通道号）
| 功能 | 引脚 | ADC 通道 |
|------|------|----------|
| 电池电压 | PC3 | ADC1_CH13 |
| 充电器电压 | PA5 | ADC1_IN5 |
| 充电器电流 | PA4 | ADC1_IN4 |
| 加热管检测 | PA6 | ADC1_IN6 |
| 前悬崖 | PA7 | ADC1_IN7 |
| 后悬崖 | PC4 | ADC1_IN14 |
| 水温 NTC | PC5 | ADC1_IN15 |
| 马桶盖位置 | PC2 | ADC1_CH12 |
| 马桶圈位置 | PC1 | ADC1_CH11 |
| 污物口位置 | PC0 | ADC1_CH10 |

### 💬 通信接口
| 功能 | 引脚 | 协议 |
|------|------|------|
| CAN 总线 | PA11/PA12 | CAN1 (2 线) |
| RS485 超声波 | PC10/PC11 | UART4 |
| TypeC 调试 | PA9/PA10 | USART1 + CH9102F |
| TypeC 上位机 | PA2/PA3 | USART2 + CH9102F |
| AI 语音 I2C | PB10/PB11 | I2C2 |
| IMU I2C | PB6/PB7 | I2C1 |
| 蓝牙手控 | PC11/PC12/PD1 | USART5 |

### 🏃 电机驱动
| 功能 | 引脚 | 类型 |
|------|------|------|
| 差速电机 | PA11/PA12 | CANopen ZLAC8015D |
| 清洁杆步进 | PD12-15 | ULN2803 OUT1-4 |
| 管路分配器 | PD8-11 | ULN2803 OUT5-8 |
| 马桶盖电机 | PE0/PE1/PC2 | MOSFET+继电器 |
| 马桶圈电机 | PB8/PB9/PC1 | MOSFET+继电器 |

### 💡 LED 与显示
| 功能 | 引脚 |
|------|------|
| OLED SPI | PB12-CS, PB13-SCK, PB14-DC, PB15-MOSI |
| LED 74HC595 | PB2-RCK, PB13-SRCLK, PB15-SER |
| Home 键 | PE9 |
| 超声波 74HC138 | PC7-A, PC8-B, PC9-C |

### 🔔 其他 IO
| 功能 | 引脚 |
|------|------|
| 蜂鸣器 | PE6 |
| UV LED | PA15 (OUT1) |
| 红外灯 | PB3 (OUT2) |
| 风扇 | PB4 (OUT3) |
| 过流检测 | PE7 (EXTI7) |
| 水位开关 | PB0(高), PB1(低) |
| 旋钮按钮 | PE10 |
| 对射管对准 | PE13/14/15 |
| 按键 SW3/SW4/SW5 | PC13/14/15 |

---

## 📁 上传的文件

### 1. **global_conf.h** - v1.0.2 最终版配置文件
- 路径：`applications/System/global_conf.h`
- 编码：**UTF-8** (中文注释正常)
- 版本：**1.0.2**

### 2. **GLOBAL_CONF_FIX_REPORT_V3.md** - 本次修正报告
- 包含：完整的修正清单、问题对比表

---

## ✅ 已修复的问题总结

### 之前的问题 → 现在的解决方案

| # | 问题 | 修复方案 |
|---|------|----------|
| 1 | 中文字符乱码 | 完全重写为 UTF-8 编码 |
| 2 | CAN 需要隔离引脚 | 删除 `CAN_ISOLATION_EN_PIN` |
| 3 | LED 没有 74HC595 引脚定义 | 新增 `LED_SRCLK/SER/RCLK_PIN` |
| 4 | 超声波 74HC138 语法错误 | 改为独立的 A/B/C 引脚宏定义 |
| 5 | 多个 ADC 没有通道配置 | 所有模拟量都添加 `XXX_PIN` 和 `XXX_ADC` |
| 6 | UART2 缺少引脚定义 | 新增 `HOST_COMM_TX/RX_PIN` |
| 7 | 步进电机命名错误 | 改为 `OUT1/OUT2/OUT3/OUT4` 格式 |
| 8 | 版本未更新 | APP_VERSION 升级为 1.0.2 |

---

## 🚀 下一步建议

1. **编译测试**：确认所有代码能正常编译
2. **检查驱动**：确保各硬件驱动使用新的宏定义名称
3. **烧录调试**：开始逐个模块的硬件验证
4. **准备 OTA**：预留足够的固件升级空间

---

*版本：v3.0 (UTF-8 Final)*  
*修正完成：2026-03-16 15:30*  
*作者：Wuji (AI Assistant)*
