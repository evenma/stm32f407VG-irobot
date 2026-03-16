# global_conf.h 最终修正报告 v2.0

## ✅ 修正完成时间：2026-03-16 13:45

根据用户确认的最终配置，已完成所有引脚定义修正。

---

## 📋 主要修正内容（v2.0）

### 1. ✅ 加热管 MOSFET 引脚修正

| 配置项 | 原值 | 修正后 | 说明 |
|--------|------|--------|------|
| HEATER_CTRL_PIN | PE6 | **PE5** | 加热管控制 MOSFET |
| BUZZER_PIN | PE6 | PE6 | 蜂鸣器（不再冲突！） |

**重要修复**：之前蜂鸣器和加热管都使用 PE6，造成硬件冲突。现已改为加热管用 PE5，完全分离！

### 2. ✅ ADC 通道映射修正

| 传感器 | 引脚 | 原 ADC 通道 | 修正后 | 说明 |
|--------|------|------------|--------|------|
| 后悬崖 | PC4 | ADC1_IN14 | ADC1_IN14 ✅ | 不变 |
| 水温 NTC | PC5 | ADC1_IN14 | **ADC1_IN15** | 不同通道，无需时分复用！ |

### 3. ✅ 超声波方案选择

```c
#define ULTRASONIC_485        // 启用 - RS485 级联 7 个防水超声波
// #define ULTRASONIC_GPIO     // 禁用 - 临时调试方案
```

---

## 🎯 所有关键引脚汇总

### ⚡ 大功率控制

| 功能 | 引脚 | 备注 |
|------|------|------|
| 加热管 MOSFET | **PE5** | 新修正，无冲突 |
| 大水泵 MOSFET | PE2 | 程序模拟 PWM |
| 小水泵 PWM | PA0 | TIM5_CH1 (PWM0) |
| 暖风电热丝 | PE3 | 必须先开风扇 |
| 充电 MOSFET | PE4 | 电池/铅酸兼容 |

### 🔌 ADC 传感器

| 功能 | 引脚 | ADC 通道 |
|------|------|----------|
| 电池电压 | PC3 | ADC1_CH13 |
| 充电器电压 | PA5 | ADC1_IN5 |
| 充电器电流采样 | PA4 | ADC1_IN4 |
| 加热管接口检测 | PA6 | ADC1_IN6 |
| 前悬崖 | PA7 | ADC1_IN7 |
| 后悬崖 | PC4 | ADC1_IN14 |
| 水温 NTC | PC5 | ADC1_IN15 ✅ |
| 马桶盖位置 | PC2 | ADC1_CH12 |
| 马桶圈位置 | PC1 | ADC1_CH11 |
| 污物口位置 | PC0 | ADC1_CH10 |

### 💬 通信接口

| 功能 | 引脚 | 协议 |
|------|------|------|
| CAN 总线 | PA11/PA12 | CAN1 + TJA1050 |
| RS485 超声波 | PC10/PC11 | UART4 |
| TypeC 调试 | PA9/PA10 | USART1 |
| TypeC 上位机 | PA2/PA3 | USART2 + CH9102F |
| AI 语音 I2C | PB10/PB11 | I2C2 |
| IMU I2C | PB6/PB7 | I2C1 |
| 蓝牙手控 | PC11/PC12/PD1 | USART5 |

### 🏃 电机驱动

| 功能 | 引脚 | 类型 |
|------|------|------|
| 差速电机 | PA11/PA12 | CANopen ZLAC8015D |
| 清洁杆步进 | PD12-PD15 | ULN2803 |
| 管路分配器 | PD8-PD11 | ULN2803 |
| 马桶盖电机 | PE0/PE1/PC2 | MOSFET+继电器 |
| 马桶圈电机 | PB8/PB9/PC1 | MOSFET+继电器 |

### 💡 LED 与显示

| 功能 | 引脚 |
|------|------|
| OLED SPI | PB12-CS, PB13-SCK, PB14-DC, PB15-MOSI |
| LED 74HC595 | PB2-RCK, PB13-SRCLK, PB15-SER |
| Home 键 | PE9 |

### 🔔 其他 IO

| 功能 | 引脚 |
|------|------|
| 蜂鸣器 | PE6 ✅ |
| UV LED | PA15 |
| 红外灯 | PB3 |
| 风扇 | PB4 |
| 过流检测 | PE7 (EXTI7) |
| 水位开关 | PB0(高), PB1(低) |
| 旋钮按钮 | PE10 |
| 对射管对准 | PE13/14/15 |
| 按键 SW3/SW4/SW5 | PC13/14/15 |

---

## ✅ 问题修复清单

### 已修复 ❌ → ✅

1. **加热管引脚冲突**
   - 原：HEATER_CTRL_PIN = PE6 (与蜂鸣器共用) ❌
   - 现：HEATER_CTRL_PIN = PE5 (独立引脚) ✅

2. **ADC 通道错误**
   - 原：WATER_TEMP_NTC_PIN = PC5 (ADC1_IN14) ❌
   - 现：WATER_TEMP_NTC_PIN = PC5 (ADC1_IN15) ✅

3. **超声波方案**
   - 原：ULTRASONIC_GPIO 启用 (HC-SR04) ❌
   - 现：ULTRASONIC_485 启用 (RS485 级联) ✅

---

## 🚀 下一步建议

1. **编译测试**：`scons --target=mdk5`
2. **检查所有驱动**：确保使用了正确的引脚宏定义
3. **烧录调试**：逐个模块验证功能
4. **OTA 升级准备**：预留空间用于后续固件更新

---

*版本：v2.0 (Final)*  
*修正完成：2026-03-16 13:45*  
*作者：Wuji (AI Assistant)*
