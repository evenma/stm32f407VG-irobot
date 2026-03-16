/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 全局配置宏定义文件（最终版 v2.0）
 * 
 * 说明：
 *   1. 所有硬件相关的配置、开关、模式都在这里定义
 *   2. 编译时通过修改宏定义来选择不同的硬件配置
 *   3. 不要在这里写具体驱动代码，只放常量定义
 */

#ifndef GLOBAL_CONF_H__
#define GLOBAL_CONF_H__


/* ======================== 系统版本 ======================== */
#define APP_VERSION_MAJOR         1
#define APP_VERSION_MINOR         0
#define APP_VERSION_PATCH         1
#define APP_VERSION_STRING        "1.0.1"


/* ======================== 硬件平台选择 ======================== */

/**
 * @brief 选择超声波传感器方案（二选一）
 *   - ULTRASONIC_485: 防水型 RS485 超声波（7 个级联）✅ 实际项目方案
 *   - ULTRASONIC_GPIO: 普通 HC-SR04 超声波（8 个 GPIO 控制）
 */
#define ULTRASONIC_485        // 使用 RS485 方案（实际项目）
// #define ULTRASONIC_GPIO     // 调试用临时方案


/* ======================== LED 指示灯配置 ======================== */

/**
 * @brief LED 数量配置
 *   实际项目：8 个 LED，4 对（绿 + 红）
 *   调试用：可以临时改为 1 个测试
 */
#define LED_TOTAL_COUNT         8
#define LED_USE_SPI_74HC595     // 启用 SPI+74HC595 驱动（默认启用）
// #define LED_USE_DIRECT_GPIO   // 如果 PCB 设计是 GPIO 直驱，使用此宏


/* ======================== 蜂鸣器配置 ======================== */

/**
 * @brief 有源蜂鸣器
 * PE6 = 蜂鸣器引脚
 */
#define BUZZER_PIN              GET_PIN(E, 6)


/* ======================== 按键配置 ======================== */

/**
 * @brief 板载按键配置
 * SW3 = PC13, SW4 = PC14, SW5 = PC15
 */
#define KEY_SW3_PIN             GET_PIN(C, 13)    // 上一页
#define KEY_SW4_PIN             GET_PIN(C, 14)    // 下一页
#define KEY_SW5_PIN             GET_PIN(C, 15)    // 未用

#define KEY_HOME_PIN            GET_PIN(E, 9)     // Home 键 - 返回充电座

#define KEY_BUTTON_PULSE_WIDTH_MS   20    // 按键去抖时间


/* ======================== CAN 电机驱动器配置 ======================== */

/**
 * @brief CAN 总线引脚定义
 * PA11 = CAN_RX, PA12 = CAN_TX
 */
#define CAN_BUS_PORT            CAN1        // CAN1 总线 (PA11/PA12)
#define CAN_TX_PIN              GET_PIN(A, 12)
#define CAN_RX_PIN              GET_PIN(A, 11)
#define CAN_ISOLATION_EN_PIN    GET_PIN(D, 7) // ACSL-6210 使能


/* ======================== RS485 超声波配置 ======================== */

/**
 * @brief RS485 引脚定义（如果启用 ULTRASONIC_485）
 * PC10 = UART4_RX, PC11 = UART4_TX, PD0 = DE/RE 切换
 */
#define UART485_PORT            UART4
#define UART485_RX_PIN          GET_PIN(C, 10)
#define UART485_TX_PIN          GET_PIN(C, 11)
#define UART485_DE_PIN          GET_PIN(D, 0)


/* ======================== GPIO 超声波配置 ======================== */

/**
 * @brief GPIO 超声波配置（如果启用 ULTRASONIC_GPIO）
 * 8 个模块通过 74HC138 译码器选择
 * PC7-9 -> 74HC138 A,B,C
 */
#define ULTRASONIC_SELECT pins {PC7, PC8, PC9}   // 74HC138 选择线
#define ULTRASONIC_TRIG_PIN     GET_PIN(A, 8)    // Trig 共用
#define ULTRASONIC_ECHO_PIN     GET_PIN(C, 6)    // Echo 输入（中断）


/* ======================== OLED 显示屏配置 ======================== */

/**
 * @brief OLED SSD1315/SSD1306 引脚定义
 * PB12 = CS, PB13 = SCK, PB14 = DC, PB15 = MOSI
 */
#define OLED_SPI_PORT           SPI2
#define OLED_CS_PIN             GET_PIN(B, 12)
#define OLED_SCK_PIN            GET_PIN(B, 13)
#define OLED_DC_PIN             GET_PIN(B, 14)
#define OLED_MOSI_PIN           GET_PIN(B, 15)
// #define OLED_RST_PIN          GET_PIN(A, 0)     // 硬件 reset - 不需要软件控制 (已移除)


/* ======================== IMU 姿态传感器配置 ======================== */

/**
 * @brief QMI8658 引脚定义
 * PB6 = I2C_SCL, PB7 = I2C_SDA, PB5 = INT2
 */
#define IMU_I2C_PORT            I2C1
#define IMU_SCL_PIN             GET_PIN(B, 6)
#define IMU_SDA_PIN             GET_PIN(B, 7)
#define IMU_INT_PIN             GET_PIN(B, 5)


/* ======================== AI 语音模块配置 ======================== */

/**
 * @brief WonderEcho Pro 引脚定义
 * PB10 = I2C_SCL, PB11 = I2C_SDA
 */
#define VOICE_I2C_PORT          I2C2
#define VOICE_SCL_PIN           GET_PIN(B, 10)
#define VOICE_SDA_PIN           GET_PIN(B, 11)


/* ======================== 电池检测配置 ======================== */

/**
 * @brief 电池电压 ADC 引脚
 * PC3 = ADC1_IN13（分压电阻 200K+22K）
 */
#define BATTERY_ADC_PIN         GET_PIN(C, 3)
#define BATTERY_ADC_CHANNEL     ADC1_CH13

#define BATTERY_FULL_VOLTAGE_MV   25200       // 25.2V 锂电池满电
#define BATTERY_LOW_VOLTAGE_MV    19200       // 19.2V ≈20% 低电量阈值


/* ======================== 充电口检测配置 ======================== */

/**
 * @brief 充电器接口检测
 * PA5 = ADC1_IN5（充电口电压）
 * PA4 = ADC1_IN4（采样电阻前端）
 * PE4 = 充电 MOSFET 控制
 */
#define CHARGER_DETECT_PIN      GET_PIN(A, 5)
#define CHARGER_SAMPLE_PIN      GET_PIN(A, 4)
#define CHARGER_CONTROL_PIN     GET_PIN(E, 4)


/* ======================== 大功率泵浦控制配置 ======================== */

/**
 * @brief 加热管 MOSFET 控制
 * PA6 = ADC1_IN6 (检测加热管供电接口连接情况 - 独立外供电)
 * PE5 = 加热管 MOSFET 开关
 */
#define HEATER_CTRL_PIN         GET_PIN(E, 5)     // 加热管 MOSFET
#define HEATER_DETECT_ADC       GET_PIN(A, 6)     // ADC1_IN6 - 检测供电接口连接

/**
 * @brief 暖风烘干电热丝
 * PE3 = 电热丝 MOSFET 控制（必须先开风扇再开电热丝）
 */
#define DRYER_HEATER_PIN        GET_PIN(E, 3)

/**
 * @brief 大水泵 MOSFET 控制 (STP55NF06L)
 * PE2 = IO 输出，无 PWM 功能，程序模拟 8KHz PWM
 */
#define LARGE_PUMP_PIN          GET_PIN(E, 2)

/**
 * @brief 小水泵 PWM 控制（温水冲洗臀部/女士）
 * PA0 = TIM5_CH1 (PWM0)
 */
#define SMALL_PUMP_PIN          GET_PIN(A, 0)    // PWM0 - TIM5_CH1 - PA0 (硬件 PWM)


/* ======================== 马桶盖/圈电机配置 ======================== */

/**
 * @brief 马桶盖电机控制
 * PE1 = MOSFET 使能，PE0 = 方向继电器，PC2 = 位置反馈 (ADC1_CH12)
 */
#define LID_MOTOR_ENABLE_PIN    GET_PIN(E, 1)     // MOSFET 使能
#define LID_MOTOR_DIR_PIN       GET_PIN(E, 0)     // 方向继电器
#define LID_MOTOR_POS_ADC       GET_PIN(C, 2)     // 位置反馈 ADC1_CH12

/**
 * @brief 马桶圈电机控制
 * PB9 = MOSFET 使能，PB8 = 方向继电器，PC1 = 位置反馈 (ADC1_CH11)
 */
#define RING_MOTOR_ENABLE_PIN   GET_PIN(B, 9)     // MOSFET 使能
#define RING_MOTOR_DIR_PIN      GET_PIN(B, 8)     // 方向继电器
#define RING_MOTOR_POS_ADC      GET_PIN(C, 1)     // 位置反馈 ADC1_CH11


/* ======================== 水位检测配置 ======================== */

/**
 * @brief 水箱高低水位开关
 * PB0 = 高水位浮球开关
 * PB1 = 低水位浮球开关
 */
#define WATER_LEVEL_HIGH_PIN    GET_PIN(B, 0)
#define WATER_LEVEL_LOW_PIN     GET_PIN(B, 1)


/* ======================== 旋转旋钮配置 ======================== */

/**
 * @brief 马桶旋钮电位器 + 按钮
 * PE10 = 按钮开关，PE11 = 女士清洁信号，PE12 = 臀部清洁信号
 */
#define KNOB_BUTTON_PIN         GET_PIN(E, 10)
#define KNOB_FEMALE_PIN         GET_PIN(E, 11)
#define KNOB_REAR_PIN           GET_PIN(E, 12)


/* ======================== 悬崖传感器配置 ======================== */

/**
 * @brief 红外测距传感器（掉崖检测）
 * PA7 = ADC1_IN7（前悬崖）
 * PC4 = ADC1_IN14（后悬崖）
 */
#define CLIFF_FRONT_ADC_PIN     GET_PIN(A, 7)
#define CLIFF_REAR_ADC_PIN      GET_PIN(C, 4)


/* ======================== 水温传感器配置 ======================== */

/**
 * @brief NTC 温度传感器 (10K, B=3950, 5V 上拉 5.1K)
 * PC5 = ADC1_IN15（与后悬崖不同通道，无需时分复用）
 */
#define WATER_TEMP_NTC_PIN      GET_PIN(C, 5)


/* ======================== 对射管充电对准配置 ======================== */

/**
 * @brief 充电座对准红外对射管
 * PE13, PE14, PE15 = 三个接收管
 */
#define CHARGER_ALIGN_PIN_1     GET_PIN(E, 13)
#define CHARGER_ALIGN_PIN_2     GET_PIN(E, 14)
#define CHARGER_ALIGN_PIN_3     GET_PIN(E, 15)


/* ======================== 步进电机控制配置 ======================== */

/**
 * @brief 清洁杆步进电机（ULN2803 驱动）
 * PD15 = STEP, PD14 = DIR, PD13 = PULSE, PD12 = EN
 */
#define CLEANING_STEPPER_STEP   GET_PIN(D, 15)
#define CLEANING_STEPPER_DIR    GET_PIN(D, 14)
#define CLEANING_STEPPER_PULSE  GET_PIN(D, 13)
#define CLEANING_STEPPER_EN     GET_PIN(D, 12)

/**
 * @brief 管路分配器步进电机（ULN2803 驱动）
 * PD11 = STEP, PD10 = DIR, PD9 = PULSE, PD8 = EN
 */
#define TUBE_STEPPER_STEP       GET_PIN(D, 11)
#define TUBE_STEPPER_DIR        GET_PIN(D, 10)
#define TUBE_STEPPER_PULSE      GET_PIN(D, 9)
#define TUBE_STEPPER_EN         GET_PIN(D, 8)


/* ======================== 其他 IO 配置 ======================== */

/**
 * @brief 左大灯 PWM
 * PA1 = TIM5_CH2 (PWM1)
 */
#define HEADLIGHT_PWM_PIN       GET_PIN(A, 1)

/**
 * @brief 紫外灯、红外灯、风扇、水阀
 * 第二个 ULN2803 驱动
 */
#define UV_LED_PIN              GET_PIN(A, 15)    // Out1
#define IR_HEAT_PIN             GET_PIN(B, 3)     // Out2
#define FAN_PIN                 GET_PIN(B, 4)     // Out3
// #define WATER_VALVE_PIN       ...  // Out4 - 预留

/**
 * @brief 水阀控制（预留）
 * PD5/6/7 - ULN2803 Out5/6/7 合并加强驱动（预留）
 */
#define WATER_VALVE_PIN_1       GET_PIN(D, 5)
#define WATER_VALVE_PIN_2       GET_PIN(D, 6)
#define WATER_VALVE_PIN_3       GET_PIN(D, 7)


/* ======================== 串口调试配置 ======================== */

/**
 * @brief Type-C 串口定义
 * UART1 = 调试控制台 (PA9/PA10)
 * UART2 = 上位机通讯 (PA2/PA3) - CH9102F
 */
#define DEBUG_UART_PORT         USART1
#define HOST_COMM_UART_PORT     USART2


/* ======================== 蓝牙手控器配置 ======================== */

/**
 * @brief DX2002 蓝牙模块
 * PC11 = RX, PC12 = TX, PD1 = WAKE/PAIR
 */
#define BLE_UART_PORT           USART5
#define BLE_RX_PIN              GET_PIN(C, 11)
#define BLE_TX_PIN              GET_PIN(C, 12)
#define BLE_WAKE_PIN            GET_PIN(D, 1)


/* ======================== 过流检测配置 ======================== */

/**
 * @brief 外供电隔离供电过流检测
 * PE7 = EXTI7 外中断信号
 */
#define OVER_CURRENT_PIN        GET_PIN(E, 7)


/* ======================== 污物口电机预留 ======================== */

/**
 * @brief 污物口电机控制（预留未实现）
 * PD3 = 方向，PD4 = 工作，PC0 = 位置反馈 (ADC1_CH10)
 */
#define SEWAGE_MOTOR_DIR_PIN    GET_PIN(D, 3)
#define SEWAGE_MOTOR_EN_PIN     GET_PIN(D, 4)
#define SEWAGE_MOTOR_POS_ADC    GET_PIN(C, 0)     // ADC1_CH10


/* ======================== 工作模式标志 ======================== */

// 系统运行状态
#define SYS_STATE_BOOTING       0
#define SYS_STATE_IDLE          1
#define SYS_STATE_NAVIGATING    2
#define SYS_STATE_CLEANING      3
#define SYS_STATE_CHARGING      4
#define SYS_STATE_ERROR         5


#endif /* GLOBAL_CONF_H__ */
