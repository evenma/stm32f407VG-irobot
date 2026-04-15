/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 全局配置宏定义文件
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
#define APP_VERSION_PATCH         2
#define APP_VERSION_STRING        "1.0.2"

/* ========== ADC Device Name ========== */
#define ADC_DEV_NAME                "adc1"
enum adc1_channel {
    ADC1_CH4  = 4,
    ADC1_CH5  = 5,
    ADC1_CH6  = 6,
    ADC1_CH7  = 7,
    ADC1_CH10 = 10,
    ADC1_CH11 = 11,
    ADC1_CH12 = 12,
    ADC1_CH13 = 13,
    ADC1_CH14 = 14,
    ADC1_CH15 = 15,
    ADC1_CH17 = 17,   // 内部参考电压（VREFINT）
};


/* ======================== 硬件平台选择 ======================== */

/* ======================== LED 指示灯配置 ======================== */

/**
 * @brief LED 数量配置
 *   实际项目：8 个 LED，4 对（绿 + 红）
 *   调试用：可以临时改为 1 个测试
 */
#define LED_TOTAL_COUNT           8
#define LED_USE_SPI_74HC595       // 启用 SPI+74HC595 驱动（默认启用）
// #define LED_USE_DIRECT_GPIO     // 如果 PCB 设计是 GPIO 直驱，使用此宏

/**
 * @brief 74HC595 移位寄存器引脚定义
 *   PB13 = SPI_SCK -> SRCLK (移位时钟)
 *   PB15 = SPI_MOSI -> SER    (数据输入)
 *   PB2  = GPIO      -> RCLK  (输出锁存时钟)
 */
#define LED_SRCLK_PIN             GET_PIN(B, 13)    // SPI SCK - 74HC595 SRCLK
#define LED_SER_PIN               GET_PIN(B, 15)    // SPI MOSI - 74HC595 SER
#define LED_RCLK_PIN              GET_PIN(B, 2)     // GPIO - 74HC595 RCLK


/* ======================== 蜂鸣器配置 ======================== */

/**
 * @brief 有源蜂鸣器
 * PE6 = 蜂鸣器引脚
 */
#define BUZZER_PIN                GET_PIN(E, 6)


/* ======================== 按键配置 ======================== */

/**
 * @brief 板载按键配置
 * SW3 = PC13, SW4 = PC14, SW5 = PC15
 */
#define KEY_SW3_PIN               GET_PIN(C, 13)    // 上一页
#define KEY_SW4_PIN               GET_PIN(C, 14)    // 下一页
#define KEY_SW5_PIN               GET_PIN(C, 15)    // 未用

#define KEY_HOME_PIN              GET_PIN(E, 9)     // Home 键 - 返回充电座

#define KEY_BUTTON_PULSE_WIDTH_MS 20                // 按键去抖时间


/* ======================== CAN 电机驱动器配置 ======================== */

/**
 * @brief CAN 总线引脚定义
 * PA11 = CAN_RX, PA12 = CAN_TX
 * 仅需 2 线，不需要隔离使能引脚
 */
#define CAN_BUS_PORT              CAN1              // CAN1 总线 (PA11/PA12)
#define CAN_TX_PIN                GET_PIN(A, 12)    // PA12 = CAN_TX
#define CAN_RX_PIN                GET_PIN(A, 11)    // PA11 = CAN_RX

/**
 * @brief 选择超声波传感器方案（二选一）
 *   - ULTRASONIC_485: 防水型 RS485 超声波（7 个级联）- 实际项目方案
 *   - ULTRASONIC_GPIO: 普通 HC-SR04 超声波（5 个 GPIO 控制 pc7,pc8,pc9=hc138+hc125 pa1=trig脉冲 + hc32 pc6=echo中断）
 */
//#define ULTRASONIC_485            // 使用 RS485 方案（实际项目）
#define ULTRASONIC_GPIO         // 使用 hc138+hc125+hc32 方案
/* ======================== RS485 超声波配置 ======================== */

/**
 * @brief RS485 引脚定义（如果启用 ULTRASONIC_485）
 * PC10 = UART4_RX, PC11 = UART4_TX, PD0 = DE/RE 切换
 */
#define UART485_PORT              UART4
#define UART485_RX_PIN            GET_PIN(C, 10)
#define UART485_TX_PIN            GET_PIN(C, 11)
#define UART485_DE_PIN            GET_PIN(D, 0)
#define RS485_DIR_PIN           	GET_PIN(D, 0) /* PD0 方向控制引脚，高电平发送，低电平接收 */

/* ======================== GPIO 超声波配置 ======================== */

/**
 * @brief GPIO 超声波配置（如果启用 ULTRASONIC_GPIO）
 * 8 个模块通过 74HC138 译码器选择
 * PC7-9 -> 74HC138 A,B,C
 */
#define ULTRASONIC_A_PIN          GET_PIN(C, 7)     // 74HC138 A
#define ULTRASONIC_B_PIN          GET_PIN(C, 8)     // 74HC138 B
#define ULTRASONIC_C_PIN          GET_PIN(C, 9)     // 74HC138 C
#define ULTRASONIC_TRIG_PIN       GET_PIN(A, 8)     // Trig 共用
#define ULTRASONIC_ECHO_PIN       GET_PIN(C, 6)     // Echo 输入（中断）
// 定时器选择（TIM3 通道无特殊要求，仅作基本定时）
#define ULTRASONIC_TIMER          TIM3
// 最大传感器数量
#define HC_SR04_NUM               8
#define ULTRASONIC_485_NUM     		7
// 单个传感器超时时间（ms）基本在32ms
#define HC_SR04_TIMEOUT_MS        50
/* 单个传感器超时时间（ms），需大于传感器最大响应时间 (处理值响应时间 320～750ms) */
#define RS485_TIMEOUT_MS      		750
// 1轮询8个超声波时长(ms)
#define POLL_INTERVAL_MS					500    // hc_sr04
#define POLL_INTERVAL_485_MS			1000    // 485
// 声速（mm/us）：340m/s = 0.34mm/us，半程往返需除以2 => 0.17mm/us
#define SOUND_SPEED_MM_PER_US     0.17f


/* ======================== OLED 显示屏配置 ======================== */

/**
 * @brief OLED SSD1315/SSD1306 引脚定义
 * PB12 = CS, PB13 = SCK, PB14 = DC, PB15 = MOSI
 */
#define OLED_SPI_PORT             SPI2
#define OLED_CS_PIN               GET_PIN(B, 12)
#define OLED_SCK_PIN              GET_PIN(B, 13)
#define OLED_DC_PIN               GET_PIN(B, 14)
#define OLED_MOSI_PIN             GET_PIN(B, 15)
// #define OLED_RST_PIN            GET_PIN(A, 0)     // 硬件 reset - 不需要软件控制（已移除）


/* ======================== IMU 姿态传感器配置 ======================== */

/**
 * @brief QMI8658 引脚定义
 * PB6 = I2C_SCL, PB7 = I2C_SDA, PB5 = INT2
 */
#define IMU_I2C_PORT              I2C1
#define IMU_SCL_PIN               GET_PIN(B, 6)
#define IMU_SDA_PIN               GET_PIN(B, 7)
#define IMU_INT_PIN               GET_PIN(B, 5)


/* ======================== AI 语音模块配置 ======================== */

/**
 * @brief WonderEcho Pro 引脚定义
 * PB10 = I2C_SCL, PB11 = I2C_SDA
 */
#define VOICE_I2C_PORT            I2C2
#define VOICE_SCL_PIN             GET_PIN(B, 10)
#define VOICE_SDA_PIN             GET_PIN(B, 11)


/* ======================== 电池检测配置 ======================== */

/**
 * @brief 电池电压 ADC 引脚
 * PC3 = ADC1_IN13（分压电阻 200K+22K）
 */
#define BATTERY_ADC_PIN           GET_PIN(C, 3)
#define BATTERY_ADC_CHANNEL       ADC1_CH13

#define BATTERY_FULL_VOLTAGE_MV   25200               // 25.2V 锂电池满电
#define BATTERY_LOW_VOLTAGE_MV    19200               // 19.2V ≈20% 低电量阈值
#define BATTERY_LOW_ALARM_MV      22000   // 22.0V 低电量报警阈值 为确保移动马桶有足够电量返回充电座，建议将低电量报警阈值设为 22.0V（约60%剩余容量）
#define BATTERY_LOW_HYSTERESIS_MV   100   // 退出阈值比进入阈值高 100mV
/* ======================== 充电口检测配置 ======================== */

/**
 * @brief 充电器接口检测
 * PA5 = ADC1_IN5（充电口电压）
 * PA4 = ADC1_IN4（采样电阻前端）
 * PE4 = 充电 MOSFET 控制
 */
#define CHARGER_DETECT_PIN        GET_PIN(A, 5)
#define CHARGER_DETECT_ADC        ADC1_CH5
#define CHARGER_SAMPLE_PIN        GET_PIN(A, 4)
#define CHARGER_SAMPLE_ADC        ADC1_CH4
#define CHARGER_CONTROL_PIN       GET_PIN(E, 4)


/* ======================== 大功率泵浦控制配置 ======================== */

/**
 * @brief 加热管 MOSFET 控制
 * PA6 = ADC1_IN6 (检测加热管供电接口连接情况 - 独立外供电)
 * PE5 = 加热管 MOSFET 开关
 */
#define HEATER_CTRL_PIN           GET_PIN(E, 5)       // 加热管 MOSFET
#define HEATER_DETECT_PIN         GET_PIN(A, 6)       // ADC1_IN6 - 检测供电接口连接
#define HEATER_DETECT_ADC         ADC1_CH6

/**
 * @brief 暖风烘干电热丝
 * PE3 = 电热丝 MOSFET 控制（必须先开风扇再开电热丝）
 */
#define DRYER_HEATER_PIN          GET_PIN(E, 3)

/**
 * @brief 大水泵 MOSFET 控制 (STP55NF06L)
 * PA1 = IO 输出，可 PWM 功能
 */
//#define LARGE_PUMP_PIN            GET_PIN(E, 2)
#define LARGE_PUMP_PIN            GET_PIN(A, 1)

/**
 * @brief 排污泵 MOSFET 控制 (STP55NF06L)
 * PE2 = IO 输出
 */
#define SEWAGE_PUMP_PIN           GET_PIN(E, 2)      

/**
 * @brief 小水泵 PWM 控制（温水冲洗臀部/女士）
 * PA0 = TIM5_CH1 (PWM0)
 */
#define SMALL_PUMP_PIN            GET_PIN(A, 0)       // PWM0 - TIM5_CH1 - PA0 (硬件 PWM)


/* ======================== 马桶盖/圈电机配置 ======================== */

/**
 * @brief 马桶盖电机控制
 * PE1 = MOSFET 使能，PE0 = 方向继电器，PC2 = 位置反馈 (ADC1_CH12)
 */
#define LID_MOTOR_ENABLE_PIN      GET_PIN(E, 1)       // MOSFET 使能
#define LID_MOTOR_DIR_PIN         GET_PIN(E, 0)       // 方向继电器
#define LID_MOTOR_POS_PIN         GET_PIN(C, 2)       // 位置反馈
#define LID_MOTOR_POS_ADC         ADC1_CH12

/**
 * @brief 马桶圈电机控制
 * PB9 = MOSFET 使能，PB8 = 方向继电器，PC1 = 位置反馈 (ADC1_CH11)
 */
#define RING_MOTOR_ENABLE_PIN     GET_PIN(B, 9)       // MOSFET 使能
#define RING_MOTOR_DIR_PIN        GET_PIN(B, 8)       // 方向继电器
#define RING_MOTOR_POS_PIN        GET_PIN(C, 1)       // 位置反馈
#define RING_MOTOR_POS_ADC        ADC1_CH11


/* ======================== 水位检测配置 ======================== */

/**
 * @brief 水箱高低水位开关
 * PB0 = 高水位浮球开关
 * PB1 = 低水位浮球开关
 */
#define WATER_LEVEL_HIGH_PIN      GET_PIN(B, 0)
#define WATER_LEVEL_LOW_PIN       GET_PIN(B, 1)


/* ======================== 旋转旋钮配置 ======================== */

/**
 * @brief 马桶旋钮电位器 + 按钮
 * PE10 = 按钮开关，PE11 = 女士清洁信号，PE12 = 臀部清洁信号
 */
#define KNOB_BUTTON_PIN           GET_PIN(E, 10)
#define KNOB_FEMALE_PIN           GET_PIN(E, 11)
#define KNOB_REAR_PIN             GET_PIN(E, 12)


/* ======================== 悬崖传感器配置 ======================== */

/**
 * @brief 红外测距传感器（掉崖检测）
 * PA7 = ADC1_IN7（前悬崖）
 * PC4 = ADC1_IN14（后悬崖）
 */
#define CLIFF_FRONT_PIN           GET_PIN(A, 7)
#define CLIFF_FRONT_ADC           ADC1_CH7
#define CLIFF_REAR_PIN            GET_PIN(C, 4)
#define CLIFF_REAR_ADC            ADC1_CH14


/* ======================== 水温传感器配置 ======================== */

/**
 * @brief NTC 温度传感器 (10K, B=3950, 5V 上拉 5.1K)
 * PC5 = ADC1_IN15（与后悬崖不同通道）
 */
#define WATER_TEMP_PIN            GET_PIN(C, 5)
#define WATER_TEMP_ADC            ADC1_CH15


/* ======================== 对射管充电对准配置 ======================== */

/**
 * @brief 充电座对准红外对射管
 * PE13, PE14, PE15 = 三个接收管
 */
#define CHARGER_ALIGN_PIN_1       GET_PIN(E, 13)
#define CHARGER_ALIGN_PIN_2       GET_PIN(E, 14)
#define CHARGER_ALIGN_PIN_3       GET_PIN(E, 15)


/* ======================== 步进电机控制配置 ======================== */

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


/* ======================== 其他 IO 配置 ======================== */

/**
 * @brief 左大灯 
 * PD6 第二个 ULN2803 驱动 Out5/6合并加强驱动
 */
#define HEADLIGHT_LEFT_PIN        GET_PIN(D, 6)

/**
 * @brief 右大灯 
 * PD7 第二个 ULN2803 驱动 Out7/8 合并加强驱动
 */
#define HEADLIGHT_RIGHT_PIN        GET_PIN(D, 7)

/**
 * @brief 紫外灯、红外灯、风扇、水阀
 * 第二个 ULN2803 驱动
 */
#define UV_LED_PIN                GET_PIN(A, 15)      // Out1
#define IR_HEAT_PIN               GET_PIN(B, 3)       // Out2
#define FAN_PIN                   GET_PIN(B, 4)       // Out3
// #define WATER_VALVE_PIN         ...                 // Out4 - 预留

/**
 * @brief 水阀控制（预留）
 * PD5 - ULN2803 
 */
#define WATER_VALVE_PIN_1         GET_PIN(D, 5)


/* ======================== 串口调试配置 ======================== */

/**
 * @brief Type-C 串口定义
 * UART1 = 调试控制台 (PA9/PA10)
 * UART2 = 上位机通讯 (PA2/PA3) - CH9102F
 */
#define DEBUG_UART_PORT           USART1
#define DEBUG_UART_TX_PIN         GET_PIN(A, 9)       // PA9 = USART1 TX
#define DEBUG_UART_RX_PIN         GET_PIN(A, 10)      // PA10 = USART1 RX

#define HOST_COMM_UART_PORT       USART2
#define HOST_COMM_TX_PIN          GET_PIN(A, 2)       // PA2 = USART2 TX
#define HOST_COMM_RX_PIN          GET_PIN(A, 3)       // PA3 = USART2 RX


/* ======================== 蓝牙手控器配置 ======================== */

/**
 * @brief DX2002 蓝牙模块
 * PC11 = RX, PC12 = TX, PD1 = WAKE/PAIR
 */
#define BLE_UART_PORT             USART5
#define BLE_RX_PIN                GET_PIN(C, 11)
#define BLE_TX_PIN                GET_PIN(C, 12)
#define BLE_WAKE_PIN              GET_PIN(D, 1)


/* ======================== 过流检测配置 ======================== */

/**
 * @brief 外供电隔离供电过流检测
 * PE7 = EXTI7 外中断信号
 */
#define OVER_CURRENT_PIN          GET_PIN(E, 7)


/* ======================== 污物口电机预留 ======================== */

/**
 * @brief 污物口电机控制（预留未实现）
 * PD3 = 方向，PD4 = 工作，PC0 = 位置反馈 (ADC1_CH10)
 */
#define SEWAGE_MOTOR_DIR_PIN      GET_PIN(D, 3)
#define SEWAGE_MOTOR_EN_PIN       GET_PIN(D, 4)
#define SEWAGE_MOTOR_POS_PIN      GET_PIN(C, 0)       // 位置反馈
#define SEWAGE_MOTOR_POS_ADC      ADC1_CH10


/* ======================== 工作模式标志 ======================== */

// 系统运行状态
#define SYS_STATE_BOOTING         0
#define SYS_STATE_IDLE            1
#define SYS_STATE_NAVIGATING      2
#define SYS_STATE_CLEANING        3
#define SYS_STATE_CHARGING        4
#define SYS_STATE_ERROR           5


#endif /* GLOBAL_CONF_H__ */
