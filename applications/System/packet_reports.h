/**
 * @file packet_reports.h
 * @author Leo.ma
 * @brief 所有串口通信数据回报
 * @version 0.1
 * @date 2026-05-3
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <stdint.h>

// 子命令定义（PKT_FUNC_SYS）
#define SYS_SUB_CHARGER_EVENT     0x01
#define SYS_SUB_HEATER_EVENT      0x02
#define SYS_SUB_CLIFF             0x03
#define SYS_SUB_BATTERY_EVENT     0x04
#define SYS_SUB_IR_SWITCH         0x05   // 红外对射管开关量

// 子命令定义（PKT_FUNC_TOILET）
#define TOILET_SUB_WATER_LEVEL    0x01
#define TOILET_SUB_WATER_TEMP     0x02
#define TOILET_SUB_SET_WATER_HEATER_AUTO   0x03   // 设置自动加热开关

// 子命令定义（PKT_FUNC_MOTOR）
#define MOTOR_SUB_VELOCITY_STATUS   0x01   // 实时速度+使能/运行状态（高频）
#define MOTOR_SUB_POSITION          0x02   // 位置脉冲（中低频）
#define MOTOR_SUB_TEMPERATURE       0x03   // 温度（低频）
#define MOTOR_SUB_FAULT             0x04   // 故障码（事件）
#define MOTOR_SUB_ONLINE            0x05   // 在线/离线状态变化（事件）
#define MOTOR_SUB_STATUSWORD        0x06   // 状态字（调试用，可选）

#pragma pack(1)
typedef union  {
    float array[4];
    struct {
        float w;
        float x;
        float y;
        float z;
    } element;
} PacketReportIMU_Quat_TypeDef;

typedef union  {
    struct {
		float accel_array[3];
		float gyro_array[3];
	}array;
    struct {
        struct {
            float x;
            float y;
            float z;
        } accel;
        struct {
            float x;
            float y;
            float z;
        } gyro;
    } element;
} PacketReportIMU_Raw_TypeDef;

// 按键事件(蓝牙手控器/遥控器/Home键/按键) (功能号 PKT_FUNC_KEY)
typedef struct  {
    uint8_t key_id;
    uint8_t event;
} PacketReportKeyEventTypeDef;

// 1. 电池电压事件 (功能号 PKT_FUNC_SYS)
typedef struct {
	uint8_t sub_cmd;						// = SYS_SUB_BATTERY_EVENT
	uint16_t voltage;
  uint8_t event;              // 0:正常, 1:低电量报警	兼容原格式，增加1字节放末尾
}PacketReportBatteryVoltageTypeDef;

// ========== 新增报告结构体 ==========
// 系统信息

// 2. 充电状态事件 (功能号 PKT_FUNC_SYS)
typedef struct {
	  uint8_t sub_cmd;            // = SYS_SUB_CHARGER_EVENT
    uint8_t event;          // 0:断开, 1:连接(未充电), 2:充电中
    uint16_t voltage_mv;    // 充电座电压或充电采样电压(mV)
    uint16_t power_mw;      // 充电功率(mW)
} PacketReportChargerEventTypeDef;

// 3. 加热器电源事件 (功能号 PKT_FUNC_SYS)
typedef struct {
	  uint8_t sub_cmd;            // = SYS_SUB_HEATER_EVENT
    uint8_t event;          // 0:断开, 1:连接
    uint16_t voltage_mv;    // 加热器供电电压(mV)
} PacketReportHeaterEventTypeDef;

// 4. 红外对射管 (功能号 PKT_FUNC_SYS)
typedef struct {
	uint8_t sub_cmd;            // = SYS_SUB_IR_SWITCH	
	uint8_t lr;                 // 1:左接收管, 2:右接收管
	uint8_t mask;               // 接收管状态：bit2:上管, bit1:左管, bit0:右管
} PacketReportIRSwitchTypeDef;

// 5. 悬崖传感器数据 (功能号 PKT_FUNC_SYS)
typedef struct {
	  uint8_t sub_cmd;            // = SYS_SUB_CLIFF
	  uint8_t cliff_trigger;      // 0:无悬崖, 1:悬崖触发
    uint16_t front_mv;          // 前悬崖电压(mV)
    uint16_t rear_mv;           // 后悬崖电压(mV)
} PacketReportCliffTypeDef;

// 智能小车 电机驱动器
// 1. 实时速度+状态（高频，如 50ms）
typedef struct {
    uint8_t sub_cmd;            // MOTOR_SUB_VELOCITY_STATUS
    int16_t left_vel_rpm;       // 实际转速 0.1rpm，负值表示反转
    int16_t right_vel_rpm;
    uint8_t left_enabled;       // 1:使能, 0:未使能
    uint8_t right_enabled;
    uint8_t left_running;       // 1:运行中, 0:停止（来自 2033h）
    uint8_t right_running;
} PacketReportMotorVelocityTypeDef;

// 2. 位置脉冲（中低频，如 500ms）
typedef struct {
    uint8_t sub_cmd;            // MOTOR_SUB_POSITION
    int32_t left_pos_pulse;
    int32_t right_pos_pulse;
} PacketReportMotorPositionTypeDef;
// 3. 电机温度（低频，如 5s）
typedef struct {
    uint8_t sub_cmd;            // MOTOR_SUB_TEMPERATURE
    int16_t left_temp_c;        // 0.1°C
    int16_t right_temp_c;
    int16_t driver_temp_c;
} PacketReportMotorTemperatureTypeDef;
// 4. 故障事件（变化时发送）
typedef struct {
    uint8_t sub_cmd;            // MOTOR_SUB_FAULT
    uint32_t fault_code;        // 0表示无故障，非0表示故障码
} PacketReportMotorFaultTypeDef;

// 5. 在线/离线状态变化事件
typedef struct {
    uint8_t sub_cmd;            // MOTOR_SUB_ONLINE
    uint8_t online;             // 0:离线, 1:在线
} PacketReportMotorOnlineTypeDef;
// 6. 状态字（调试用，可周期性或手动触发）
typedef struct {
    uint8_t sub_cmd;            // MOTOR_SUB_STATUSWORD
    uint16_t left_statusword;   // 6041h 低16位（左电机）
    uint16_t right_statusword;  // 6041h 高16位（右电机）
} PacketReportMotorStatusWordTypeDef;

//智能马桶
// 1. 水温(智能马桶) (功能号 PKT_FUNC_TOILET)
typedef struct {
		uint8_t sub_cmd;         // = SYS_SUB_WATER_TEMPER
    uint16_t temperature_c;     // 0.1°C
} PacketReportWaterTempTypeDef;

// 2. 水箱水位状态 (功能号 PKT_FUNC_TOILET)
typedef struct {
	  uint8_t sub_cmd;         // = SYS_SUB_WATER_LEVEL
    uint8_t high_level;     // 0:无水, 1:有水
    uint8_t low_level;      // 0:有水, 1:缺水
} PacketReportWaterLevelTypeDef;

// 其他
// 1. 超声波单传感器立即发送 (功能号 PKT_FUNC_ULTRASONIC)
#define ULTRASONIC_MAX 8
typedef struct {
    uint8_t sensor_id;          // 传感器ID(0~N-1)
    uint32_t distance_mm;       // 距离(mm)
} PacketReportUltrasonicSingleTypeDef;

// 2. 超声波批量发送(一轮结束) (功能号 PKT_FUNC_ULTRASONIC)
typedef struct {
    uint8_t count;              // 有效传感器数量
    uint32_t distances[ULTRASONIC_MAX];
} PacketReportUltrasonicBatchTypeDef;

// 3. 语音模块指令 (功能号 PKT_FUNC_VOICE) — 预留
typedef struct {
    uint8_t command_id;					// 命令词 ID
//		char    text[64];           // JSON 字符串，最大 63 字符（结束符 '\0'）
} PacketReportVoiceTypeDef;

#pragma pack()
