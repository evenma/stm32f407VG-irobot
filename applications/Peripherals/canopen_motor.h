#ifndef PERIPHERALS_CANOPEN_MOTOR_H__
#define PERIPHERALS_CANOPEN_MOTOR_H__

#include <rtthread.h>


/**
 * @brief CAN 总线配置 (STM32F407VG)
 * PA11 = CAN_RX, PA12 = CAN_TX (CAN1)
 */
#ifndef MOTOR_CAN_BUS
#define MOTOR_CAN_BUS         "can1"    // STM32F4 CAN1
#endif


/**
 * @brief ZLAC8015D CANopen 参数
 * 参考《ZLAC8015D CANopen 通信例程 Version 1.00》
 * 
 * PDO 映射表:
 *   - RPDO1 (0x1A0): 状态字 0x6041, 实际位置 0x6064, 实际速度 0x606C
 *   - TPDO1 (0x181): 目标速度 0x60FF, 操作模式 0x6060
 *   
 * SDO 对象字典:
 *   0x6040  : 控制字 (Control Word)
 *   0x6041  : 状态字 (Status Word)
 *   0x6060  : 操作模式设定值
 *   0x6064  : 当前位置 (编码器计数)
 *   0x606C  : 实际速度 (RPM*100)
 *   0x60FF  : 目标速度
 *   0x2800  : NMT 心跳请求
 */

/**
 * @brief 电机节点 ID (CANopen NMT 地址)
 */
#define MOTOR_LEFT_NODE_ID    1        // 左轮电机 (节点 1)
#define MOTOR_RIGHT_NODE_ID   2        // 右轮电机 (节点 2)
#define MOTOR_MAX_NODES       2        // 最多支持 2 个电机


/**
 * @brief CANopen PDO 索引
 */
#define OBJ_CONTROL_WORD    0x6040    // 控制字 (Control Word)
#define OBJ_STATUS_WORD     0x6041    // 状态字 (Status Word) - 注意是 0x6041
#define OBJ_OP_MODE         0x6060    // 操作模式设定值
#define OBJ_TARGET_VELOCITY 0x60FF    // 目标速度
#define OBJ_CURRENT_POSITION 0x6064   // 当前位置 (编码器脉冲数)
#define OBJECT_VELOCITY     0x606C    // 实际速度 (RPM*100)


/**
 * @brief 工作模式 (Operation Mode)
 * 根据 ZLAC8015D 手册定义
 */
typedef enum
{
    MODE_PROFILE_POSITION = 6,      // CSP Profile Position ⭐ 推荐
    MODE_PROFILE_VELOCITY = 8,      // CSV Profile Velocity
    MODE_PROFILE_TORQUE = 1,        // CST Profile Torque
    MODE_INTERPOLATED_POSITION = 7, // 插补位置模式
    MODE_CYCLIC_SYNCHRONOUS_POS = 5, // Sync cyclic 位置
    MODE_CYCLIC_SYNCHRONOUS_VEL = 9, // Sync cyclic 速度
    MODE_VELOCITY_LIMIT_MODE = 3,   // 速度限制模式
    MODE_HOMING_MODE = 6            // 回原点 (需配合 CSP)
} MotorOperationMode_t;


/* ========== 数据结构 ========== */

/**
 * @brief ZLAC8015D 状态字位定义 (0x6041)
 */
typedef struct {
    uint16_t not_ready_to_switch_on : 1;    // Bit 0
    uint16_t switch_on_disabled : 1;        // Bit 1
    uint16_t ready_to_switch_on : 1;        // Bit 2
    uint16_t switched_on : 1;               // Bit 3
    uint16_t operation_enabled : 1;         // Bit 4
    uint16_t fault : 1;                     // Bit 5
    uint16_t quick_stop_active : 1;         // Bit 6
    uint16_t switching_on_pending : 1;      // Bit 7
    uint16_t external_position_limit : 1;   // Bit 8
    uint16_t internal_position_limit : 1;   // Bit 9
    uint16_t position_within_tolerance : 1; // Bit 10
    uint16_t target_reached : 1;            // Bit 11
    uint16_t velocity_within_tolerance : 1; // Bit 12
    uint16_t position_error_overflow : 1;   // Bit 13
    uint16_t reserved1 : 1;                 // Bit 14
    uint16_t reserved2 : 1;                 // Bit 15
} StatusWordBits_t;


/**
 * @brief 电机反馈数据
 */
typedef struct
{
    uint16_t status_word;           // 状态字 (0x6041)
    int32_t position_ticks;         // 当前位置 (编码器脉冲数)
    float velocity_rpm;             // 实际速度 (RPM)
    float torque_percent;           // 当前扭矩 (%)
    StatusWordBits_t bits;          // 位解析结构
    MotorStateBits_t state_enum;    // 状态枚举转换
} MotorFeedback_t;


/**
 * @brief 电机控制命令 (TPDO1)
 */
typedef struct
{
    uint16_t control_word;          // 控制字 (0x6040)
    float target_velocity;          // 目标速度 (CSV 模式) 或增量位置
    int32_t target_position;        // 目标位置 (CSP 模式绝对位置)
    uint8_t operation_mode;         // 操作模式 (0x6060)
} MotorCommand_t;


/**
 * @brief ZLAC8015D 电机对象
 */
typedef struct
{
    rt_uint8_t node_id;             // CANopen 节点 ID (1~127)
    rt_bool_t initialized;          // 是否已初始化
    
    /* 配置参数 */
    struct
    {
        MotorOperationMode_t op_mode;           // 当前操作模式
        float encoder_ticks_per_rev;            // 每圈编码器计数 (默认 4096)
        float gear_ratio;                       // 减速比
        float max_velocity_rpm;                 // 最大速度 (RPM)
        float acceleration_rpm_per_sec;         // 加速度 (RPM/s)
        float deceleration_rpm_per_sec;         // 减速度 (RPM/s)
        int32_t homing_offset;                  // 原点偏移
        uint16_t quick_stop_decel;              // 快速停止减速度
    } config;
    
    /* 当前状态 */
    MotorFeedback_t feedback;           // 实时反馈数据
    MotorCommand_t command;             // 待发送命令
    
    /* PID 控制器参数 */
    float kp;                         // 比例增益
    float ki;                         // 积分增益
    float kd;                         // 微分增益
    float target_value;               // PID 目标值
    float last_error;                 // 上次误差
    float integral_sum;               // 积分累加
    float integral_limit;             // 积分限幅
    
    /* CAN 通信统计 */
    uint32_t tx_count;                // 发送帧数
    uint32_t rx_count;                // 接收帧数
    uint32_t timeout_errors;          // 超时错误
    uint32_t crc_errors;              // CRC 错误
    
    /* 回调函数 */
    void (*on_state_changed)(uint8_t old_state, uint8_t new_state);
    void (*on_fault)(uint16_t fault_code);
    void (*on_home_complete)(void);
} CanMotorObject_t;


/* ========== 公共 API ========== */

int canopen_motor_init(rt_uint8_t* nodes, rt_uint8_t count);
void canopen_motor_deinit(void);
CanMotorObject_t* canopen_motor_get_object(uint8_t idx);

/* 基本控制 */
int canopen_motor_enable(uint8_t motor_idx);
int canopen_motor_disable(uint8_t motor_idx);
int canopen_motor_reset_fault(uint8_t motor_idx);
int canopen_motor_homing(uint8_t motor_idx);

/* 运动控制 */
int canopen_motor_set_velocity(uint8_t motor_idx, float rpm);
int canopen_motor_quick_stop(uint8_t motor_idx);
int canopen_motor_set_position(uint8_t motor_idx, float positions);
int canopen_motor_move_relative(uint8_t motor_idx, float angle_deg);
rt_bool_t canopen_motor_is_reached(uint8_t motor_idx);

/* 状态查询 */
int canopen_motor_read_feedback(uint8_t motor_idx);
MotorStateBits_t canopen_motor_get_state(uint8_t motor_idx);
rt_bool_t canopen_motor_has_fault(uint8_t motor_idx);
uint16_t canopen_motor_get_fault_code(uint8_t motor_idx);

/* PID 闭环控制 */
void canopen_motor_pid_update(uint8_t motor_idx);
int canopen_motor_set_pid(uint8_t motor_idx, float kp, float ki, float kd, float limit);
float canopen_motor_pid_get_output(uint8_t motor_idx);


/* MSH 调试命令 */
#ifdef RT_USING_MSH
void msh_canopen_motor_status(int argc, char** argv);
void msh_canopen_motor_velocity(int argc, char** argv);
void msh_canopen_motor_pid(int argc, char** argv);
void msh_canopen_motor_test(int argc, char** argv);
#endif

#endif /* PERIPHERALS_CANOPEN_MOTOR_H__ */
