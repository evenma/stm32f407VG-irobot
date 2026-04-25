#ifndef __ZLTECH_CAN_MOTOR_H__
#define __ZLTECH_CAN_MOTOR_H__

#include <rtthread.h>
#include <rtdevice.h>

/* ======================== 基础配置 ======================== */
#define ZLAC_NODE_ID                1           /* 驱动器节点地址 (1-127) */
#define ZLAC_CAN_DEV_NAME           "can1"      /* CAN 设备名称 */
#define ZLAC_CAN_FILTER_ID          0x000       /* 滤波器 ID（接收所有） */
#define ZLAC_CAN_FILTER_MASK        0x000

/* 通信超时 (ms) */
#define ZLAC_SDO_TIMEOUT_MS         200
#define ZLAC_PDO_TIMEOUT_MS         1000
#define ZLAC_CAN_TIMEOUT_MS  				500					/* 通讯超时*/
#define ZLAC_HEARTBEAT_PRODTIME_MS  1000				/* 心跳周期*/
#define ZLAC_HEARTBEAT_TIMEOUT_MS   3000        /* 心跳超时，离线检测 */

/* 电机参数 (ZLLG65ASM250-4096-B) */
#define ZLAC_MOTOR_POLE_PAIRS       15          /* 极对数 */
#define ZLAC_MOTOR_ENCODER_LINES    4096        /* 编码器线数 */
#define ZLAC_MOTOR_RATED_CURRENT_MA 60        /* 额定电流 6A */
#define ZLAC_MOTOR_PEAK_CURRENT_MA  180       /* 峰值电流 18A */
#define ZLAC_MOTOR_MAX_RPM          160         /* 最大转速 (rpm) */
#define ZLAC_MOTOR_ACCEL_TIME_MS    200         /* 加速时间 (ms) */
#define ZLAC_MOTOR_DECEL_TIME_MS    200         /* 减速时间 (ms) */
#define ZLAC_MOTOR_QUICKSTOP_TIME_MS 20         /* 急停减速时间 (ms) */

/* ======================== CANopen 对象字典索引 ======================== */
/* CiA301 通信参数 */
#define ZLAC_OD_DEVICE_TYPE         0x1000
#define ZLAC_OD_ERROR_REG           0x1001
#define ZLAC_OD_COB_ID_SYNC         0x1005
#define ZLAC_OD_HARDWARE_VERSION    0x1009
#define ZLAC_OD_SOFTWARE_VERSION    0x100A
#define ZLAC_OD_HEARTBEAT_PROD      0x1017
#define ZLAC_OD_MANUFACTURER_DEVICE 0x1018
#define ZLAC_OD_IDENTITY            0x1200

/* RPDO 通信参数 (1400h-1403h) */
#define ZLAC_OD_RPDO_COMM_PARAM     0x1400
/* TPDO 通信参数 (1800h-1803h) */
#define ZLAC_OD_TPDO_COMM_PARAM     0x1800
/* RPDO 映射参数 (1600h-1603h) */
#define ZLAC_OD_RPDO_MAPPING        0x1600
/* TPDO 映射参数 (1A00h-1A03h) */
#define ZLAC_OD_TPDO_MAPPING        0x1A00

/* 厂家自定义参数 (2000h-2FFFh) */
#define ZLAC_OD_OFFLINE_TIME        0x2000
#define ZLAC_OD_485_ADDR            0x2001
#define ZLAC_OD_485_BAUD            0x2002
#define ZLAC_OD_INPUT_STATUS        0x2003
#define ZLAC_OD_OUTPUT_STATUS       0x2004
#define ZLAC_OD_POS_CLEAR           0x2005
#define ZLAC_OD_SET_HOME            0x2006
#define ZLAC_OD_POWER_ON_LOCK       0x2007
#define ZLAC_OD_MAX_SPEED           0x2008
#define ZLAC_OD_RESTORE_DEFAULT     0x2009
#define ZLAC_OD_CAN_NODE_ID         0x200A
#define ZLAC_OD_CAN_BAUDRATE        0x200B
#define ZLAC_OD_MOTOR_POLES         0x200C        /* 子索引1左,2右 */
#define ZLAC_OD_START_SPEED         0x200D
#define ZLAC_OD_ENCODER_LINES       0x200E
#define ZLAC_OD_SYNC_ASYNC          0x200F
#define ZLAC_OD_SAVE_PARAMS         0x2010
#define ZLAC_OD_HALL_OFFSET         0x2011
#define ZLAC_OD_OVERLOAD_FACTOR     0x2012
#define ZLAC_OD_TEMP_PROTECT        0x2013
#define ZLAC_OD_RATED_CURRENT       0x2014
#define ZLAC_OD_MAX_CURRENT         0x2015
#define ZLAC_OD_OVERLOAD_TIME       0x2016
#define ZLAC_OD_POSITION_ERROR      0x2017
#define ZLAC_OD_VEL_SMOOTH          0x2018        /* 速度平滑系数 */
#define ZLAC_OD_CUR_KP              0x2019
#define ZLAC_OD_CUR_KI              0x201A
#define ZLAC_OD_FEEDFORWARD_SMOOTH  0x201B
#define ZLAC_OD_TORQUE_SMOOTH       0x201C
#define ZLAC_OD_VEL_KP              0x201D
#define ZLAC_OD_VEL_KI              0x201E
#define ZLAC_OD_VEL_KF              0x201F
#define ZLAC_OD_POS_KP              0x2020
#define ZLAC_OD_POS_KF              0x2021
#define ZLAC_OD_POS_KFF             0x2022
#define ZLAC_OD_IO_CONFIG           0x2023
#define ZLAC_OD_EMERGENCY_STOP      0x2026
#define ZLAC_OD_BRAKE_CONFIG        0x2030        /* 子索引4:有效电平, 7:B0, 8:B1 */
#define ZLAC_OD_TEMPERATURE         0x2032
#define ZLAC_OD_MOTOR_STATUS        0x2033
#define ZLAC_OD_HALL_STATUS         0x2034
#define ZLAC_OD_BUS_VOLTAGE         0x2035

/* CiA402 运动控制参数 (6000h-6FFFh) */
#define ZLAC_OD_CONTROLWORD         0x6040
#define ZLAC_OD_STATUSWORD          0x6041
#define ZLAC_OD_MODE_OF_OPERATION   0x6060
#define ZLAC_OD_MODE_DISPLAY        0x6061
#define ZLAC_OD_ACTUAL_POSITION     0x6064
#define ZLAC_OD_ACTUAL_VELOCITY     0x606C        /* 子索引1左,2右,3组合 */
#define ZLAC_OD_TARGET_VELOCITY     0x60FF        /* 子索引1左,2右,3组合 */
#define ZLAC_OD_TARGET_TORQUE       0x6071			/* 目标转矩 子索引1左,2右,3组合*/
#define ZLAC_OD_ACTUAL_TORQUE       0x6077			/* 实际转矩 子索引1左,2右,3组合*/
#define ZLAC_OD_TARGET_POSITION     0x607A        /* 子索引1左,2右 */
#define ZLAC_OD_PROFILE_VELOCITY    0x6081		 	/* 位置模式时最大速度 */
#define ZLAC_OD_POS_START_VELOCITY  0x6082			/* 位置模式时启停速度 */
#define ZLAC_OD_PROFILE_ACCEL       0x6083
#define ZLAC_OD_PROFILE_DECEL       0x6084
#define ZLAC_OD_QUICKSTOP_DECEL     0x6085
#define ZLAC_OD_TORQUE_SLOPE        0x6087
#define ZLAC_OD_FAULT_CODE          0x603F
#define ZLAC_OD_HALT_OPTION         0x605D
#define ZLAC_OD_QUICKSTOP_OPTION    0x605A
#define ZLAC_OD_SHUTDOWN_OPTION     0x605B
#define ZLAC_OD_DISABLE_OPTION      0x605C

/* ======================== COB-ID 宏 ======================== */
#define ZLAC_COBID_NMT              0x000
#define ZLAC_COBID_SYNC             0x080
#define ZLAC_COBID_EMCY(addr)       (0x080 + (addr))
#define ZLAC_COBID_TPDO0(addr)      (0x180 + (addr))
#define ZLAC_COBID_RPDO0(addr)      (0x200 + (addr))
#define ZLAC_COBID_TPDO1(addr)      (0x280 + (addr))
#define ZLAC_COBID_RPDO1(addr)      (0x300 + (addr))
#define ZLAC_COBID_TPDO2(addr)      (0x380 + (addr))
#define ZLAC_COBID_RPDO2(addr)      (0x400 + (addr))
#define ZLAC_COBID_TPDO3(addr)      (0x480 + (addr))
#define ZLAC_COBID_RPDO3(addr)      (0x500 + (addr))
#define ZLAC_COBID_TSDO(addr)       (0x580 + (addr))
#define ZLAC_COBID_RSDO(addr)       (0x600 + (addr))
#define ZLAC_COBID_HB(addr)         (0x700 + (addr))

/* ======================== 错误码 ======================== */
#define ZLAC_ERR_OK                 0
#define ZLAC_ERR_TIMEOUT            -1
#define ZLAC_ERR_CAN_TX             -2
#define ZLAC_ERR_CAN_RX             -3
#define ZLAC_ERR_SDO_ABORT          -4
#define ZLAC_ERR_INVALID_PARAM      -5
#define ZLAC_ERR_OFFLINE            -6
#define ZLAC_ERR_INVALID_STATE      -7
#define ZLAC_ERR_BRAKE_UNRELEASE    -8
#define ZLAC_ERR_MODE_UNREADY    		-9


/* ======================== 驱动器状态 ======================== */
typedef enum {
    ZLAC_STATE_NOT_READY = 0,
    ZLAC_STATE_SWITCH_ON_DISABLED,
    ZLAC_STATE_READY_TO_SWITCH_ON,
    ZLAC_STATE_SWITCHED_ON,
    ZLAC_STATE_OPERATION_ENABLED,
    ZLAC_STATE_QUICK_STOP_ACTIVE,
    ZLAC_STATE_FAULT_REACTION_ACTIVE,
    ZLAC_STATE_FAULT,
} ZlacState_t;

/* ======================== 工作模式 ======================== */
typedef enum {
    ZLAC_MODE_UNKNOWN = 0,
    ZLAC_MODE_PROFILE_POSITION = 1,
    ZLAC_MODE_PROFILE_VELOCITY = 3,
    ZLAC_MODE_TORQUE = 4,
} ZlacOpMode_t;

/* 位置模式类型 */
typedef enum {
    ZLAC_POS_MODE_ABSOLUTE = 0,   /* 绝对位置模式 */
    ZLAC_POS_MODE_RELATIVE = 1,   /* 相对位置模式 */
} ZlacPositionMode_t;

/* ======================== 内部函数 ======================== */

/* NMT 服务 */
static rt_err_t zlac_nmt_start(void);                      /* 启动节点 (进入操作状态) */
static rt_err_t zlac_nmt_stop(void);                       /* 停止节点 */
static rt_err_t zlac_nmt_reset_communication(void);        /* 复位通讯 */
static rt_err_t zlac_nmt_reset_node(void);                 /* 复位节点 */
/* 配置驱动器参数 (一次性，可保存到 EEPROM) */
static rt_err_t zlac_motor_config_default(void);           /* 根据电机参数配置驱动器 */
static rt_err_t zlac_save_parameters(void);                /* 保存所有 RW 参数到 EEPROM */
/* ======================== 位置环 PID 配置 ======================== */
static rt_err_t zlac_set_position_kp(uint16_t left_kp, uint16_t right_kp);
static rt_err_t zlac_get_position_kp(uint16_t *left_kp, uint16_t *right_kp);
static rt_err_t zlac_set_position_kf(uint16_t left_kf, uint16_t right_kf);
static rt_err_t zlac_get_position_kf(uint16_t *left_kf, uint16_t *right_kf);

/* ======================== 平滑系数配置 ======================== */
static rt_err_t zlac_set_vel_smooth(uint16_t left_smooth, uint16_t right_smooth);
static rt_err_t zlac_get_vel_smooth(uint16_t *left_smooth, uint16_t *right_smooth);
static rt_err_t zlac_set_feedforward_smooth(uint16_t left_smooth, uint16_t right_smooth);
static rt_err_t zlac_get_feedforward_smooth(uint16_t *left_smooth, uint16_t *right_smooth);
/* ======================== 电机基本参数 ======================== */
static rt_err_t zlac_set_motor_poles(uint16_t left_poles, uint16_t right_poles);
static rt_err_t zlac_get_motor_poles(uint16_t *left_poles, uint16_t *right_poles);
static rt_err_t zlac_set_start_speed(uint16_t left_rpm, uint16_t right_rpm);
static rt_err_t zlac_get_start_speed(uint16_t *left_rpm, uint16_t *right_rpm);
static rt_err_t zlac_set_overload_factor(uint16_t left_factor, uint16_t right_factor);
static rt_err_t zlac_get_overload_factor(uint16_t *left_factor, uint16_t *right_factor);
static rt_err_t zlac_set_temp_threshold(uint16_t left_motor_c, uint16_t right_motor_c, uint16_t driver_c);
static rt_err_t zlac_get_temp_threshold(uint16_t *left_motor_c, uint16_t *right_motor_c, uint16_t *driver_c);
/* ======================== 电机方向 ======================== */
static rt_err_t zlac_set_init_direction(uint16_t direction);   // 0:CW, 1:CCW
static uint16_t zlac_get_init_direction(void);

/* 工作模式设置 */
static rt_err_t zlac_set_op_mode(ZlacOpMode_t mode);
static ZlacOpMode_t zlac_get_op_mode(void);
/* 位置控制 (使用 PDO) */
static rt_err_t zlac_set_position(int32_t left_pulses, int32_t right_pulses, rt_bool_t relative);
/* ======================== 通讯与诊断 ======================== */
static rt_err_t zlac_set_offline_time(uint16_t ms);
static uint16_t zlac_get_offline_time(void);
static uint16_t zlac_get_output_status(void);
static rt_err_t zlac_restore_default(void);
/* PDO 配置 (需要在 NMT 预操作状态下配置) */
static rt_err_t zlac_config_pdo_for_velocity_mode(void);   /* 配置速度模式 PDO */
static rt_err_t zlac_config_pdo_for_position_mode(void);   /* 配置位置模式 PDO */

/* ======================== 公共接口 ======================== */

/* 初始化驱动 (CAN 设备，启动心跳监测) */
rt_err_t zlac_motor_init(void);

/* 状态机控制 */
rt_err_t zlac_control_enable(void);                 /* 使能电机 */
rt_err_t zlac_control_disable(void);                /* 停止，但保持力矩（功率级仍使能） */
rt_err_t zlac_control_free(void);										 /* 完全解轴，电机自由 */
rt_err_t zlac_control_quickstop(void);              /* 快速停止 */
rt_err_t zlac_control_shutdown(void);               /* 关机 (进入 Ready to switch on) */
rt_err_t zlac_control_fault_reset(void);            /* 故障复位 */
ZlacState_t zlac_get_state(uint16_t sw);            /* 获取左右电机当前状态 */
uint32_t read_full_statusword(void);								/* 获取全部状态*/

void zlac_set_right_motor_reverse(rt_bool_t reverse);  /*左右轮同向或反向*/
// 注意： 不管是速度模式还是位置模式，此驱动代码针对左右两轮电机只有同步控制，不做单独的异步控制
/* 速度控制 (使用 PDO，前提是已使能并配置 PDO) */
rt_err_t zlac_init_velocity_mode(void);	/*初始化速度模式并使能电机*/
rt_err_t zlac_set_velocity(int16_t left_rpm, int16_t right_rpm);
rt_err_t zlac_get_velocity(int16_t *left_rpm, int16_t *right_rpm);   /* 读取实际速度 */

/* 位置控制*/
rt_err_t zlac_init_position_mode(ZlacPositionMode_t mode); /*初始化位置模式并使能电机*/
rt_err_t zlac_set_position_by_mode(int32_t left_pulses, int32_t right_pulses);
rt_err_t zlac_get_position(int32_t *left_pulses, int32_t *right_pulses); 
ZlacPositionMode_t zlac_get_position_mode(void); /* 获取当前设置的位置运动模式 */

/* 状态读取*/
rt_bool_t zlac_is_left_target_reached(void);	/* 查询左右电机是否已到达目标位置或目标速度（bit10 = 1 && bit12 = 1）*/
rt_bool_t zlac_is_right_target_reached(void);
uint16_t zlac_get_motor_status_left(void);	/* 电机运动状态*/
uint16_t zlac_get_motor_status_right(void);

/* 绝对位置模式，需要设置原点 */
rt_err_t zlac_set_position_abs_home(void);

/* 抱闸控制 */
rt_err_t zlac_brake_release(void);                  /* 释放抱闸 */
rt_err_t zlac_brake_engage(void);                   /* 抱闸锁紧 */
rt_err_t zlac_get_brake(uint16_t *left_val, uint16_t *right_val) ;

/* 心跳检测 (查询离线状态) */
void zlac_check_heartbeat(void);
rt_bool_t zlac_is_online(void);                     /* 是否在线 */

/* 读取驱动器温度等状态 (通过 SDO) */
int16_t zlac_get_motor_temp_left(void);
int16_t zlac_get_motor_temp_right(void);
int16_t zlac_get_driver_temp(void);
uint16_t zlac_get_bus_voltage(void);                /* 0.01V 单位 */

/* 获取故障码 (32位) */
uint32_t zlac_get_fault_code(void);                 
rt_err_t zlac_clear_fault(void);                    /* 清除故障 (同 fault_reset) */

#endif /* __ZLTECH_CAN_MOTOR_H__ */
