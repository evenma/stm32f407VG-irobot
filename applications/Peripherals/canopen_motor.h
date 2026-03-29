/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief ZLAC8015D V4.0 CANopen dual-hub motor driver + differential drive helper
 */

#ifndef PERIPHERALS_CANOPEN_MOTOR_H__
#define PERIPHERALS_CANOPEN_MOTOR_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/dev_can.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== basic config ======================== */

#ifndef ZLAC_CAN_DEV_NAME
#define ZLAC_CAN_DEV_NAME                      "can1"
#endif

#ifndef ZLAC_CAN_BAUD
#define ZLAC_CAN_BAUD                          CAN500kBaud
#endif

#ifndef ZLAC_NODE_ID_DEFAULT
#define ZLAC_NODE_ID_DEFAULT                   1
#endif

#define ZLAC_SDO_TIMEOUT_MS                    200
#define ZLAC_OFFLINE_TIMEOUT_MS                3000
#define ZLAC_RX_THREAD_STACK_SIZE              2048
#define ZLAC_RX_THREAD_PRIORITY                12
#define ZLAC_RX_THREAD_TICK                    10

#define ZLAC_DEFAULT_ACCEL_MS                  200
#define ZLAC_DEFAULT_DECEL_MS                  200
#define ZLAC_DEFAULT_SYNC_ENABLE               RT_FALSE
#define ZLAC_DEFAULT_WHEEL_TRACK_MM            420
#define ZLAC_DEFAULT_WHEEL_DIAMETER_MM         165
#define ZLAC_DEFAULT_MAX_RPM                   300

#define ZLAC_SPEED_MIN_RPM                     (-1000)
#define ZLAC_SPEED_MAX_RPM                     (1000)

/* ======================== CiA301 / CiA402 ======================== */

#define ZLAC_COB_NMT                           0x000
#define ZLAC_COB_EMCY(node)                    (0x080 + (node))
#define ZLAC_COB_TPDO0(node)                   (0x180 + (node))
#define ZLAC_COB_RPDO0(node)                   (0x200 + (node))
#define ZLAC_COB_TPDO1(node)                   (0x280 + (node))
#define ZLAC_COB_RPDO1(node)                   (0x300 + (node))
#define ZLAC_COB_TPDO2(node)                   (0x380 + (node))
#define ZLAC_COB_RPDO2(node)                   (0x400 + (node))
#define ZLAC_COB_TPDO3(node)                   (0x480 + (node))
#define ZLAC_COB_RPDO3(node)                   (0x500 + (node))
#define ZLAC_COB_SDO_TX(node)                  (0x580 + (node))
#define ZLAC_COB_SDO_RX(node)                  (0x600 + (node))
#define ZLAC_COB_HEARTBEAT(node)               (0x700 + (node))

/* object dictionary */
#define ZLAC_OBJ_HEARTBEAT_PRODUCER            0x1017
#define ZLAC_OBJ_SAVE_PARAM                    0x2010
#define ZLAC_OBJ_SYNC_ASYNC_FLAG               0x200F
#define ZLAC_OBJ_TEMP_INFO                     0x2032
#define ZLAC_OBJ_MOTOR_STATE                   0x2033
#define ZLAC_OBJ_BUS_VOLTAGE                   0x2035
#define ZLAC_OBJ_FAULT_CODE                    0x603F
#define ZLAC_OBJ_CONTROL_WORD                  0x6040
#define ZLAC_OBJ_STATUS_WORD                   0x6041
#define ZLAC_OBJ_QUICK_STOP_CODE               0x605A
#define ZLAC_OBJ_DISABLE_CODE                  0x605C
#define ZLAC_OBJ_HALT_CODE                     0x605D
#define ZLAC_OBJ_MODE_OF_OPERATION             0x6060
#define ZLAC_OBJ_MODE_DISPLAY                  0x6061
#define ZLAC_OBJ_ACTUAL_POSITION               0x6064
#define ZLAC_OBJ_ACTUAL_VELOCITY               0x606C
#define ZLAC_OBJ_TARGET_POSITION               0x607A
#define ZLAC_OBJ_TARGET_TORQUE                 0x6071
#define ZLAC_OBJ_MAX_SPEED                     0x6081
#define ZLAC_OBJ_ACCEL_TIME                    0x6083
#define ZLAC_OBJ_DECEL_TIME                    0x6084
#define ZLAC_OBJ_QUICKSTOP_DECEL               0x6085
#define ZLAC_OBJ_TORQUE_SLOPE                  0x6087
#define ZLAC_OBJ_TARGET_VELOCITY               0x60FF

/* PDO objects (optional) */
#define ZLAC_OBJ_RPDO1_COMM                    0x1401
#define ZLAC_OBJ_RPDO1_MAP                     0x1601
#define ZLAC_OBJ_TPDO0_COMM                    0x1800
#define ZLAC_OBJ_TPDO0_MAP                     0x1A00

/* NMT */
#define ZLAC_NMT_START                         0x01
#define ZLAC_NMT_STOP                          0x02
#define ZLAC_NMT_PREOP                         0x80
#define ZLAC_NMT_RESET_NODE                    0x81
#define ZLAC_NMT_RESET_COMM                    0x82

/* control word */
#define ZLAC_CTRL_DISABLE_VOLTAGE              0x0000
#define ZLAC_CTRL_SHUTDOWN                     0x0006
#define ZLAC_CTRL_SWITCH_ON                    0x0007
#define ZLAC_CTRL_ENABLE_OPERATION             0x000F
#define ZLAC_CTRL_QUICK_STOP                   0x0002
#define ZLAC_CTRL_FAULT_RESET                  0x0080

#define ZLAC_SAVE_PARAM_MAGIC                  0x0001

typedef enum
{
    MOTOR_SIDE_LEFT = 0,
    MOTOR_SIDE_RIGHT,
    MOTOR_SIDE_BOTH,
} MotorSide_t;

typedef enum
{
    ZLAC_MODE_UNDEFINED = 0,
    ZLAC_MODE_POSITION  = 1,
    ZLAC_MODE_SPEED     = 3,
    ZLAC_MODE_TORQUE    = 4,
} ZlacMotorMode_t;

typedef enum
{
    ZLAC_NMT_STATE_BOOTUP       = 0x00,
    ZLAC_NMT_STATE_STOPPED      = 0x04,
    ZLAC_NMT_STATE_OPERATIONAL  = 0x05,
    ZLAC_NMT_STATE_PREOP        = 0x7F,
} ZlacNmtState_t;

typedef struct
{
    rt_int32_t position_counts;
    rt_int16_t actual_speed_rpm_x10;
    rt_uint16_t status_word;

    rt_bool_t enabled;
    rt_bool_t fault;
    rt_bool_t quick_stopped;
    rt_bool_t target_reached;
} ZlacWheelFeedback_t;

typedef struct
{
    rt_bool_t initialized;
    rt_bool_t online;
    rt_bool_t heartbeat_seen;
    rt_bool_t pdo_speed_feedback_enabled;

    rt_uint8_t node_id;
    rt_uint8_t nmt_state;

    rt_tick_t last_rx_tick;
    rt_tick_t last_heartbeat_tick;

    rt_device_t can_dev;
    rt_thread_t rx_thread;
    rt_sem_t rx_sem;
    struct rt_mutex lock;
    struct rt_completion sdo_completion;

    struct
    {
        rt_bool_t pending;
        rt_err_t result;
        rt_uint8_t cs;
        rt_uint16_t index;
        rt_uint8_t subindex;
        rt_uint8_t data[4];
        rt_uint8_t len;
        rt_uint32_t abort_code;
    } sdo_resp;

    struct
    {
        ZlacMotorMode_t mode;
        rt_int16_t target_left_rpm;
        rt_int16_t target_right_rpm;
        ZlacWheelFeedback_t left;
        ZlacWheelFeedback_t right;
        rt_uint32_t fault_code;
        rt_int16_t motor_temp_left_x10;
        rt_int16_t motor_temp_right_x10;
        rt_int16_t driver_temp_x10;
        rt_uint16_t bus_voltage_x100;
    } feedback;

    rt_uint32_t tx_count;
    rt_uint32_t rx_count;
    rt_uint32_t sdo_timeout_count;
    rt_uint32_t sdo_abort_count;
    rt_uint32_t parse_error_count;
} CanopenMotorObject_t;

typedef struct
{
    rt_bool_t enabled;
    rt_bool_t direct_rpm_mode;
    rt_bool_t dirty;

    rt_uint16_t wheel_track_mm;
    rt_uint16_t wheel_diameter_mm;
    rt_int16_t max_rpm;

    rt_int32_t linear_mm_s;
    rt_int32_t angular_mrad_s;

    rt_int16_t target_left_rpm;
    rt_int16_t target_right_rpm;
} DifferentialDriveState_t;

typedef void (*CanopenMotorRxHook_t)(const struct rt_can_msg *msg, void *user_data);

/* ======================== motor API ======================== */

int canopen_motor_init(rt_uint8_t node_id);
void canopen_motor_deinit(void);
const CanopenMotorObject_t *canopen_motor_get_object(void);
void canopen_motor_set_rx_hook(CanopenMotorRxHook_t hook, void *user_data);

int canopen_motor_nmt_command(rt_uint8_t command);
int canopen_motor_start_node(void);
int canopen_motor_stop_node(void);
int canopen_motor_enter_preop(void);

int canopen_motor_set_mode(ZlacMotorMode_t mode);
int canopen_motor_set_sync_mode(rt_bool_t sync_enable);
int canopen_motor_set_accel_decel(rt_uint32_t accel_ms, rt_uint32_t decel_ms);
int canopen_motor_set_quickstop_decel(rt_uint32_t decel_ms);
int canopen_motor_enable(void);
int canopen_motor_disable(void);
int canopen_motor_quick_stop(void);
int canopen_motor_clear_fault(void);
int canopen_motor_stop(void);
int canopen_motor_prepare_speed_mode(rt_bool_t sync_enable, rt_uint32_t accel_ms, rt_uint32_t decel_ms);

int canopen_motor_set_velocity(rt_int16_t left_rpm, rt_int16_t right_rpm);
int canopen_motor_set_single_velocity(MotorSide_t side, rt_int16_t rpm);

int canopen_motor_read_status(void);
int canopen_motor_read_velocity(void);
int canopen_motor_read_position(void);
int canopen_motor_read_fault(void);
int canopen_motor_read_temperature(void);
int canopen_motor_read_bus_voltage(void);
int canopen_motor_refresh_feedback(void);
int canopen_motor_service(void);

int canopen_motor_config_tpdo0_speed(rt_uint8_t transmission_type, rt_uint16_t event_time_ms);
int canopen_motor_config_rpdo1_target_velocity(void);
int canopen_motor_save_parameters(void);

rt_bool_t canopen_motor_is_online(void);

/* ======================== differential drive API ======================== */

int differential_drive_init(void);
void differential_drive_reset(void);
const DifferentialDriveState_t *differential_drive_get_state(void);
int differential_drive_set_geometry(rt_uint16_t wheel_track_mm, rt_uint16_t wheel_diameter_mm);
int differential_drive_set_max_rpm(rt_int16_t max_rpm);
int differential_drive_set_twist(rt_int32_t linear_mm_s, rt_int32_t angular_mrad_s);
int differential_drive_set_twist_deg(rt_int32_t linear_mm_s, rt_int32_t angular_deg_s);
int differential_drive_set_wheel_rpm(rt_int16_t left_rpm, rt_int16_t right_rpm);
int differential_drive_stop(void);
int differential_drive_apply(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_CANOPEN_MOTOR_H__ */
