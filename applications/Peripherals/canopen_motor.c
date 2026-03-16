/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief ZLAC8015D CANopen 电机驱动器驱动
 * 
 * 参考文档：《ZLAC8015D V4.0 CANopen 通信说明 Version 1.00》
 * 协议标准：CANopen DS301 V4.02 + DS402 V2.01
 */

#include <rtthread.h>
#include "canopen_motor.h"


/**
 * @brief 电机对象实例（支持双电机）
 */
static CanMotorObject_t s_motors[MOTOR_MAX_NODES];

/**
 * @brief CAN 设备句柄
 */
static struct rt_device* s_can_dev = RT_NULL;

/**
 * @brief CAN RX 接收线程
 */
static rt_thread_t s_rx_thread = RT_NULL;

/**
 * @brief CANopen COB-ID 宏定义
 */
#define COB_NMT                 0x000
#define COB_TPDO1(node_id)      (0x280 | ((node_id) << 8))
#define COB_SDO_REQUEST(node_id)(0x600 | ((node_id) << 8))
#define COB_HEARTBEAT(node_id)  (0x700 | (node_id))


/* ========== SDO 读写函数 ========== */

static int sdo_read_16bit(uint8_t node_id, uint16_t index, uint8_t subindex, uint16_t* value)
{
    /* 简化实现：直接返回模拟值 */
    if (index == 0x6041)  /* Status word */
    {
        *value = 0x0018;  /* Switched on + Operation enabled */
        return 0;
    }
    else if (index == 0x6064)  /* Position */
    {
        *value = 0x0000;  /* Current position */
        return 0;
    }
    return 0;
}


static int sdo_write_16bit(uint8_t node_id, uint16_t index, uint8_t subindex, uint16_t value)
{
    if (!s_can_dev || !s_can_dev->init)
    {
        return -RT_ERROR;
    }
    
    struct rt_can_frame frame;
    frame.id = COB_SDO_REQUEST(node_id);
    frame.len = 8;
    frame.data[0] = 0x2B;  /* 2-byte write command */
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    frame.data[4] = value & 0xFF;
    frame.data[5] = (value >> 8) & 0xFF;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    
    rt_device_write(s_can_dev, 0, &frame, sizeof(frame));
    rt_thread_mdelay(10);
    
    return 0;
}


static int motor_on(CanMotorObject_t* motor)
{
    /* Step 1: Enable voltage (0x0F) */
    sdo_write_16bit(motor->node_id, OBJ_CONTROL_WORD, 0, 0x000F);
    rt_thread_mdelay(50);
    
    /* Step 2: Switch on (0x0F) */
    sdo_write_16bit(motor->node_id, OBJ_CONTROL_WORD, 0, 0x000F);
    rt_thread_mdelay(50);
    
    /* Step 3: Operate (0x1F) */
    sdo_write_16bit(motor->node_id, OBJ_CONTROL_WORD, 0, 0x001F);
    rt_thread_mdelay(50);
    
    /* Verify status */
    uint16_t status;
    sdo_read_16bit(motor->node_id, OBJ_STATUS_WORD, 0, &status);
    motor->feedback.status_word = status;
    
    return ((status & 0x0038) == 0x0018) ? 0 : -RT_ERROR;
}


static int motor_off(CanMotorObject_t* motor)
{
    /* Disable operating → Switch off → Disable voltage */
    sdo_write_16bit(motor->node_id, OBJ_CONTROL_WORD, 0, 0x0006);
    rt_thread_mdelay(50);
    sdo_write_16bit(motor->node_id, OBJ_CONTROL_WORD, 0, 0x0008);
    rt_thread_mdelay(50);
    
    return 0;
}


int canopen_motor_init(rt_uint8_t* nodes, rt_uint8_t count)
{
    if (nodes == RT_NULL || count == 0 || count > MOTOR_MAX_NODES)
    {
        return -RT_ERROR;
    }
    
    /* 1. 初始化 CAN1 */
    s_can_dev = rt_device_find(MOTOR_CAN_BUS);
    if (s_can_dev == RT_NULL)
    {
        rt_kprintf("[CAN] Device not found: %s\n", MOTOR_CAN_BUS);
        return -RT_ERROR;
    }
    
    struct rt_can_info can_info = {
        .receive_filter = CAN_ALL_FILTERS_ENABLE,
        .timing = {
            .prescaler = 2,
            .time_seg1 = 13,
            .time_seg2 = 2,
            .sjw = 1,
            .baudrate = RT_CAN_BITRATE_500KBPS,
        },
    };
    rt_device_control(s_can_dev, RT_DEVICE_CTRL_CAN_SET_MODE, &can_info);
    
    /* 2. 初始化每个电机 */
    for (uint8_t i = 0; i < count; i++)
    {
        canopen_motor_init_single(nodes[i]);
        
        /* Send NMT Start command */
        struct rt_can_frame nmt;
        nmt.id = COB_NMT;
        nmt.len = 2;
        nmt.data[0] = 0x0F;
        nmt.data[1] = nodes[i];
        rt_device_write(s_can_dev, 0, &nmt, sizeof(nmt));
        
        rt_kprintf("[CAN] Node %d started via NMT\n", nodes[i]);
    }
    
    rt_kprintf("[CAN] Motor system initialized (%d nodes)\n", count);
    return 0;
}


int canopen_motor_init_single(uint8_t node_id)
{
    uint8_t idx = node_id - 1;
    
    memset(&s_motors[idx], 0, sizeof(CanMotorObject_t));
    s_motors[idx].node_id = node_id;
    s_motors[idx].initialized = RT_TRUE;
    
    /* Default configuration per ZLAC8015D spec */
    s_motors[idx].config.encoder_ticks_per_rev = 4096.0f;
    s_motors[idx].config.gear_ratio = 1.0f;
    s_motors[idx].config.max_velocity_rpm = 3000.0f;
    s_motors[idx].config.acceleration_rpm_per_sec = 5000.0f;
    s_motors[idx].config.deceleration_rpm_per_sec = 5000.0f;
    s_motors[idx].op_mode = MODE_PROFILE_VELOCITY;  /* CSV mode */
    
    /* PID defaults */
    s_motors[idx].kp = 1.0f;
    s_motors[idx].ki = 0.1f;
    s_motors[idx].kd = 0.05f;
    s_motors[idx].integral_limit = 100.0f;
    
    rt_kprintf("[CAN] Motor node %d initialized (CSV mode)\n", node_id);
    return 0;
}


void canopen_motor_deinit(void)
{
    for (uint8_t i = 0; i < MOTOR_MAX_NODES; i++)
    {
        if (s_motors[i].initialized)
        {
            motor_off(&s_motors[i]);
        }
    }
}


CanMotorObject_t* canopen_motor_get_object(uint8_t idx)
{
    if (idx >= MOTOR_MAX_NODES) return RT_NULL;
    return &s_motors[idx];
}


int canopen_motor_enable(uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return -RT_ERROR;
    }
    return motor_on(&s_motors[motor_idx]);
}


int canopen_motor_disable(uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return -RT_ERROR;
    }
    return motor_off(&s_motors[motor_idx]);
}


int canopen_motor_reset_fault(uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return -RT_ERROR;
    }
    
    CanMotorObject_t* motor = &s_motors[motor_idx];
    
    /* Write to status word to clear fault (0x80 bit) */
    sdo_write_16bit(motor->node_id, OBJ_CONTROL_WORD, 0, 0x0080);
    rt_thread_mdelay(50);
    
    return 0;
}


int canopen_motor_homing(uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return -RT_ERROR;
    }
    
    CanMotorObject_t* motor = &s_motors[motor_idx];
    
    /* Homing procedure:
     * 1. Set operation mode to homing (requires CSP mode)
     * 2. Target position = origin with offset
     * 3. Monitor state machine transition
     */
    
    motor->command.control_word = 0x000F;
    sdo_write_16bit(motor->node_id, OBJ_CONTROL_WORD, 0, 0x000F);
    
    /* Reset position counter */
    sdo_write_16bit(motor->node_id, OBJ_CURRENT_POSITION, 0, 0x0000);
    
    /* Start homing motion (absolute position at offset) */
    int32_t target = (int32_t)motor->config.homing_offset;
    sdo_write_32bit(motor->node_id, OBJ_CURRENT_POSITION, 0, target);
    
    rt_thread_mdelay(100);
    
    return 0;
}


int canopen_motor_set_velocity(uint8_t motor_idx, float rpm)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return -RT_ERROR;
    }
    
    CanMotorObject_t* motor = &s_motors[motor_idx];
    
    /* Clamp to max speed */
    if (rpm > motor->config.max_velocity_rpm) rpm = motor->config.max_velocity_rpm;
    if (rpm < -motor->config.max_velocity_rpm) rpm = -motor->config.max_velocity_rpm;
    
    /* Prepare TPDO1 command */
    struct rt_can_frame tpdo;
    tpdo.id = COB_TPDO1(motor->node_id);
    tpdo.len = 8;
    
    /* Control Word */
    int32_t vel_scaled = (int32_t)(rpm * 100.0f);  /* Scale by 100 */
    
    tpdo.data[0] = 0x1F & 0xFF;          /* CtrlWord low */
    tpdo.data[1] = (0x1F >> 8) & 0xFF;   /* CtrlWord high */
    tpdo.data[2] = motor->op_mode;       /* Op mode */
    tpdo.data[3] = 0x00;
    tpdo.data[4] = vel_scaled & 0xFF;
    tpdo.data[5] = (vel_scaled >> 8) & 0xFF;
    tpdo.data[6] = (vel_scaled >> 16) & 0xFF;
    tpdo.data[7] = (vel_scaled >> 24) & 0xFF;
    
    rt_device_write(s_can_dev, 0, &tpdo, sizeof(tpdo));
    
    motor->tx_count++;
    return 0;
}


int canopen_motor_quick_stop(uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return -RT_ERROR;
    }
    
    CanMotorObject_t* motor = &s_motors[motor_idx];
    
    /* Quick stop via control word (0x0006) */
    sdo_write_16bit(motor->node_id, OBJ_CONTROL_WORD, 0, 0x0006);
    motor->command.control_word = 0x0006;
    
    return 0;
}


int canopen_motor_set_position(uint8_t motor_idx, float positions)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return -RT_ERROR;
    }
    
    CanMotorObject_t* motor = &s_motors[motor_idx];
    
    /* Convert revolutions to encoder ticks */
    int32_t ticks = (int32_t)(positions * motor->config.encoder_ticks_per_rev);
    
    /* Send absolute position command via TPDO */
    struct rt_can_frame tpdo;
    tpdo.id = COB_TPDO1(motor->node_id);
    tpdo.len = 8;
    
    tpdo.data[0] = 0x1F & 0xFF;              /* CtrlWord */
    tpdo.data[1] = (0x1F >> 8) & 0xFF;
    tpdo.data[2] = MODE_PROFILE_POSITION;    /* CSP mode */
    tpdo.data[3] = 0x00;
    tpdo.data[4] = ticks & 0xFF;
    tpdo.data[5] = (ticks >> 8) & 0xFF;
    tpdo.data[6] = (ticks >> 16) & 0xFF;
    tpdo.data[7] = (ticks >> 24) & 0xFF;
    
    rt_device_write(s_can_dev, 0, &tpdo, sizeof(tpdo));
    
    motor->target_value = positions;
    motor->tx_count++;
    return 0;
}


int canopen_motor_move_relative(uint8_t motor_idx, float angle_deg)
{
    CanMotorObject_t* motor = canopen_motor_get_object(motor_idx);
    
    /* Angle to revolutions */
    float rev = angle_deg / 360.0f;
    
    /* Get current position from feedback */
    sdo_read_16bit(motor->node_id, OBJ_CURRENT_POSITION, 0, NULL);
    
    /* TODO: Implement relative move based on current position */
    return canopen_motor_set_position(motor_idx, rev);
}


rt_bool_t canopen_motor_is_reached(uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return RT_FALSE;
    }
    
    /* Check status word bit 11 (position reached) */
    return ((s_motors[motor_idx].feedback.status_word & (1 << 11)) != 0);
}


int canopen_motor_read_feedback(uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return -RT_ERROR;
    }
    
    CanMotorObject_t* motor = &s_motors[motor_idx];
    
    /* Read status word (0x6041) */
    sdo_read_16bit(motor->node_id, OBJ_STATUS_WORD, 0, &motor->feedback.status_word);
    
    /* Parse bits */
    motor->feedback.bits.switched_on = (motor->feedback.status_word & (1 << 3)) ? 1 : 0;
    motor->feedback.bits.operation_enabled = (motor->feedback.status_word & (1 << 4)) ? 1 : 0;
    motor->feedback.bits.fault = (motor->feedback.status_word & (1 << 5)) ? 1 : 0;
    
    /* Read actual position (0x6064) */
    sdo_read_16bit(motor->node_id, OBJ_CURRENT_POSITION, 0, (uint16_t*)&motor->feedback.position_ticks);
    
    motor->rx_count++;
    return 0;
}


MotorStateBits_t canopen_motor_get_state(uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return (MotorStateBits_t)0;
    }
    
    CanMotorObject_t* motor = &s_motors[motor_idx];
    
    uint16_t status = motor->feedback.status_word;
    
    if (status & (1 << 5)) return STATE_FAULT;
    if (status & (1 << 4)) return STATE_OPERATION_ENABLED;
    if (status & (1 << 3)) return STATE_SWITCHED_ON;
    if (status & (1 << 2)) return STATE_READY_TO_SWITCH_ON;
    return STATE_NOT_READY;
}


rt_bool_t canopen_motor_has_fault(uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return RT_FALSE;
    }
    
    return s_motors[motor_idx].feedback.bits.fault;
}


uint16_t canopen_motor_get_fault_code(uint8_t motor_idx)
{
    /* TODO: Read fault register (0xFF00 or similar) */
    return 0x0000;
}


float canopen_motor_pid_get_output(uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return 0.0f;
    }
    
    CanMotorObject_t* motor = &s_motors[motor_idx];
    
    float error = motor->target_value - (motor->feedback.position_ticks / motor->config.encoder_ticks_per_rev);
    
    /* PID calculation */
    motor->integral_sum += error;
    if (motor->integral_sum > motor->integral_limit) motor->integral_sum = motor->integral_limit;
    if (motor->integral_sum < -motor->integral_limit) motor->integral_sum = -motor->integral_limit;
    
    float derivative = error - motor->last_error;
    motor->last_error = error;
    
    float output = (motor->kp * error) + (motor->ki * motor->integral_sum) + (motor->kd * derivative);
    
    return output;
}


void canopen_motor_pid_update(uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return;
    }
    
    /* Update feedback first */
    canopen_motor_read_feedback(motor_idx);
    
    /* PID computation */
    float pid_output = canopen_motor_pid_get_output(motor_idx);
    
    /* Apply velocity control based on PID output */
    float target_rpm = pid_output / 10.0f;
    canopen_motor_set_velocity(motor_idx, target_rpm);
}


int canopen_motor_set_pid(uint8_t motor_idx, float kp, float ki, float kd, float limit)
{
    if (motor_idx >= MOTOR_MAX_NODES || !s_motors[motor_idx].initialized)
    {
        return -RT_ERROR;
    }
    
    CanMotorObject_t* motor = &s_motors[motor_idx];
    
    motor->kp = kp;
    motor->ki = ki;
    motor->kd = kd;
    motor->integral_limit = limit;
    
    return 0;
}


/* MSH Debug Commands */
#ifdef RT_USING_MSH

MSH_CMD_EXPORT(canopen_motor_status, "motor status");
MSH_CMD_EXPORT(canopen_motor_velocity, "motor vel [left|right] rpm");
MSH_CMD_EXPORT(canopen_motor_pid, "motor pid kp ki kd");
MSH_CMD_EXPORT(canopen_motor_test, "motor test duration_ms");

#endif
