/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief ZLAC8015D V4.0 CANopen dual-hub motor driver + differential drive helper
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/dev_can.h>
#include <string.h>
#include <stdlib.h>

#include "canopen_motor.h"
#include "../System/global_conf.h"

#define ZLAC_SDO_OK_WRITE                 0x60
#define ZLAC_SDO_OK_READ_1BYTE            0x4F
#define ZLAC_SDO_OK_READ_2BYTE            0x4B
#define ZLAC_SDO_OK_READ_3BYTE            0x47
#define ZLAC_SDO_OK_READ_4BYTE            0x43
#define ZLAC_SDO_ABORT                    0x80
#define ZLAC_PI_X1000                     3142

#define ZLAC_STATUS_LOW16(v)              ((rt_uint16_t)((v) & 0xFFFFU))
#define ZLAC_STATUS_HIGH16(v)             ((rt_uint16_t)(((v) >> 16) & 0xFFFFU))

static CanopenMotorObject_t s_motor_obj;
static DifferentialDriveState_t s_diff_state;
static rt_uint8_t s_service_divider = 0;
static rt_bool_t s_lock_inited = RT_FALSE;
static CanopenMotorRxHook_t s_rx_hook = RT_NULL;
static void *s_rx_hook_user_data = RT_NULL;

static rt_err_t _can_rx_indicate(rt_device_t dev, rt_size_t size);
static void _can_rx_thread_entry(void *parameter);
static int _can_open_and_prepare(rt_uint8_t node_id);
static void _can_cleanup(void);
static int _can_install_filters(rt_uint8_t node_id);
static int _can_send_frame(rt_uint32_t can_id, const rt_uint8_t *data, rt_uint8_t len);
static void _handle_rx_frame(const struct rt_can_msg *msg);
static void _handle_sdo_response(const struct rt_can_msg *msg);
static void _handle_heartbeat(const struct rt_can_msg *msg);
static void _handle_tpdo0_speed(const struct rt_can_msg *msg);
static void _update_wheel_status(ZlacWheelFeedback_t *wheel, rt_uint16_t status_word);
static rt_bool_t _wait_sdo_response(rt_int32_t timeout_ms);
static int _sdo_write_raw(rt_uint16_t index, rt_uint8_t subindex, rt_uint8_t cmd, rt_uint32_t value);
static int _sdo_write_u8(rt_uint16_t index, rt_uint8_t subindex, rt_uint8_t value);
static int _sdo_write_u16(rt_uint16_t index, rt_uint8_t subindex, rt_uint16_t value);
static int _sdo_write_u32(rt_uint16_t index, rt_uint8_t subindex, rt_uint32_t value);
static int _sdo_read_raw(rt_uint16_t index, rt_uint8_t subindex, rt_uint32_t *value, rt_uint8_t *value_len);
static int _sdo_read_u16(rt_uint16_t index, rt_uint8_t subindex, rt_uint16_t *value);
static int _sdo_read_u32(rt_uint16_t index, rt_uint8_t subindex, rt_uint32_t *value);
static int _sdo_read_i32(rt_uint16_t index, rt_uint8_t subindex, rt_int32_t *value);
static rt_int16_t _clamp_speed(rt_int16_t rpm);
static rt_int16_t _linear_to_rpm(rt_int32_t linear_mm_s, rt_uint16_t wheel_diameter_mm);
static void _diff_update_from_twist(rt_int32_t linear_mm_s, rt_int32_t angular_mrad_s);
static const char *_nmt_state_name(rt_uint8_t state);
static const char *_mode_name(ZlacMotorMode_t mode);

static rt_err_t _can_rx_indicate(rt_device_t dev, rt_size_t size)
{
    RT_UNUSED(dev);
    RT_UNUSED(size);

    if (s_motor_obj.rx_sem != RT_NULL)
    {
        rt_sem_release(s_motor_obj.rx_sem);
    }

    return RT_EOK;
}

static void _can_rx_thread_entry(void *parameter)
{
    struct rt_can_msg msg;

    RT_UNUSED(parameter);

    while (1)
    {
        if (rt_sem_take(s_motor_obj.rx_sem, RT_WAITING_FOREVER) != RT_EOK)
        {
            continue;
        }

        while (1)
        {
            rt_memset(&msg, 0, sizeof(msg));
            msg.hdr_index = -1;

            if (rt_device_read(s_motor_obj.can_dev, 0, &msg, sizeof(msg)) != sizeof(msg))
            {
                break;
            }

            s_motor_obj.rx_count++;
            s_motor_obj.last_rx_tick = rt_tick_get();
            s_motor_obj.online = RT_TRUE;
            _handle_rx_frame(&msg);
        }
    }
}

static void _can_cleanup(void)
{
    if (s_motor_obj.can_dev != RT_NULL)
    {
        rt_device_set_rx_indicate(s_motor_obj.can_dev, RT_NULL);
    }

    if (s_motor_obj.rx_thread != RT_NULL)
    {
        rt_thread_delete(s_motor_obj.rx_thread);
        s_motor_obj.rx_thread = RT_NULL;
    }

    if (s_motor_obj.rx_sem != RT_NULL)
    {
        rt_sem_delete(s_motor_obj.rx_sem);
        s_motor_obj.rx_sem = RT_NULL;
    }

    if (s_motor_obj.can_dev != RT_NULL)
    {
        rt_device_close(s_motor_obj.can_dev);
        s_motor_obj.can_dev = RT_NULL;
    }

    if (s_lock_inited)
    {
        rt_mutex_detach(&s_motor_obj.lock);
        s_lock_inited = RT_FALSE;
    }

    s_rx_hook = RT_NULL;
    s_rx_hook_user_data = RT_NULL;
    rt_memset(&s_motor_obj, 0, sizeof(s_motor_obj));
}

static int _can_open_and_prepare(rt_uint8_t node_id)
{
    rt_uint32_t arg;
    rt_err_t result;

    rt_memset(&s_motor_obj, 0, sizeof(s_motor_obj));
    s_lock_inited = RT_FALSE;
    s_motor_obj.node_id = node_id;
    s_motor_obj.nmt_state = ZLAC_NMT_STATE_BOOTUP;
    s_motor_obj.feedback.mode = ZLAC_MODE_UNDEFINED;

    s_motor_obj.can_dev = rt_device_find(ZLAC_CAN_DEV_NAME);
    if (s_motor_obj.can_dev == RT_NULL)
    {
        rt_kprintf("[ZLAC] can device not found: %s\n", ZLAC_CAN_DEV_NAME);
        return -RT_ERROR;
    }

    result = rt_mutex_init(&s_motor_obj.lock, "zlaclk", RT_IPC_FLAG_PRIO);
    if (result != RT_EOK)
    {
        rt_kprintf("[ZLAC] mutex init failed\n");
        return result;
    }
    s_lock_inited = RT_TRUE;

    rt_completion_init(&s_motor_obj.sdo_completion);

    s_motor_obj.rx_sem = rt_sem_create("zlacrx", 0, RT_IPC_FLAG_FIFO);
    if (s_motor_obj.rx_sem == RT_NULL)
    {
        _can_cleanup();
        return -RT_ENOMEM;
    }

    result = rt_device_open(s_motor_obj.can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    if (result != RT_EOK)
    {
        rt_kprintf("[ZLAC] open %s failed: %d\n", ZLAC_CAN_DEV_NAME, result);
        _can_cleanup();
        return result;
    }

    arg = ZLAC_CAN_BAUD;
    result = rt_device_control(s_motor_obj.can_dev, RT_CAN_CMD_SET_BAUD, (void *)arg);
    if (result != RT_EOK)
    {
        rt_kprintf("[ZLAC] set baud failed: %d\n", result);
        _can_cleanup();
        return result;
    }

    arg = RT_CAN_MODE_NORMAL;
    result = rt_device_control(s_motor_obj.can_dev, RT_CAN_CMD_SET_MODE, (void *)arg);
    if (result != RT_EOK)
    {
        rt_kprintf("[ZLAC] set mode failed: %d\n", result);
        _can_cleanup();
        return result;
    }

    result = _can_install_filters(node_id);
    if (result != RT_EOK)
    {
        rt_kprintf("[ZLAC] install filters failed: %d\n", result);
        _can_cleanup();
        return result;
    }

    rt_device_set_rx_indicate(s_motor_obj.can_dev, _can_rx_indicate);

    arg = 1;
    result = rt_device_control(s_motor_obj.can_dev, RT_CAN_CMD_START, (void *)arg);
    if (result != RT_EOK)
    {
        rt_kprintf("[ZLAC] start can failed: %d\n", result);
        _can_cleanup();
        return result;
    }

    s_motor_obj.rx_thread = rt_thread_create("zlac_rx",
                                             _can_rx_thread_entry,
                                             RT_NULL,
                                             ZLAC_RX_THREAD_STACK_SIZE,
                                             ZLAC_RX_THREAD_PRIORITY,
                                             ZLAC_RX_THREAD_TICK);
    if (s_motor_obj.rx_thread == RT_NULL)
    {
        _can_cleanup();
        return -RT_ENOMEM;
    }

    rt_thread_startup(s_motor_obj.rx_thread);
    s_motor_obj.initialized = RT_TRUE;
    return RT_EOK;
}

static int _can_install_filters(rt_uint8_t node_id)
{
#ifdef RT_CAN_USING_HDR
    struct rt_can_filter_item items[] =
    {
        RT_CAN_FILTER_STD_INIT(ZLAC_COB_SDO_TX(node_id), RT_NULL, RT_NULL),
        RT_CAN_FILTER_STD_INIT(ZLAC_COB_HEARTBEAT(node_id), RT_NULL, RT_NULL),
        RT_CAN_FILTER_STD_INIT(ZLAC_COB_TPDO0(node_id), RT_NULL, RT_NULL),
        RT_CAN_FILTER_STD_INIT(ZLAC_COB_EMCY(node_id), RT_NULL, RT_NULL),
    };
    struct rt_can_filter_config cfg =
    {
        sizeof(items) / sizeof(items[0]),
        1,
        items,
    };

    return rt_device_control(s_motor_obj.can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
#else
    RT_UNUSED(node_id);
    return RT_EOK;
#endif
}

static int _can_send_frame(rt_uint32_t can_id, const rt_uint8_t *data, rt_uint8_t len)
{
    struct rt_can_msg msg;

    if (s_motor_obj.can_dev == RT_NULL || data == RT_NULL || len > 8)
    {
        return -RT_ERROR;
    }

    rt_memset(&msg, 0, sizeof(msg));
    msg.id = can_id;
    msg.ide = RT_CAN_STDID;
    msg.rtr = RT_CAN_DTR;
    msg.len = len;
    msg.hdr_index = -1;
    rt_memcpy(msg.data, data, len);

    if (rt_device_write(s_motor_obj.can_dev, 0, &msg, sizeof(msg)) != sizeof(msg))
    {
        return -RT_ERROR;
    }

    s_motor_obj.tx_count++;
    return RT_EOK;
}

static void _handle_rx_frame(const struct rt_can_msg *msg)
{
    if (msg == RT_NULL || msg->ide != RT_CAN_STDID)
    {
        return;
    }

    if (s_rx_hook != RT_NULL)
    {
        s_rx_hook(msg, s_rx_hook_user_data);
    }

    if (msg->id == ZLAC_COB_SDO_TX(s_motor_obj.node_id))
    {
        _handle_sdo_response(msg);
    }
    else if (msg->id == ZLAC_COB_HEARTBEAT(s_motor_obj.node_id))
    {
        _handle_heartbeat(msg);
    }
    else if (msg->id == ZLAC_COB_TPDO0(s_motor_obj.node_id))
    {
        _handle_tpdo0_speed(msg);
    }
    else if (msg->id == ZLAC_COB_EMCY(s_motor_obj.node_id))
    {
        if (msg->len >= 2)
        {
            s_motor_obj.feedback.fault_code = (rt_uint16_t)(msg->data[0] | (msg->data[1] << 8));
        }
    }
}

static void _handle_sdo_response(const struct rt_can_msg *msg)
{
    if (msg == RT_NULL || msg->len < 4)
    {
        s_motor_obj.parse_error_count++;
        return;
    }

    s_motor_obj.sdo_resp.cs = msg->data[0];
    s_motor_obj.sdo_resp.index = (rt_uint16_t)(msg->data[1] | (msg->data[2] << 8));
    s_motor_obj.sdo_resp.subindex = msg->data[3];
    s_motor_obj.sdo_resp.abort_code = 0;
    s_motor_obj.sdo_resp.len = 0;

    if (msg->data[0] == ZLAC_SDO_ABORT)
    {
        s_motor_obj.sdo_resp.abort_code = (rt_uint32_t)msg->data[4] |
                                          ((rt_uint32_t)msg->data[5] << 8) |
                                          ((rt_uint32_t)msg->data[6] << 16) |
                                          ((rt_uint32_t)msg->data[7] << 24);
        s_motor_obj.sdo_resp.result = -RT_ERROR;
        s_motor_obj.sdo_abort_count++;
    }
    else if (msg->data[0] == ZLAC_SDO_OK_WRITE)
    {
        s_motor_obj.sdo_resp.result = RT_EOK;
    }
    else if (msg->data[0] == ZLAC_SDO_OK_READ_1BYTE)
    {
        s_motor_obj.sdo_resp.len = 1;
        s_motor_obj.sdo_resp.data[0] = msg->data[4];
        s_motor_obj.sdo_resp.result = RT_EOK;
    }
    else if (msg->data[0] == ZLAC_SDO_OK_READ_2BYTE)
    {
        s_motor_obj.sdo_resp.len = 2;
        s_motor_obj.sdo_resp.data[0] = msg->data[4];
        s_motor_obj.sdo_resp.data[1] = msg->data[5];
        s_motor_obj.sdo_resp.result = RT_EOK;
    }
    else if (msg->data[0] == ZLAC_SDO_OK_READ_3BYTE)
    {
        s_motor_obj.sdo_resp.len = 3;
        s_motor_obj.sdo_resp.data[0] = msg->data[4];
        s_motor_obj.sdo_resp.data[1] = msg->data[5];
        s_motor_obj.sdo_resp.data[2] = msg->data[6];
        s_motor_obj.sdo_resp.result = RT_EOK;
    }
    else if (msg->data[0] == ZLAC_SDO_OK_READ_4BYTE)
    {
        s_motor_obj.sdo_resp.len = 4;
        s_motor_obj.sdo_resp.data[0] = msg->data[4];
        s_motor_obj.sdo_resp.data[1] = msg->data[5];
        s_motor_obj.sdo_resp.data[2] = msg->data[6];
        s_motor_obj.sdo_resp.data[3] = msg->data[7];
        s_motor_obj.sdo_resp.result = RT_EOK;
    }
    else
    {
        s_motor_obj.sdo_resp.result = -RT_ERROR;
        s_motor_obj.parse_error_count++;
    }

    if (s_motor_obj.sdo_resp.pending)
    {
        s_motor_obj.sdo_resp.pending = RT_FALSE;
        rt_completion_done(&s_motor_obj.sdo_completion);
    }
}

static void _handle_heartbeat(const struct rt_can_msg *msg)
{
    if (msg == RT_NULL || msg->len < 1)
    {
        return;
    }

    s_motor_obj.last_heartbeat_tick = rt_tick_get();
    s_motor_obj.last_rx_tick = s_motor_obj.last_heartbeat_tick;
    s_motor_obj.heartbeat_seen = RT_TRUE;
    s_motor_obj.nmt_state = msg->data[0];
    s_motor_obj.online = RT_TRUE;
}

static void _handle_tpdo0_speed(const struct rt_can_msg *msg)
{
    if (msg == RT_NULL || msg->len < 4)
    {
        return;
    }

    s_motor_obj.feedback.left.actual_speed_rpm_x10 = (rt_int16_t)(msg->data[0] | (msg->data[1] << 8));
    s_motor_obj.feedback.right.actual_speed_rpm_x10 = (rt_int16_t)(msg->data[2] | (msg->data[3] << 8));
    s_motor_obj.pdo_speed_feedback_enabled = RT_TRUE;
}

static void _update_wheel_status(ZlacWheelFeedback_t *wheel, rt_uint16_t status_word)
{
    if (wheel == RT_NULL)
    {
        return;
    }

    wheel->status_word = status_word;
    wheel->quick_stopped = ((status_word & 0x0020U) == 0U) ? RT_TRUE : RT_FALSE;
    wheel->fault = ((status_word & 0x0008U) != 0U) ? RT_TRUE : RT_FALSE;
    wheel->enabled = ((status_word & 0x0004U) != 0U) ? RT_TRUE : RT_FALSE;
    wheel->target_reached = ((status_word & 0x0400U) != 0U) ? RT_TRUE : RT_FALSE;
}

static rt_bool_t _wait_sdo_response(rt_int32_t timeout_ms)
{
    rt_err_t result;

    result = rt_completion_wait(&s_motor_obj.sdo_completion,
                                rt_tick_from_millisecond(timeout_ms));
    if (result != RT_EOK)
    {
        s_motor_obj.sdo_timeout_count++;
        return RT_FALSE;
    }

    return RT_TRUE;
}

static int _sdo_write_raw(rt_uint16_t index, rt_uint8_t subindex, rt_uint8_t cmd, rt_uint32_t value)
{
    rt_uint8_t data[8];
    int result;

    if (!s_motor_obj.initialized)
    {
        return -RT_ERROR;
    }

    rt_mutex_take(&s_motor_obj.lock, RT_WAITING_FOREVER);

    rt_memset(data, 0, sizeof(data));
    data[0] = cmd;
    data[1] = (rt_uint8_t)(index & 0xFFU);
    data[2] = (rt_uint8_t)((index >> 8) & 0xFFU);
    data[3] = subindex;
    data[4] = (rt_uint8_t)(value & 0xFFU);
    data[5] = (rt_uint8_t)((value >> 8) & 0xFFU);
    data[6] = (rt_uint8_t)((value >> 16) & 0xFFU);
    data[7] = (rt_uint8_t)((value >> 24) & 0xFFU);

    rt_completion_init(&s_motor_obj.sdo_completion);
    s_motor_obj.sdo_resp.pending = RT_TRUE;
    s_motor_obj.sdo_resp.result = -RT_ERROR;
    s_motor_obj.sdo_resp.abort_code = 0;

    result = _can_send_frame(ZLAC_COB_SDO_RX(s_motor_obj.node_id), data, 8);
    if (result != RT_EOK)
    {
        s_motor_obj.sdo_resp.pending = RT_FALSE;
        rt_mutex_release(&s_motor_obj.lock);
        return result;
    }

    if (!_wait_sdo_response(ZLAC_SDO_TIMEOUT_MS))
    {
        rt_mutex_release(&s_motor_obj.lock);
        return -RT_ETIMEOUT;
    }

    if (s_motor_obj.sdo_resp.index != index || s_motor_obj.sdo_resp.subindex != subindex)
    {
        rt_mutex_release(&s_motor_obj.lock);
        return -RT_ERROR;
    }

    result = s_motor_obj.sdo_resp.result;
    rt_mutex_release(&s_motor_obj.lock);
    return result;
}

static int _sdo_write_u8(rt_uint16_t index, rt_uint8_t subindex, rt_uint8_t value)
{
    return _sdo_write_raw(index, subindex, 0x2F, value);
}

static int _sdo_write_u16(rt_uint16_t index, rt_uint8_t subindex, rt_uint16_t value)
{
    return _sdo_write_raw(index, subindex, 0x2B, value);
}

static int _sdo_write_u32(rt_uint16_t index, rt_uint8_t subindex, rt_uint32_t value)
{
    return _sdo_write_raw(index, subindex, 0x23, value);
}

static int _sdo_read_raw(rt_uint16_t index, rt_uint8_t subindex, rt_uint32_t *value, rt_uint8_t *value_len)
{
    rt_uint8_t data[8];
    int result;
    rt_uint32_t temp = 0;

    if (!s_motor_obj.initialized || value == RT_NULL || value_len == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_mutex_take(&s_motor_obj.lock, RT_WAITING_FOREVER);

    rt_memset(data, 0, sizeof(data));
    data[0] = 0x40;
    data[1] = (rt_uint8_t)(index & 0xFFU);
    data[2] = (rt_uint8_t)((index >> 8) & 0xFFU);
    data[3] = subindex;

    rt_completion_init(&s_motor_obj.sdo_completion);
    s_motor_obj.sdo_resp.pending = RT_TRUE;
    s_motor_obj.sdo_resp.result = -RT_ERROR;
    s_motor_obj.sdo_resp.abort_code = 0;

    result = _can_send_frame(ZLAC_COB_SDO_RX(s_motor_obj.node_id), data, 8);
    if (result != RT_EOK)
    {
        s_motor_obj.sdo_resp.pending = RT_FALSE;
        rt_mutex_release(&s_motor_obj.lock);
        return result;
    }

    if (!_wait_sdo_response(ZLAC_SDO_TIMEOUT_MS))
    {
        rt_mutex_release(&s_motor_obj.lock);
        return -RT_ETIMEOUT;
    }

    if (s_motor_obj.sdo_resp.index != index || s_motor_obj.sdo_resp.subindex != subindex)
    {
        rt_mutex_release(&s_motor_obj.lock);
        return -RT_ERROR;
    }

    if (s_motor_obj.sdo_resp.result == RT_EOK)
    {
        temp = (rt_uint32_t)s_motor_obj.sdo_resp.data[0] |
               ((rt_uint32_t)s_motor_obj.sdo_resp.data[1] << 8) |
               ((rt_uint32_t)s_motor_obj.sdo_resp.data[2] << 16) |
               ((rt_uint32_t)s_motor_obj.sdo_resp.data[3] << 24);
        *value = temp;
        *value_len = s_motor_obj.sdo_resp.len;
    }

    result = s_motor_obj.sdo_resp.result;
    rt_mutex_release(&s_motor_obj.lock);
    return result;
}

static int _sdo_read_u16(rt_uint16_t index, rt_uint8_t subindex, rt_uint16_t *value)
{
    rt_uint32_t temp;
    rt_uint8_t len;
    int result;

    if (value == RT_NULL)
    {
        return -RT_ERROR;
    }

    result = _sdo_read_raw(index, subindex, &temp, &len);
    if (result == RT_EOK && len >= 2)
    {
        *value = (rt_uint16_t)(temp & 0xFFFFU);
        return RT_EOK;
    }

    return -RT_ERROR;
}

static int _sdo_read_u32(rt_uint16_t index, rt_uint8_t subindex, rt_uint32_t *value)
{
    rt_uint8_t len;
    return _sdo_read_raw(index, subindex, value, &len);
}

static int _sdo_read_i32(rt_uint16_t index, rt_uint8_t subindex, rt_int32_t *value)
{
    rt_uint32_t temp;
    rt_uint8_t len;
    int result;

    if (value == RT_NULL)
    {
        return -RT_ERROR;
    }

    result = _sdo_read_raw(index, subindex, &temp, &len);
    if (result == RT_EOK && len >= 4)
    {
        *value = (rt_int32_t)temp;
        return RT_EOK;
    }

    return -RT_ERROR;
}

static rt_int16_t _clamp_speed(rt_int16_t rpm)
{
    rt_int16_t max_abs = ZLAC_SPEED_MAX_RPM;

    if (s_diff_state.max_rpm > 0 && s_diff_state.max_rpm < max_abs)
    {
        max_abs = s_diff_state.max_rpm;
    }

    if (rpm > max_abs) return max_abs;
    if (rpm < -max_abs) return -max_abs;
    return rpm;
}

static rt_int16_t _linear_to_rpm(rt_int32_t linear_mm_s, rt_uint16_t wheel_diameter_mm)
{
    rt_int64_t numerator;
    rt_int64_t denominator;
    rt_int64_t milli_rpm;

    if (wheel_diameter_mm == 0)
    {
        return 0;
    }

    numerator = (rt_int64_t)linear_mm_s * 60000LL * 1000LL;
    denominator = (rt_int64_t)ZLAC_PI_X1000 * (rt_int64_t)wheel_diameter_mm;
    milli_rpm = numerator / denominator;

    if (milli_rpm >= 0)
    {
        milli_rpm = (milli_rpm + 500LL) / 1000LL;
    }
    else
    {
        milli_rpm = (milli_rpm - 500LL) / 1000LL;
    }

    return _clamp_speed((rt_int16_t)milli_rpm);
}

static void _diff_update_from_twist(rt_int32_t linear_mm_s, rt_int32_t angular_mrad_s)
{
    rt_int32_t delta_mm_s;
    rt_int32_t left_mm_s;
    rt_int32_t right_mm_s;

    delta_mm_s = (rt_int32_t)(((rt_int64_t)angular_mrad_s * (rt_int64_t)s_diff_state.wheel_track_mm) / 2000LL);
    left_mm_s = linear_mm_s - delta_mm_s;
    right_mm_s = linear_mm_s + delta_mm_s;

    s_diff_state.target_left_rpm = _linear_to_rpm(left_mm_s, s_diff_state.wheel_diameter_mm);
    s_diff_state.target_right_rpm = _linear_to_rpm(right_mm_s, s_diff_state.wheel_diameter_mm);
    s_diff_state.dirty = RT_TRUE;
}

static const char *_nmt_state_name(rt_uint8_t state)
{
    switch (state)
    {
    case ZLAC_NMT_STATE_BOOTUP:      return "BOOTUP";
    case ZLAC_NMT_STATE_STOPPED:     return "STOPPED";
    case ZLAC_NMT_STATE_OPERATIONAL: return "OPERATIONAL";
    case ZLAC_NMT_STATE_PREOP:       return "PRE-OP";
    default:                         return "UNKNOWN";
    }
}

static const char *_mode_name(ZlacMotorMode_t mode)
{
    switch (mode)
    {
    case ZLAC_MODE_POSITION: return "POSITION";
    case ZLAC_MODE_SPEED:    return "SPEED";
    case ZLAC_MODE_TORQUE:   return "TORQUE";
    default:                 return "UNDEFINED";
    }
}

int canopen_motor_init(rt_uint8_t node_id)
{
    int result;

    if (node_id == 0 || node_id > 127)
    {
        return -RT_ERROR;
    }

    if (s_motor_obj.initialized)
    {
        if (s_motor_obj.node_id == node_id)
        {
            return RT_EOK;
        }

        canopen_motor_deinit();
    }

    result = _can_open_and_prepare(node_id);
    if (result != RT_EOK)
    {
        return result;
    }

    differential_drive_init();
    rt_thread_mdelay(30);
    canopen_motor_read_status();
    rt_kprintf("[ZLAC] init ok, dev=%s node=%d\n", ZLAC_CAN_DEV_NAME, node_id);
    return RT_EOK;
}

void canopen_motor_deinit(void)
{
    if (!s_motor_obj.initialized)
    {
        return;
    }

    _can_cleanup();
}

const CanopenMotorObject_t *canopen_motor_get_object(void)
{
    return &s_motor_obj;
}

void canopen_motor_set_rx_hook(CanopenMotorRxHook_t hook, void *user_data)
{
    s_rx_hook = hook;
    s_rx_hook_user_data = user_data;
}

int canopen_motor_nmt_command(rt_uint8_t command)
{
    rt_uint8_t data[2];

    if (!s_motor_obj.initialized)
    {
        return -RT_ERROR;
    }

    data[0] = command;
    data[1] = s_motor_obj.node_id;
    return _can_send_frame(ZLAC_COB_NMT, data, 2);
}

int canopen_motor_start_node(void)
{
    return canopen_motor_nmt_command(ZLAC_NMT_START);
}

int canopen_motor_stop_node(void)
{
    return canopen_motor_nmt_command(ZLAC_NMT_STOP);
}

int canopen_motor_enter_preop(void)
{
    return canopen_motor_nmt_command(ZLAC_NMT_PREOP);
}

int canopen_motor_set_mode(ZlacMotorMode_t mode)
{
    int result;

    result = _sdo_write_u8(ZLAC_OBJ_MODE_OF_OPERATION, 0x00, (rt_uint8_t)mode);
    if (result == RT_EOK)
    {
        s_motor_obj.feedback.mode = mode;
    }

    return result;
}

int canopen_motor_set_sync_mode(rt_bool_t sync_enable)
{
    return _sdo_write_u16(ZLAC_OBJ_SYNC_ASYNC_FLAG, 0x00, sync_enable ? 1U : 0U);
}

int canopen_motor_set_accel_decel(rt_uint32_t accel_ms, rt_uint32_t decel_ms)
{
    int result;

    result = _sdo_write_u32(ZLAC_OBJ_ACCEL_TIME, 0x01, accel_ms);
    if (result != RT_EOK) return result;
    result = _sdo_write_u32(ZLAC_OBJ_ACCEL_TIME, 0x02, accel_ms);
    if (result != RT_EOK) return result;
    result = _sdo_write_u32(ZLAC_OBJ_DECEL_TIME, 0x01, decel_ms);
    if (result != RT_EOK) return result;
    result = _sdo_write_u32(ZLAC_OBJ_DECEL_TIME, 0x02, decel_ms);
    if (result != RT_EOK) return result;
    return RT_EOK;
}

int canopen_motor_set_quickstop_decel(rt_uint32_t decel_ms)
{
    int result;

    result = _sdo_write_u32(ZLAC_OBJ_QUICKSTOP_DECEL, 0x01, decel_ms);
    if (result != RT_EOK) return result;
    return _sdo_write_u32(ZLAC_OBJ_QUICKSTOP_DECEL, 0x02, decel_ms);
}

int canopen_motor_enable(void)
{
    int result;

    result = _sdo_write_u16(ZLAC_OBJ_CONTROL_WORD, 0x00, ZLAC_CTRL_DISABLE_VOLTAGE);
    if (result != RT_EOK) return result;
    rt_thread_mdelay(5);

    result = _sdo_write_u16(ZLAC_OBJ_CONTROL_WORD, 0x00, ZLAC_CTRL_SHUTDOWN);
    if (result != RT_EOK) return result;
    rt_thread_mdelay(5);

    result = _sdo_write_u16(ZLAC_OBJ_CONTROL_WORD, 0x00, ZLAC_CTRL_SWITCH_ON);
    if (result != RT_EOK) return result;
    rt_thread_mdelay(5);

    result = _sdo_write_u16(ZLAC_OBJ_CONTROL_WORD, 0x00, ZLAC_CTRL_ENABLE_OPERATION);
    if (result != RT_EOK) return result;
    rt_thread_mdelay(5);

    return canopen_motor_read_status();
}

int canopen_motor_disable(void)
{
    return _sdo_write_u16(ZLAC_OBJ_CONTROL_WORD, 0x00, ZLAC_CTRL_DISABLE_VOLTAGE);
}

int canopen_motor_quick_stop(void)
{
    return _sdo_write_u16(ZLAC_OBJ_CONTROL_WORD, 0x00, ZLAC_CTRL_QUICK_STOP);
}

int canopen_motor_clear_fault(void)
{
    return _sdo_write_u16(ZLAC_OBJ_CONTROL_WORD, 0x00, ZLAC_CTRL_FAULT_RESET);
}

int canopen_motor_stop(void)
{
    s_motor_obj.feedback.target_left_rpm = 0;
    s_motor_obj.feedback.target_right_rpm = 0;
    return canopen_motor_set_velocity(0, 0);
}

int canopen_motor_prepare_speed_mode(rt_bool_t sync_enable, rt_uint32_t accel_ms, rt_uint32_t decel_ms)
{
    int result;

    result = canopen_motor_start_node();
    if (result != RT_EOK) return result;
    rt_thread_mdelay(20);

    canopen_motor_clear_fault();
    rt_thread_mdelay(20);

    result = canopen_motor_set_sync_mode(sync_enable);
    if (result != RT_EOK) return result;

    result = canopen_motor_set_mode(ZLAC_MODE_SPEED);
    if (result != RT_EOK) return result;

    result = canopen_motor_set_accel_decel(accel_ms, decel_ms);
    if (result != RT_EOK) return result;

    result = canopen_motor_enable();
    if (result != RT_EOK) return result;

    return canopen_motor_set_velocity(0, 0);
}

int canopen_motor_set_velocity(rt_int16_t left_rpm, rt_int16_t right_rpm)
{
    rt_uint32_t value;
    int result;

    left_rpm = _clamp_speed(left_rpm);
    right_rpm = _clamp_speed(right_rpm);
    value = (rt_uint16_t)left_rpm | ((rt_uint32_t)(rt_uint16_t)right_rpm << 16);

    result = _sdo_write_u32(ZLAC_OBJ_TARGET_VELOCITY, 0x03, value);
    if (result != RT_EOK)
    {
        return result;
    }

    s_motor_obj.feedback.target_left_rpm = left_rpm;
    s_motor_obj.feedback.target_right_rpm = right_rpm;
    return RT_EOK;
}

int canopen_motor_set_single_velocity(MotorSide_t side, rt_int16_t rpm)
{
    rpm = _clamp_speed(rpm);

    switch (side)
    {
    case MOTOR_SIDE_LEFT:
        return canopen_motor_set_velocity(rpm, s_motor_obj.feedback.target_right_rpm);
    case MOTOR_SIDE_RIGHT:
        return canopen_motor_set_velocity(s_motor_obj.feedback.target_left_rpm, rpm);
    case MOTOR_SIDE_BOTH:
        return canopen_motor_set_velocity(rpm, rpm);
    default:
        return -RT_ERROR;
    }
}

int canopen_motor_read_status(void)
{
    rt_uint32_t value;

    if (_sdo_read_u32(ZLAC_OBJ_STATUS_WORD, 0x00, &value) != RT_EOK)
    {
        return -RT_ERROR;
    }

    _update_wheel_status(&s_motor_obj.feedback.left, ZLAC_STATUS_LOW16(value));
    _update_wheel_status(&s_motor_obj.feedback.right, ZLAC_STATUS_HIGH16(value));
    return RT_EOK;
}

int canopen_motor_read_velocity(void)
{
    rt_uint32_t value;

    if (_sdo_read_u32(ZLAC_OBJ_ACTUAL_VELOCITY, 0x03, &value) != RT_EOK)
    {
        return -RT_ERROR;
    }

    s_motor_obj.feedback.left.actual_speed_rpm_x10 = (rt_int16_t)(value & 0xFFFFU);
    s_motor_obj.feedback.right.actual_speed_rpm_x10 = (rt_int16_t)((value >> 16) & 0xFFFFU);
    return RT_EOK;
}

int canopen_motor_read_position(void)
{
    rt_int32_t value;

    if (_sdo_read_i32(ZLAC_OBJ_ACTUAL_POSITION, 0x01, &value) != RT_EOK)
    {
        return -RT_ERROR;
    }
    s_motor_obj.feedback.left.position_counts = value;

    if (_sdo_read_i32(ZLAC_OBJ_ACTUAL_POSITION, 0x02, &value) != RT_EOK)
    {
        return -RT_ERROR;
    }
    s_motor_obj.feedback.right.position_counts = value;

    return RT_EOK;
}

int canopen_motor_read_fault(void)
{
    return _sdo_read_u32(ZLAC_OBJ_FAULT_CODE, 0x00, &s_motor_obj.feedback.fault_code);
}

int canopen_motor_read_temperature(void)
{
    rt_uint16_t value;

    if (_sdo_read_u16(ZLAC_OBJ_TEMP_INFO, 0x01, &value) != RT_EOK)
    {
        return -RT_ERROR;
    }
    s_motor_obj.feedback.motor_temp_left_x10 = (rt_int16_t)value;

    if (_sdo_read_u16(ZLAC_OBJ_TEMP_INFO, 0x02, &value) != RT_EOK)
    {
        return -RT_ERROR;
    }
    s_motor_obj.feedback.motor_temp_right_x10 = (rt_int16_t)value;

    if (_sdo_read_u16(ZLAC_OBJ_TEMP_INFO, 0x03, &value) != RT_EOK)
    {
        return -RT_ERROR;
    }
    s_motor_obj.feedback.driver_temp_x10 = (rt_int16_t)value;

    return RT_EOK;
}

int canopen_motor_read_bus_voltage(void)
{
    return _sdo_read_u16(ZLAC_OBJ_BUS_VOLTAGE, 0x00, &s_motor_obj.feedback.bus_voltage_x100);
}

int canopen_motor_refresh_feedback(void)
{
    int result;

    result = canopen_motor_read_status();
    if (result != RT_EOK) return result;

    result = canopen_motor_read_velocity();
    if (result != RT_EOK) return result;

    result = canopen_motor_read_position();
    if (result != RT_EOK) return result;

    result = canopen_motor_read_fault();
    if (result != RT_EOK) return result;

    result = canopen_motor_read_temperature();
    if (result != RT_EOK) return result;

    result = canopen_motor_read_bus_voltage();
    if (result != RT_EOK) return result;

    return RT_EOK;
}

int canopen_motor_service(void)
{
    int result;

    if (!s_motor_obj.initialized)
    {
        return -RT_ERROR;
    }

    result = canopen_motor_read_status();
    if (result != RT_EOK) return result;

    result = canopen_motor_read_velocity();
    if (result != RT_EOK) return result;

    s_service_divider++;
    if ((s_service_divider % 10U) == 0U)
    {
        canopen_motor_read_fault();
        canopen_motor_read_bus_voltage();
    }
    if ((s_service_divider % 20U) == 0U)
    {
        canopen_motor_read_temperature();
        canopen_motor_read_position();
    }

    return RT_EOK;
}

int canopen_motor_config_tpdo0_speed(rt_uint8_t transmission_type, rt_uint16_t event_time_ms)
{
    int result;

    if (transmission_type != 0xFEU && transmission_type != 0xFFU)
    {
        return -RT_ERROR;
    }

    result = _sdo_write_u8(ZLAC_OBJ_TPDO0_MAP, 0x00, 0x00);
    if (result != RT_EOK) return result;

    result = _sdo_write_u32(ZLAC_OBJ_TPDO0_MAP, 0x01, 0x606C0320UL);
    if (result != RT_EOK) return result;

    result = _sdo_write_u8(ZLAC_OBJ_TPDO0_COMM, 0x02, transmission_type);
    if (result != RT_EOK) return result;

    if (transmission_type == 0xFFU)
    {
        result = _sdo_write_u16(ZLAC_OBJ_TPDO0_COMM, 0x05, (rt_uint16_t)(event_time_ms * 2U));
        if (result != RT_EOK) return result;
    }

    result = _sdo_write_u8(ZLAC_OBJ_TPDO0_MAP, 0x00, 0x01);
    if (result == RT_EOK)
    {
        s_motor_obj.pdo_speed_feedback_enabled = RT_TRUE;
    }

    return result;
}

int canopen_motor_config_rpdo1_target_velocity(void)
{
    int result;

    result = _sdo_write_u8(ZLAC_OBJ_RPDO1_MAP, 0x00, 0x00);
    if (result != RT_EOK) return result;

    result = _sdo_write_u32(ZLAC_OBJ_RPDO1_MAP, 0x01, 0x60FF0120UL);
    if (result != RT_EOK) return result;

    result = _sdo_write_u32(ZLAC_OBJ_RPDO1_MAP, 0x02, 0x60FF0220UL);
    if (result != RT_EOK) return result;

    result = _sdo_write_u8(ZLAC_OBJ_RPDO1_MAP, 0x00, 0x02);
    if (result != RT_EOK) return result;

    result = _sdo_write_u8(ZLAC_OBJ_RPDO1_COMM, 0x02, 0xFF);
    if (result != RT_EOK) return result;

    return RT_EOK;
}

int canopen_motor_save_parameters(void)
{
    return _sdo_write_u16(ZLAC_OBJ_SAVE_PARAM, 0x00, ZLAC_SAVE_PARAM_MAGIC);
}

rt_bool_t canopen_motor_is_online(void)
{
    rt_tick_t now;

    if (!s_motor_obj.initialized || s_motor_obj.last_rx_tick == 0)
    {
        return RT_FALSE;
    }

    now = rt_tick_get();
    if ((now - s_motor_obj.last_rx_tick) > rt_tick_from_millisecond(ZLAC_OFFLINE_TIMEOUT_MS))
    {
        return RT_FALSE;
    }

    return RT_TRUE;
}

int differential_drive_init(void)
{
    rt_memset(&s_diff_state, 0, sizeof(s_diff_state));
    s_diff_state.wheel_track_mm = ZLAC_DEFAULT_WHEEL_TRACK_MM;
    s_diff_state.wheel_diameter_mm = ZLAC_DEFAULT_WHEEL_DIAMETER_MM;
    s_diff_state.max_rpm = ZLAC_DEFAULT_MAX_RPM;
    return RT_EOK;
}

void differential_drive_reset(void)
{
    differential_drive_init();
}

const DifferentialDriveState_t *differential_drive_get_state(void)
{
    return &s_diff_state;
}

int differential_drive_set_geometry(rt_uint16_t wheel_track_mm, rt_uint16_t wheel_diameter_mm)
{
    if (wheel_track_mm == 0 || wheel_diameter_mm == 0)
    {
        return -RT_ERROR;
    }

    s_diff_state.wheel_track_mm = wheel_track_mm;
    s_diff_state.wheel_diameter_mm = wheel_diameter_mm;
    return RT_EOK;
}

int differential_drive_set_max_rpm(rt_int16_t max_rpm)
{
    if (max_rpm <= 0)
    {
        return -RT_ERROR;
    }

    s_diff_state.max_rpm = max_rpm;
    return RT_EOK;
}

int differential_drive_set_twist(rt_int32_t linear_mm_s, rt_int32_t angular_mrad_s)
{
    s_diff_state.enabled = RT_TRUE;
    s_diff_state.direct_rpm_mode = RT_FALSE;
    s_diff_state.linear_mm_s = linear_mm_s;
    s_diff_state.angular_mrad_s = angular_mrad_s;
    _diff_update_from_twist(linear_mm_s, angular_mrad_s);
    return RT_EOK;
}

int differential_drive_set_twist_deg(rt_int32_t linear_mm_s, rt_int32_t angular_deg_s)
{
    rt_int32_t angular_mrad_s;

    angular_mrad_s = (rt_int32_t)(((rt_int64_t)angular_deg_s * 17453LL) / 1000LL);
    return differential_drive_set_twist(linear_mm_s, angular_mrad_s);
}

int differential_drive_set_wheel_rpm(rt_int16_t left_rpm, rt_int16_t right_rpm)
{
    s_diff_state.enabled = RT_TRUE;
    s_diff_state.direct_rpm_mode = RT_TRUE;
    s_diff_state.linear_mm_s = 0;
    s_diff_state.angular_mrad_s = 0;
    s_diff_state.target_left_rpm = _clamp_speed(left_rpm);
    s_diff_state.target_right_rpm = _clamp_speed(right_rpm);
    s_diff_state.dirty = RT_TRUE;
    return RT_EOK;
}

int differential_drive_stop(void)
{
    s_diff_state.enabled = RT_FALSE;
    s_diff_state.linear_mm_s = 0;
    s_diff_state.angular_mrad_s = 0;
    s_diff_state.target_left_rpm = 0;
    s_diff_state.target_right_rpm = 0;
    s_diff_state.dirty = RT_TRUE;
    return RT_EOK;
}

int differential_drive_apply(void)
{
    if (!s_diff_state.dirty)
    {
        return RT_EOK;
    }

    if (!s_motor_obj.initialized)
    {
        return -RT_ERROR;
    }

    if (canopen_motor_set_velocity(s_diff_state.target_left_rpm,
                                   s_diff_state.target_right_rpm) != RT_EOK)
    {
        return -RT_ERROR;
    }

    s_diff_state.dirty = RT_FALSE;
    return RT_EOK;
}

#ifdef RT_USING_MSH
#include <msh.h>

static void zlac_status(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("\n===== ZLAC8015D =====\n");
    rt_kprintf("init         : %s\n", s_motor_obj.initialized ? "YES" : "NO");
    rt_kprintf("online       : %s\n", canopen_motor_is_online() ? "YES" : "NO");
    rt_kprintf("node         : %d\n", s_motor_obj.node_id);
    rt_kprintf("nmt          : %s (0x%02X)\n", _nmt_state_name(s_motor_obj.nmt_state), s_motor_obj.nmt_state);
    rt_kprintf("mode         : %s (%d)\n", _mode_name(s_motor_obj.feedback.mode), s_motor_obj.feedback.mode);
    rt_kprintf("target rpm   : L=%d R=%d\n", s_motor_obj.feedback.target_left_rpm, s_motor_obj.feedback.target_right_rpm);
    rt_kprintf("actual rpm   : L=%d.%d R=%d.%d\n",
               s_motor_obj.feedback.left.actual_speed_rpm_x10 / 10,
               abs(s_motor_obj.feedback.left.actual_speed_rpm_x10 % 10),
               s_motor_obj.feedback.right.actual_speed_rpm_x10 / 10,
               abs(s_motor_obj.feedback.right.actual_speed_rpm_x10 % 10));
    rt_kprintf("status word  : L=0x%04X R=0x%04X\n",
               s_motor_obj.feedback.left.status_word,
               s_motor_obj.feedback.right.status_word);
    rt_kprintf("fault        : 0x%08lX\n", s_motor_obj.feedback.fault_code);
    rt_kprintf("bus voltage  : %d.%02d V\n",
               s_motor_obj.feedback.bus_voltage_x100 / 100,
               s_motor_obj.feedback.bus_voltage_x100 % 100);
    rt_kprintf("tx/rx        : %lu/%lu\n", s_motor_obj.tx_count, s_motor_obj.rx_count);
}

static void zlac_init_cmd(int argc, char **argv)
{
    rt_uint8_t node_id = ZLAC_NODE_ID_DEFAULT;

    if (argc >= 2)
    {
        node_id = (rt_uint8_t)atoi(argv[1]);
    }

    rt_kprintf("[ZLAC] init %s\n", canopen_motor_init(node_id) == RT_EOK ? "ok" : "failed");
}

static void zlac_up_cmd(int argc, char **argv)
{
    rt_uint32_t accel_ms = ZLAC_DEFAULT_ACCEL_MS;
    rt_uint32_t decel_ms = ZLAC_DEFAULT_DECEL_MS;

    if (argc >= 2) accel_ms = (rt_uint32_t)atoi(argv[1]);
    if (argc >= 3) decel_ms = (rt_uint32_t)atoi(argv[2]);

    rt_kprintf("[ZLAC] speed mode %s\n",
               canopen_motor_prepare_speed_mode(ZLAC_DEFAULT_SYNC_ENABLE, accel_ms, decel_ms) == RT_EOK ? "ok" : "failed");
}

static void zlac_vel_cmd(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("usage: zlac_vel <left_rpm> <right_rpm>\n");
        return;
    }

    rt_kprintf("[ZLAC] set velocity %s\n",
               canopen_motor_set_velocity((rt_int16_t)atoi(argv[1]), (rt_int16_t)atoi(argv[2])) == RT_EOK ? "ok" : "failed");
}

static void zlac_stop_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    rt_kprintf("[ZLAC] stop %s\n", canopen_motor_stop() == RT_EOK ? "ok" : "failed");
}

static void dd_cfg_cmd(int argc, char **argv)
{
    rt_uint16_t track = ZLAC_DEFAULT_WHEEL_TRACK_MM;
    rt_uint16_t diam = ZLAC_DEFAULT_WHEEL_DIAMETER_MM;
    rt_int16_t max_rpm = ZLAC_DEFAULT_MAX_RPM;

    if (argc >= 2) track = (rt_uint16_t)atoi(argv[1]);
    if (argc >= 3) diam = (rt_uint16_t)atoi(argv[2]);
    if (argc >= 4) max_rpm = (rt_int16_t)atoi(argv[3]);

    if (differential_drive_set_geometry(track, diam) != RT_EOK ||
        differential_drive_set_max_rpm(max_rpm) != RT_EOK)
    {
        rt_kprintf("[DD] config failed\n");
        return;
    }

    rt_kprintf("[DD] cfg ok, track=%dmm diam=%dmm max=%d\n", track, diam, max_rpm);
}

static void dd_twist_cmd(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("usage: dd_twist <linear_mm_s> <angular_deg_s>\n");
        return;
    }

    if (differential_drive_set_twist_deg((rt_int32_t)atoi(argv[1]), (rt_int32_t)atoi(argv[2])) != RT_EOK)
    {
        rt_kprintf("[DD] set twist failed\n");
        return;
    }

    rt_kprintf("[DD] apply %s\n", differential_drive_apply() == RT_EOK ? "ok" : "failed");
}

static void dd_rpm_cmd(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("usage: dd_rpm <left_rpm> <right_rpm>\n");
        return;
    }

    differential_drive_set_wheel_rpm((rt_int16_t)atoi(argv[1]), (rt_int16_t)atoi(argv[2]));
    rt_kprintf("[DD] apply %s\n", differential_drive_apply() == RT_EOK ? "ok" : "failed");
}

static void dd_stop_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    differential_drive_stop();
    rt_kprintf("[DD] stop %s\n", differential_drive_apply() == RT_EOK ? "ok" : "failed");
}

static void dd_status_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    rt_kprintf("\n===== Diff Drive =====\n");
    rt_kprintf("enabled      : %s\n", s_diff_state.enabled ? "YES" : "NO");
    rt_kprintf("mode         : %s\n", s_diff_state.direct_rpm_mode ? "DIRECT_RPM" : "TWIST");
    rt_kprintf("track/diam   : %d / %d mm\n", s_diff_state.wheel_track_mm, s_diff_state.wheel_diameter_mm);
    rt_kprintf("max rpm      : %d\n", s_diff_state.max_rpm);
    rt_kprintf("twist        : linear=%ld mm/s angular=%ld mrad/s\n", s_diff_state.linear_mm_s, s_diff_state.angular_mrad_s);
    rt_kprintf("target rpm   : L=%d R=%d\n", s_diff_state.target_left_rpm, s_diff_state.target_right_rpm);
}

MSH_CMD_EXPORT(zlac_init_cmd, init zlac canopen driver: zlac_init_cmd [node_id]);
MSH_CMD_EXPORT(zlac_up_cmd, start speed mode: zlac_up_cmd [accel_ms] [decel_ms]);
MSH_CMD_EXPORT(zlac_status, show zlac status);
MSH_CMD_EXPORT(zlac_vel_cmd, set wheel rpm: zlac_vel_cmd <left_rpm> <right_rpm>);
MSH_CMD_EXPORT(zlac_stop_cmd, stop zlac motor);
MSH_CMD_EXPORT(dd_cfg_cmd, set diff-drive geometry: dd_cfg_cmd [track_mm] [wheel_diam_mm] [max_rpm]);
MSH_CMD_EXPORT(dd_twist_cmd, run twist: dd_twist_cmd <linear_mm_s> <angular_deg_s>);
MSH_CMD_EXPORT(dd_rpm_cmd, run wheel rpm: dd_rpm_cmd <left_rpm> <right_rpm>);
MSH_CMD_EXPORT(dd_stop_cmd, stop differential drive);
MSH_CMD_EXPORT(dd_status_cmd, show differential drive status);
#endif
