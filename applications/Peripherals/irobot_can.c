/**
 * @file    irobot_can.c
 * @brief   iRobot CAN compatibility layer (backed by canopen_motor)
 * @version 2.1
 * @date    2026-03-29
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/dev_can.h>

#include "irobot_can.h"
#include "canopen_motor.h"
#include "../System/global_conf.h"

#ifdef ULOG_ENABLE
#include <ulog.h>
#else
#define LOG_E(...)
#define LOG_W(...)
#define LOG_I(...)
#define LOG_D(...)
#endif

#define IROBOT_CAN_RX_MQ_DEPTH              32

static rt_bool_t s_can_connected = RT_FALSE;
static rt_mq_t s_can_rx_mq = RT_NULL;

static rt_err_t _raw_send(rt_uint32_t id, const rt_uint8_t *data, rt_uint8_t len)
{
    struct rt_can_msg msg;
    const CanopenMotorObject_t *obj = canopen_motor_get_object();

    if (obj == RT_NULL || obj->can_dev == RT_NULL || data == RT_NULL || len > 8)
    {
        return -RT_ERROR;
    }

    rt_memset(&msg, 0, sizeof(msg));
    msg.id = id;
    msg.ide = RT_CAN_STDID;
    msg.rtr = RT_CAN_DTR;
    msg.len = len;
    msg.hdr_index = -1;
    rt_memcpy(msg.data, data, len);

    if (rt_device_write(obj->can_dev, 0, &msg, sizeof(msg)) != sizeof(msg))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static void _compat_rx_hook(const struct rt_can_msg *msg, void *user_data)
{
    struct can_msg out;
    rt_err_t result;

    RT_UNUSED(user_data);

    if (msg == RT_NULL || s_can_rx_mq == RT_NULL)
    {
        return;
    }

    if (msg->ide != RT_CAN_STDID || msg->len > 8)
    {
        return;
    }

    out.id = msg->id;
    out.dlc = msg->len;
    rt_memcpy(out.data, msg->data, msg->len);

    result = rt_mq_send(s_can_rx_mq, &out, sizeof(out));
    if (result == -RT_EFULL)
    {
        struct can_msg drop;
        (void)rt_mq_recv(s_can_rx_mq, &drop, sizeof(drop), 0);
        (void)rt_mq_send(s_can_rx_mq, &out, sizeof(out));
    }
}

static rt_err_t _compat_queue_create(void)
{
    if (s_can_rx_mq != RT_NULL)
    {
        rt_mq_delete(s_can_rx_mq);
        s_can_rx_mq = RT_NULL;
    }

    s_can_rx_mq = rt_mq_create("icanrx",
                               sizeof(struct can_msg),
                               IROBOT_CAN_RX_MQ_DEPTH,
                               RT_IPC_FLAG_FIFO);
    if (s_can_rx_mq == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    return RT_EOK;
}

rt_err_t irobot_can_init(void)
{
    rt_err_t result;

    result = canopen_motor_init(ZLAC_NODE_ID_DEFAULT);
    if (result != RT_EOK)
    {
        LOG_E("canopen_motor_init failed: %d", result);
        s_can_connected = RT_FALSE;
        return result;
    }

    result = _compat_queue_create();
    if (result != RT_EOK)
    {
        s_can_connected = RT_FALSE;
        return result;
    }

    canopen_motor_set_rx_hook(_compat_rx_hook, RT_NULL);

    result = canopen_motor_prepare_speed_mode(ZLAC_DEFAULT_SYNC_ENABLE,
                                              ZLAC_DEFAULT_ACCEL_MS,
                                              ZLAC_DEFAULT_DECEL_MS);
    if (result != RT_EOK)
    {
        LOG_E("canopen_motor_prepare_speed_mode failed: %d", result);
        canopen_motor_set_rx_hook(RT_NULL, RT_NULL);
        s_can_connected = RT_FALSE;
        return result;
    }

    differential_drive_init();
    s_can_connected = RT_TRUE;
    LOG_I("irobot_can compatibility layer ready");
    return RT_EOK;
}

rt_err_t irobot_can_send_msg(rt_uint32_t id, rt_uint8_t *data, rt_uint8_t len)
{
    return _raw_send(id, data, len);
}

rt_err_t irobot_can_send_data(rt_uint32_t can_id, rt_uint8_t dlc, rt_uint8_t *data)
{
    return _raw_send(can_id, data, dlc);
}

rt_err_t irobot_can_recv_msg(struct can_msg *msg, rt_uint32_t timeout)
{
    if (msg == RT_NULL || s_can_rx_mq == RT_NULL)
    {
        return -RT_ERROR;
    }

    return rt_mq_recv(s_can_rx_mq, msg, sizeof(*msg), timeout);
}

rt_err_t irobot_can_control(rt_uint8_t cmd, void *arg)
{
    rt_uint8_t node_id = ZLAC_NODE_ID_DEFAULT;

    if (arg != RT_NULL)
    {
        node_id = *(rt_uint8_t *)arg;
        if (node_id == 0 || node_id > 127)
        {
            node_id = ZLAC_NODE_ID_DEFAULT;
        }
    }

    switch (cmd)
    {
    case IROBOT_CAN_CMD_START:
        if (canopen_motor_init(node_id) != RT_EOK)
        {
            s_can_connected = RT_FALSE;
            return -RT_ERROR;
        }
        if (_compat_queue_create() != RT_EOK)
        {
            s_can_connected = RT_FALSE;
            return -RT_ERROR;
        }
        canopen_motor_set_rx_hook(_compat_rx_hook, RT_NULL);
        if (canopen_motor_prepare_speed_mode(ZLAC_DEFAULT_SYNC_ENABLE,
                                             ZLAC_DEFAULT_ACCEL_MS,
                                             ZLAC_DEFAULT_DECEL_MS) != RT_EOK)
        {
            s_can_connected = RT_FALSE;
            return -RT_ERROR;
        }
        differential_drive_init();
        s_can_connected = RT_TRUE;
        return RT_EOK;

    case IROBOT_CAN_CMD_STOP:
        differential_drive_stop();
        differential_drive_apply();
        canopen_motor_stop();
        canopen_motor_disable();
        s_can_connected = RT_FALSE;
        return RT_EOK;

    case IROBOT_CAN_CMD_RESET:
        canopen_motor_set_rx_hook(RT_NULL, RT_NULL);
        canopen_motor_deinit();
        rt_thread_mdelay(50);
        return irobot_can_control(IROBOT_CAN_CMD_START, &node_id);

    default:
        return -RT_ERROR;
    }
}

rt_bool_t irobot_can_get_state(void)
{
    return (s_can_connected && canopen_motor_is_online()) ? RT_TRUE : RT_FALSE;
}

void irobot_can_disconnect(void)
{
    differential_drive_stop();
    differential_drive_apply();
    canopen_motor_stop();
    canopen_motor_set_rx_hook(RT_NULL, RT_NULL);
    canopen_motor_deinit();

    if (s_can_rx_mq != RT_NULL)
    {
        rt_mq_delete(s_can_rx_mq);
        s_can_rx_mq = RT_NULL;
    }

    s_can_connected = RT_FALSE;
}

#ifdef RT_USING_MSH
#include <msh.h>

static void irobot_can_status_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("\n===== irobot_can =====\n");
    rt_kprintf("connected : %s\n", s_can_connected ? "YES" : "NO");
    rt_kprintf("online    : %s\n", canopen_motor_is_online() ? "YES" : "NO");
    rt_kprintf("rx queue  : %s\n", s_can_rx_mq ? "READY" : "NULL");
}

MSH_CMD_EXPORT(irobot_can_status_cmd, show compatibility-layer can status);
#endif
