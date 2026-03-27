/**
 * @file    irobot_can.c
 * @brief   CAN通讯驱动（iRobot专用封装）
 * @version 1.0
 * @date    2026-03-26
 * @author  马哥
 *
 * @note
 * - 封装RT-Thread的CAN设备驱动
 * - 支持ZLAC8015D驱动器CANopen协议
 * - 支持ACSL-6210-50RE隔离电路和TJA1050收发器
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "global_conf.h"

#ifdef ULOG_ENABLE
#include <ulog.h>
#else
#define LOG_E(...)
#define LOG_W(...)
#define LOG_I(...)
#define LOG_D(...)
#endif

/* CAN设备句柄 */
static struct rt_device *can_dev = RT_NULL;

/* CAN通讯状态 */
static rt_bool_t can_connected = RT_FALSE;

/* CAN接收消息队列 */
static rt_queue_t can_rx_queue = RT_NULL;

/*============================================================================*/
/* CAN消息结构定义 */
/*============================================================================*/

/**
 * @brief CAN消息结构体
 */
struct can_msg {
    rt_uint32_t id;
    rt_uint8_t  dlc;
    rt_uint8_t  data[8];
};

/*============================================================================*/
/* CAN驱动函数 */
/*============================================================================*/

/**
 * @brief CAN初始化
 * @return RT_EOK 初始化成功
 *         RT_ERROR 初始化失败
 */
rt_err_t irobot_can_init(void)
{
    rt_err_t ret = RT_EOK;

    LOG_I("CAN driver initializing...");

    /* 查找CAN设备 */
#if CAN_ENABLE
    can_dev = rt_device_find("can1");
    if (can_dev == RT_NULL) {
        LOG_E("CAN device not found!");
        return RT_ERROR;
    }

    /* 打开CAN设备 */
    ret = rt_device_open(can_dev, RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK) {
        LOG_E("Failed to open CAN device: %d", ret);
        return ret;
    }

    /* 初始化接收队列 */
    can_rx_queue = rt_queue_create("can_rx_q", 20 * sizeof(struct can_msg),
                                   RT_IPC_FLAG_FIFO);
    if (can_rx_queue == RT_NULL) {
        LOG_E("Failed to create CAN RX queue");
        rt_device_close(can_dev);
        return RT_ERROR;
    }

    /* 配置CAN设备 */
    struct can_configure cfg;
    cfg.mode = CAN_MODE_NORMAL;
    cfg.timing = 0;
    cfg.max_recv_buf_size = 20;
    cfg.max_send_buf_size = 20;
    cfg.priv = RT_NULL;

    ret = rt_device_control(can_dev, RT_DEVICE_CTRL_CONFIG, &cfg);
    if (ret != RT_EOK) {
        LOG_E("Failed to configure CAN device: %d", ret);
        rt_queue_destroy(can_rx_queue);
        rt_device_close(can_dev);
        return ret;
    }

    /* 注册CAN接收回调 */
    rt_device_set_rx_indicate(can_dev, can_rx_callback);

    can_connected = RT_TRUE;
    LOG_I("CAN driver initialized successfully");
#else
    LOG_W("CAN is disabled");
#endif

    return ret;
}

/**
 * @brief CAN接收回调函数
 * @param dev CAN设备
 * @param size 接收数据长度
 */
static rt_err_t can_rx_callback(struct rt_device *dev, rt_size_t size)
{
    struct can_msg msg;
    rt_err_t ret;

    if (can_dev == RT_NULL || can_rx_queue == RT_NULL) {
        return RT_ERROR;
    }

    /* 读取CAN消息 */
    ret = rt_device_read(can_dev, 0, &msg, sizeof(msg));
    if (ret != sizeof(msg)) {
        LOG_E("Failed to read CAN message");
        return RT_ERROR;
    }

    /* 将消息放入队列 */
    ret = rt_queue_send(can_rx_queue, &msg, RT_WAITING_FOREVER);
    if (ret != RT_EOK) {
        LOG_E("Failed to send CAN message to queue");
        return ret;
    }

    return RT_EOK;
}

/**
 * @brief CAN发送消息
 * @param id CAN ID
 * @param data 数据指针
 * @param len 数据长度
 * @return RT_EOK 发送成功
 *         RT_ERROR 发送失败
 */
rt_err_t irobot_can_send_msg(rt_uint32_t id, rt_uint8_t *data, rt_uint8_t len)
{
    struct can_msg msg;
    rt_err_t ret;

    if (can_dev == RT_NULL) {
        LOG_E("CAN device not initialized");
        return RT_ERROR;
    }

    if (len > 8) {
        LOG_E("CAN message length too long: %d", len);
        return RT_ERROR;
    }

    /* 填充CAN消息 */
    msg.id = id;
    msg.dlc = len;
    rt_memcpy(msg.data, data, len);

    /* 发送CAN消息 */
    ret = rt_device_write(can_dev, 0, &msg, sizeof(msg));
    if (ret != sizeof(msg)) {
        LOG_E("Failed to send CAN message");
        return RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief CAN发送数据帧
 * @param can_id CAN ID
 * @param dlc 数据长度
 * @param data 数据指针
 * @return RT_EOK 发送成功
 *         RT_ERROR 发送失败
 */
rt_err_t irobot_can_send_data(rt_uint32_t can_id, rt_uint8_t dlc, rt_uint8_t *data)
{
    return irobot_can_send_msg(can_id, data, dlc);
}

/**
 * @brief CAN接收消息
 * @param msg 消息指针
 * @param timeout 超时时间（单位：tick）
 * @return RT_EOK 接收成功
 *         RT_ETIMEOUT 超时
 *         RT_ERROR 接收失败
 */
rt_err_t irobot_can_recv_msg(struct can_msg *msg, rt_uint32_t timeout)
{
    rt_err_t ret;

    if (can_rx_queue == RT_NULL || msg == RT_NULL) {
        return RT_ERROR;
    }

    /* 从队列中接收消息 */
    ret = rt_queue_recv(can_rx_queue, msg, timeout);
    if (ret != RT_EOK) {
        return ret;
    }

    return RT_EOK;
}

/**
 * @brief CAN设备控制
 * @param cmd 控制命令
 * @param arg 控制参数
 * @return RT_EOK 控制成功
 *         RT_ERROR 控制失败
 */
rt_err_t irobot_can_control(rt_uint8_t cmd, void *arg)
{
    rt_err_t ret = RT_EOK;

    if (can_dev == RT_NULL) {
        return RT_ERROR;
    }

    switch (cmd) {
        case 0x01:  /* 启动CAN */
            ret = rt_device_control(can_dev, RT_DEVICE_CTRL_CONFIG, NULL);
            break;

        case 0x02:  /* 停止CAN */
            ret = rt_device_control(can_dev, RT_DEVICE_CTRL_SUSPEND, NULL);
            break;

        case 0x03:  /* 复位CAN */
            LOG_W("CAN reset not supported");
            break;

        default:
            LOG_E("Unknown CAN command: %d", cmd);
            ret = RT_ERROR;
            break;
    }

    return ret;
}

/**
 * @brief CAN读取状态
 * @return RT_TRUE 已连接
 *         RT_FALSE 未连接
 */
rt_bool_t irobot_can_get_state(void)
{
    return can_connected;
}

/**
 * @brief CAN断开连接
 */
void irobot_can_disconnect(void)
{
    if (can_dev != RT_NULL) {
        rt_device_close(can_dev);
    }

    if (can_rx_queue != RT_NULL) {
        rt_queue_destroy(can_rx_queue);
        can_rx_queue = RT_NULL;
    }

    can_connected = RT_FALSE;
    LOG_I("CAN disconnected");
}

/*============================================================================*/
/* MSH命令实现 */
/*============================================================================*/

#ifdef MSH_ENABLE

/* 打印CAN状态 */
static int cmd_can_status(int argc, char **argv)
{
    rt_kprintf("========================================\n");
    rt_kprintf("CAN Status\n");
    rt_kprintf("========================================\n");
    rt_kprintf("Device: %s\n", can_dev ? "Open" : "Closed");
    rt_kprintf("Connected: %s\n", can_connected ? "Yes" : "No");
    rt_kprintf("RX Queue: %s\n", can_rx_queue ? "Ready" : "Not created");
    rt_kprintf("========================================\n");

    return 0;
}
MSH_CMD_EXPORT(cmd_can_status, CAN status);

#endif /* MSH_ENABLE */
