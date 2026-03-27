/**
 * @file    irobot_can.h
 * @brief   CAN通讯驱动头文件
 * @version 1.0
 * @date    2026-03-26
 * @author  马哥
 */

#ifndef __IROBOT_CAN_H__
#define __IROBOT_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtdef.h>

/*============================================================================*/
/* CAN命令定义 */
/*============================================================================*/

#define IROBOT_CAN_CMD_START                 0x01
#define IROBOT_CAN_CMD_STOP                  0x02
#define IROBOT_CAN_CMD_RESET                 0x03

/*============================================================================*/
/* CAN ID定义 */
/*============================================================================*/

/* ZLAC8015D 驱动器CANopen对象字典ID */
#define IROBOT_CAN_NODE_ID                   1      // CANopen节点ID

/* COB-ID定义 */
#define IROBOT_CAN_COB_ID_HEARTBEAT          0x700  // 心跳消息
#define IROBOT_CAN_COB_ID_NODE_STATUS        0x700  // 节点状态
#define IROBOT_CAN_COB_ID_EMERGENCY          0x080  // 紧急错误消息
#define IROBOT_CAN_COB_ID_TPDO1              0x180  // TPDO1
#define IROBOT_CAN_COB_ID_RPDO1              0x200  // RPDO1
#define IROBOT_CAN_COB_ID_TPDO2              0x280  // TPDO2
#define IROBOT_CAN_COB_ID_RPDO2              0x300  // RPDO2
#define IROBOT_CAN_COB_ID_TPDO3              0x380  // TPDO3
#define IROBOT_CAN_COB_ID_RPDO3              0x400  // RPDO3
#define IROBOT_CAN_COB_ID_TPDO4              0x480  // TPDO4
#define IROBOT_CAN_COB_ID_RPDO4              0x500  // RPDO4
#define IROBOT_CAN_COB_ID_SYNC               0x080  // 同步消息
#define IROBOT_CAN_COB_ID_TIME               0x080  // 时间戳

/* CANopen NMT命令 */
#define IROBOT_CAN_NMT_START_UP                  0x01
#define IROBOT_CAN_NMT_STOP_UP                   0x02
#define IROBOT_CAN_NMT_ENTER_OPERATIONAL          0x04
#define IROBOT_CAN_NMT_ENTER_PRE_OPERATIONAL      0x05
#define IROBOT_CAN_NMT_ENTER_STOPPED              0x06
#define IROBOT_CAN_NMT_RESET_NODE                 0x10
#define IROBOT_CAN_NMT_RESET_COMMUNICATION        0x11

/* CANopen NMT状态 */
#define IROBOT_CAN_NMT_STATE_UNKNOWN             0x00
#define IROBOT_CAN_NMT_STATE_INITIALIZING         0x01
#define IROBOT_CAN_NMT_STATE_STOPPED              0x02
#define IROBOT_CAN_NMT_STATE_OPERATIONAL          0x04
#define IROBOT_CAN_NMT_STATE_PRE_OPERATIONAL      0x05
#define IROBOT_CAN_NMT_STATE_RESET_COMM          0x06
#define IROBOT_CAN_NMT_STATE_RESET_NODE           0x07

/* CANopen心跳时间 */
#define IROBOT_CAN_HEARTBEAT_TIME_MS              1000   // 心跳时间1秒

/*============================================================================*/
/* CAN数据帧定义 */
/*============================================================================*/

/* TPDO1 - 驱动器状态 */
#define IROBOT_CAN_TPDO1_ID                  0x180 + IROBOT_CAN_NODE_ID
#define IROBOT_CAN_TPDO1_LEN                  8
#define IROBOT_CAN_TPDO1_STATUS_WORD          0
#define IROBOT_CAN_TPDO1_TARGET_SPEED         1
#define IROBOT_CAN_TPDO1_CURRENT_SPEED        2
#define IROBOT_CAN_TPDO1_CURRENT              3
#define IROBOT_CAN_TPDO1_VOLTAGE              4
#define IROBOT_CAN_TPDO1_TEMPERATURE          5

/* RPDO1 - 目标速度 */
#define IROBOT_CAN_RPDO1_ID                  0x200 + IROBOT_CAN_NODE_ID
#define IROBOT_CAN_RPDO1_LEN                  8
#define IROBOT_CAN_RPDO1_ENABLE               0
#define IROBOT_CAN_RPDO1_SPEED                1
#define IROBOT_CAN_RPDO1_DIR                  2
#define IROBOT_CAN_RPDO1_ESTOP                3

/* NMT命令帧 */
#define IROBOT_CAN_NMT_CMD_ID                 0x000  // 广播ID
#define IROBOT_CAN_NMT_LENGTH                 2

/* 心跳帧 */
#define IROBOT_CAN_HEARTBEAT_ID               0x700 + IROBOT_CAN_NODE_ID
#define IROBOT_CAN_HEARTBEAT_LENGTH           1

/*============================================================================*/
/* 函数声明 */
/*============================================================================*/

/**
 * @brief CAN初始化
 * @return RT_EOK 初始化成功
 *         RT_ERROR 初始化失败
 */
rt_err_t irobot_can_init(void);

/**
 * @brief CAN发送消息
 * @param id CAN ID
 * @param data 数据指针
 * @param len 数据长度
 * @return RT_EOK 发送成功
 *         RT_ERROR 发送失败
 */
rt_err_t irobot_can_send_msg(rt_uint32_t id, rt_uint8_t *data, rt_uint8_t len);

/**
 * @brief CAN发送数据帧
 * @param can_id CAN ID
 * @param dlc 数据长度
 * @param data 数据指针
 * @return RT_EOK 发送成功
 *         RT_ERROR 发送失败
 */
rt_err_t irobot_can_send_data(rt_uint32_t can_id, rt_uint8_t dlc, rt_uint8_t *data);

/**
 * @brief CAN接收消息
 * @param msg 消息指针
 * @param timeout 超时时间（单位：tick）
 * @return RT_EOK 接收成功
 *         RT_ETIMEOUT 超时
 *         RT_ERROR 接收失败
 */
rt_err_t irobot_can_recv_msg(struct can_msg *msg, rt_uint32_t timeout);

/**
 * @brief CAN设备控制
 * @param cmd 控制命令
 * @param arg 控制参数
 * @return RT_EOK 控制成功
 *         RT_ERROR 控制失败
 */
rt_err_t irobot_can_control(rt_uint8_t cmd, void *arg);

/**
 * @brief CAN读取状态
 * @return RT_TRUE 已连接
 *         RT_FALSE 未连接
 */
rt_bool_t irobot_can_get_state(void);

/**
 * @brief CAN断开连接
 */
void irobot_can_disconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* __IROBOT_CAN_H__ */
