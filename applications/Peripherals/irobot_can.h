/**
 * @file    irobot_can.h
 * @brief   iRobot CAN compatibility layer (backed by canopen_motor)
 * @version 2.0
 * @date    2026-03-29
 */

#ifndef __IROBOT_CAN_H__
#define __IROBOT_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtdef.h>

#define IROBOT_CAN_CMD_START                 0x01
#define IROBOT_CAN_CMD_STOP                  0x02
#define IROBOT_CAN_CMD_RESET                 0x03

#define IROBOT_CAN_NODE_ID                   1
#define IROBOT_CAN_COB_ID_EMERGENCY(node)    (0x080 + (node))
#define IROBOT_CAN_COB_ID_TPDO1(node)        (0x180 + (node))
#define IROBOT_CAN_COB_ID_RPDO1(node)        (0x200 + (node))
#define IROBOT_CAN_COB_ID_TPDO2(node)        (0x280 + (node))
#define IROBOT_CAN_COB_ID_RPDO2(node)        (0x300 + (node))
#define IROBOT_CAN_COB_ID_TPDO3(node)        (0x380 + (node))
#define IROBOT_CAN_COB_ID_RPDO3(node)        (0x400 + (node))
#define IROBOT_CAN_COB_ID_TPDO4(node)        (0x480 + (node))
#define IROBOT_CAN_COB_ID_RPDO4(node)        (0x500 + (node))
#define IROBOT_CAN_COB_ID_HEARTBEAT(node)    (0x700 + (node))

struct can_msg
{
    rt_uint32_t id;
    rt_uint8_t  dlc;
    rt_uint8_t  data[8];
};

rt_err_t irobot_can_init(void);
rt_err_t irobot_can_send_msg(rt_uint32_t id, rt_uint8_t *data, rt_uint8_t len);
rt_err_t irobot_can_send_data(rt_uint32_t can_id, rt_uint8_t dlc, rt_uint8_t *data);
rt_err_t irobot_can_recv_msg(struct can_msg *msg, rt_uint32_t timeout);
rt_err_t irobot_can_control(rt_uint8_t cmd, void *arg);
rt_bool_t irobot_can_get_state(void);
void irobot_can_disconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* __IROBOT_CAN_H__ */
