/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 小车运动控制模块（基于 ZLAC8015D 驱动器）
 */

#ifndef __CAR_ACTION_H__
#define __CAR_ACTION_H__

#include <rtthread.h>

/* 命令类型（邮箱消息低8位为命令，高24位为参数）*/
typedef enum {
		CAR_CMD__NONE = 0,
		CAR_CMD_CLEAR_FAULT,			 /* 清除故障代码 */
    CAR_CMD_VEL_MODE,          /* 切换到速度模式 */
    CAR_CMD_POS_MODE_ABS,      /* 切换到绝对位置模式 */
    CAR_CMD_POS_MODE_REL,      /* 切换到相对位置模式 */
    CAR_CMD_SET_VEL,           /* 设置速度：参数 = (left_rpm << 16) | (right_rpm & 0xFFFF) */
    CAR_CMD_SET_POS,           /* 设置位置：参数 = (left_pulses << 16) | (right_pulses & 0xFFFF) */
    CAR_CMD_STOP,              /* 停止/结束工作（解轴+抱闸）*/
    CAR_CMD_ENABLE,            /* 使能电机 */
    CAR_CMD_DISABLE,           /* 禁用电机（停止但锁轴）*/
    CAR_CMD_FREE,              /* 完全解轴 */
    CAR_CMD_BRAKE_ON,          /* 抱闸锁紧 */
    CAR_CMD_BRAKE_OFF,         /* 释放抱闸 */
    CAR_CMD_LIGHT_ON,          /* 大灯开 */
    CAR_CMD_LIGHT_OFF,         /* 大灯关 */
    CAR_CMD_CHARGER_ON,        /* 充电器开 */
    CAR_CMD_CHARGER_OFF,       /* 充电器关 */
    CAR_CMD_ZERO_POS,          /* 回到原点（绝对位置模式）*/
    CAR_CMD_QUICK_STOP,        /* 安全停车（因悬崖）或者紧急停*/
		CAR_CMD_MOVE_START,				 /* 速度模式 松闸 电机使能*/
} CarCmd_t;

#ifndef CAR_PACK_VEL
#define CAR_PACK_VEL(left, right)  ( ((uint32_t)( (uint16_t)(left) ) << 16) | (uint32_t)( (uint16_t)(right) ) )
#endif

/* 初始化小车动作模块 */
void car_action_init(void);

/* 发送命令到小车动作线程（非阻塞）*/
void car_action_send_cmd(CarCmd_t cmd, uint32_t param);

void car_action_set_position_global(int32_t left, int32_t right);

#endif /* __CAR_ACTION_H__ */

