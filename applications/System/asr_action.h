/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 语音识别动作线程头文件
 *
 * 提供语音模块初始化、命令轮询及动作执行的对外接口
 */

#ifndef __ASR_ACTION_H__
#define __ASR_ACTION_H__

#include <rtthread.h>

/**
 * @brief 启动语音识别动作线程
 *
 * 该函数会创建并启动一个独立线程，完成：
 *   - 初始化 WonderEcho Pro 语音模块（I2C2）
 *   - 轮询识别结果并执行对应动作
 *   - 支持主动播报系统状态（可通过其他模块调用 asr_speak）
 *
 * @return RT_EOK 成功；其他值表示失败
 */
int asr_action_init(void);

void asr_action_speak_state(uint8_t talk_id);
void asr_action_set_single_mode(rt_bool_t enable);
/**
 * @brief 获取当前是否为单机模式
 * @return RT_TRUE 单机模式，RT_FALSE 正常模式
 */
rt_bool_t asr_action_is_single_mode(void);
#endif /* __ASR_ACTION_H__ */

