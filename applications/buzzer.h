/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BUZZER_H__
#define BUZZER_H__

#include <rtthread.h>

/**
 * @brief 蜂鸣器引脚定义 (根据实际硬件调整)
 * 有源蜂鸣器通常使用 GPIO 输出控制
 */
#ifndef BUZZER_PIN
#define BUZZER_PIN    GET_PIN(C, 6)   // 默认 PC6，可根据 PCB 调整
#endif

/**
 * @brief 音调时间参数 (毫秒)
 */
#define TONE_SHORT        50      /* 短音时长 */
#define TONE_LONG         200     /* 长音时长 */
#define PAUSE_SHORT       50      /* 短间隔 */
#define PAUSE_LONG        200     /* 长间隔 */

/**
 * @brief 蜂鸣器初始化函数
 */
void buzzer_init(void);

/**
 * @brief 开启蜂鸣器
 */
static __inline void buzzer_on(void)
{
    rt_pin_write(BUZZER_PIN, PIN_HIGH);
}

/**
 * @brief 关闭蜂鸣器
 */
static __inline void buzzer_off(void)
{
    rt_pin_write(BUZZER_PIN, PIN_LOW);
}

/* ========== 音效函数 ========== */

/**
 * @brief 开机提示音 - 短 - 短 - 稍长序列
 * @note 设备启动完成时调用
 */
void buzz_poweron(void);

/**
 * @brief 关机提示音 - 单声中等长度
 * @note 设备正常关机前调用
 */
void buzz_shutdown(void);

/**
 * @brief 任务开始提示 - 三声快速短音
 * @note 新任务启动时调用
 */
void buzz_task_start(void);

/**
 * @brief 任务完成提示 - 两声长音（愉快）
 * @note 任务成功执行后调用
 */
void buzz_task_done(void);

/**
 * @brief 任务失败提示 - 连续三声较长音（沮丧）
 * @note 任务执行失败时调用
 */
void buzz_task_fail(void);

/**
 * @brief 低电量警告 - 间歇性短 - 短 - 长
 * @note 电池电量低于 20% 循环播放
 * @return 0 成功，-1 需停止报警
 */
int buzz_low_battery(void);

/**
 * @brief 电量充满提示 - 连续五声短音
 * @note 电池充电完成时调用
 */
void buzz_charged(void);

/**
 * @brief 严重故障警报 - 持续长音循环
 * @note 硬件故障、传感器异常等紧急情况
 * @warning 需要人工干预才能停止
 */
void buzz_critical_error(void);

/**
 * @brief 一般故障提示 - 长 - 短 - 短×3
 * @note 非致命错误（通信超时等）
 */
void buzz_general_error(void);

/**
 * @brief 按键确认音 - 清脆短音
 * @note 物理按键或触摸被按下时调用
 */
void buzz_keypress(void);

/**
 * @brief 普通提示音 - 单声短音
 * @note 一般提示信息
 */
void buzz_info(void);

#endif /* BUZZER_H__ */
