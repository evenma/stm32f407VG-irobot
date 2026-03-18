/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PERIPHERALS_LED_H__
#define PERIPHERALS_LED_H__

#include <rtthread.h>

/**
 * @brief LED 指示灯数量定义
 * 8 个指示灯：LED0~LED5 本地状态 + LED6~LED7 上位机控制
 */
#define LED_TOTAL_COUNT         8

/**
 * @brief LED 配对索引定义
 * 0 = LED0 (工作绿灯), 1 = LED1 (故障红灯)
 * 2 = LED2 (满电绿灯), 3 = LED3 (低电红灯)
 * 4 = LED4 (满水绿灯), 5 = LED5 (缺水红灯)
 * 6 = LED6 (导航绿灯←上位机), 7 = LED7 (异常红灯←上位机)
 */
#define LED_IDX_WORKING_GREEN   0
#define LED_IDX_FAULT_RED       1
#define LED_IDX_FULL_BAT_GREEN  2
#define LED_IDX_LOW_BAT_RED     3
#define LED_IDX_FULL_WATER_GREEN    4
#define LED_IDX_LOW_WATER_RED   5
#define LED_IDX_NAV_GREEN       6    /* ← 上位机 LED7 */
#define LED_IDX_ERROR_RED       7    /* ← 上位机 LED8 */


/**
 * @brief LED 颜色状态枚举
 */
typedef enum
{
    LED_COLOR_OFF = 0,      // 熄灭
    LED_COLOR_ON,           // 亮 (单色 LED 只有一种亮度)
    LED_COLOR_FLASH_FAST,   // 快速闪烁 (100ms)
    LED_COLOR_FLASH_SLOW,   // 慢速闪烁 (500ms)
} LedColorState_t;

#define LED_COLOR_GREEN_ON LED_COLOR_ON
#define LED_COLOR_RED_ON LED_COLOR_ON

/**
 * @brief LED 对象结构体
 */
typedef struct
{
    rt_uint8_t id;                // LED ID (0-7)
    LedColorState_t color_state;  // 当前颜色状态
    rt_bool_t enabled;            // 是否启用此 LED
} LedObject_t;


/**
 * ========== 初始化函数 ==========
 */

/**
 * @brief 初始化 LED 系统
 * @note SPI 和 74HC595 初始化在此完成
 */
void led_init(void);


/**
 * ========== 基础控制接口 ==========
 */

/**
 * @brief 设置单个 LED 的颜色状态
 * @param id LED 编号 (0-7)
 * @param state 颜色状态 (LedColorState_t)
 * @return 0 成功，负值失败
 */
int led_set_color(rt_uint8_t id, LedColorState_t state);

/**
 * @brief 获取单个 LED 的当前颜色状态
 * @param id LED 编号 (0-7)
 * @return 当前颜色状态
 */
LedColorState_t led_get_color(rt_uint8_t id);

/**
 * @brief 开启或关闭单个 LED
 * @param id LED 编号 (0-7)
 * @param on true 开启，false 关闭
 * @return 0 成功，负值失败
 */
int led_enable(rt_uint8_t id, rt_bool_t on);

/**
 * @brief 检查 LED 是否已启用
 * @param id LED 编号 (0-7)
 * @return true 已启用
 */
rt_bool_t led_is_enabled(rt_uint8_t id);


/**
 * ========== 高级功能接口 ==========
 */

/**
 * @brief 批量设置多个 LED 的状态
 * @param start_id 起始 LED ID (0-7)
 * @param count LED 数量 (1-8)
 * @param states LED 状态数组指针
 * @return 0 成功，负值失败
 */
int led_set_multiple(rt_uint8_t start_id, rt_uint8_t count, const LedColorState_t* states);

/**
 * @brief 全部关闭所有 LED
 */
void led_off_all(void);

/**
 * @brief 全部开启所有绿色 LED (0,2,4,6)
 */
void led_on_all_green(void);

/**
 * @brief 全部开启所有红色 LED (1,3,5,7)
 */
void led_on_all_red(void);

/**
 * @brief 扫描 LED 效果 - 从 0 到 7 顺序点亮
 */
void led_scan_effect(void);


/**
 * ========== 预设场景接口 ==========
 */

/**
 * @brief 工作指示灯状态设置
 * @param running true 工作中 (绿灯常亮)
 */
void led_set_working(rt_bool_t running);

/**
 * @brief 故障指示灯状态设置
 * @param faulted true 有故障 (红灯常亮)
 */
void led_set_fault(rt_bool_t faulted);

/**
 * @brief 电池电量指示灯设置
 * @param full true 电量满
 */
void led_set_battery_full(rt_bool_t full);

/**
 * @brief 低电量警告设置
 * @param low true 电量低
 */
void led_set_battery_low(rt_bool_t low);

/**
 * @brief 水箱水位指示灯设置
 * @param full true 水位满
 */
void led_set_water_full(rt_bool_t full);

/**
 * @brief 低水位警告设置
 * @param low true 水位不足
 */
void led_set_water_low(rt_bool_t low);

/**
 * @brief 开机完整提示序列
 * @note 先扫面一次，然后绿灯常亮表示就绪
 */
void led_poweron_sequence(void);

/**
 * @brief 关机提示序列
 * @note 停止所有上位机 LED 并全灭
 */
void led_shutdown_sequence(void);


/* ========== 上位机 LED 控制接口 (新增) ========== */

/**
 * @brief 停止 LED 的软定时器闪烁 (上位机控制)
 * @param led_idx 0=LED7(导航), 1=LED8(异常)
 */
void led_stop_flash(uint8_t led_idx);

/**
 * @brief 启动 LED 的软定时器闪烁 (上位机指令)
 * @param led_idx 0=LED7(导航), 1=LED8(异常)
 * @param on_time_ms 亮的时间 (毫秒)
 * @param off_time_ms 灭的时间 (毫秒)
 * @param repeat 重复次数 (0=无限循环)
 * @return 0 成功，负值失败
 */
int led_flash(uint8_t led_idx, uint16_t on_time_ms, uint16_t off_time_ms, uint16_t repeat);


#endif /* PERIPHERALS_LED_H__ */
