/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief OLED 显示句柄 - 系统服务层
 * 
 * 功能：
 *   1. 多页面管理（启动页、主页、设置页等）
 *   2. SW3/SW4 轮询切换（不用外中断）
 *   3. 与 LED SPI 资源的互斥保护
 *   4. 定时刷新 + 信号量触发刷新
 */

#ifndef SYSTEM_OLED_HANDLE_H__
#define SYSTEM_OLED_HANDLE_H__

#include <rtthread.h>


/**
 * @brief OLED 屏幕配置
 */
#define OLED_WIDTH            128
#define OLED_HEIGHT           64


/**
 * @brief 屏幕分页定义
 * 每个页面包含：页面 ID、标题、渲染函数指针
 */
typedef enum
{
    PAGE_BOOT = 0,          // 启动页 - 开机时显示
    PAGE_HOME,              // 主页 - 默认显示
    PAGE_PID_TUNING,        // PID 调参页
    PAGE_ULTRASONIC,        // 超声波传感器数据页
    PAGE_IR_SENSOR,         // 红外传感器数据页
    PAGE_BATTERY_INFO,      // 电池电量信息页
    PAGE_WATER_LEVEL,       // 水位信息页
    PAGE_MOTOR_STATUS,      // 电机状态页
    PAGE_IMU_DATA,          // IMU 姿态数据页 ⭐ 新增
    PAGE_FAULT_LOG,         // 故障日志页
    PAGE_SETTINGS,          // 系统设置页
    
    PAGE_COUNT              // 页面总数 (11 个)
} OledPageId_t;


/**
 * @brief 页面数据结构
 */
typedef struct
{
    OledPageId_t id;                // 页面 ID
    const char* title;              // 页面标题（顶部栏显示）
    void (*render_func)(void);      // 页面渲染函数指针
    rt_uint8_t refresh_interval_ms; // 自动刷新间隔（0=不自动刷新）
} OledPage_t;


/**
 * ========== 初始化函数 ==========
 */

/**
 * @brief 初始化 OLED 显示系统
 * @note 创建显示任务、注册页面、获取 SPI 互斥锁
 */
void oled_handle_init(void);


/**
 * ========== 页面管理 API ==========
 */

/**
 * @brief 注册一个显示页面
 * @param page_id 页面 ID
 * @param title 页面标题
 * @param render_func 页面渲染函数（绘制内容的回调）
 * @return 0 成功，负值失败
 */
int oled_register_page(OledPageId_t page_id, const char* title, void (*render_func)(void));

/**
 * @brief 切换到指定页面
 * @param page_id 目标页面 ID
 * @return 0 成功
 */
int oled_switch_page(OledPageId_t page_id);

/**
 * @brief 上一上一页
 */
void oled_prev_page(void);

/**
 * @brief 下一页
 */
void oled_next_page(void);

/**
 * @brief 获取当前页面 ID
 */
OledPageId_t oled_get_current_page(void);


/**
 * ========== 刷新控制 API ==========
 */

/**
 * @brief 触发屏幕刷新（信号量）
 * @note 由其他任务调用，通知 OLED 刷新最新数据
 */
void oled_trigger_refresh(void);

/**
 * @brief 立即强制刷新当前页面
 */
void oled_force_refresh(void);

/**
 * @brief 设置页面自动刷新间隔
 * @param page_id 页面 ID
 * @param interval_ms 刷新间隔（单位毫秒），0 表示禁用自动刷新
 */
void oled_set_auto_refresh(OledPageId_t page_id, rt_uint32_t interval_ms);


/**
 * ========== UI 组件绘制接口 ==========
 * 
 * 这些函数供各页面渲染函数调用，提供常用 UI 组件
 */

/**
 * @brief 绘制顶部标题栏
 * @param title 标题文本（最长不超过 12 字符）
 * @param show_page_indicator 是否显示页码指示器（如 "[1/9]"）
 */
void oled_draw_title_bar(const char* title, rt_bool_t show_page_indicator);

/**
 * @brief 绘制进度条
 * @param x 左上角 X 坐标
 * @param y 左上角 Y 坐标
 * @param width 宽度
 * @param height 高度
 * @param progress 进度值 (0-100%)
 * @param color_color true=绿色，false=红色
 */
void oled_draw_progress(rt_uint8_t x, rt_uint8_t y, rt_uint8_t width, 
                        rt_uint8_t height, rt_uint8_t progress, rt_bool_t is_good);

/**
 * @brief 绘制数值显示框（带单位）
 * @param x X 坐标
 * @param y Y 坐标
 * @param value 数值（整数）
 * @param unit 单位字符串（如"V", "mm", "%", "°C"）
 * @param precision 小数位数
 */
void oled_draw_value_box(rt_uint8_t x, rt_uint8_t y, int32_t value, 
                         const char* unit, rt_uint8_t precision);

/**
 * @brief 绘制状态指示灯（圆点）
 * @param x 圆心 X 坐标
 * @param y 圆心 Y 坐标
 * @param radius 半径
 * @param active true=亮绿，false=暗红
 */
void oled_draw_status_dot(rt_uint8_t x, rt_uint8_t y, rt_uint8_t radius, rt_bool_t active);

/**
 * @brief 绘制图标
 * @param x X 坐标
 * @param y Y 坐标
 * @param icon_id 图标 ID
 */
void oled_draw_icon(rt_uint8_t x, rt_uint8_t y, rt_uint8_t icon_id);


/**
 * ========== 电池电量全局变量（u8g2 示例程序中使用）==========
 */

extern int g_oled_battery_mv;    // 当前电池电压 (mV)


/**
 * ========== SPI 资源互斥锁 API - 供其他模块调用 ==========
 * 
 * OLED 和 LED(74HC595) 共用同一个 SPI 总线
 * LED 模块在发送数据前必须调用此锁保护机制
 */

/**
 * @brief 获取 SPI 互斥锁（等待）
 * @note 必须在 LED_shift_out() 等 SPI 操作前调用
 */
void oled_spi_lock(void);

/**
 * @brief 释放 SPI 互斥锁
 * @note 必须在完成 SPI 操作后调用对应的 unlock
 */
void oled_spi_unlock(void);


#endif /* SYSTEM_OLED_HANDLE_H__ */
