/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief OLED display handle implementation v1.1.4
 * 
 * Function description:
 *   1. Use u8g2 library with SPI2 driver for SSD1306 OLED
 *   2. Multi-page management and switching (11 pages)
 *   3. SW3/SW4 polling detection (non-interrupt method)
 *   4. Mutual exclusion protection for SPI resources shared with LED
 *   5. MSH console test commands with correct export aliases (v1.1.4)
 */

#include <rtthread.h>
#include <u8g2_port.h>
#include "oled_handle.h"
#include "global_conf.h"
#include "button.h"
#include <string.h>
#include <u8g2.h>
#include <stdlib.h>
#include "qmi8658.h"
#include <math.h>
#include "print_utils.h"
#include "monitor.h"
#include "user_action.h"
/**
 * ========== Global Variables ==========
 */
#ifndef rt_tick_get_millisecond
#define rt_tick_get_millisecond()  (rt_tick_get() )  //RT_TICK_PER_SECOND = 1000
#endif
/**
 * @brief Battery level global variable (shared with u8g2 example program)
 */
int g_oled_battery_mv = 24000;
int g_charge_power_mw;
/**
 * @brief OLED update task handle (extern declaration for MSH commands)
 */
rt_thread_t s_oled_thread;

/**
 * @brief U8g2 display object
 */
static u8g2_t s_u8g2;

/**
 * @brief SPI resource mutex - resolves OLED and LED SPI conflict
 */
static struct rt_mutex s_spi_mutex;

/**
 * @brief Refresh semaphore - triggers refresh from other tasks
 */
static struct rt_semaphore s_refresh_sem;

/**
 * @brief Current page index
 */
static OledPageId_t s_current_page = PAGE_BOOT;

/**
 * @brief Page count
 */
static uint8_t s_page_count = 0;

/**
 * @brief Pages array (max 16 pages)
 */
static OledPage_t s_pages[PAGE_COUNT];


/**
 * ========== Internal Function Prototypes ==========
 */
static void oled_gpio_init(void);
static void u8g2_init(void);
static void render_boot_page(u8g2_t* u8g2);
static void render_home_page(u8g2_t* u8g2);
static void render_pid_tuning_page(u8g2_t* u8g2);
static void render_ultrasonic_page(u8g2_t* u8g2);
static void render_ir_sensor_page(u8g2_t* u8g2);
static void render_battery_info_page(u8g2_t* u8g2);
static void render_water_level_page(u8g2_t* u8g2);
static void render_motor_status_page(u8g2_t* u8g2);
static void render_imu_data_page(u8g2_t* u8g2);
static void render_fault_log_page(u8g2_t* u8g2);
static void render_settings_page(u8g2_t* u8g2);

static void oled_update_task(void *parameter);
static void oled_key_task(void *parameter);

/**
 * ========== Public Functions ==========
 */
/**
 * @brief OLED hardware initialization
 */
void oled_handle_init(void)
{
    oled_gpio_init();
    u8g2_init();
    
    // Create mutex with priority inheritance
    rt_mutex_init(&s_spi_mutex, "oled_spi", RT_IPC_FLAG_PRIO);
    
    // Create semaphore for refresh trigger
    rt_sem_init(&s_refresh_sem, "oled_ref", 0, RT_IPC_FLAG_PRIO);
    
    // Register all pages
    oled_register_page(PAGE_BOOT, "Booting...", render_boot_page);
    oled_register_page(PAGE_HOME, "Main Menu", render_home_page);
    oled_register_page(PAGE_PID_TUNING, "PID Tuning", render_pid_tuning_page);
    oled_register_page(PAGE_ULTRASONIC, "Ultrasonic", render_ultrasonic_page);
    oled_register_page(PAGE_IR_SENSOR, "IR Sensors", render_ir_sensor_page);
    oled_register_page(PAGE_BATTERY_INFO, "Battery Info", render_battery_info_page);
    oled_register_page(PAGE_WATER_LEVEL, "Water Level", render_water_level_page);
    oled_register_page(PAGE_MOTOR_STATUS, "Motor Status", render_motor_status_page);
    oled_register_page(PAGE_IMU_DATA, "IMU Data", render_imu_data_page);
    oled_register_page(PAGE_FAULT_LOG, "Fault Log", render_fault_log_page);
    oled_register_page(PAGE_SETTINGS, "Settings", render_settings_page);
    
    s_page_count = PAGE_COUNT;
    
    // Create OLED update task
    s_oled_thread = rt_thread_create("oled",
                                     oled_update_task,
                                     NULL,
                                     1024,
                                     RT_THREAD_PRIORITY_MAX - 5,
                                     20);
    
    if (s_oled_thread != RT_NULL)
    {
        rt_thread_startup(s_oled_thread);
    }
		
		    // 创建按键处理任务，接收消息队列并切换页面
    rt_thread_t key_thread = rt_thread_create("oled_key",
                                              oled_key_task,
                                              NULL,
                                              512,                       // 栈大小
                                              RT_THREAD_PRIORITY_MAX - 4, // 优先级略高于普通任务
                                              20);                       // 时间片
    if (key_thread != RT_NULL)
    {
        rt_thread_startup(key_thread);
    }
		
//		rt_mutex_release(&s_spi_mutex);
//		rt_sem_release(&s_refresh_sem);
		rt_kprintf("OLED init ok\r\n");
}


/**
 * @brief Lock SPI bus before OLED operations
 */
void oled_spi_lock(void)
{
    rt_mutex_take(&s_spi_mutex, RT_WAITING_FOREVER);
}


/**
 * @brief Unlock SPI bus after OLED operations
 */
void oled_spi_unlock(void)
{
    rt_mutex_release(&s_spi_mutex);
}


/**
 * @brief Trigger OLED refresh from any task
 */
void oled_trigger_refresh(void)
{
    rt_sem_release(&s_refresh_sem);
}


/**
 * @brief Switch to specified page
 * @param page_id Page ID to switch to
 * @return RT_EOK on success, RT_ERROR on invalid page
 */
rt_err_t oled_switch_page(OledPageId_t page_id)
{
    if (page_id >= PAGE_COUNT) return -RT_ERROR;
    
    rt_kprintf("[OLED] Switching page %d -> %d\r\n", s_current_page, page_id);
    s_current_page = page_id;
    oled_trigger_refresh();
    
    return RT_EOK;
}

/**
 * @brief Switch to previous page
 */
void oled_prev_page(void)
{
    if (s_current_page > 0) {
        oled_switch_page((OledPageId_t)(s_current_page - 1));
    }
    // 如果在第 0 页，什么也不做
}

/**
 * @brief Switch to next page
 */
void oled_next_page(void)
{
    if (s_current_page < PAGE_COUNT - 1) {
        oled_switch_page((OledPageId_t)(s_current_page + 1));
    }
    // 如果在最后一页，什么也不做
}

/**
 * @brief Get current page ID
 * @return Current page ID
 */
OledPageId_t oled_get_current_page(void)
{
    return s_current_page;
}


/**
 * @brief Force screen refresh
 */
void oled_force_refresh(void)
{
    rt_mutex_take(&s_spi_mutex, RT_WAITING_FOREVER);
    u8g2_ClearDisplay(&s_u8g2);
    s_pages[s_current_page].render(&s_u8g2);
    u8g2_SendBuffer(&s_u8g2);
    rt_mutex_release(&s_spi_mutex);
}


/**
 * @brief Register a display page
 * @param page_id Unique page identifier
 * @param title Short title string
 * @param render_callback Render function pointer
 */
int oled_register_page(OledPageId_t page_id, const char* title, void (*render_callback)(u8g2_t*))
{
    if (page_id >= PAGE_COUNT) return -RT_ERROR;
    s_pages[page_id].id = page_id;
    s_pages[page_id].title = title;
    s_pages[page_id].render = render_callback;
    s_pages[page_id].refresh_interval_ms = 1000;
    return RT_EOK;
}


/**
 * @brief Set page refresh interval
 * @param page_id Page ID
 * @param interval_ms Refresh interval in milliseconds
 */
void oled_set_page_interval(OledPageId_t page_id, uint16_t interval_ms)
{
    if (page_id >= PAGE_COUNT) return;
    
    s_pages[page_id].refresh_interval_ms = interval_ms;
}


/**
 * @brief Draw title bar at top of screen
 * @param title Title text (max 12 characters)
 * @param show_page_indicator Whether to show page indicator like "[1/9]"
 */
void oled_draw_title_bar(const char* title, rt_bool_t show_page_indicator)
{
   uint8_t y = 18;  // 基线位置，可根据实际效果微调（范围 16~20）

    // 1. 绘制标题（粗体）
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&s_u8g2, 8, y, title);

    // 2. 如果需要显示页码
    if (show_page_indicator)
    {
        char page_buf[16];
        rt_snprintf(page_buf, sizeof(page_buf), "%d/%d", s_current_page + 1, s_page_count);

        // 切换为页码字体（常规体），计算宽度并绘制
        u8g2_SetFont(&s_u8g2, u8g2_font_ncenR08_tr);
        uint8_t str_width = u8g2_GetStrWidth(&s_u8g2, page_buf);
        u8g2_DrawStr(&s_u8g2, OLED_WIDTH - 8 - str_width, y, page_buf);
    }
}


/**
 * @brief Draw a value box with label
 * @param x X coordinate
 * @param y Y coordinate
 * @param value Numeric value
 * @param unit Unit string
 * @param decimal_digits Number of decimal places
 */
void oled_draw_value_box(uint8_t x, uint8_t y, int32_t value, const char* unit, uint8_t decimal_digits)
{
    char buf[32];
    
    if (decimal_digits == 0)
    {
        rt_snprintf(buf, sizeof(buf), "%ld%s", value, unit);
    }
    else if (decimal_digits == 1)
    {
        rt_snprintf(buf, sizeof(buf), "%ld.%d%s", value / 10, value % 10, unit);
    }
    else if (decimal_digits == 2)
    {
        rt_snprintf(buf, sizeof(buf), "%ld.%02d%s", value / 100, value % 100, unit);
    }
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB10_tr);
    u8g2_DrawStr(&s_u8g2, x, y, buf);
}


/**
 * @brief Draw battery voltage indicator
 */
static void draw_battery_indicator(uint8_t x, uint8_t y)
{    
    uint32_t mv = g_oled_battery_mv;
    // 限制电压范围
    if (mv > BATTERY_FULL_VOLTAGE_MV) mv = BATTERY_FULL_VOLTAGE_MV;
    if (mv < BATTERY_LOW_VOLTAGE_MV) mv = BATTERY_LOW_VOLTAGE_MV;
    
    // 计算填充宽度（0~30像素）
    uint32_t range = BATTERY_FULL_VOLTAGE_MV - BATTERY_LOW_VOLTAGE_MV;
    uint32_t fill = (mv - BATTERY_LOW_VOLTAGE_MV) * 30 / range;
    uint8_t fill_width = (uint8_t)fill;
    
    // 绘制电池外框和尖端（不变）
    u8g2_DrawFrame(&s_u8g2, x, y, 30, 16);
    u8g2_DrawLine(&s_u8g2, x + 30, y + 4, x + 34, y + 8);
    u8g2_DrawLine(&s_u8g2, x + 30, y + 12, x + 34, y + 8);
    
    // 绘制电量填充
    if (fill_width > 0) {
        u8g2_DrawBox(&s_u8g2, x + 2, y + 2, fill_width, 12);
    }
}


/**
 * @brief Draw progress bar
 * @param x X coordinate
 * @param y Y coordinate  
 * @param width Bar width
 * @param height Bar height
 * @param percent Percentage (0-100)
 */
static void draw_progress_bar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t percent)
{
    // Border
    u8g2_DrawFrame(&s_u8g2, x, y, width, height);
    
    // Fill
    if (percent > 100) percent = 100;
    uint8_t fill_width = width * percent / 100;
    u8g2_DrawBox(&s_u8g2, x, y, fill_width, height);
}

/**
 * @brief Draw progress bar (public API)
 */
void oled_draw_progress(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t progress, rt_bool_t is_good)
{
    draw_progress_bar(x, y, width, height, progress);  // 调用内部静态函数
}

/**
 * @brief Draw status indicator dot (public API)
 */
void oled_draw_status_dot(uint8_t x, uint8_t y, uint8_t radius, rt_bool_t active)
{
    if (active)
    {
        u8g2_DrawDisc(&s_u8g2, x, y, radius,U8G2_DRAW_ALL);   // 实心圆
    }
    else
    {
        u8g2_DrawCircle(&s_u8g2, x, y, radius,U8G2_DRAW_ALL); // 空心圆
    }
}

/**
 * @brief Draw icon (public API) - 暂未实现
 */
void oled_draw_icon(uint8_t x, uint8_t y, uint8_t icon_id)
{
    // 可根据需要添加图标绘制，目前仅输出调试信息
    rt_kprintf("[OLED] oled_draw_icon(%d,%d,%d) not implemented\n", x, y, icon_id);
}

/**
 * @brief GPIO initialization for OLED
 */
static void oled_gpio_init(void)
{
#if defined(LED_USE_DIRECT_GPIO)
    // Direct GPIO control mode
    rt_pin_mode(OLED_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(OLED_DC_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(OLED_CS_PIN, HIGH);
    rt_pin_write(OLED_DC_PIN, LOW);
#else
    // SPI mode - CS and DC controlled by SPI driver
    // No additional GPIO setup needed
#endif
}


/**
 * @brief Initialize u8g2 display object (SPI2 DRIVER v1.1.4)
 * NOTE: Two-stage init - first call enables SPI2, second call does actual display init
 *       RESET pin set to U8X8_PIN_NONE (255) because OLED_RST_PIN removed from global_conf.h
 */
static void u8g2_init(void)
{
    rt_kprintf("OLED display\r\n");
    
    // 1. Initialize with SPI2 driver (CRITICAL!)
    u8g2_Setup_ssd1306_128x64_noname_f( &s_u8g2, U8G2_R0, 
        u8x8_byte_rtthread_4wire_hw_spi, u8x8_gpio_and_delay_rtthread);
    
    // Set SPI pins for CS, DC
    // RESET set to U8X8_PIN_NONE (255) = no software reset control
    u8x8_SetPin(u8g2_GetU8x8(&s_u8g2), U8X8_PIN_CS, OLED_CS_PIN);
    u8x8_SetPin(u8g2_GetU8x8(&s_u8g2), U8X8_PIN_DC, OLED_DC_PIN);
    u8x8_SetPin(u8g2_GetU8x8(&s_u8g2), U8X8_PIN_RESET, U8X8_PIN_NONE);  // Hardware reset not needed
    
    // First u8g2_InitDisplay() call - ENABLES SPI2 BUS if not already enabled
    rt_kprintf("[OLED] Stage 1: Enabling SPI2 bus...\r\n");
    u8g2_InitDisplay(&s_u8g2);
    
    // 2. Power-on delay - CRITICAL FOR OLED STABILITY!
    rt_thread_mdelay(10);  
    
    // Second u8g2_InitDisplay() call - ACTUAL DISPLAY INITIALIZATION
    rt_kprintf("[OLED] Stage 2: Initializing display...\r\n");
    u8g2_InitDisplay(&s_u8g2);
    
    rt_kprintf("[OLED] Display initialized successfully!\r\n");
    u8g2_ClearDisplay(&s_u8g2);
    u8g2_SetPowerSave(&s_u8g2, 0);
}


/**
 * @brief OLED update task
 */
void oled_update_task(void* parameter)
{
    uint32_t tick_last = 0;
    rt_int32_t result;
    static OledPageId_t s_last_page = PAGE_COUNT;  // 初始化为无效值，确保首次刷新硬件清屏
	
    while (1)
    {
        result = rt_sem_take(&s_refresh_sem, 100);
        
        // Check timeout - still refresh if needed
        uint32_t tick_now = rt_tick_get_millisecond();
        
        if (result == RT_EOK || 
            (tick_now - tick_last) >= s_pages[s_current_page].refresh_interval_ms)
        {
						OledPageId_t current_page = s_current_page;  // 读取当前页（可能已被其他任务修改）
            // Acquire SPI mutex
            rt_mutex_take(&s_spi_mutex, RT_WAITING_FOREVER);
            
					// 判断是否发生页面切换
            if (current_page != s_last_page)
            {
                // 页面切换：硬件清屏（清除屏幕和缓冲区）
                u8g2_ClearDisplay(&s_u8g2);
                s_last_page = current_page;  // 更新记录的上次页面
            }
            else
            {
                // 同一页面刷新：仅清内存缓冲区，避免硬件闪烁
                u8g2_ClearBuffer(&s_u8g2);
            }
            // Clear and redraw
   //         u8g2_ClearDisplay(&s_u8g2);
            s_pages[s_current_page].render(&s_u8g2);
            
            // Update display
            u8g2_SendBuffer(&s_u8g2);
            
            // Release SPI mutex
            rt_mutex_release(&s_spi_mutex);
            
            tick_last = tick_now;
        }
    }
}

/**
 * @brief OLED 按键处理任务，接收按键消息队列
 */
static void oled_key_task(void *parameter)
{
		rt_kprintf("[OLED] oled_key_task started\n");
    rt_uint32_t msg=0;
    rt_err_t result;
	
		rt_mq_t page_queue = button_get_page_queue();
    if (page_queue == RT_NULL) {
        rt_kprintf("[OLED] Error: Failed to get button page queue!\n");
        return;
    }
    while (1)
    {
        // 阻塞等待按键消息
        result = rt_mq_recv(page_queue, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result > 0)
        {
					//rt_kprintf("[OLED] Received msg: 0x%08lx\n", msg);
            if (msg == 0xFFFFFFFF)          // 上一页
            {
                oled_prev_page();
            }
            else if (msg == 0xFFFFFFFE)      // 下一页
            {
                oled_next_page();
            }
            else if (msg < PAGE_COUNT)       // 直接跳转到指定页面（预留）
            {
                oled_switch_page((OledPageId_t)msg);
            }
            else
            {
                rt_kprintf("[OLED] Unknown page msg: 0x%08lx\n", msg);
            }
        } else {
            rt_kprintf("[OLED] mq_recv error: %d\n", result);
						rt_thread_mdelay(100); // 避免过频错误打印
        }
    }
}

/**
 * ========== Page Renderers (v1.1.4 - Full Display Content Restored) ==========
 */

static void render_boot_page(u8g2_t* u8g2)
{
    static rt_tick_t start_tick = 0;
    static rt_bool_t switched = RT_FALSE;
    if (start_tick == 0) {
        start_tick = rt_tick_get();
    }
    rt_tick_t elapsed = rt_tick_get() - start_tick;
    uint8_t progress = (elapsed * 100) / (RT_TICK_PER_SECOND * 10); // 10秒满
    if (progress > 100) progress = 100;

    // iHomeRobot logo
    u8g2_SetFont(u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(u8g2, 8, 15, "iHomeRobot");
    u8g2_DrawStr(u8g2, 8, 25, "Smart Toilet");

    // 进度条
    u8g2_DrawFrame(u8g2, 8, 35, 112, 8);
    u8g2_DrawBox(u8g2, 8, 35, progress * 112 / 100, 8);

    // 提示文字
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    u8g2_DrawStr(u8g2, 40, 55, "Booting...");

    // 进度满则自动跳转到主页
    if (progress >= 100 && !switched) {
        switched = RT_TRUE;
        oled_switch_page(PAGE_HOME);
    }
}

static void render_home_page(u8g2_t* u8g2)
{  
    // Page title
    oled_draw_title_bar("Main Menu", RT_TRUE);
    
    // 电池指示器（绘制在右上角，与标题栏共存）
    draw_battery_indicator(90, 15);  // 位置微调，避免覆盖页码
    
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
   uint8_t y = 30;  // 起始Y坐标，从标题栏下方开始

    // 电池电压
    long bat_v = g_oled_battery_mv / 1000;
    long bat_mv = g_oled_battery_mv % 1000;
    char bat_str[20];
    rt_snprintf(bat_str, sizeof(bat_str), "Bat: %ld.%03dV", bat_v, bat_mv);
    u8g2_DrawStr(u8g2, 8, y, bat_str);
    y += 8;

	//进度条 计算电量百分比 (Vmin=19.2V, Vmax=25.2V)
		uint8_t bat_percent = (g_oled_battery_mv - BATTERY_LOW_VOLTAGE_MV) * 100 / 
													(BATTERY_FULL_VOLTAGE_MV - BATTERY_LOW_VOLTAGE_MV);
		if (bat_percent > 100) bat_percent = 100;
		draw_progress_bar(8, y, 112, 6, bat_percent);
    y += 15;
		rt_snprintf(bat_str, sizeof(bat_str), "Power: %dW", g_charge_power_mw/1000);
    u8g2_DrawStr(u8g2, 8, y, bat_str);
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Mode: Standby");

}

static void render_pid_tuning_page(u8g2_t* u8g2)
{
    oled_draw_title_bar("PID Tuning", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
	
	// 声明外部 PID 变量（需在实际代码中定义）
 //   extern float g_pid_kp, g_pid_ki, g_pid_kd;
	float g_pid_kp = 2.5;
	float	g_pid_ki = 0.05;
	float g_pid_kd = 0.85;

    char buf[20];

    // 显示 Kp（比例系数）
    int16_t kp_int = (int16_t)(g_pid_kp * 100);
	rt_snprintf(buf, sizeof(buf), "Kp: %d.%02d", kp_int / 100, kp_int % 100);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 12;

    // 显示 Ki（积分系数）
    int16_t ki_int = (int16_t)(g_pid_ki * 100);
	rt_snprintf(buf, sizeof(buf), "Ki: %d.%02d", ki_int / 100, ki_int % 100);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 12;

    // 显示 Kd（微分系数）
    int16_t kd_int = (int16_t)(g_pid_kd * 100);
    rt_snprintf(buf, sizeof(buf), "Kd: %d.%02d", kd_int / 100, kd_int % 100);
    u8g2_DrawStr(u8g2, 8, y, buf);
}

static void render_ultrasonic_page(u8g2_t* u8g2)
{
	  oled_draw_title_bar("Ultrasonic", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
    u8g2_DrawStr(u8g2, 8, y, "Front: 152 mm");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Back:   89 mm");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Left:  124 mm");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Right: 118 mm");
}

static void render_ir_sensor_page(u8g2_t* u8g2)
{
	  oled_draw_title_bar("IR Sensors", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
    u8g2_DrawStr(u8g2, 8, y, "Cliff Front: OK");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Cliff Rear:  OK");
    y += 2;
    u8g2_DrawHLine(u8g2, 8, y, 112);
    y += 8;
    u8g2_DrawStr(u8g2, 8, y, "Charging Align:");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Ready");
}

static void render_battery_info_page(u8g2_t* u8g2)
{
    oled_draw_title_bar("Battery Info", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;

    // 电池电压
    char buf[32];
    rt_snprintf(buf, sizeof(buf), "Voltage: %d.%03dV", 
                g_oled_battery_mv / 1000, g_oled_battery_mv % 1000);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;

    // 电量百分比
    uint8_t percent = (g_oled_battery_mv - BATTERY_LOW_VOLTAGE_MV) * 100 /
                      (BATTERY_FULL_VOLTAGE_MV - BATTERY_LOW_VOLTAGE_MV);
    if (percent > 100) percent = 100;
    rt_snprintf(buf, sizeof(buf), "Level: %d%%", percent);
    u8g2_DrawStr(u8g2, 8, y, buf);
//    y += 12; 
		y += 2;
    draw_progress_bar(8, y, 112, 6, percent);
    y += 12;

    // 充电状态（使用 monitor 的充电检测电压）
    rt_uint32_t charger_mv = monitor_get_charger_voltage();   // 需要包含 monitor.h
    if (rt_pin_read(CHARGER_CONTROL_PIN) == PIN_HIGH && charger_mv > 5000) {
        u8g2_DrawStr(u8g2, 8, y, "Charge: Active");
    } else if(rt_pin_read(CHARGER_CONTROL_PIN) == PIN_LOW && charger_mv > 5000){
        u8g2_DrawStr(u8g2, 8, y, "Charge: Idle");
    }else{
        u8g2_DrawStr(u8g2, 8, y, "Charge: Disconnected");
    }
    y += 8;

    // 其他信息
    u8g2_DrawStr(u8g2, 8, y, "Status: Normal");
}

static void render_water_level_page(u8g2_t* u8g2)
{
    // Water tank status
    oled_draw_title_bar("Water Level", RT_TRUE);
    
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
	
    // 低水位状态
    if (monitor_get_water_low_level()) {
        u8g2_DrawStr(u8g2, 8, y, "Low Water:  LOW");
    } else {
        u8g2_DrawStr(u8g2, 8, y, "Low Water:  OK");
    }
    y += 10;

   // 工作状态：清洁臀部
    const WorkStatus_t* status = user_action_get_work_status();
    if (status->clean_rear) {
        u8g2_DrawStr(u8g2, 8, y, "Rear Clean: Working");
    } else {
        u8g2_DrawStr(u8g2, 8, y, "Rear Clean: Idle");
    }
    y += 10;
		
	// 工作状态：清洁女性
    if (status->clean_female) {
        u8g2_DrawStr(u8g2, 8, y, "Female Clean: Working");
    } else {
        u8g2_DrawStr(u8g2, 8, y, "Female Clean: Idle");
    }
    y += 10;
    // 工作状态：烘干
    if (status->dry) {
        u8g2_DrawStr(u8g2, 8, y, "Dry: Working");
    } else {
        u8g2_DrawStr(u8g2, 8, y, "Dry: Idle");
    }		
}

static void render_motor_status_page(u8g2_t* u8g2)
{
    // Motor operational status
    oled_draw_title_bar("Motor Status", RT_TRUE);
    
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
    u8g2_DrawStr(u8g2, 8, y, "Left Motor:  0 RPM");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Right Motor: 0 RPM");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Cleaning Rod:");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Tube Switcher:");
}

static void render_imu_data_page(u8g2_t* u8g2)
{
    // IMU attitude data
    oled_draw_title_bar("IMU Data", RT_TRUE);
    
     u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
		
	   if(s_report_raw) {
        // 显示原始加速度和陀螺仪
//        char buf[32];
//			 	int acc_x = s_latest_data.acc_x_g*1000;
//				int acc_y = s_latest_data.acc_y_g*1000; 
//				int acc_z = s_latest_data.acc_z_g*1000;
//        rt_snprintf(buf, sizeof(buf), "ACC: X%+4d/1000",
//                    acc_x);
//        u8g2_DrawStr(u8g2, 0, y, buf);
//        rt_snprintf(buf, sizeof(buf), "Y%+4d/1000 Z%+4d/1000",
//                    acc_y, acc_z);
//        y += 10;
//				u8g2_DrawStr(u8g2, 0, y, buf);
//				
//			 	int gyro_x = s_latest_data.gyro_x_deg*100;
//				int gyro_y = s_latest_data.gyro_y_deg*100; 
//				int gyro_z = s_latest_data.gyro_z_deg*100;				
//        rt_snprintf(buf, sizeof(buf), "GYRO:X%+3d/100",
//                    gyro_x);
//        y += 12;
//				u8g2_DrawStr(u8g2, 0, y, buf);
//        rt_snprintf(buf, sizeof(buf), "Y%+3d/100 Z%+3d/100",
//                    gyro_y, gyro_z);
//        y += 10;
//				u8g2_DrawStr(u8g2, 0, y, buf);				
       // 显示原始加速度和陀螺仪（浮点形式）
        char buf[48];
        // 加速度一行
        format_float(s_latest_data.acc_x_g, buf, 0, 3, 1);
        u8g2_DrawStr(u8g2, 0, y, "ACC X:");
        u8g2_DrawStr(u8g2, 50, y, buf);
        y += 10;				
        format_float(s_latest_data.acc_y_g, buf, 0, 3, 1);
				u8g2_DrawStr(u8g2, 0, y, "Y:");
        u8g2_DrawStr(u8g2, 20, y, buf);        				

        format_float(s_latest_data.acc_z_g, buf, 0, 3, 1);
        u8g2_DrawStr(u8g2, 70, y, "Z:");
        u8g2_DrawStr(u8g2, 90, y, buf);
        y += 10;

        // 陀螺仪一行
        format_float(s_latest_data.gyro_x_deg, buf, 0, 1, 1);
        u8g2_DrawStr(u8g2, 0, y, "GYRO X:");
        u8g2_DrawStr(u8g2, 50, y, buf);
				y += 10;
        format_float(s_latest_data.gyro_y_deg, buf, 0, 1, 1);
        u8g2_DrawStr(u8g2, 0, y, "Y:");
        u8g2_DrawStr(u8g2, 20, y, buf);        				

        format_float(s_latest_data.gyro_z_deg, buf, 0, 1, 1);
        u8g2_DrawStr(u8g2, 70, y, "Z:");
        u8g2_DrawStr(u8g2, 90, y, buf);
				
    } else {
//				int pitch = s_latest_data.pitch*100;
//				int yaw = s_latest_data.yaw*100; 
//				int roll = s_latest_data.roll*100;
//				 
//				char imustr[20];
//				rt_snprintf(imustr, sizeof(imustr), "Pitch: %+5d/100", pitch);
//				u8g2_DrawStr(u8g2, 8, y, imustr);
//				y += 10;
//				rt_snprintf(imustr, sizeof(imustr), "Roll:  %+5d/100", roll);
//				u8g2_DrawStr(u8g2, 8, y, imustr);
//				y += 10;
//				rt_snprintf(imustr, sizeof(imustr), "Yaw:   %+5d/100", yaw);
//				u8g2_DrawStr(u8g2, 8, y, imustr);
//				y += 10;
//				u8g2_DrawStr(u8g2, 8, y, "Status: Calibrated");
			      // 显示欧拉角（浮点形式）
        char buf[24];
        format_float(s_latest_data.pitch, buf, 6, 2, 1);
        u8g2_DrawStr(u8g2, 8, y, "Pitch: ");
        u8g2_DrawStr(u8g2, 70, y, buf);
        u8g2_DrawStr(u8g2, 110, y, "°");
        y += 10;
        format_float(s_latest_data.roll, buf, 6, 2, 1);
        u8g2_DrawStr(u8g2, 8, y, "Roll:  ");
        u8g2_DrawStr(u8g2, 70, y, buf);
        u8g2_DrawStr(u8g2, 110, y, "°");
        y += 10;
        format_float(s_latest_data.yaw, buf, 6, 2, 0);
        u8g2_DrawStr(u8g2, 8, y, "Yaw:   ");
        u8g2_DrawStr(u8g2, 70, y, buf);
        u8g2_DrawStr(u8g2, 110, y, "°");
        y += 10;
        u8g2_DrawStr(u8g2, 8, y, "Status: Calibrated");
    }
}

static void render_fault_log_page(u8g2_t* u8g2)
{
    // Fault/error log display
    oled_draw_title_bar("Fault Log", RT_TRUE);
    
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
    u8g2_DrawStr(u8g2, 8, y, "No Active Faults");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "System OK");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Last Reset:");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Power-on");
}

static void render_settings_page(u8g2_t* u8g2)
{
    // System settings menu
    oled_draw_title_bar("Settings", RT_TRUE);
    
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
    u8g2_DrawStr(u8g2, 8, y, "Volume: Med");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Lang: EN");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "LED: 100%");
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Auto-off: 30min");
}


/**
 * ========== MSH Console Commands for Testing (v1.1.4 - CORRECT EXPORT ALIASES) ==========
 * 
 * IMPORTANT NOTE ON MSH_CMD_EXPORT_ALIAS MACRO USAGE:
 * MSH_CMD_EXPORT_ALIAS(function_name, command_name, description)
 *   - function_name = C function that MUST be defined in code
 *   - command_name = Name user types in msh console (can be different from function_name)
 *   - description = Help text displayed in 'help' command
 * 
 * Pattern: Define function FIRST, then add MSH_CMD_EXPORT_ALIAS macro
 */

/**
 * @brief MSH command implementation for oled_test
 * Usage: oled_test [page_id]
 *   page_id: 0=boot, 1=home, 2=pid, 3=ultrasonic, 4=ir, 5=battery, 
 *            6=water, 7=motor, 8=imu, 9=fault, 10=settings
 */
void oled_test(int argc, char *argv[])
{
    int page_id = -1;
    
    if (argc > 1)
    {
        page_id = strtol(argv[1], NULL, 10);
        if (page_id < 0 || page_id >= PAGE_COUNT)
        {
            rt_kprintf("[OLED] Invalid page ID! Use 0-%d\r\n", PAGE_COUNT - 1);
            return;
        }
    }
    
    rt_kprintf("[OLED] ===========================================\r\n");
    rt_kprintf("[OLED] OLED Display Control Test (v1.1.4)\r\n");
    rt_kprintf("[OLED] ===========================================\r\n");
    rt_kprintf("[OLED] Page count: %d\r\n", PAGE_COUNT);
    
    if (page_id >= 0)
    {
        rt_kprintf("[OLED] Switching to page %d...\r\n", page_id);
        
        switch (page_id)
        {
            case PAGE_BOOT:      rt_kprintf("[OLED] Page 0: Boot Logo + Progress Bar\r\n");       break;
            case PAGE_HOME:      rt_kprintf("[OLED] Page 1: Main Menu + Battery Info\r\n");   break;
            case PAGE_PID_TUNING: rt_kprintf("[OLED] Page 2: PID Tuning Interface\r\n");     break;
            case PAGE_ULTRASONIC: rt_kprintf("[OLED] Page 3: Ultrasonic Sensor Data\r\n");    break;
            case PAGE_IR_SENSOR: rt_kprintf("[OLED] Page 4: IR Cliff Sensors\r\n");          break;
            case PAGE_BATTERY_INFO: rt_kprintf("[OLED] Page 5: Detailed Battery Status\r\n"); break;
            case PAGE_WATER_LEVEL: rt_kprintf("[OLED] Page 6: Water Tank Level\r\n");         break;
            case PAGE_MOTOR_STATUS: rt_kprintf("[OLED] Page 7: Motor RPM and Steppers\r\n");  break;
            case PAGE_IMU_DATA:   rt_kprintf("[OLED] Page 8: IMU Pitch/Roll/Yaw\r\n");        break;
            case PAGE_FAULT_LOG:  rt_kprintf("[OLED] Page 9: System Fault Log\r\n");         break;
            case PAGE_SETTINGS:   rt_kprintf("[OLED] Page 10: System Settings Menu\r\n");     break;
            default:              rt_kprintf("[OLED] Unknown page!\r\n");                      break;
        }
        
        oled_switch_page((OledPageId_t)page_id);
        rt_kprintf("[OLED] Page switched to %d\r\n", page_id);
    }
    
    rt_kprintf("[OLED] ===========================================\r\n");
    rt_kprintf("[OLED] Available commands:\r\n");
    rt_kprintf("[OLED]   oled_test         - Show info and boot page\r\n");
    rt_kprintf("[OLED]   oled_test 1       - Switch to home page\r\n");
    rt_kprintf("[OLED]   oled_test 5       - Switch to battery page\r\n");
    rt_kprintf("[OLED]   oled_refresh      - Force refresh current page\r\n");
    rt_kprintf("[OLED]   oled_status       - Show OLED status\r\n");
    rt_kprintf("[OLED]   oled_cycle N      - Cycle through N pages\r\n");
    rt_kprintf("[OLED] ===========================================\r\n");
}
MSH_CMD_EXPORT_ALIAS(oled_test, oled_test, OLED Display Control Test (v1.1.4));

/**
 * @brief MSH command implementation for oled_refresh
 * Usage: oled_refresh
 */
void oled_refresh_cmd(int argc, char *argv[])
{
    rt_kprintf("[OLED] Forcing screen refresh...\r\n");
    oled_force_refresh();
    rt_kprintf("[OLED] Refresh complete!\r\n");
}
MSH_CMD_EXPORT_ALIAS(oled_refresh_cmd, oled_refresh, OLED Force Screen Refresh);

/**
 * @brief MSH command implementation for oled_status
 * Usage: oled_status
 */
void oled_status(int argc, char *argv[])
{
    OledPageId_t page = oled_get_current_page();
    rt_kprintf("[OLED] OLED Status Information\r\n");
    rt_kprintf("[OLED] =========================\r\n");
    rt_kprintf("[OLED] Current page: ");
    
    switch (page)
    {
        case PAGE_BOOT:      rt_kprintf("0 (Boot)\r\n");       break;
        case PAGE_HOME:      rt_kprintf("1 (Home)\r\n");      break;
        case PAGE_PID_TUNING:rt_kprintf("2 (PID Tuning)\r\n");break;
        case PAGE_ULTRASONIC:rt_kprintf("3 (Ultrasonic)\r\n");break;
        case PAGE_IR_SENSOR: rt_kprintf("4 (IR Sensors)\r\n");break;
        case PAGE_BATTERY_INFO: rt_kprintf("5 (Battery)\r\n");break;
        case PAGE_WATER_LEVEL: rt_kprintf("6 (Water Level)\r\n");break;
        case PAGE_MOTOR_STATUS: rt_kprintf("7 (Motor Status)\r\n");break;
        case PAGE_IMU_DATA:   rt_kprintf("8 (IMU Data)\r\n"); break;
        case PAGE_FAULT_LOG:  rt_kprintf("9 (Fault Log)\r\n");break;
        case PAGE_SETTINGS:   rt_kprintf("10 (Settings)\r\n");break;
        default:              rt_kprintf("Unknown (%d)\r\n", page); break;
    }
    
    rt_kprintf("[OLED] Total pages: %d\r\n", s_page_count);
    rt_kprintf("[OLED] Battery voltage: %d mV\r\n", g_oled_battery_mv);
    rt_kprintf("[OLED] OLED initialized: Yes\r\n");
}
MSH_CMD_EXPORT_ALIAS(oled_status, oled_status, OLED Show Status Information);

/**
 * @brief MSH command implementation for oled_cycle
 * Usage: oled_cycle [count]
 *   count: number of pages to cycle (default: 3)
 */
void oled_cycle(int argc, char *argv[])
{
    int cycles = 3;
    
    if (argc > 1)
    {
        cycles = strtol(argv[1], NULL, 10);
        if (cycles < 1) cycles = 1;
        if (cycles > PAGE_COUNT) cycles = PAGE_COUNT;
    }
    
    rt_kprintf("[OLED] Cycling through %d pages...\r\n", cycles);
    
    for (int i = 0; i < cycles; i++)
    {
        OledPageId_t next_page = (OledPageId_t)((s_current_page + 1) % PAGE_COUNT);
        rt_kprintf("[OLED] Switching from %d -> %d\r\n", s_current_page, next_page);
        oled_switch_page(next_page);
        rt_thread_mdelay(5000);  // Give OLED time to update
    }
    
    rt_kprintf("[OLED] Cycle complete!\r\n");
}
MSH_CMD_EXPORT_ALIAS(oled_cycle, oled_cycle, OLED Cycle Through Pages);
