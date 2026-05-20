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
//#ifdef ULTRASONIC_GPIO
    #include "ultrasonic_hc_sr04.h"
//#elif defined(ULTRASONIC_485)
    #include "ultrasonic_485.h"
//#endif
#include "zltech_can_motor.h"
#include "ir_receiver.h"
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

/* 上位机发送数据显示缓冲区（供 packet_handle 使用）*/
char oled_l1[21] = {0};
char oled_l2[21] = {0};
char oled_l3[21] = {0};
char oled_l4[21] = {0};

/**
 * ========== Internal Function Prototypes ==========
 */
static void oled_gpio_init(void);
static void u8g2_init(void);
static void render_boot_page(u8g2_t* u8g2);
static void render_home_page(u8g2_t* u8g2);
static void render_ultrasonic_page(u8g2_t* u8g2);
static void render_ultrasonic_485_page(u8g2_t* u8g2);
static void render_ir_sensor_page(u8g2_t* u8g2);
static void render_battery_info_page(u8g2_t* u8g2);
static void render_ir_align_page(u8g2_t *u8g2);
static void render_water_level_page(u8g2_t* u8g2);
static void render_imu_data_page(u8g2_t* u8g2);
static void render_fault_log_page(u8g2_t* u8g2);
static void render_settings_page(u8g2_t* u8g2);
static void render_zlac_monitor1_page(u8g2_t* u8g2);
static void render_zlac_monitor2_page(u8g2_t* u8g2);
static void render_zlac_vel_pid_page(u8g2_t* u8g2);
static void render_zlac_pos_pid_page(u8g2_t* u8g2);
static void render_zlac_motor_param_page(u8g2_t* u8g2);
static void render_zlac_config_page(u8g2_t* u8g2);
static void render_uart_data_page(u8g2_t* u8g2);

static void oled_update_task(void *parameter);
static void oled_key_task(void *parameter);
void oled_set_page_interval(OledPageId_t page_id, uint16_t interval_ms);
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
	
    oled_register_page(PAGE_BATTERY_INFO, "Battery Info", render_battery_info_page);
	  oled_register_page(PAGE_IMU_DATA, "IMU Data", render_imu_data_page);
    oled_register_page(PAGE_ULTRASONIC, "Ultrasonic sr04", render_ultrasonic_page);
    oled_register_page(PAGE_ULTRASONIC_485, "Ultrasonic 485", render_ultrasonic_485_page);	
    oled_register_page(PAGE_IR_SENSOR, "IR Sensors", render_ir_sensor_page);
		oled_register_page(PAGE_IR_ALIGN, "IR Alignment", render_ir_align_page);    
		oled_register_page(PAGE_WATER_LEVEL, "Water Level", render_water_level_page);

	    // 注册 ZLAC8015D 电机相关页面
    oled_register_page(PAGE_ZLAC_MONITOR1, "ZLtech Monitor1", render_zlac_monitor1_page);
    oled_register_page(PAGE_ZLAC_MONITOR2, "ZLtech Monitor2", render_zlac_monitor2_page);
    oled_register_page(PAGE_ZLAC_VEL_PID,  "ZLtech Vel PID",  render_zlac_vel_pid_page);
    oled_register_page(PAGE_ZLAC_POS_PID,  "ZLtech Pos PID",  render_zlac_pos_pid_page);
    oled_register_page(PAGE_ZLAC_MOTOR_PARAM, "ZLtech Motor", render_zlac_motor_param_page);
    oled_register_page(PAGE_ZLAC_CONFIG,   "ZLtech Config",  render_zlac_config_page);
	
    oled_register_page(PAGE_FAULT_LOG, "Fault Log", render_fault_log_page);
    oled_register_page(PAGE_SETTINGS, "Settings", render_settings_page);
		
		oled_register_page(PAGE_UART_DATA, "UART Data", render_uart_data_page);

    s_page_count = PAGE_COUNT;
		
		    // 为实时监控页面设置更快的刷新间隔（500ms）
    oled_set_page_interval(PAGE_ZLAC_MONITOR1, 500);
    oled_set_page_interval(PAGE_ZLAC_MONITOR2, 500);
    
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
    if (mv < BATTERY_NULL_VOLTAGE_MV) mv = BATTERY_NULL_VOLTAGE_MV;
    
    // 计算填充宽度（0~30像素）
    uint32_t range = BATTERY_FULL_VOLTAGE_MV - BATTERY_NULL_VOLTAGE_MV;
    uint32_t fill = (mv - BATTERY_NULL_VOLTAGE_MV) * 30 / range;
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

	//进度条 计算电量百分比 (Vmin=16.8V, Vmax=25.2V)
		int voltage = g_oled_battery_mv;
		if (voltage > BATTERY_FULL_VOLTAGE_MV) voltage = BATTERY_FULL_VOLTAGE_MV;
		if (voltage < BATTERY_NULL_VOLTAGE_MV) voltage = BATTERY_NULL_VOLTAGE_MV;
		uint8_t bat_percent = (voltage - BATTERY_NULL_VOLTAGE_MV) * 100 / 
													(BATTERY_FULL_VOLTAGE_MV - BATTERY_NULL_VOLTAGE_MV);
		if (bat_percent > 100) bat_percent = 100;
		draw_progress_bar(8, y, 112, 6, bat_percent);
    y += 15;
		rt_snprintf(bat_str, sizeof(bat_str), "Power: %dW", g_charge_power_mw/1000);
    u8g2_DrawStr(u8g2, 8, y, bat_str);
    y += 10;
    u8g2_DrawStr(u8g2, 8, y, "Mode: Standby");

}

static void render_ultrasonic_page(u8g2_t* u8g2)
{
    oled_draw_title_bar("Ultrasonic SR04", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
    char buf[24];   // 足够大

#ifdef ULTRASONIC_GPIO
    uint32_t dist[8] = {0};
    hc_sr04_get_distances(dist, 8);
    // 第一行
    u8g2_DrawStr(u8g2, 8, y, "F:");
    rt_snprintf(buf, sizeof(buf), dist[0] ? "%4dmm" : "---", dist[0]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    rt_snprintf(buf, sizeof(buf), dist[1] ? "%4dmm" : "---", dist[1]);
    u8g2_DrawStr(u8g2, 85, y, buf);
    y += 10;
    // 第二行
    u8g2_DrawStr(u8g2, 8, y, "TS:");
    rt_snprintf(buf, sizeof(buf), dist[2] ? "%4dmm" : "---", dist[2]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    rt_snprintf(buf, sizeof(buf), dist[3] ? "%4dmm" : "---", dist[3]);
    u8g2_DrawStr(u8g2, 85, y, buf);
    y += 10;
    // 第三行
    u8g2_DrawStr(u8g2, 8, y, "BS:");
    rt_snprintf(buf, sizeof(buf), dist[4] ? "%4dmm" : "---", dist[4]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    rt_snprintf(buf, sizeof(buf), dist[5] ? "%4dmm" : "---", dist[5]);
    u8g2_DrawStr(u8g2, 85, y, buf);
    y += 10;
    // 第四行
    u8g2_DrawStr(u8g2, 8, y, "R:");
    rt_snprintf(buf, sizeof(buf), dist[6] ? "%4dmm" : "---", dist[6]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    rt_snprintf(buf, sizeof(buf), dist[7] ? "%4dmm" : "---", dist[7]);
    u8g2_DrawStr(u8g2, 85, y, buf);

#elif defined(ULTRASONIC_485)
    uint32_t dist[7] = {0};
    ultrasonic_485_get_distances(dist, 7);
    // 第一行
    u8g2_DrawStr(u8g2, 8, y, "F:");
    rt_snprintf(buf, sizeof(buf), dist[0] ? "%4dmm" : "---", dist[0]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    rt_snprintf(buf, sizeof(buf), dist[1] ? "%4dmm" : "---", dist[1]);
    u8g2_DrawStr(u8g2, 85, y, buf);
    y += 10;
    // 第二行
    u8g2_DrawStr(u8g2, 8, y, "TS:");
    rt_snprintf(buf, sizeof(buf), dist[2] ? "%4dmm" : "---", dist[2]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    rt_snprintf(buf, sizeof(buf), dist[3] ? "%4dmm" : "---", dist[3]);
    u8g2_DrawStr(u8g2, 85, y, buf);
    y += 10;
    // 第三行
    u8g2_DrawStr(u8g2, 8, y, "BS:");
    rt_snprintf(buf, sizeof(buf), dist[4] ? "%4dmm" : "---", dist[4]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    rt_snprintf(buf, sizeof(buf), dist[5] ? "%4dmm" : "---", dist[5]);
    u8g2_DrawStr(u8g2, 85, y, buf);
    y += 10;
    // 第四行（后方只有中间一个）
    u8g2_DrawStr(u8g2, 8, y, "R:");
    rt_snprintf(buf, sizeof(buf), dist[6] ? "%4dmm" : "---", dist[6]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    u8g2_DrawStr(u8g2, 85, y, "---");   // 右列留空

#else
    u8g2_DrawStr(u8g2, 8, 30, "Ultrasonic: None");
#endif
}

static void render_ultrasonic_485_page(u8g2_t* u8g2)
{
    oled_draw_title_bar("Ultrasonic 485", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
    char buf[24];   // 足够大

    uint32_t dist[7] = {0};
    ultrasonic_485_get_distances(dist, 7);
    // 第一行
    u8g2_DrawStr(u8g2, 8, y, "F:");
    rt_snprintf(buf, sizeof(buf), dist[0] ? "%4dmm" : "---", dist[0]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    rt_snprintf(buf, sizeof(buf), dist[1] ? "%4dmm" : "---", dist[1]);
    u8g2_DrawStr(u8g2, 85, y, buf);
    y += 10;
    // 第二行
    u8g2_DrawStr(u8g2, 8, y, "TS:");
    rt_snprintf(buf, sizeof(buf), dist[2] ? "%4dmm" : "---", dist[2]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    rt_snprintf(buf, sizeof(buf), dist[3] ? "%4dmm" : "---", dist[3]);
    u8g2_DrawStr(u8g2, 85, y, buf);
    y += 10;
    // 第三行
    u8g2_DrawStr(u8g2, 8, y, "BS:");
    rt_snprintf(buf, sizeof(buf), dist[4] ? "%4dmm" : "---", dist[4]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    rt_snprintf(buf, sizeof(buf), dist[5] ? "%4dmm" : "---", dist[5]);
    u8g2_DrawStr(u8g2, 85, y, buf);
    y += 10;
    // 第四行（后方只有中间一个）
    u8g2_DrawStr(u8g2, 8, y, "R:");
    rt_snprintf(buf, sizeof(buf), dist[6] ? "%4dmm" : "---", dist[6]);
    u8g2_DrawStr(u8g2, 35, y, buf);
    u8g2_DrawStr(u8g2, 85, y, "---");   // 右列留空
}
// 距离转换表（电压 mV -> 距离 mm），按电压从高到低排列
typedef struct {
    uint16_t voltage_mv;
    uint16_t distance_mm;
} DistMap_t;

static const DistMap_t s_dist_map[] = {
    { 2500, 15 },   // 1.5cm 对应 2.5V，但实际1.5cm已是最小可靠距离，故取15mm
    { 2060, 20 },   // 2.0cm
    { 1050, 50 },   // 5.0cm
    { 596,  100 },  // 10.0cm
    { 400,  150 },  // 15.0cm
    { 300,  200 },  // 20.0cm
};
#define DIST_MAP_SIZE (sizeof(s_dist_map) / sizeof(s_dist_map[0]))

/**
 * @brief 将悬崖传感器电压转换为距离（毫米）
 * @param voltage_mv 传感器输出电压（mV）
 * @return 距离（mm），若超出范围则返回最大值200mm或最小值15mm
 */
static uint16_t cliff_voltage_to_distance(uint16_t voltage_mv)
{
    // 超出最大值（小于最小电压）返回最大距离
    if (voltage_mv <= s_dist_map[DIST_MAP_SIZE - 1].voltage_mv)
        return s_dist_map[DIST_MAP_SIZE - 1].distance_mm;
    // 超出最小值（大于最大电压）返回最小距离
    if (voltage_mv >= s_dist_map[0].voltage_mv)
        return s_dist_map[0].distance_mm;

    // 查找所在区间（电压递减）
    for (uint8_t i = 0; i < DIST_MAP_SIZE - 1; i++) {
        if (voltage_mv <= s_dist_map[i].voltage_mv && voltage_mv >= s_dist_map[i+1].voltage_mv) {
            // 线性插值
            uint16_t v_high = s_dist_map[i].voltage_mv;
            uint16_t v_low  = s_dist_map[i+1].voltage_mv;
            uint16_t d_high = s_dist_map[i].distance_mm;
            uint16_t d_low  = s_dist_map[i+1].distance_mm;
            // 距离 mm = d_high + (d_low - d_high) * (v_high - voltage_mv) / (v_high - v_low)
            uint32_t diff = (uint32_t)(d_low - d_high) * (v_high - voltage_mv) / (v_high - v_low);
            return d_high + diff;
        }
    }
    // fallback
    return s_dist_map[DIST_MAP_SIZE - 1].distance_mm;
}


static void render_ir_sensor_page(u8g2_t* u8g2)
{
    oled_draw_title_bar("IR Sensors", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
    char buf[24];
    uint16_t front_mv, rear_mv;
    rt_bool_t cliff_trigger;

    // 读取悬崖传感器数据
    read_cliff_sensor(&front_mv, &rear_mv, &cliff_trigger);

		// 转换为距离（毫米）
    uint16_t front_mm = cliff_voltage_to_distance(front_mv);
    uint16_t rear_mm  = cliff_voltage_to_distance(rear_mv);

    // 显示前悬崖距离
    rt_snprintf(buf, sizeof(buf), "Front: %3d mm", front_mm);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    // 显示后悬崖距离
    rt_snprintf(buf, sizeof(buf), "Rear:  %3d mm", rear_mm);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    u8g2_DrawHLine(u8g2, 8, y, 112);
    y += 8;
    // 显示悬崖触发状态
    rt_snprintf(buf, sizeof(buf), "Cliff: %s", cliff_trigger ? "WARNING" : "OK");
    u8g2_DrawStr(u8g2, 8, y, buf);
}

// 充电座红外接收管
static void render_ir_align_page(u8g2_t *u8g2)
{
    uint8_t left_status = ir_get_left_status();
    uint8_t right_status = ir_get_right_status();
    uint8_t left_up   = (left_status >> 2) & 1;
    uint8_t left_left = (left_status >> 1) & 1;
    uint8_t left_right = left_status & 1;

    uint8_t right_up   = (right_status >> 2) & 1;
    uint8_t right_left = (right_status >> 1) & 1;
    uint8_t right_right = right_status & 1;

    oled_draw_title_bar("IRM-8601M2", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
    uint8_t line_height = 10;

    // 第一行：Left:
    u8g2_DrawStr(u8g2, 8, y, "Left:");
    // 第二行：U:x L:x R:x
    char buf[24];
    rt_snprintf(buf, sizeof(buf), "U:%d L:%d R:%d", left_up, left_left, left_right);
    u8g2_DrawStr(u8g2, 8, y + line_height, buf);
    // 第三行：Right:
    u8g2_DrawStr(u8g2, 8, y + 2 * line_height, "Right:");
    // 第四行：U:x L:x R:x
    rt_snprintf(buf, sizeof(buf), "U:%d L:%d R:%d", right_up, right_left, right_right);
    u8g2_DrawStr(u8g2, 8, y + 3 * line_height, buf);
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
		int voltage = g_oled_battery_mv;
		if (voltage > BATTERY_FULL_VOLTAGE_MV) voltage = BATTERY_FULL_VOLTAGE_MV;
		if (voltage < BATTERY_NULL_VOLTAGE_MV) voltage = BATTERY_NULL_VOLTAGE_MV;
		int percent = (voltage - BATTERY_NULL_VOLTAGE_MV) * 100 / (BATTERY_FULL_VOLTAGE_MV - BATTERY_NULL_VOLTAGE_MV);
//    uint8_t percent = (g_oled_battery_mv - BATTERY_LOW_VOLTAGE_MV) * 100 /
//                      (BATTERY_FULL_VOLTAGE_MV - BATTERY_LOW_VOLTAGE_MV);
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
	
		rt_uint32_t heater_mv = monitor_get_heater_voltage();
    if (rt_pin_read(HEATER_CTRL_PIN) == PIN_HIGH && heater_mv > 5000) {
        u8g2_DrawStr(u8g2, 8, y, "Heater: Active");
    } else if(rt_pin_read(HEATER_CTRL_PIN) == PIN_LOW && heater_mv > 5000){
        u8g2_DrawStr(u8g2, 8, y, "Heater: Idle");
    }else{
        u8g2_DrawStr(u8g2, 8, y, "Heater: Disconnected");
    }
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
		// 第二行：高水位状态
    if (monitor_get_water_high_level()) {
        u8g2_DrawStr(u8g2, 8, y, "High Water: FULL");
    } else {
        u8g2_DrawStr(u8g2, 8, y, "High Water: ---");
    }
    y += 10;
    u8g2_DrawHLine(u8g2, 8, y, 112);
    y += 8;	
		// 第三行：水温值
    int16_t water_temp = wc_get_water_temperature();
    char buf[24];
    if (water_temp != -32768) {
        rt_snprintf(buf, sizeof(buf), "WaterTemp:%d.%d C", water_temp / 10, water_temp % 10);
    } else {
        rt_snprintf(buf, sizeof(buf), "WaterTemp: ERR");
    }
    u8g2_DrawStr(u8g2, 8, y, buf);
	
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

//1. 实时动态数据页面1（速度、位置、温度、运行状态）
static void render_zlac_monitor1_page(u8g2_t* u8g2)
{
    oled_draw_title_bar("ZLtech Monitor1", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    
    int16_t vel_l, vel_r;
    int32_t pos_l, pos_r;
    int16_t temp_l, temp_r, temp_d;
    uint16_t status_l = zlac_get_motor_status_left();
    uint16_t status_r = zlac_get_motor_status_right();
    
    zlac_get_velocity(&vel_l, &vel_r);
    zlac_get_position(&pos_l, &pos_r);
    temp_l = zlac_get_motor_temp_left();
    temp_r = zlac_get_motor_temp_right();
    temp_d = zlac_get_driver_temp();
    
    char buf[32];
    uint8_t y = 28;
    
    // 速度（使用 format_float）
    format_float(vel_l / 10.0f, buf, 5, 1, 0);
    u8g2_DrawStr(u8g2, 8, y, "Vel L:");
    u8g2_DrawStr(u8g2, 45, y, buf);
    format_float(vel_r / 10.0f, buf, 5, 1, 0);
    u8g2_DrawStr(u8g2, 75, y, " R:");
    u8g2_DrawStr(u8g2, 90, y, buf);
    y += 9;
    
    // 位置：左电机一行
    rt_snprintf(buf, sizeof(buf), "Pos L: %10ld", pos_l);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 9;
    // 右电机一行
    rt_snprintf(buf, sizeof(buf), "Pos R: %10ld", pos_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 9;
    
    // 温度：合并左右温度，运行状态并入此行
    format_float(temp_l / 10.0f, buf, 5, 1, 0);
    u8g2_DrawStr(u8g2, 8, y, "Tmp L:");
    u8g2_DrawStr(u8g2, 45, y, buf);
    format_float(temp_r / 10.0f, buf, 5, 1, 0);
    u8g2_DrawStr(u8g2, 75, y, " R:");
    u8g2_DrawStr(u8g2, 90, y, buf);
    y += 9;
    
    // 电机运行状态
    rt_snprintf(buf, sizeof(buf), "Run: L=%s R=%s", status_l ? "RUN" : "STOP", status_r ? "RUN" : "STOP");
    u8g2_DrawStr(u8g2, 8, y, buf);
}

//2. 实时动态数据页面2（在线、抱闸、状态字、故障码）
static void render_zlac_monitor2_page(u8g2_t* u8g2)
{
    oled_draw_title_bar("ZLtech Monitor2", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    
    rt_bool_t online = zlac_is_online();
    uint16_t brake_left, brake_right;
    zlac_get_brake(&brake_left, &brake_right);
    uint32_t full_status = read_full_statusword();
    uint32_t fault = zlac_get_fault_code();
    
    char buf[32];
    uint8_t y = 28;
    // 在线状态
    u8g2_DrawStr(u8g2, 8, y, online ? "Online: Yes" : "Online: No");
    y += 10;
    // 抱闸状态
    rt_snprintf(buf, sizeof(buf), "Brake:L=%s R=%s", brake_left ? "Eng" : "Rel", brake_right ? "Eng" : "Rel");
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    // 全状态字
    rt_snprintf(buf, sizeof(buf), "Status:0x%08X", full_status);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    // 故障码（只显示低16位或简化显示）
    if (fault)
        rt_snprintf(buf, sizeof(buf), "Fault: 0x%08X", fault);
    else
        rt_snprintf(buf, sizeof(buf), "Fault: None");
    u8g2_DrawStr(u8g2, 8, y, buf);
}

//3. 速度环 PID 页面
static void render_zlac_vel_pid_page(u8g2_t* u8g2)
{
    oled_draw_title_bar("ZLtech Vel PID", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    
    uint16_t kp_l, kp_r, ki_l, ki_r, kf_l, kf_r;
    uint16_t smooth_l, smooth_r;
    zlac_get_velocity_pid_kp(&kp_l, &kp_r);
    zlac_get_velocity_pid_ki(&ki_l, &ki_r);
    zlac_get_velocity_pid_kf(&kf_l, &kf_r);
    zlac_get_vel_smooth(&smooth_l, &smooth_r);
    
    char buf[32];
    uint8_t y = 28;
    rt_snprintf(buf, sizeof(buf), "Kp: %4d %4d", kp_l, kp_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    rt_snprintf(buf, sizeof(buf), "Ki: %4d %4d", ki_l, ki_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    rt_snprintf(buf, sizeof(buf), "Kf: %4d %4d", kf_l, kf_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    rt_snprintf(buf, sizeof(buf), "Smooth:%4d %4d", smooth_l, smooth_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
}

//4. 位置环 PID 页面（Kp, Kf）
static void render_zlac_pos_pid_page(u8g2_t* u8g2)
{
    uint16_t ff_smooth_l, ff_smooth_r; 
		oled_draw_title_bar("ZLtech Pos PID", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    
    uint16_t kp_l, kp_r, kf_l, kf_r;
    zlac_get_position_kp(&kp_l, &kp_r);
    zlac_get_position_kf(&kf_l, &kf_r);
    
		zlac_get_ff_smooth(&ff_smooth_l, &ff_smooth_r);   
    
		char buf[32];
    uint8_t y = 28;
    rt_snprintf(buf, sizeof(buf), "Kp: %4d %4d", kp_l, kp_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    rt_snprintf(buf, sizeof(buf), "Kf: %4d %4d", kf_l, kf_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
	  y += 10;
    rt_snprintf(buf, sizeof(buf), "FF Sm:%4d %4d", ff_smooth_l, ff_smooth_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
}

//5. 电流环 PID 及平滑系数页面  删除

//6. 电机基本参数页面
static void render_zlac_motor_param_page(u8g2_t* u8g2)
{
    oled_draw_title_bar("ZLtech Motor", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    
    uint16_t encoder_l, encoder_r;
    uint16_t poles_l, poles_r;
    uint16_t peak_l, peak_r;
    uint16_t max_speed;
    zlac_get_encoder_lines(&encoder_l, &encoder_r);
    zlac_get_motor_poles(&poles_l, &poles_r);
    zlac_get_current_peak(&peak_l, &peak_r);
    max_speed = zlac_get_max_speed();
    
    char buf[32];
    uint8_t y = 28;
    rt_snprintf(buf, sizeof(buf), "Encoder:%4d %4d", encoder_l, encoder_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    rt_snprintf(buf, sizeof(buf), "Poles:  %4d %4d", poles_l, poles_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    rt_snprintf(buf, sizeof(buf), "Peak A: %4d %4d", peak_l/10, peak_r/10); // 单位0.1A -> A
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    rt_snprintf(buf, sizeof(buf), "Max rpm: %4d", max_speed);
    u8g2_DrawStr(u8g2, 8, y, buf);
}

//7. 设置参数页面（初始方向、工作模式、加速减速时间
static void render_zlac_config_page(u8g2_t* u8g2)
{
    oled_draw_title_bar("ZLtech Config", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    
    uint16_t init_dir = zlac_get_init_direction();
    uint16_t op_mode = zlac_get_op_mode();
    uint32_t accel_l, accel_r, decel_l, decel_r;
    zlac_get_accel_time(&accel_l, &accel_r);
    zlac_get_decel_time(&decel_l, &decel_r);
    const char* mode_str = (op_mode == 1) ? "Pos" : ((op_mode == 3) ? "Vel" : ((op_mode == 4) ? "Torq" : "Unk"));
    if(op_mode == 1){
			ZlacPositionMode_t pos = zlac_get_position_mode();
			mode_str = (pos == ZLAC_POS_MODE_ABSOLUTE) ? "Abs Pos" : "Rel Pos";
    }
		char buf[32];
    uint8_t y = 28;
    rt_snprintf(buf, sizeof(buf), "Dir: %s", init_dir ? "CCW" : "CW");
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    rt_snprintf(buf, sizeof(buf), "Mode: %s", mode_str);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    rt_snprintf(buf, sizeof(buf), "Acc: %4ld %4ld ms", accel_l, accel_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
    y += 10;
    rt_snprintf(buf, sizeof(buf), "Dec: %4ld %4ld ms", decel_l, decel_r);
    u8g2_DrawStr(u8g2, 8, y, buf);
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

// 上位机数据显示
static void render_uart_data_page(u8g2_t* u8g2)
{
    oled_draw_title_bar("UART Data", RT_TRUE);
    u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf);
    uint8_t y = 30;
    uint8_t line_height = 10;

    // 显示第 1 行
    if (oled_l1[0] != '\0')
        u8g2_DrawStr(u8g2, 8, y, oled_l1);
    else
        u8g2_DrawStr(u8g2, 8, y, "---");

    // 第 2 行
    if (oled_l2[0] != '\0')
        u8g2_DrawStr(u8g2, 8, y + line_height, oled_l2);
    else
        u8g2_DrawStr(u8g2, 8, y + line_height, "---");

    // 第 3 行（如果定义了子命令 0x03）
    if (oled_l3[0] != '\0')
        u8g2_DrawStr(u8g2, 8, y + line_height * 2, oled_l3);
    // 第 4 行
    if (oled_l4[0] != '\0')
        u8g2_DrawStr(u8g2, 8, y + line_height * 3, oled_l4);
}
// 方便 packet_handle 更新任意行
void oled_set_uart_line(uint8_t line, const char* text)
{
    char* target = NULL;
    switch (line) {
        case 1: target = oled_l1; break;
        case 2: target = oled_l2; break;
        case 3: target = oled_l3; break;
        case 4: target = oled_l4; break;
        default: return;
    }
    strncpy(target, text, 20);
    target[20] = '\0';
    oled_trigger_refresh();  // 立即刷新
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
            case PAGE_BOOT:      rt_kprintf("[OLED] Page %d: Boot Logo + Progress Bar\r\n",PAGE_BOOT);       break;
            case PAGE_HOME:      rt_kprintf("[OLED] Page %d: Main Menu + Battery Info\r\n",PAGE_HOME);   break;
            case PAGE_BATTERY_INFO: rt_kprintf("[OLED] Page %d: Detailed Battery Status\r\n",PAGE_BATTERY_INFO); break;
            case PAGE_IMU_DATA:   rt_kprintf("[OLED] Page %d: IMU Pitch/Roll/Yaw\r\n",PAGE_IMU_DATA);        break;
						case PAGE_ULTRASONIC: rt_kprintf("[OLED] Page %d: Ultrasonic sr04 Sensor Data\r\n",PAGE_ULTRASONIC);    break;
						case PAGE_ULTRASONIC_485: rt_kprintf("[OLED] Page %d: Ultrasonic 485 Sensor Data\r\n",PAGE_ULTRASONIC_485);    break;
					case PAGE_IR_SENSOR: rt_kprintf("[OLED] Page %d: IR Cliff Sensors\r\n",PAGE_IR_SENSOR);          break;

            case PAGE_WATER_LEVEL: rt_kprintf("[OLED] Page %d: Water Tank Level\r\n",PAGE_WATER_LEVEL);         break;

            case PAGE_ZLAC_MONITOR1: rt_kprintf("[OLED] Page %d: ZLtech Monitor1\r\n",PAGE_ZLAC_MONITOR1);         break;
            case PAGE_ZLAC_MONITOR2: rt_kprintf("[OLED] Page %d: ZLtech Monitor2\r\n",PAGE_ZLAC_MONITOR2);         break;
            case PAGE_ZLAC_VEL_PID: rt_kprintf("[OLED] Page %d: ZLtech Vel PID\r\n",PAGE_ZLAC_VEL_PID);         break;
            case PAGE_ZLAC_POS_PID: rt_kprintf("[OLED] Page %d: ZLtech Pos PID\r\n",PAGE_ZLAC_POS_PID);         break;
            case PAGE_ZLAC_MOTOR_PARAM: rt_kprintf("[OLED] Page %d: ZLtech Motor param\r\n",PAGE_ZLAC_MOTOR_PARAM);         break;
            case PAGE_ZLAC_CONFIG: rt_kprintf("[OLED] Page %d: ZLtech Other Config\r\n",PAGE_ZLAC_CONFIG);         break;

						case PAGE_FAULT_LOG:  rt_kprintf("[OLED] Page %d: System Fault Log\r\n",PAGE_FAULT_LOG);         break;
            case PAGE_SETTINGS:   rt_kprintf("[OLED] Page %d: System Settings Menu\r\n",PAGE_FAULT_LOG);     break;
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
				case PAGE_BOOT:      rt_kprintf("[OLED] Page %d: Boot Logo + Progress Bar\r\n",PAGE_BOOT);       break;
				case PAGE_HOME:      rt_kprintf("[OLED] Page %d: Main Menu + Battery Info\r\n",PAGE_HOME);   break;
				case PAGE_BATTERY_INFO: rt_kprintf("[OLED] Page %d: Detailed Battery Status\r\n",PAGE_BATTERY_INFO); break;
				case PAGE_IMU_DATA:   rt_kprintf("[OLED] Page %d: IMU Pitch/Roll/Yaw\r\n",PAGE_IMU_DATA);        break;
				case PAGE_ULTRASONIC: rt_kprintf("[OLED] Page %d: Ultrasonic sr04 Sensor Data\r\n",PAGE_ULTRASONIC);    break;
				case PAGE_ULTRASONIC_485: rt_kprintf("[OLED] Page %d: Ultrasonic 485 Sensor Data\r\n",PAGE_ULTRASONIC_485);    break;
			case PAGE_IR_SENSOR: rt_kprintf("[OLED] Page %d: IR Cliff Sensors\r\n",PAGE_IR_SENSOR);          break;

				case PAGE_WATER_LEVEL: rt_kprintf("[OLED] Page %d: Water Tank Level\r\n",PAGE_WATER_LEVEL);         break;

				case PAGE_ZLAC_MONITOR1: rt_kprintf("[OLED] Page %d: ZLtech Monitor1\r\n",PAGE_ZLAC_MONITOR1);         break;
				case PAGE_ZLAC_MONITOR2: rt_kprintf("[OLED] Page %d: ZLtech Monitor2\r\n",PAGE_ZLAC_MONITOR2);         break;
				case PAGE_ZLAC_VEL_PID: rt_kprintf("[OLED] Page %d: ZLtech Vel PID\r\n",PAGE_ZLAC_VEL_PID);         break;
				case PAGE_ZLAC_POS_PID: rt_kprintf("[OLED] Page %d: ZLtech Pos PID\r\n",PAGE_ZLAC_POS_PID);         break;
				case PAGE_ZLAC_MOTOR_PARAM: rt_kprintf("[OLED] Page %d: ZLtech Motor param\r\n",PAGE_ZLAC_MOTOR_PARAM);         break;
				case PAGE_ZLAC_CONFIG: rt_kprintf("[OLED] Page %d: ZLtech Other Config\r\n",PAGE_ZLAC_CONFIG);         break;

				case PAGE_FAULT_LOG:  rt_kprintf("[OLED] Page %d: System Fault Log\r\n",PAGE_FAULT_LOG);         break;
				case PAGE_SETTINGS:   rt_kprintf("[OLED] Page %d: System Settings Menu\r\n",PAGE_FAULT_LOG);     break;

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
