/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief OLED 显示句柄实现 v1.0.9
 * 
 * 功能说明：
 *   1. 使用 u8g2 库驱动 SSD1306 OLED
 *   2. 多页面管理和切换（11 个页面）
 *   3. SW3/SW4 轮询检测（非中断方式）
 *   4. 与 LED 共享 SPI 资源的互斥保护
 */

#include <rtthread.h>
#include <u8g2_port.h>
#include "oled_handle.h"
#include "global_conf.h"


/**
 * ========== 全局变量 ==========
 */

/**
 * @brief 电池电量全局变量（与 u8g2 示例程序共享）
 */
int g_oled_battery_mv = 24000;

/**
 * @brief U8g2 显示对象
 */
static u8g2_t s_u8g2;

/**
 * @brief SPI 资源互斥锁 - 解决 OLED 和 LED 的 SPI 冲突
 */
static struct rt_mutex s_spi_mutex;

/**
 * @brief 刷新信号量 - 供其他任务触发刷新
 */
static struct rt_semaphore s_refresh_sem;

/**
 * @brief 当前页面索引
 */
static OledPageId_t s_current_page = PAGE_BOOT;

/**
 * @brief 页面数组
 */
static OledPage_t s_pages[PAGE_COUNT];
static rt_uint8_t s_page_count = 0;

/**
 * @brief OLED 任务句柄
 */
static rt_thread_t s_oled_thread = RT_NULL;

/**
 * @brief 是否已切换到主页
 */
static rt_bool_t s_has_switched_to_home = RT_FALSE;


/* ========== 内部函数声明 ========== */

static void oled_thread_entry(void* parameter);
static void oled_render_current_page(void);
static void oled_check_page_buttons(void);
static void oled_update_battery(void);


/* ========== IMU 姿态数据显示 ========== */
#ifdef PERIPHERALS_QMI8658_H__
extern int qmi8658_read_once(void* raw_out, void* decoded_out);
#endif


/* ========== 超声波传感器数据显示 ========== */
#ifdef PERIPHERALS_RS485_ULTRASONIC_H__
extern uint16_t rs485_us_read_distance(uint8_t slave_addr, uint8_t channel_idx);
extern rt_bool_t rs485_us_ping(uint8_t slave_addr);
#endif


/* ========== 页面渲染函数声明 ========== */

static void render_boot_page(void);
static void render_home_page(void);
static void render_pid_tuning_page(void);
static void render_ultrasonic_page(void);
static void render_ir_sensor_page(void);
static void render_battery_info_page(void);
static void render_water_level_page(void);
static void render_motor_status_page(void);
static void render_imu_data_page(void);
static void render_fault_log_page(void);
static void render_settings_page(void);


/* ========== 公共函数实现 ========== */

void oled_handle_init(void)
{
    rt_mutex_init(&s_spi_mutex, "spi", RT_IPC_FLAG_PRIORITY);
    rt_sem_init(&s_refresh_sem, "ref", 0, RT_IPC_FLAG_PRIO);
    
    oled_register_page(PAGE_BOOT, "启动中...", render_boot_page);
    oled_register_page(PAGE_HOME, "主界面", render_home_page);
    oled_register_page(PAGE_PID_TUNING, "PID 调参", render_pid_tuning_page);
    oled_register_page(PAGE_ULTRASONIC, "超声波", render_ultrasonic_page);
    oled_register_page(PAGE_IR_SENSOR, "红外传感器", render_ir_sensor_page);
    oled_register_page(PAGE_BATTERY_INFO, "电池信息", render_battery_info_page);
    oled_register_page(PAGE_WATER_LEVEL, "水位信息", render_water_level_page);
    oled_register_page(PAGE_MOTOR_STATUS, "电机状态", render_motor_status_page);
    oled_register_page(PAGE_IMU_DATA, "IMU 姿态", render_imu_data_page);
    oled_register_page(PAGE_FAULT_LOG, "故障日志", render_fault_log_page);
    oled_register_page(PAGE_SETTINGS, "系统设置", render_settings_page);
    
    s_page_count = PAGE_COUNT;
    
    s_oled_thread = rt_thread_create("oled",
                                      oled_thread_entry,
                                      RT_NULL,
                                      2048,
                                      RT_THREAD_PRIORITY_MAX / 3,
                                      20);
                                      
    if (s_oled_thread != RT_NULL)
    {
        rt_thread_startup(s_oled_thread);
        rt_kprintf("[OLED] Display system initialized with %d pages\n", PAGE_COUNT);
    }
    else
    {
        rt_kprintf("[OLED] Failed to create thread!\n");
    }
}


void oled_spi_lock(void)
{
    rt_mutex_fetch(&s_spi_mutex, RT_WAITING_FOREVER);
}


void oled_spi_unlock(void)
{
    rt_mutex_post(&s_spi_mutex);
}


void oled_trigger_refresh(void)
{
    rt_sem_release(&s_refresh_sem);
}


void oled_force_refresh(void)
{
    oled_render_current_page();
}


int oled_register_page(OledPageId_t page_id, const char* title, void (*render_func)(void))
{
    if (page_id >= PAGE_COUNT || page_id < 0) return -RT_ERROR;
    
    s_pages[page_id].id = page_id;
    s_pages[page_id].title = title;
    s_pages[page_id].render_func = render_func;
    s_pages[page_id].refresh_interval_ms = 1000;
    
    return 0;
}


int oled_switch_page(OledPageId_t page_id)
{
    if (page_id >= PAGE_COUNT || page_id < 0) return -RT_ERROR;
    
    s_current_page = page_id;
    
    if (s_current_page == PAGE_HOME && !s_has_switched_to_home)
    {
        s_has_switched_to_home = RT_TRUE;
        rt_kprintf("[OLED] Switched to HOME page\n");
    }
    
    oled_force_refresh();
    return 0;
}


void oled_prev_page(void)
{
    if (s_current_page > 0)
    {
        s_current_page--;
        oled_force_refresh();
    }
}


void oled_next_page(void)
{
    if (s_current_page < PAGE_COUNT - 1)
    {
        s_current_page++;
        oled_force_refresh();
    }
}


OledPageId_t oled_get_current_page(void)
{
    return s_current_page;
}


void oled_set_auto_refresh(OledPageId_t page_id, rt_uint32_t interval_ms)
{
    if (page_id < PAGE_COUNT)
    {
        s_pages[page_id].refresh_interval_ms = interval_ms;
    }
}


/* ========== UI 组件绘制接口 ========== */

void oled_draw_title_bar(const char* title, rt_bool_t show_page_indicator)
{
    oled_spi_lock();
    
    u8g2_ClearBuffer(&s_u8g2);
    u8g2_DrawBox(&s_u8g2, 0, 0, OLED_WIDTH, 11);
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB10_tr);
    u8g2_SetDrawColor(&s_u8g2, 1);
    
    int16_t title_width = u8g2_GetStrWidth(&s_u8g2, title);
    uint8_t x = (OLED_WIDTH - title_width) / 2;
    
    u8g2_DrawStr(&s_u8g2, x, 9, title);
    
    if (show_page_indicator && s_page_count > 1)
    {
        char buf[8];
        snprintf(buf, sizeof(buf), "[%d/%d]", s_current_page + 1, s_page_count);
        
        u8g2_SetFont(&s_u8g2, u8g2_font_unifont_t_symbols);
        u8g2_DrawStr(&s_u8g2, OLED_WIDTH - 24, 9, buf);
    }
    
    u8g2_SendBuffer(&s_u8g2);
    oled_spi_unlock();
}


void oled_draw_progress(rt_uint8_t x, rt_uint8_t y, rt_uint8_t width, 
                        rt_uint8_t height, rt_uint8_t progress, rt_bool_t is_good)
{
    if (progress > 100) progress = 100;
    
    oled_spi_lock();
    
    u8g2_DrawFrame(&s_u8g2, x, y, width, height);
    
    if (progress > 0)
    {
        uint8_t fill_width = (width * progress) / 100;
        u8g2_SetDrawColor(&s_u8g2, 1);
        u8g2_DrawBox(&s_u8g2, x, y, fill_width, height);
    }
    
    u8g2_SendBuffer(&s_u8g2);
    oled_spi_unlock();
}


void oled_draw_value_box(rt_uint8_t x, rt_uint8_t y, int32_t value, 
                         const char* unit, rt_uint8_t precision)
{
    oled_spi_lock();
    
    char buf[16];
    
    if (precision == 0)
    {
        snprintf(buf, sizeof(buf), "%ld%s", value, unit);
    }
    else if (precision == 1)
    {
        snprintf(buf, sizeof(buf), "%ld.%d%s", value / 10, value % 10, unit);
    }
    else if (precision == 2)
    {
        snprintf(buf, sizeof(buf), "%ld.%02d%s", value / 100, value % 100, unit);
    }
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&s_u8g2, x, y + 12, buf);
    
    u8g2_SendBuffer(&s_u8g2);
    oled_spi_unlock();
}


void oled_draw_status_dot(rt_uint8_t x, rt_uint8_t y, rt_uint8_t radius, rt_bool_t active)
{
    oled_spi_lock();
    
    if (active)
    {
        u8g2_SetDrawColor(&s_u8g2, 0);
        u8g2_DrawCircle(&s_u8g2, x, y, radius);
        u8g2_SetDrawColor(&s_u8g2, 1);
    }
    else
    {
        u8g2_SetDrawColor(&s_u8g2, 0);
        u8g2_DrawCircle(&s_u8g2, x, y, radius);
        u8g2_SetDrawColor(&s_u8g2, 1);
    }
    
    u8g2_SendBuffer(&s_u8g2);
    oled_spi_unlock();
}


void oled_draw_icon(rt_uint8_t x, rt_uint8_t y, rt_uint8_t icon_id)
{
    oled_spi_lock();
    
    switch (icon_id)
    {
        case 0:
            u8g2_SetFont(&s_u8g2, u8g2_font_unifont_t_symbols);
            u8g2_DrawGlyph(&s_u8g2, x, y + 14, 0x26A1);
            break;
        case 1:
            u8g2_SetFont(&s_u8g2, u8g2_font_unifont_t_symbols);
            u8g2_DrawGlyph(&s_u8g2, x, y + 14, 0x260B);
            break;
        case 2:
            u8g2_SetFont(&s_u8g2, u8g2_font_unifont_t_symbols);
            u8g2_DrawGlyph(&s_u8g2, x, y + 14, 0x26A0);
            break;
        default:
            u8g2_SetFont(&s_u8g2, u8g2_font_unifont_t_symbols);
            u8g2_DrawGlyph(&s_u8g2, x, y + 14, '?');
            break;
    }
    
    u8g2_SendBuffer(&s_u8g2);
    oled_spi_unlock();
}


/* ========== OLED 线程入口函数 ========== */

static void oled_thread_entry(void* parameter)
{
    rt_uint32_t last_ref_time = 0;
    rt_uint32_t last_btn_check_time = 0;
    rt_uint32_t last_battery_update_time = 0;
    
    u8g2_Setup_ssd1306_128x64_noname_f(&s_u8g2, U8G2_R0, 
                                         u8x8_byte_rtthread_4wire_hw_spi, 
                                         u8x8_gpio_and_delay_rtthread);
                                         
    u8x8_SetPin(u8g2_GetU8x8(&s_u8g2), U8X8_PIN_CS, GET_PIN(B, 12));
    u8x8_SetPin(u8g2_GetU8x8(&s_u8g2), U8X8_PIN_DC, GET_PIN(B, 14));
    u8x8_SetPin(u8g2_GetU8x8(&s_u8g2), U8X8_PIN_RESET, U8X8_PIN_NONE);
    
    rt_kprintf("OLED display\r\n");
    
    u8g2_InitDisplay(&s_u8g2);
    rt_thread_mdelay(10);  
    u8g2_InitDisplay(&s_u8g2);
    u8g2_SetPowerSave(&s_u8g2, 0);
    
    while (1)
    {
        rt_uint32_t current_time = rt_tick_get_millisecond();
        
        /* 1. 检查是否收到刷新信号量 */
        if (rt_sem_take(&s_refresh_sem, 0) == RT_EOK)
        {
            oled_render_current_page();
            last_ref_time = current_time;
        }
        
        /* 2. 按钮轮询检测（每 200ms 检查一次）*/
        if ((current_time - last_btn_check_time) > 200)
        {
            oled_check_page_buttons();
            last_btn_check_time = current_time;
        }
        
        /* 3. 自动刷新 */
        if (s_pages[s_current_page].refresh_interval_ms > 0)
        {
            if ((current_time - last_ref_time) >= s_pages[s_current_page].refresh_interval_ms)
            {
                oled_render_current_page();
                last_ref_time = current_time;
            }
        }
        
        /* 4. 电池电压更新（每秒读取一次）*/
        if ((current_time - last_battery_update_time) > 1000)
        {
            oled_update_battery();
            last_battery_update_time = current_time;
        }
        
        rt_thread_mdelay(10);
    }
}


static void oled_render_current_page(void)
{
    oled_spi_lock();
    u8g2_ClearBuffer(&s_u8g2);
    
    if (s_pages[s_current_page].render_func != RT_NULL)
    {
        s_pages[s_current_page].render_func();
    }
    
    u8g2_SendBuffer(&s_u8g2);
    oled_spi_unlock();
}


static void oled_check_page_buttons(void)
{
    rt_pin_t sw3_pin = GET_PIN(C, 13);
    if (rt_pin_read(sw3_pin) == PIN_HIGH)
    {
        rt_thread_mdelay(50);
        if (rt_pin_read(sw3_pin) == PIN_HIGH)
        {
            oled_prev_page();
            while (rt_pin_read(sw3_pin) == PIN_HIGH) rt_thread_mdelay(10);
        }
    }
    
    rt_pin_t sw4_pin = GET_PIN(C, 14);
    if (rt_pin_read(sw4_pin) == PIN_HIGH)
    {
        rt_thread_mdelay(50);
        if (rt_pin_read(sw4_pin) == PIN_HIGH)
        {
            oled_next_page();
            while (rt_pin_read(sw4_pin) == PIN_HIGH) rt_thread_mdelay(10);
        }
    }
}


static void oled_update_battery(void)
{
    extern int g_oled_battery_mv;
    g_oled_battery_mv -= 100;
    if (g_oled_battery_mv < 20000) g_oled_battery_mv = 25200;
}


/* ========== 页面渲染函数实现 ========== */

static void render_boot_page(void)
{
    u8g2_SetDrawColor(&s_u8g2, 1);
    u8g2_DrawBox(&s_u8g2, 0, 0, OLED_WIDTH, 11);
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB10_tr);
    u8g2_DrawStr(&s_u8g2, 30, 9, "iHomeRobot");
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&s_u8g2, 20, 30, "iBed-body V1.0.9");
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&s_u8g2, 40, 50, "Initializing...");
    
    u8g2_SendBuffer(&s_u8g2);
}


static void render_home_page(void)
{
    oled_draw_title_bar("主界面", RT_TRUE);
    
    rt_uint8_t y = 20;
    
    oled_draw_value_box(10, y, g_oled_battery_mv / 100, "V", 1);
    y += 20;
    
    oled_draw_status_dot(20, y, 10, RT_TRUE);
    oled_draw_status_dot(50, y, 10, RT_FALSE);
    y += 15;
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&s_u8g2, 10, y + 5, "Status: Ready");
    u8g2_DrawStr(&s_u8g2, 90, y + 5, "[SW3/SW4]");
    
    u8g2_SendBuffer(&s_u8g2);
}


static void render_pid_tuning_page(void)
{
    oled_draw_title_bar("PID 调参", RT_TRUE);
    
    rt_uint8_t y = 25;
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    
    u8g2_DrawStr(&s_u8g2, 10, y, "Kp: [待调]");
    y += 12;
    u8g2_DrawStr(&s_u8g2, 10, y, "Ki: [待调]");
    y += 12;
    u8g2_DrawStr(&s_u8g2, 10, y, "Kd: [待调]");
    
    u8g2_DrawStr(&s_u8g2, 80, y, ">");
    
    u8g2_SendBuffer(&s_u8g2);
}


static void render_ultrasonic_page(void)
{
    oled_draw_title_bar("超声波", RT_TRUE);
    
    rt_uint8_t y = 25;
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    
#ifdef PERIPHERALS_RS485_ULTRASONIC_H__
    // RS485 Modbus 超声波传感器数据
    for (int i = 0; i < 7; i++)
    {
        char buf[16];
        
        // 模拟距离数据（实际应通过 rs485_us_read_distance() 获取）
        uint16_t distance = 150 + (i * 120);  // 模拟 150mm~930mm
        
        snprintf(buf, sizeof(buf), "US%d: %dmm", i + 1, distance);
        u8g2_DrawStr(&s_u8g2, 10, y, buf);
        y += 12;
    }
    
#else
    // 未启用 RS485 时的占位显示
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&s_u8g2, 20, 25, "RS485 module");
    u8g2_DrawStr(&s_u8g2, 25, 35, "not enabled");
    u8g2_DrawStr(&s_u8g2, 20, 45, "in build config");
#endif
    
    u8g2_SendBuffer(&s_u8g2);
}


static void render_ir_sensor_page(void)
{
    oled_draw_title_bar("红外传感器", RT_TRUE);
    
    rt_uint8_t y = 25;
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    
    u8g2_DrawStr(&s_u8g2, 10, y, "CLF: --mm");
    y += 12;
    u8g2_DrawStr(&s_u8g2, 10, y, "CLR: --mm");
    y += 12;
    
    u8g2_DrawStr(&s_u8g2, 10, y, "CHA: OFF");
    y += 12;
    u8g2_DrawStr(&s_u8g2, 10, y, "CHB: OFF");
    y += 12;
    u8g2_DrawStr(&s_u8g2, 10, y, "CHC: OFF");
    
    u8g2_SendBuffer(&s_u8g2);
}


static void render_battery_info_page(void)
{
    oled_draw_title_bar("电池信息", RT_TRUE);
    
    rt_uint8_t y = 25;
    
    oled_draw_value_box(10, y, g_oled_battery_mv / 100, "V", 1);
    y += 15;
    
    rt_uint8_t progress = ((g_oled_battery_mv - 16800) * 100) / (25200 - 16800);
    if (progress > 100) progress = 100;
    
    oled_draw_progress(10, y, 100, 10, progress, RT_TRUE);
    y += 15;
    
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", progress);
    u8g2_DrawStr(&s_u8g2, 110, y, buf);
    
    u8g2_SendBuffer(&s_u8g2);
}


static void render_water_level_page(void)
{
    oled_draw_title_bar("水位信息", RT_TRUE);
    
    rt_uint8_t y = 25;
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    
    u8g2_DrawStr(&s_u8g2, 10, y, "Water: ----%");
    y += 12;
    u8g2_DrawStr(&s_u8g2, 10, y, "Temp: --.°C");
    
    u8g2_SendBuffer(&s_u8g2);
}


static void render_motor_status_page(void)
{
    oled_draw_title_bar("电机状态", RT_TRUE);
    
    rt_uint8_t y = 25;
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    
    u8g2_DrawStr(&s_u8g2, 10, y, "Left: ---rpm");
    y += 12;
    u8g2_DrawStr(&s_u8g2, 10, y, "Right: ---rpm");
    
    u8g2_SendBuffer(&s_u8g2);
}


static void render_imu_data_page(void)
{
    oled_draw_title_bar("IMU 姿态", RT_TRUE);
    
    rt_uint8_t y = 20;
    
#ifdef PERIPHERALS_QMI8658_H__
    // 尝试包含 IMU 头文件
    static float pitch_last = 0, roll_last = 0;
    
    pitch_last += 0.5f;
    roll_last -= 0.3f;
    
    if (pitch_last > 90) pitch_last = 90;
    if (pitch_last < -90) pitch_last = -90;
    if (roll_last > 180) roll_last = 180;
    if (roll_last < -180) roll_last = -180;
    
    int16_t pitch_int = (int16_t)(pitch_last * 10);
    int16_t roll_int = (int16_t)(roll_last * 10);
    
    char buf_pitch[16], buf_roll[16];
    snprintf(buf_pitch, sizeof(buf_pitch), "%d.%d°", pitch_int / 10, abs(pitch_int % 10));
    snprintf(buf_roll, sizeof(buf_roll), "%d.%d°", roll_int / 10, abs(roll_int % 10));
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&s_u8g2, 10, y, "Pitch:");
    u8g2_DrawStr(&s_u8g2, 70, y, buf_pitch);
    
    y += 20;
    u8g2_DrawStr(&s_u8g2, 10, y, "Roll: ");
    u8g2_DrawStr(&s_u8g2, 70, y, buf_roll);
    
#else
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&s_u8g2, 20, 25, "IMU module");
    u8g2_DrawStr(&s_u8g2, 25, 35, "not enabled");
    u8g2_DrawStr(&s_u8g2, 20, 45, "in build config");
#endif
    
    y += 25;
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawLine(&s_u8g2, 10, y, 118, y);
    u8g2_DrawStr(&s_u8g2, 10, y + 10, "-90°  <--->  90°");
    
    u8g2_SendBuffer(&s_u8g2);
}


static void render_fault_log_page(void)
{
    oled_draw_title_bar("故障日志", RT_TRUE);
    
    rt_uint8_t y = 25;
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    
    u8g2_DrawStr(&s_u8g2, 10, y, "No fault log");
    
    u8g2_SendBuffer(&s_u8g2);
}


static void render_settings_page(void)
{
    oled_draw_title_bar("系统设置", RT_TRUE);
    
    rt_uint8_t y = 25;
    
    u8g2_SetFont(&s_u8g2, u8g2_font_ncenB08_tr);
    
    u8g2_DrawStr(&s_u8g2, 10, y, "Brightness: ---");
    y += 12;
    u8g2_DrawStr(&s_u8g2, 10, y, "Volume:  ---");
    y += 12;
    u8g2_DrawStr(&s_u8g2, 10, y, "Language: CN");
    
    u8g2_SendBuffer(&s_u8g2);
}
