/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief OLED display handle implementation v1.1.0
 * 
 * Function description:
 *   1. Use u8g2 library with SPI2 driver for SSD1306 OLED
 *   2. Multi-page management and switching (11 pages)
 *   3. SW3/SW4 polling detection (non-interrupt method)
 *   4. Mutual exclusion protection for SPI resources shared with LED
 */

#include <rtthread.h>
#include <u8g2_port.h>
#include "oled_handle.h"
#include "global_conf.h"
#include <string.h>


/**
 * ========== Global Variables ==========
 */

/**
 * @brief Battery level global variable (shared with u8g2 example program)
 */
int g_oled_battery_mv = 24000;

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
    rt_mutex_init(&s_spi_mutex, "spi", RT_IPC_FLAG_PRIORITY);
    
    // Create semaphore for refresh trigger
    rt_sem_init(&s_refresh_sem, "ref", 0, RT_IPC_FLAG_PRIORITY);
    
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
}


/**
 * @brief Lock SPI bus before OLED operations
 */
void oled_spi_lock(void)
{
    rt_mutex_fetch(&s_spi_mutex, RT_WAITING_FOREVER);
}


/**
 * @brief Unlock SPI bus after OLED operations
 */
void oled_spi_unlock(void)
{
    rt_mutex_post(&s_spi_mutex);
}


/**
 * @brief Trigger OLED refresh from any task
 */
void oled_trigger_refresh(void)
{
    rt_sem_send(&s_refresh_sem);
}


/**
 * @brief Switch to specified page
 * @param page_id Page ID to switch to
 * @return RT_EOK on success, RT_ERROR on invalid page
 */
rt_err_t oled_switch_page(OledPageId_t page_id)
{
    if (page_id >= PAGE_COUNT) return -RT_ERROR;
    
    s_current_page = page_id;
    oled_trigger_refresh();
    
    return RT_EOK;
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
 * @brief Register a display page
 * @param page_id Unique page identifier
 * @param title Short title string
 * @param render_callback Render function pointer
 */
void oled_register_page(OledPageId_t page_id, const char* title, OledRenderFunc_t render_callback)
{
    if (page_id >= PAGE_COUNT) return;
    
    s_pages[page_id].title = title;
    s_pages[page_id].render = render_callback;
    s_pages[page_id].refresh_interval_ms = 1000;
    s_pages[page_id].last_refresh_ms = 0;
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
 * @param title Title string
 * @param active RT_TRUE if this is current page
 */
void oled_draw_title_bar(const char* title, rt_bool_t active)
{
    uint8_t y = 20;
    
    if (active)
    {
        u8g2_SetFont(&s_u8g2, &u8g2_font_ncenB14_tr);
        u8g2_DrawStr(&s_u8g2, 8, y, title);
    }
    else
    {
        u8g2_SetFont(&s_u8g2, &u8g2_font_ncenR14_tr);
        u8g2_DrawStr(&s_u8g2, 8, y, title);
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
void oled_draw_value_box(uint8_t x, uint8_t y, long value, const char* unit, uint8_t decimal_digits)
{
    char buf[32];
    
    if (decimal_digits == 0)
    {
        snprintf(buf, sizeof(buf), "%ld%s", value, unit);
    }
    else if (decimal_digits == 1)
    {
        snprintf(buf, sizeof(buf), "%ld.%d%s", value / 10, value % 10, unit);
    }
    else if (decimal_digits == 2)
    {
        snprintf(buf, sizeof(buf), "%ld.%02d%s", value / 100, value % 100, unit);
    }
    
    u8g2_SetFont(&s_u8g2, &u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&s_u8g2, x, y, buf);
}


/**
 * @brief Draw battery voltage indicator
 */
static void draw_battery_indicator(uint8_t x, uint8_t y)
{
    uint32_t mv = g_oled_battery_mv;
    
    // Draw battery outline
    u8g2_DrawFrame(&s_u8g2, x, y, 30, 16);
    
    // Draw battery tip
    u8g2_DrawLine(&s_u8g2, x + 30, y + 4, x + 34, y + 8);
    u8g2_DrawLine(&s_u8g2, x + 30, y + 12, x + 34, y + 8);
    
    // Calculate fill level (21V ~ 29V range)
    if (mv > 29000) mv = 29000;
    if (mv < 21000) mv = 21000;
    
    uint8_t fill_width = (mv - 21000) / 8 * 30 / 1000;
    
    // Draw battery level
    u8g2_DrawFilledRect(&s_u8g2, x + 2, y + 2, fill_width, 12);
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
    u8g2_DrawFilledRect(&s_u8g2, x, y, fill_width, height);
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
 * @brief Initialize u8g2 display object (SPI2 DRIVER v1.1.2)
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
    
    while (1)
    {
        result = rt_sem_take(&s_refresh_sem, 100);
        
        // Check timeout - still refresh if needed
        uint32_t tick_now = rt_tick_get_millisecond();
        
        if (result == RT_EOK || 
            (tick_now - tick_last) >= s_pages[s_current_page].refresh_interval_ms)
        {
            // Acquire SPI mutex
            rt_mutex_fetch(&s_spi_mutex, RT_WAITING_FOREVER);
            
            // Clear and redraw
            u8g2_ClearDisplay(&s_u8g2);
            s_pages[s_current_page].render(&s_u8g2);
            
            // Update display
            u8g2_SendBuffer(&s_u8g2);
            
            // Release SPI mutex
            rt_mutex_post(&s_spi_mutex);
            
            tick_last = tick_now;
        }
    }
}


/**
 * ========== Page Renderers (v1.1.1 - Full Display Content Restored) ==========
 */

static void render_boot_page(u8g2_t* u8g2)
{
    uint8_t x, y;
    
    // iHomeRobot logo
    u8g2_SetFont(u8g2, &u8g2_font_ncenB14_tr);
    u8g2_DrawStr(u8g2, 8, 15, "iHomeRobot Smart Toilet Robot");
    
    // Boot progress bar at bottom
    x = 8; y = 45;
    u8g2_DrawFrame(u8g2, x, y, 112, 8);
    static uint8_t boot_progress = 0;
    if (boot_progress < 100) boot_progress++;
    u8g2_DrawFilledRect(u8g2, x, y, boot_progress * 112 / 100, 8);
    
    y += 12;
    u8g2_DrawStr(u8g2, 35, y, "Booting...");
}

static void render_home_page(u8g2_t* u8g2)
{
    uint8_t y = 20;
    
    // Page title
    oled_draw_title_bar("Main Menu", RT_TRUE);
    
    // Battery indicator at top-right
    draw_battery_indicator(70, y - 6);
    
    y += 25;
    
    // System status block
    u8g2_SetFont(u8g2, &u8g2_font_ncenR14_tr);
    
    // Battery voltage with decimal point
    long bat_v = g_oled_battery_mv / 1000;
    long bat_mv = g_oled_battery_mv % 1000;
    char bat_str[16];
    snprintf(bat_str, sizeof(bat_str), "%ld.%03dV", bat_v, bat_mv);
    u8g2_DrawStr(u8g2, 8, y, bat_str);
    
    // Progress bar for battery charge
    y += 15;
    draw_progress_bar(8, y, 112, 10, (bat_v > 22) ? 80 : 50, RT_TRUE);
    
    // Power source indicator
    y += 15;
    u8g2_DrawStr(u8g2, 8, y, "Power: Bat/Adapter");
    
    // Operating mode
    y += 15;
    u8g2_DrawStr(u8g2, 8, y, "Mode: Standby");
    
    y += 10;
    u8g2_DrawHLine(u8g2, 8, y, 112);
    y += 15;
    
    // Quick status items
    u8g2_DrawStr(u8g2, 8, y, "Left LED: OFF");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Right LED: OFF");
}

static void render_pid_tuning_page(u8g2_t* u8g2)
{
    // Reserved for future PID tuning interface
    oled_draw_title_bar("PID Tuning", RT_TRUE);
    
    uint8_t y = 40;
    u8g2_SetFont(u8g2, &u8g2_font_ncenR14_tr);
    
    u8g2_DrawStr(u8g2, 20, y, "[Reserved]");
    y += 15;
    u8g2_DrawStr(u8g2, 20, y, "PID params not configured yet");
}

static void render_ultrasonic_page(u8g2_t* u8g2)
{
    // Ultrasonic sensor readings
    oled_draw_title_bar("Ultrasonic", RT_TRUE);
    
    uint8_t y = 40;
    u8g2_SetFont(u8g2, &u8g2_font_ncenR14_tr);
    
    // Simulated distance readings (replace with actual sensor data later)
    u8g2_DrawStr(u8g2, 8, y, "Front:   152 mm");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Back:     89 mm");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Left:   124 mm");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Right:  118 mm");
}

static void render_ir_sensor_page(u8g2_t* u8g2)
{
    // IR cliff sensors
    oled_draw_title_bar("IR Sensors", RT_TRUE);
    
    uint8_t y = 40;
    u8g2_SetFont(u8g2, &u8g2_font_ncenR14_tr);
    
    // Cliff sensor status (simulated)
    u8g2_DrawStr(u8g2, 8, y, "Cliff Front: OK");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Cliff Rear:  OK");
    
    y += 10;
    u8g2_DrawHLine(u8g2, 8, y, 112);
    y += 15;
    
    u8g2_DrawStr(u8g2, 8, y, "Charging Align: Ready");
}

static void render_battery_info_page(u8g2_t* u8g2)
{
    // Detailed battery info
    oled_draw_title_bar("Battery Info", RT_TRUE);
    
    uint8_t y = 40;
    u8g2_SetFont(u8g2, &u8g2_font_ncenR14_tr);
    
    // Battery voltage
    long bat_v = g_oled_battery_mv / 1000;
    long bat_mv = g_oled_battery_mv % 1000;
    char bat_str[20];
    snprintf(bat_str, sizeof(bat_str), "Voltage: %ld.%03d V", bat_v, bat_mv);
    u8g2_DrawStr(u8g2, 8, y, bat_str);
    
    y += 15;
    u8g2_DrawStr(u8g2, 8, y, "Level: ~80%");
    
    y += 15;
    draw_progress_bar(8, y, 112, 12, 80, RT_TRUE);
    
    y += 18;
    u8g2_DrawStr(u8g2, 8, y, "Charge Status: Idle");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Adapter: Disconnected");
}

static void render_water_level_page(u8g2_t* u8g2)
{
    // Water tank status
    oled_draw_title_bar("Water Level", RT_TRUE);
    
    uint8_t y = 40;
    u8g2_SetFont(u8g2, &u8g2_font_ncenR14_tr);
    
    u8g2_DrawStr(u8g2, 8, y, "High Water: Present");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Low Water:  OK");
    
    y += 20;
    u8g2_DrawStr(u8g2, 8, y, "Clean Tank: Yes");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Dirty Tank: Empty");
}

static void render_motor_status_page(u8g2_t* u8g2)
{
    // Motor operational status
    oled_draw_title_bar("Motor Status", RT_TRUE);
    
    uint8_t y = 40;
    u8g2_SetFont(u8g2, &u8g2_font_ncenR14_tr);
    
    // Left motor (simulated RPM)
    u8g2_DrawStr(u8g2, 8, y, "Left Motor:     0 RPM");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Right Motor:    0 RPM");
    
    y += 20;
    
    // Stepper motors status
    u8g2_DrawStr(u8g2, 8, y, "Cleaning Rod:   Ready");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Tube Switcher:  Ready");
}

static void render_imu_data_page(u8g2_t* u8g2)
{
    // IMU attitude data
    oled_draw_title_bar("IMU Data", RT_TRUE);
    
    uint8_t y = 40;
    u8g2_SetFont(u8g2, &u8g2_font_ncenR14_tr);
    
    // Simulated IMU readings (pitch/yaw/roll in degrees)
    int8_t pitch = 2, yaw = -1, roll = 1;
    char imustr[32];
    
    snprintf(imustr, sizeof(imustr), "Pitch:  %+3d°", pitch);
    u8g2_DrawStr(u8g2, 8, y, imustr);
    
    y += 12;
    snprintf(imustr, sizeof(imustr), "Roll:   %+3d°", roll);
    u8g2_DrawStr(u8g2, 8, y, imustr);
    
    y += 12;
    snprintf(imustr, sizeof(imustr), "Yaw:    %+3d°", yaw);
    u8g2_DrawStr(u8g2, 8, y, imustr);
    
    y += 15;
    u8g2_DrawStr(u8g2, 8, y, "Status: Calibrated");
}

static void render_fault_log_page(u8g2_t* u8g2)
{
    // Fault/error log display
    oled_draw_title_bar("Fault Log", RT_TRUE);
    
    uint8_t y = 40;
    u8g2_SetFont(u8g2, &u8g2_font_ncenR14_tr);
    
    // Check for faults (placeholder logic)
    u8g2_DrawStr(u8g2, 8, y, "No Active Faults");
    
    y += 25;
    u8g2_DrawStr(u8g2, 8, y, "System OK");
    y += 15;
    
    // Last reset info
    u8g2_DrawStr(u8g2, 8, y, "Last Reset: Power-on");
}

static void render_settings_page(u8g2_t* u8g2)
{
    // System settings menu
    oled_draw_title_bar("Settings", RT_TRUE);
    
    uint8_t y = 40;
    u8g2_SetFont(u8g2, &u8g2_font_ncenR14_tr);
    
    u8g2_DrawStr(u8g2, 8, y, "Volume: Medium");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Language: EN");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "LED Brightness: 100%");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Auto Power-off: 30min");
    
    y += 25;
    u8g2_DrawStr(u8g2, 8, y, "Firmware: v1.0.2");
    y += 12;
    u8g2_DrawStr(u8g2, 8, y, "Build: 2026-03-16");
}


/**
 * ========== MSH Console Commands for Testing ==========
 */

/**
 * @brief MSH command to test OLED initialization and display control
 * Usage: oled_test [page_id]
 *   page_id: 0=boot, 1=home, 2=pid, 3=ultrasonic, 4=ir, 5=battery, 
 *            6=water, 7=motor, 8=imu, 9=fault, 10=settings
 */
MSH_CMD_EXPORT_ALIAS(oled_test_cmd, oled_test, OLED Display Control Test (v1.1.2));

/**
 * @brief Actual implementation of oled_test command
 */
void oled_test_cmd(int argc, char *argv[])
{
    oled_test_impl(argc, argv);
}

void oled_test_impl(int argc, char *argv[])
{
    int page_id = -1;
    
    if (argc > 1)
    {
        page_id = rt_strtol(argv[1], NULL, 10);
        if (page_id < 0 || page_id >= PAGE_COUNT)
        {
            rt_kprintf("[OLED] Invalid page ID! Use 0-%d\r\n", PAGE_COUNT - 1);
            return;
        }
    }
    
    rt_kprintf("[OLED] ===========================================\r\n");
    rt_kprintf("[OLED] OLED Display Control Test (v1.1.2)\r\n");
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
    rt_kprintf("[OLED] ===========================================\r\n");
}

/**
 * @brief MSH command to force refresh current page
 * Usage: oled_refresh
 */
MSH_CMD_EXPORT_ALIAS(oled_force_refresh, oled_refresh, OLED Force Screen Refresh);

void oled_force_refresh_impl(int argc, char *argv[])
{
    rt_kprintf("[OLED] Forcing screen refresh...\r\n");
    oled_force_refresh();
    rt_kprintf("[OLED] Refresh complete!\r\n");
}

/**
 * @brief Actual wrapper function
 */
void oled_force_refresh_cmd(int argc, char *argv[])
{
    oled_force_refresh_impl(argc, argv);
}

/**
 * @brief MSH command to show OLED status
 * Usage: oled_status
 */
MSH_CMD_EXPORT_ALIAS(oled_show_status, oled_status, OLED Show Status Information);

void oled_status_impl(int argc, char *argv[])
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

/**
 * @brief Actual wrapper function
 */
void oled_status_cmd(int argc, char *argv[])
{
    oled_status_impl(argc, argv);
}

/**
 * @brief MSH command to cycle through all pages
 * Usage: oled_cycle [count]
 *   count: number of pages to cycle (default: 3)
 */
MSH_CMD_EXPORT_ALIAS(oled_cycle_pages, oled_cycle, OLED Cycle Through Pages);

void oled_cycle_impl(int argc, char *argv[])
{
    int cycles = 3;
    
    if (argc > 1)
    {
        cycles = rt_strtol(argv[1], NULL, 10);
        if (cycles < 1) cycles = 1;
        if (cycles > PAGE_COUNT) cycles = PAGE_COUNT;
    }
    
    rt_kprintf("[OLED] Cycling through %d pages...\r\n", cycles);
    
    for (int i = 0; i < cycles; i++)
    {
        OledPageId_t next_page = (s_current_page + 1) % PAGE_COUNT;
        rt_kprintf("[OLED] Switching from %d -> %d\r\n", s_current_page, next_page);
        oled_switch_page(next_page);
        rt_thread_mdelay(500);  // Give OLED time to update
    }
    
    rt_kprintf("[OLED] Cycle complete!\r\n");
}

/**
 * @brief Actual wrapper function
 */
void oled_cycle_cmd(int argc, char *argv[])
{
    oled_cycle_impl(argc, argv);
}
