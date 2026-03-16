/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief OLED Display Handle - System Service Layer v1.1.4
 * 
 * Function:
 *   1. Multi-page management (boot page, home page, settings page, etc.)
 *   2. SW3/SW4 polling switch (without external interrupt)
 *   3. Mutual exclusion protection with LED SPI resource
 *   4. Timed refresh + semaphore-triggered refresh
 *   5. MSH console test commands for rapid debugging
 */

#ifndef SYSTEM_OLED_HANDLE_H__
#define SYSTEM_OLED_HANDLE_H__

#include <rtthread.h>


/**
 * @brief OLED Screen Configuration
 */
#define OLED_WIDTH            128
#define OLED_HEIGHT           64


/**
 * @brief Screen Page Definition
 * Each page contains: page ID, title, render function pointer
 */
typedef enum
{
    PAGE_BOOT = 0,          // Boot page - displayed at startup
    PAGE_HOME,              // Home page - default display
    PAGE_PID_TUNING,        // PID tuning page
    PAGE_ULTRASONIC,        // Ultrasonic sensor data page
    PAGE_IR_SENSOR,         // IR sensor data page
    PAGE_BATTERY_INFO,      // Battery level info page
    PAGE_WATER_LEVEL,       // Water level info page
    PAGE_MOTOR_STATUS,      // Motor status page
    PAGE_IMU_DATA,          // IMU attitude data page
    PAGE_FAULT_LOG,         // Fault log page
    PAGE_SETTINGS,          // System settings page
    
    PAGE_COUNT              // Total pages (11 pages)
} OledPageId_t;


/**
 * @brief Page Data Structure
 */
typedef struct
{
    OledPageId_t id;                // Page ID
    const char* title;              // Page title (displayed in top bar)
    void (*render)(void);           // Page render function pointer
    uint8_t refresh_interval_ms;    // Auto refresh interval (0=disabled)
} OledPage_t;


/**
 * ========== Initialization Functions ==========
 */

/**
 * @brief Initialize OLED display system
 * @note Create display task, register pages, acquire SPI mutex
 */
void oled_handle_init(void);


/**
 * ========== Page Management API ==========
 */

/**
 * @brief Register a display page
 * @param page_id Page ID
 * @param title Page title
 * @param render_func Page render function (callback for drawing content)
 * @return 0 on success, negative value on failure
 */
int oled_register_page(OledPageId_t page_id, const char* title, void (*render_func)(void));

/**
 * @brief Switch to specified page
 * @param page_id Target page ID
 * @return 0 on success
 */
int oled_switch_page(OledPageId_t page_id);

/**
 * @brief Previous page
 */
void oled_prev_page(void);

/**
 * @brief Next page
 */
void oled_next_page(void);

/**
 * @brief Get current page ID
 */
OledPageId_t oled_get_current_page(void);


/**
 * ========== Refresh Control API ==========
 */

/**
 * @brief Trigger screen refresh (semaphore)
 * @note Called by other tasks to notify OLED of new data
 */
void oled_trigger_refresh(void);

/**
 * @brief Immediately force refresh current page
 */
void oled_force_refresh(void);

/**
 * @brief Set page auto-refresh interval
 * @param page_id Page ID
 * @param interval_ms Refresh interval (milliseconds), 0 means disable auto-refresh
 */
void oled_set_auto_refresh(OledPageId_t page_id, uint32_t interval_ms);


/**
 * ========== UI Component Drawing Interfaces ==========
 * 
 * These functions are called by page renderers to provide common UI components
 */

/**
 * @brief Draw top title bar
 * @param title Title text (max 12 characters)
 * @param show_page_indicator Whether to show page indicator like "[1/9]"
 */
void oled_draw_title_bar(const char* title, rt_bool_t show_page_indicator);

/**
 * @brief Draw progress bar
 * @param x Top-left X coordinate
 * @param y Top-left Y coordinate
 * @param width Width
 * @param height Height
 * @param progress Progress value (0-100%)
 * @param is_good true=green, false=red
 */
void oled_draw_progress(uint8_t x, uint8_t y, uint8_t width, 
                        uint8_t height, uint8_t progress, rt_bool_t is_good);

/**
 * @brief Draw numeric value box (with unit)
 * @param x X coordinate
 * @param y Y coordinate
 * @param value Numeric value (integer)
 * @param unit Unit string (e.g., "V", "mm", "%", "°C")
 * @param precision Number of decimal places
 */
void oled_draw_value_box(uint8_t x, uint8_t y, int32_t value, 
                         const char* unit, uint8_t precision);

/**
 * @brief Draw status indicator light (dot)
 * @param x Center X coordinate
 * @param y Center Y coordinate
 * @param radius Radius
 * @param active true=bright green, false=dim red
 */
void oled_draw_status_dot(uint8_t x, uint8_t y, uint8_t radius, rt_bool_t active);

/**
 * @brief Draw icon
 * @param x X coordinate
 * @param y Y coordinate
 * @param icon_id Icon ID
 */
void oled_draw_icon(uint8_t x, uint8_t y, uint8_t icon_id);


/**
 * ========== Battery Level Global Variable (used in u8g2 example program) ==========
 */

extern int g_oled_battery_mv;    // Current battery voltage (mV)
extern rt_thread_t s_oled_thread;   // OLED update task handle


/**
 * ========== SPI Resource Mutex API - For use by other modules ==========
 * 
 * OLED and LED(74HC595) share the same SPI bus
 * LED modules must call this lock mechanism before SPI operations
 */

/**
 * @brief Acquire SPI mutex (wait)
 * @note Must be called before SPI operations like LED_shift_out()
 */
void oled_spi_lock(void);

/**
 * @brief Release SPI mutex
 * @note Corresponding unlock must be called after completing SPI operation
 */
void oled_spi_unlock(void);


/* ============================================ */
/*        MSH Console Test Commands API       */
/* ============================================ */
/* 
 * These commands are exported via MSH_CMD_EXPORT_ALIAS macro
 * Available in msh console after oled initialization
 */

/**
 * @brief MSH command: oled_test [page_id]
 * Usage: Show OLED info or switch to specific page (0-10)
 *   page_id: 0=boot, 1=home, 2=pid, 3=ultrasonic, 4=ir, 5=battery, 
 *            6=water, 7=motor, 8=imu, 9=fault, 10=settings
 */
extern void oled_test(int argc, char *argv[]);

/**
 * @brief MSH command: oled_refresh
 * Usage: Force immediate screen refresh
 */
extern void oled_refresh_cmd(int argc, char *argv[]);

/**
 * @brief MSH command: oled_status
 * Usage: Display current OLED status information
 */
extern void oled_status(int argc, char *argv[]);

/**
 * @brief MSH command: oled_cycle [count]
 * Usage: Cycle through N consecutive pages (default: 3)
 *   count: number of pages to cycle through (1-11)
 */
extern void oled_cycle(int argc, char *argv[]);


#endif /* SYSTEM_OLED_HANDLE_H__ */
