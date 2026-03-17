/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief Button object driver - Key input handling with debouncing
 * 
 * Hardware connections:
 *   - SW3 (PC13): Previous page / Back
 *   - SW4 (PC14): Next page / Forward  
 *   - SW5 (PC15): Not used
 *   - HOME (PE9): Return to charging dock
 * 
 * Architecture:
 *   - Timer-based scan (10ms interval)
 *   - Event detection via IPC (message queue + semaphore)
 *   - NO blocking operations in timer callback!
 */

#ifndef PERIPHERALS_BUTTON_H__
#define PERIPHERALS_BUTTON_H__

#include <rtthread.h>


/**
 * @brief Button ID enumeration
 */
typedef enum
{
    BUTTON_ID_SW3 = 0,      // PC13 - Previous page
    BUTTON_ID_SW4,          // PC14 - Next page
    BUTTON_ID_SW5,          // PC15 - Not used
    BUTTON_ID_HOME,         // PE9 - Return home
    
    BUTTON_TOTAL_COUNT      // Total number of buttons
} ButtonId_t;


/**
 * @brief Button event types
 */
typedef enum
{
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SHORT_PRESS,   // Single short press
    BUTTON_EVENT_LONG_PRESS,    // Long press held
    BUTTON_EVENT_RELEASE        // Release after press
} ButtonEvent_t;


/**
 * @brief Button callback function type
 * @param id Button ID that triggered the event
 * @param event Event type (short/long/release)
 */
typedef void (*ButtonCallback_t)(ButtonId_t id, ButtonEvent_t event);


/**
 * @brief Button state structure
 */
typedef struct
{
    rt_uint8_t id;                // Button ID (0-3)
    rt_bool_t enabled;            // Is this button enabled?
    rt_int32_t debounce_ticks;    // Current debounce counter
    rt_tick_t last_press_time;    // Time when button was pressed
    rt_bool_t is_pressed;         // Current pressed state
    rt_bool_t event_fired;        // Event already delivered flag
    ButtonCallback_t callback;    // User callback function
} ButtonObject_t;


/**
 * ========== Initialization Functions ==========
 */

/**
 * @brief Initialize all button objects and IPC primitives
 * @note Creates message queue for page changes and semaphore for HOME signal
 */
void button_init(void);


/**
 * ========== Core Event Handling ==========
 */

/**
 * @brief Process button scan (alternative manual method if no timer)
 */
void button_scan(void);


/**
 * ========== Callback Registration (Optional) ==========
 */

/**
 * @brief Register callback for specific button
 * @param id Button ID to register for
 * @param callback Function pointer to call on events
 */
void button_register_callback(ButtonId_t id, ButtonCallback_t callback);


/**
 * @brief Enable/disable a specific button
 * @param id Button ID
 * @param enable RT_TRUE to enable, RT_FALSE to disable
 */
void button_enable(ButtonId_t id, rt_bool_t enable);


/**
 * @brief Check if a button is enabled
 * @param id Button ID
 * @return RT_TRUE if enabled, RT_FALSE otherwise
 */
rt_bool_t button_is_enabled(ButtonId_t id);


/**
 * ========== Helper Functions ==========
 */

/**
 * @brief Get current press state of a button
 * @param id Button ID
 * @return RT_TRUE if currently pressed
 */
rt_bool_t button_is_pressed(ButtonId_t id);


/**
 * @brief Get time since last press (in milliseconds)
 * @param id Button ID
 * @return Milliseconds since press, or 0xFFFFFFFF if not pressed recently
 */
rt_uint32_t button_get_time_since_press(ButtonId_t id);


/**
 * ========== IPC Interface Functions ==========
 */

/**
 * @brief Get the page change message queue handle
 * @return Message queue pointer, can be used by OLED task to receive page requests
 * 
 * Example usage in OLED task:
 * @code
 * rt_uint32_t page_id;
 * rt_err_t result = rt_mq_recv(s_page_queue, &page_id, sizeof(page_id), 100);
 * if (result == RT_EOK) {
 *     oled_switch_page(page_id);
 * }
 * @endcode
 */
rt_messageq_t button_get_page_queue(void);

/**
 * @brief Get the HOME press semaphore handle
 * @return Semaphore pointer, can be used by upper host navigation task
 * 
 * Example usage in navigation task:
 * @code
 * rt_sem_take(button_get_home_semaphore(), RT_WAITING_FOREVER);
 * navigate_to_charging_dock();
 * @endcode
 */
rt_sem_t button_get_home_semaphore(void);


#endif /* PERIPHERALS_BUTTON_H__ */