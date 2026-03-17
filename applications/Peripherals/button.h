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
 * Features:
 *   - Software debouncing (20ms configurable)
 *   - Short press / long press detection
 *   - Callback registration for each key
 *   - MSH console test commands
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
 * @brief Initialize all button objects
 * @note Configures GPIO pins as input with pull-up
 */
void button_init(void);


/**
 * ========== Core Event Handling ==========
 */

/**
 * @brief Process button scan (call periodically from main loop or timer)
 * @note Should be called every 10-20ms for responsive detection
 */
void button_scan(void);


/**
 * ========== Callback Registration ==========
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
 * @return Milliseconds since press, or 0 if not pressed recently
 */
rt_uint32_t button_get_time_since_press(ButtonId_t id);


#endif /* PERIPHERALS_BUTTON_H__ */