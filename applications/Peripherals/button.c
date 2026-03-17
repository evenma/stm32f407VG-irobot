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

#include <rtthread.h>
#include <board.h>
#include "button.h"
#include "global_conf.h"


/**
 * @brief LED state array (8 individual LEDs)
 */
static ButtonObject_t s_buttons[BUTTON_TOTAL_COUNT];

/**
 * @brief Scan timer handle
 */
static rt_timer_t s_scan_timer = RT_NULL;

/**
 * @brief Long press threshold (in milliseconds)
 */
#define LONG_PRESS_THRESHOLD_MS  1000

/**
 * @brief Minimum interval between events (debounce)
 */
#define MIN_EVENT_INTERVAL_MS    200


/* ========== Internal Function Prototypes ========== */

/**
 * @brief Scan timer callback (called every 10ms)
 */
static void button_scan_timer_callback(void *parameter);

/**
 * @brief Detect and process a single button press
 * @param btn_idx Button index (0-3)
 */
static void button_detect_press(rt_uint8_t btn_idx);

/**
 * @brief Deliver event to registered callback
 * @param btn_idx Button index
 * @param event Event type
 */
static void button_deliver_event(rt_uint8_t btn_idx, ButtonEvent_t event);


/* ========== Public Functions Implementation ========== */

/**
 * @brief Initialize all button objects
 */
void button_init(void)
{
    rt_int8_t i;
    
    /* 1. Configure all pins as input with pull-up */
    /* SW3 = PC13 */
    rt_pin_mode(KEY_SW3_PIN, PIN_MODE_INPUT_PULLUP);
    /* SW4 = PC14 */
    rt_pin_mode(KEY_SW4_PIN, PIN_MODE_INPUT_PULLUP);
    /* SW5 = PC15 */
    rt_pin_mode(KEY_SW5_PIN, PIN_MODE_INPUT_PULLUP);
    /* HOME = PE9 */
    rt_pin_mode(KEY_HOME_PIN, PIN_MODE_INPUT_PULLUP);
    
    /* 2. Initialize button objects */
    for (i = 0; i < BUTTON_TOTAL_COUNT; i++)
    {
        s_buttons[i].id = i;
        s_buttons[i].enabled = RT_TRUE;     // Enable by default
        s_buttons[i].debounce_ticks = 0;
        s_buttons[i].last_press_time = 0;
        s_buttons[i].is_pressed = RT_FALSE;
        s_buttons[i].event_fired = RT_FALSE;
        s_buttons[i].callback = RT_NULL;
    }
    
    /* 3. Create scan timer (10ms period) */
    s_scan_timer = rt_timer_create("btnscn", 
                                    button_scan_timer_callback, 
                                    RT_NULL, 
                                    10,        // 10ms period
                                    RT_TIMER_FLAG_PERIODIC);
    
    if (s_scan_timer != RT_NULL)
    {
        rt_timer_start(s_scan_timer);
        rt_kprintf("[Button] Scan timer started (10ms)\n");
    }
    else
    {
        rt_kprintf("[Button] Error: Failed to create scan timer!\n");
    }
    
    rt_kprintf("[Button] Initialized %d buttons\n", BUTTON_TOTAL_COUNT);
}


/**
 * @brief Scan timer callback (called every 10ms)
 */
static void button_scan_timer_callback(void *parameter)
{
    rt_uint8_t i;
    
    for (i = 0; i < BUTTON_TOTAL_COUNT; i++)
    {
        button_detect_press(i);
    }
}


/**
 * @brief Detect and process a single button press
 */
static void button_detect_press(rt_uint8_t btn_idx)
{
    rt_bool_t current_state;
    ButtonId_t button_id;
    
    switch (btn_idx)
    {
        case BUTTON_ID_SW3:
            button_id = BUTTON_ID_SW3;
            current_state = !rt_pin_read(KEY_SW3_PIN);
            break;
        case BUTTON_ID_SW4:
            button_id = BUTTON_ID_SW4;
            current_state = !rt_pin_read(KEY_SW4_PIN);
            break;
        case BUTTON_ID_SW5:
            button_id = BUTTON_ID_SW5;
            current_state = !rt_pin_read(KEY_SW5_PIN);
            break;
        case BUTTON_ID_HOME:
            button_id = BUTTON_ID_HOME;
            current_state = !rt_pin_read(KEY_HOME_PIN);
            break;
        default:
            return;
    }
    
    ButtonObject_t* btn = &s_buttons[btn_idx];
    
    /* If disabled, skip processing */
    if (!btn->enabled)
    {
        return;
    }
    
    /* Handle pressed state */
    if (current_state)
    {
        if (!btn->is_pressed)
        {
            /* Rising edge: button just pressed */
            btn->debounce_ticks = KEY_BUTTON_PULSE_WIDTH_MS / 10;  // 20ms / 10ms = 2 ticks
            btn->last_press_time = rt_tick_get_millisecond();
            btn->is_pressed = RT_TRUE;
            btn->event_fired = RT_FALSE;
        }
        else
        {
            /* Button already pressed, continue debounce */
            if (btn->debounce_ticks > 0)
            {
                btn->debounce_ticks--;
            }
            
            if (btn->debounce_ticks == 0 && !btn->event_fired)
            {
                /* Debounce complete, check for long press */
                rt_int32_t duration_ms = rt_tick_get_millisecond() - btn->last_press_time;
                
                if (duration_ms >= LONG_PRESS_THRESHOLD_MS)
                {
                    /* Long press detected */
                    button_deliver_event(btn_idx, BUTTON_EVENT_LONG_PRESS);
                    btn->event_fired = RT_TRUE;
                }
            }
        }
    }
    else
    {
        /* Falling edge: button released */
        if (btn->is_pressed)
        {
            btn->is_pressed = RT_FALSE;
            btn->debounce_ticks = 0;
            
            /* Only deliver release event if no long press fired */
            if (!btn->event_fired && btn->callback != RT_NULL)
            {
                /* Check if short press */
                rt_int32_t duration_ms = rt_tick_get_millisecond() - btn->last_press_time;
                
                if (duration_ms < LONG_PRESS_THRESHOLD_MS)
                {
                    button_deliver_event(btn_idx, BUTTON_EVENT_SHORT_PRESS);
                }
                
                button_deliver_event(btn_idx, BUTTON_EVENT_RELEASE);
            }
            
            btn->event_fired = RT_FALSE;
        }
    }
}


/**
 * @brief Deliver event to registered callback
 */
static void button_deliver_event(rt_uint8_t btn_idx, ButtonEvent_t event)
{
    ButtonObject_t* btn = &s_buttons[btn_idx];
    
    if (btn->callback != RT_NULL)
    {
        btn->callback((ButtonId_t)btn_idx, event);
    }
}


/**
 * @brief Process button scan (alternative manual method)
 */
void button_scan(void)
{
    rt_uint8_t i;
    
    for (i = 0; i < BUTTON_TOTAL_COUNT; i++)
    {
        button_detect_press(i);
    }
}


/**
 * @brief Register callback for specific button
 */
void button_register_callback(ButtonId_t id, ButtonCallback_t callback)
{
    if (id >= BUTTON_TOTAL_COUNT)
    {
        rt_kprintf("[Button] Error: Invalid button ID %d\n", id);
        return;
    }
    
    s_buttons[id].callback = callback;
    rt_kprintf("[Button] Callback registered for button %d\n", id);
}


/**
 * @brief Enable/disable a specific button
 */
void button_enable(ButtonId_t id, rt_bool_t enable)
{
    if (id >= BUTTON_TOTAL_COUNT)
    {
        rt_kprintf("[Button] Error: Invalid button ID %d\n", id);
        return;
    }
    
    s_buttons[id].enabled = enable;
}


/**
 * @brief Check if a button is enabled
 */
rt_bool_t button_is_enabled(ButtonId_t id)
{
    if (id >= BUTTON_TOTAL_COUNT)
    {
        return RT_FALSE;
    }
    
    return s_buttons[id].enabled;
}


/**
 * @brief Get current press state of a button
 */
rt_bool_t button_is_pressed(ButtonId_t id)
{
    if (id >= BUTTON_TOTAL_COUNT)
    {
        return RT_FALSE;
    }
    
    return s_buttons[id].is_pressed;
}


/**
 * @brief Get time since last press (in milliseconds)
 */
rt_uint32_t button_get_time_since_press(ButtonId_t id)
{
    if (id >= BUTTON_TOTAL_COUNT)
    {
        return 0xFFFFFFFF;
    }
    
    return rt_tick_get_millisecond() - s_buttons[id].last_press_time;
}


/* ========== Debug MSH Commands ========== */

#ifdef RT_USING_MSH

#include <msh.h>

static void button_test(int argc, char** argv)
{
    rt_kprintf("========== Button Test Mode ==========\n");
    rt_kprintf("Testing all buttons...\n");
    rt_kprintf("Press any button to see event log\n");
    rt_kprintf("--------------------------------------\n\n");
    
    /* Register temporary callback */
    static uint8_t short_count[BUTTON_TOTAL_COUNT] = {0};
    static uint8_t long_count[BUTTON_TOTAL_COUNT] = {0};
    
    void temp_callback(ButtonId_t id, ButtonEvent_t event)
    {
        const char* event_name = "";
        
        switch (event)
        {
            case BUTTON_EVENT_SHORT_PRESS:
                event_name = "SHORT";
                short_count[id]++;
                break;
            case BUTTON_EVENT_LONG_PRESS:
                event_name = "LONG";
                long_count[id]++;
                break;
            case BUTTON_EVENT_RELEASE:
                event_name = "RELEASE";
                break;
            default:
                event_name = "NONE";
                break;
        }
        
        const char* button_name = "";
        switch (id)
        {
            case BUTTON_ID_SW3: button_name = "SW3(PC13)"; break;
            case BUTTON_ID_SW4: button_name = "SW4(PC14)"; break;
            case BUTTON_ID_SW5: button_name = "SW5(PC15)"; break;
            case BUTTON_ID_HOME: button_name = "HOME(PE9)"; break;
            default: button_name = "UNKNOWN"; break;
        }
        
        rt_kprintf("%-12s => %-12s [Short:%d Long:%d]\n", 
                   button_name, event_name, short_count[id], long_count[id]);
    }
    
    button_register_callback(BUTTON_ID_SW3, temp_callback);
    button_register_callback(BUTTON_ID_SW4, temp_callback);
    button_register_callback(BUTTON_ID_SW5, temp_callback);
    button_register_callback(BUTTON_ID_HOME, temp_callback);
    
    rt_kprintf("\nTest started! Press Ctrl+C to exit.\n\n");
}


static void button_status(int argc, char** argv)
{
    rt_kprintf("\n========== Button Status ==========\n");
    rt_kprintf("%-12s %-10s %-10s %-10s\n", "Button", "Enabled", "Pressed", "Callback");
    rt_kprintf("------------------------------------");
    
    for (int i = 0; i < BUTTON_TOTAL_COUNT; i++)
    {
        const char* name = "";
        const char* enabled_str = s_buttons[i].enabled ? "Yes" : "No";
        const char* pressed_str = s_buttons[i].is_pressed ? "Yes" : "No";
        const char* callback_str = s_buttons[i].callback ? "Yes" : "No";
        
        switch (i)
        {
            case BUTTON_ID_SW3: name = "SW3(PC13)"; break;
            case BUTTON_ID_SW4: name = "SW4(PC14)"; break;
            case BUTTON_ID_SW5: name = "SW5(PC15)"; break;
            case BUTTON_ID_HOME: name = "HOME(PE9)"; break;
            default: name = "Unknown"; break;
        }
        
        rt_kprintf("\n%-12s %-10s %-10s %-10s", name, enabled_str, pressed_str, callback_str);
    }
    rt_kprintf("\n====================================\n\n");
}

MSH_CMD_EXPORT(button_test, "button test - Start testing mode");
MSH_CMD_EXPORT(button_status, "button status - Show current status");

#endif /* RT_USING_MSH */
