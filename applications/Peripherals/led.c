/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief LED indicator driver - Based on 74HC595 shift register
 * 
 * Hardware connections:
 * - SPI2: PB13(SCK), PB15(MOSI) -> 74HC595 SRCLK/SER
 * - GPIO: PB2(RCK) -> 74HC595 RCLK
 * - LED: 8 individual LEDs (1x 74HC595, 1 byte = 8 LEDs)
 * LED0~LED5: Local status indicators (working/fault/battery/water level)
 * LED6(LED7): Host navigation green light (NEW)
 * LED7(LED8): Host error red light (NEW)
 * 
 * Note: OLED and LED share SPI2 bus, must acquire mutex lock before access!
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "led.h"
#include "oled_handle.h" // Include oled_spi_lock/unlock for SPI mutual exclusion
#include "global_conf.h"


/**
 * @brief LED state array (8 individual LEDs)
 */
static LedObject_t led_objects[LED_TOTAL_COUNT];

/**
 * @brief 74HC595 shift register output buffer
 * 8 bits = 8 individual LEDs, 1 bit per LED
 */
static rt_uint8_t shift_register_output = 0x00;

/**
 * @brief SPI device handle
 */
static struct rt_device* s_spi_dev = RT_NULL;

/**
 * @brief Initialization flag
 */
static rt_bool_t s_initialized = RT_FALSE;

/**
 * @brief LED7/LED8 software timer structure (for host commands)
 */
typedef struct
{
 struct rt_timer timer_obj; // Timer handle
 rt_tick_t on_ticks; // ON duration (ticks)
 rt_tick_t off_ticks; // OFF duration (ticks)
 uint16_t repeat_count; // Repeat count (0=infinite loop)
 uint16_t remaining_repeats; // Remaining repeat count
 rt_bool_t is_flashing; // Is currently flashing
 rt_bool_t last_state; // Last state
} LedSoftTimer_t;

static LedSoftTimer_t s_led_timers[2]; // Index 0=LED7(object ID 6), Index 1=LED8(object ID 7)


/* ========== Internal Function Prototypes ========== */

/**
 * @brief Update LED state to shift register buffer
 */
static void led_update_shift_register(void);

/**
 * @brief Send data to 74HC595 via SPI (with mutex lock)
 */
static void led_send_to_spi(void);

/**
 * @brief Pull down RCK to prepare latch
 */
static void led_prepare_rck_low(void);

/**
 * @brief Pull up RCK to trigger output
 */
static void led_trigger_rck_high(void);

/**
 * @brief LED software timer callback function
 */
static void led_soft_timer_callback(void *parameter);


/* ========== Public Functions Implementation ========== */

/**
 * @brief Initialize LED driver
 */
void led_init(void)
{
 uint8_t i;
 /* 1. 配置 RCLK 引脚为输出，并初始化为低电平 */
 rt_pin_mode(LED_RCLK_PIN, PIN_MODE_OUTPUT);
 rt_pin_write(LED_RCLK_PIN, PIN_LOW);

 /* 1. Find SPI2 device */
 s_spi_dev = rt_device_find("spi2");
 if (s_spi_dev == RT_NULL)
 {
 rt_kprintf("[LED] Error: SPI2 device not found!\n");
 return;
 }
 rt_kprintf("[LED] SPI2 device found\n");

 /* 3. 打开 SPI 设备（配置为读写模式）*/
 rt_err_t ret = rt_device_open(s_spi_dev, RT_DEVICE_FLAG_RDWR);
 if (ret != RT_EOK && ret != -RT_EBUSY) // -RT_EBUSY 表示已被 OLED 打开，可接受
 {
 rt_kprintf("[LED] Error: Failed to open SPI2, ret=%d\n", ret);
 return;
 }
 rt_kprintf("[LED] SPI2 opened (or already opened by OLED)\n");

 /* 2. Configure all LED objects */
 for (i = 0; i < LED_TOTAL_COUNT; i++)
 {
 led_objects[i].id = i;
 led_objects[i].color_state = LED_COLOR_OFF;

 led_objects[i].enabled = RT_TRUE; // Enable by default
 }

 /* 3. Initialize host LED software timers */
 for (int i = 0; i < 2; i++)
 {
 rt_memset(&s_led_timers[i], 0, sizeof(LedSoftTimer_t));
 }

 /* 4. Initialize to all OFF state */
 led_off_all();

 /* 5. Mark as initialized */
 s_initialized = RT_TRUE;

 rt_kprintf("[LED] Initialized %d LEDs via SPI+74HC595\n", LED_TOTAL_COUNT);
}


/**
 * @brief Update LED states to shift register buffer
 */
static void led_update_shift_register(void)
{
 shift_register_output = 0x00;

 for (int i = 0; i < LED_TOTAL_COUNT; i++)
 {
 if (led_objects[i].enabled && led_objects[i].color_state != LED_COLOR_OFF)
 {
 shift_register_output |= (1 << i); // Set ith bit
 }
 }
}


/**
 * @brief Pull down RCK (PB2) to prepare latch
 */
static void led_prepare_rck_low(void)
{
 rt_pin_write(LED_RCLK_PIN, PIN_LOW);
}


/**
 * @brief Short pulse pull up RCK to trigger 74HC595 output
 */
static void led_trigger_rck_high(void)
{
 rt_pin_write(LED_RCLK_PIN, PIN_HIGH);
 rt_thread_mdelay(1);
 rt_pin_write(LED_RCLK_PIN, PIN_LOW);
}


/**
 * @brief Send to SPI with mutual exclusion protection (ensure no conflict with OLED)
 */
static void led_send_to_spi(void)
{
 if (!s_initialized || s_spi_dev == RT_NULL)
 {
 rt_kprintf("[LED] Not initialized or no SPI device\n");
 return;
 }

 uint8_t tx_byte = shift_register_output;
 rt_kprintf("[LED] Sending 0x%02x to SPI\n", tx_byte); // 调试打印

 /* Acquire SPI mutex protection */
 oled_spi_lock();

 /* 1. Pull down RCK, start shift */
 led_prepare_rck_low();

 /* 2. Write 1 byte via SPI2 */
 /* SPI2: PB13=SCK, PB15=MOSI */
 int ret = rt_device_write(s_spi_dev, 0, &tx_byte, 1);
 if (ret != 1)
 {
 rt_kprintf("[LED] SPI write error: %d\n", ret);
 }

 /* 3. Delay then pull up RCK to trigger output latch */
 rt_thread_mdelay(1);
 led_trigger_rck_high();

 /* Release mutex */
 oled_spi_unlock();
}


/**
 * @brief Set color of specified LED
 * @param id LED ID (0-7)
 * @param state Color state to set
 * @return 0 on success, -RT_ERROR on failure
 */
int led_set_color(rt_uint8_t id, LedColorState_t state)
{
 if (!s_initialized)
 {
 rt_kprintf("[LED] Error: Not initialized yet!\n");
 return -RT_ERROR;
 }

 if (id >= LED_TOTAL_COUNT)
 {
 rt_kprintf("[LED] Error: Invalid ID %d\n", id);
 return -RT_ERROR;
 }

 led_objects[id].color_state = state;

 /* Update shift register and send immediately */
 led_update_shift_register();
 led_send_to_spi();

 return 0;
}


/**
 * @brief Get color state of specified LED
 * @param id LED ID (0-7)
 * @return Current color state
 */
LedColorState_t led_get_color(rt_uint8_t id)
{
 if (id >= LED_TOTAL_COUNT || !s_initialized)
 {
 return LED_COLOR_OFF;
 }

 return led_objects[id].color_state;
}


/**
 * @brief Enable or disable specific LED
 * @param id LED ID (0-7)
 * @param on RT_TRUE to enable, RT_FALSE to disable
 * @return 0 on success, -RT_ERROR on failure
 */
int led_enable(rt_uint8_t id, rt_bool_t on)
{
 if (id >= LED_TOTAL_COUNT || !s_initialized)
 {
 return -RT_ERROR;
 }

 led_objects[id].enabled = on;

 if (!on)
 {
 led_objects[id].color_state = LED_COLOR_OFF;
 led_update_shift_register();
 led_send_to_spi();
 }

 return 0;
}


/**
 * @brief Check if LED is enabled
 * @param id LED ID (0-7)
 * @return RT_TRUE if enabled, RT_FALSE otherwise
 */
rt_bool_t led_is_enabled(rt_uint8_t id)
{
 if (id >= LED_TOTAL_COUNT || !s_initialized)
 {
 return RT_FALSE;
 }

 return led_objects[id].enabled;
}


/**
 * @brief Set multiple LEDs at once
 * @param start_id Starting LED ID
 * @param count Number of LEDs to set
 * @param states Array of color states
 * @return 0 on success, -RT_ERROR on failure
 */
int led_set_multiple(rt_uint8_t start_id, rt_uint8_t count, const LedColorState_t* states)
{
 if (start_id + count > LED_TOTAL_COUNT || states == NULL || !s_initialized)
 {
 return -RT_ERROR;
 }

 for (int i = 0; i < count; i++)
 {
 led_set_color(start_id + i, states[i]);
 }

 return 0;
}


/**
 * @brief Turn all LEDs OFF
 */
void led_off_all(void)
{
 if (!s_initialized)
 {
 return;
 }

 for (uint8_t i = 0; i < LED_TOTAL_COUNT; i++)
 {
 led_objects[i].color_state = LED_COLOR_OFF;
 }

 shift_register_output = 0x00;
 led_send_to_spi();
}


/**
 * @brief Turn ALL GREEN LEDs ON (LED0, LED2, LED4, LED6)
 */
void led_on_all_green(void)
{
 if (!s_initialized)
 {
 return;
 }

 /* LED0, LED2, LED4, LED6 are green LEDs */
 led_objects[0].color_state = LED_COLOR_GREEN_ON;
 led_objects[2].color_state = LED_COLOR_GREEN_ON;
 led_objects[4].color_state = LED_COLOR_GREEN_ON;
 led_objects[6].color_state = LED_COLOR_GREEN_ON;

 led_update_shift_register();
 led_send_to_spi();
}


/**
 * @brief Turn ALL RED LEDs ON (LED1, LED3, LED5, LED7)
 */
void led_on_all_red(void)
{
 if (!s_initialized)
 {
 return;
 }

 /* LED1, LED3, LED5, LED7 are red LEDs */
 led_objects[1].color_state = LED_COLOR_RED_ON;
 led_objects[3].color_state = LED_COLOR_RED_ON;
 led_objects[5].color_state = LED_COLOR_RED_ON;
 led_objects[7].color_state = LED_COLOR_RED_ON;

 led_update_shift_register();
 led_send_to_spi();
}


/**
 * @brief Run LED scan effect (chasing lights)
 */
void led_scan_effect(void)
{
 if (!s_initialized)
 {
 return;
 }

 for (uint8_t i = 0; i < LED_TOTAL_COUNT; i++)
 {
 led_objects[i].color_state = LED_COLOR_GREEN_ON;
 led_update_shift_register();
 led_send_to_spi();
 rt_thread_mdelay(500);

 led_objects[i].color_state = LED_COLOR_OFF;
 led_update_shift_register();
 led_send_to_spi();
 rt_thread_mdelay(500);
 }
}


/* ========== Preset Scenario Interfaces ========== */

/**
 * @brief Set working status indicator
 * @param running RT_TRUE if running, RT_FALSE if stopped
 */
void led_set_working(rt_bool_t running)
{
 if (!s_initialized)
 {
 return;
 }

 LedColorState_t state = running ? LED_COLOR_GREEN_ON : LED_COLOR_OFF;
 led_set_color(LED_IDX_WORKING_GREEN, state);
}


/**
 * @brief Set fault status indicator
 * @param faulted RT_TRUE if faulted, RT_FALSE if normal
 */
void led_set_fault(rt_bool_t faulted)
{
 if (!s_initialized)
 {
 return;
 }

 LedColorState_t state = faulted ? LED_COLOR_RED_ON : LED_COLOR_OFF;
 led_set_color(LED_IDX_FAULT_RED, state);
}


/**
 * @brief Set battery full indicator
 * @param full RT_TRUE if full, RT_FALSE otherwise
 */
void led_set_battery_full(rt_bool_t full)
{
 if (!s_initialized)
 {
 return;
 }

 LedColorState_t state = full ? LED_COLOR_GREEN_ON : LED_COLOR_OFF;
 led_set_color(LED_IDX_FULL_BAT_GREEN, state);
}


/**
 * @brief Set battery low warning indicator
 * @param low RT_TRUE if low, RT_FALSE otherwise
 */
void led_set_battery_low(rt_bool_t low)
{
 if (!s_initialized)
 {
 return;
 }

 LedColorState_t state = low ? LED_COLOR_FLASH_FAST : LED_COLOR_OFF;
 led_set_color(LED_IDX_LOW_BAT_RED, state);
}


/**
 * @brief Set water tank full indicator
 * @param full RT_TRUE if full, RT_FALSE otherwise
 */
void led_set_water_full(rt_bool_t full)
{
 if (!s_initialized)
 {
 return;
 }

 LedColorState_t state = full ? LED_COLOR_GREEN_ON : LED_COLOR_OFF;
 led_set_color(LED_IDX_FULL_WATER_GREEN, state);
}


/**
 * @brief Set water tank low warning indicator
 * @param low RT_TRUE if low, RT_FALSE otherwise
 */
void led_set_water_low(rt_bool_t low)
{
 if (!s_initialized)
 {
 return;
 }

 LedColorState_t state = low ? LED_COLOR_FLASH_SLOW : LED_COLOR_OFF;
 led_set_color(LED_IDX_LOW_WATER_RED, state);
}


/* ========== Host Control LED Functions (NEW) ========== */

/**
 * @brief Stop LED soft timer flashing
 * @param led_idx 0=LED7, 1=LED8
 */
void led_stop_flash(uint8_t led_idx)
{
 if (led_idx >= 2 || !s_initialized)
 {
 return;
 }

 LedSoftTimer_t* timer = &s_led_timers[led_idx];

 if (timer->is_flashing)
 {
 rt_timer_stop(&timer->timer_obj);
 timer->is_flashing = RT_FALSE;
 timer->remaining_repeats = 0;

 /* Force turn off LED */
 led_objects[led_idx + 6].color_state = LED_COLOR_OFF; /* LED7=index 6, LED8=index 7 */
 led_update_shift_register();
 led_send_to_spi();
 }
}


/**
 * @brief Start LED soft timer flashing
 * @param led_idx 0=LED7, 1=LED8
 * @param on_time_ms ON duration in milliseconds
 * @param off_time_ms OFF duration in milliseconds
 * @param repeat Repeat count (0=infinite loop)
 * @return 0 on success, -RT_ERROR on failure
 */
int led_flash(uint8_t led_idx, uint16_t on_time_ms, uint16_t off_time_ms, uint16_t repeat)
{
 if (led_idx >= 2 || !s_initialized)
 {
 return -RT_ERROR;
 }

 LedSoftTimer_t* timer = &s_led_timers[led_idx];
// int led_obj_idx = led_idx + 6; /* LED7=6, LED8=7 */

 /* If already flashing, stop old one first */
 if (timer->is_flashing)
 {
 rt_timer_stop(&timer->timer_obj);
 }

 /* Configure timer parameters */
 timer->on_ticks = on_time_ms * RT_TICK_PER_SECOND / 1000;
 timer->off_ticks = off_time_ms * RT_TICK_PER_SECOND / 1000;
 timer->repeat_count = repeat;
 timer->remaining_repeats = repeat;
 timer->last_state = RT_TRUE;
 timer->is_flashing = RT_TRUE;

 // 初始化定时器对象（如果尚未初始化，可重复调用）
 rt_timer_init(&timer->timer_obj,
 "ledftmr",
 led_soft_timer_callback,
 timer, // 传递 timer 指针作为参数
 timer->on_ticks, // 初始超时时间设为 on_ticks
 RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER); // 使用软定时器（线程上下文）

 /* Start timer */
 rt_timer_start(&timer->timer_obj);
 timer->is_flashing = RT_TRUE;

 rt_kprintf("[LED] Flash started: LED%d on=%dms off=%dms repeat=%d\n", 
 led_idx + 7, on_time_ms, off_time_ms, repeat);

 return 0;
}


/**
 * @brief LED soft timer callback (checks every 1ms)
 */
static void led_soft_timer_callback(void *parameter)
{ 
 LedSoftTimer_t *led_timer = (LedSoftTimer_t *)parameter;
 int timer_idx = (led_timer == &s_led_timers[0]) ? 0 : 1;
 int led_obj_idx = timer_idx + 6; /* LED7=6, LED8=7 */

 static rt_tick_t s_elapsed_ms[2] = {0}; /* Record elapsed time */

 s_elapsed_ms[timer_idx]++;

 /* Calculate current state based on timing */
 rt_tick_t total_cycle_ticks = led_timer->on_ticks + led_timer->off_ticks;
 rt_tick_t elapsed_in_cycle = s_elapsed_ms[timer_idx] % total_cycle_ticks;

 rt_bool_t should_be_on = (elapsed_in_cycle < led_timer->on_ticks);

 if (should_be_on != led_timer->last_state)
 {
 /* State changed */
 led_timer->last_state = should_be_on;

 if (should_be_on)
 {
 /* Turn ON */
 led_objects[led_obj_idx].color_state = LED_COLOR_ON;

 /* Check if need to stop */
 if (led_timer->remaining_repeats > 0)
 {
 led_timer->remaining_repeats--;
 if (led_timer->remaining_repeats == 0)
 {
 /* All flashes complete, keep final ON state and stop timer */
 s_elapsed_ms[timer_idx] = 0;
 return; /* Keep ON, don't turn off */
 }
 }
 }
 else
 {
 /* Turn OFF */
 led_objects[led_obj_idx].color_state = LED_COLOR_OFF;
 }

 /* Update shift register and send via SPI (with mutex) */
 led_update_shift_register();
 oled_spi_lock();
 if (s_initialized && s_spi_dev)
 {
 led_prepare_rck_low();
 uint8_t tx_byte = shift_register_output;
 rt_device_write(s_spi_dev, 0, &tx_byte, 1);
 rt_thread_mdelay(1);
 led_trigger_rck_high();
 }
 oled_spi_unlock();
 }
}


/**
 * @brief Power-on LED sequence effect
 */
void led_poweron_sequence(void)
{
 led_scan_effect();
 led_set_working(RT_TRUE);
 rt_kprintf("[LED] Power-on sequence complete\n");
}


/**
 * @brief Shutdown LED sequence effect
 */
void led_shutdown_sequence(void)
{
 /* First stop all host LED flashing */
 led_stop_flash(0); /* LED7 */
 led_stop_flash(1); /* LED8 */

 led_off_all();
 rt_kprintf("[LED] Shutdown sequence complete\n");
}


/* ========== Debug MSH Commands ========== */

#ifdef RT_USING_MSH

#include <msh.h>

static void led_on(int argc, char** argv)
{
 if (argc < 2)
 {
 rt_kprintf("Usage: led on <0-7>\n");
 return;
 }

 int id = atoi(argv[1]);
 if (id < 0 || id >= LED_TOTAL_COUNT)
 {
 rt_kprintf("Invalid LED ID: %d\n", id);
 return;
 }

 led_enable(id, RT_TRUE);
 led_set_color(id, LED_COLOR_GREEN_ON);
 rt_kprintf("LED%d turned on\n", id);
}


static void led_off(int argc, char** argv)
{
 if (argc < 2)
 {
 rt_kprintf("Usage: led off <0-7>\n");
 return;
 }

 int id = atoi(argv[1]);
 if (id < 0 || id >= LED_TOTAL_COUNT)
 {
 rt_kprintf("Invalid LED ID: %d\n", id);
 return;
 }

 led_enable(id, RT_FALSE);
 rt_kprintf("LED%d turned off\n", id);
}


static void led_test(int argc, char** argv)
{
 rt_kprintf("Running LED scan effect...\n");
 led_scan_effect();
 rt_kprintf("Done!\n");
}


static void led_status(int argc, char** argv)
{
 rt_kprintf("\n========== LED Status ==========\n");
 for (int i = 0; i < LED_TOTAL_COUNT; i++)
 {
 const char* name = "";
 switch (i)
 {
 case 0: name = "Working(Green)"; break;
 case 1: name = "Fault(Red)"; break;
 case 2: name = "BatFull(Green)"; break;
 case 3: name = "BatLow(Red)"; break;
 case 4: name = "WaterFull(Green)"; break;
 case 5: name = "WaterLow(Red)"; break;
 case 6: name = "Nav-Green"; break; /* Host LED7 */
 case 7: name = "Error-Red"; break; /* Host LED8 */
 default: name = "Unknown";
 }

 rt_kprintf("%-16s [%d]: Enabled=%s, State=0x%02X\n", 
 name, i,
 led_is_enabled(i) ? "Yes" : "No",
 led_get_color(i));
 }
 rt_kprintf("================================\n\n");
}

MSH_CMD_EXPORT(led_on, "led on <0-7>");
MSH_CMD_EXPORT(led_off, "led off <0-7>");
MSH_CMD_EXPORT(led_test, "led test");
MSH_CMD_EXPORT(led_status, "led status");

#endif /* RT_USING_MSH */