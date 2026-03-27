/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief Monitor - Analog signal monitoring (battery, charger, heater)
 */

#ifndef PERIPHERALS_MONITOR_H__
#define PERIPHERALS_MONITOR_H__

#include <rtthread.h>

/**
 * @brief Battery voltage in mV (0~25200)
 */
rt_uint32_t monitor_get_battery_voltage(void);

/**
 * @brief Charger port voltage in mV (0~3300)
 */
rt_uint32_t monitor_get_charger_voltage(void);
// 获取充电采样电压（PA4）
rt_uint32_t monitor_get_charger_sample_voltage(void);   

/**
 * @brief Heater power supply detection voltage in mV (0~3300)
 *        A value > 500mV indicates external power is connected.
 */
rt_uint32_t monitor_get_heater_voltage(void);

/**
 * @brief Initialize monitoring system (creates a thread to periodically read ADC)
 */
void monitor_init(void);

#endif /* PERIPHERALS_MONITOR_H__ */
