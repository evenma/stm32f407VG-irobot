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

// 定义通道索引，方便数组访问
enum monitor_channel {
    MONITOR_CH_BATTERY = 0,      // PC3, CH13
    MONITOR_CH_CHARGER_DET,      // PA5, CH5
    MONITOR_CH_HEATER_DET,       // PA6, CH6
    MONITOR_CH_CHARGER_SAMPLE,   // PA4, CH4
    MONITOR_CH_COUNT
};
typedef struct {
    uint32_t magic;               // 校验魔数，用于判断数据是否有效
    uint32_t version;             // 版本号，便于将来扩展
    float scale[MONITOR_CH_COUNT]; // 各通道比例系数
    int32_t charge_diff_offset;    // 充电压差偏移（mV）
    // 可以继续添加其他需要保存的校准参数
		uint32_t ultrasonic_baudrate;  // 超声波传感器波特率
} MonitorCalibData_t;

#define MONITOR_CALIB_MAGIC  0x4D4F4E49   // "MONI" 的 ASCII 码
#define MONITOR_CALIB_VERSION 2


rt_bool_t monitor_get_water_high_level(void);
rt_bool_t monitor_get_water_low_level(void);

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

uint32_t monitor_get_ultrasonic_baudrate(void);

#endif /* PERIPHERALS_MONITOR_H__ */
