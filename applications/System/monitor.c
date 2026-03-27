#include <rtthread.h>
#include <rtdevice.h>
#include "monitor.h"
#include "global_conf.h"
#include "print_utils.h"
#include "led.h"
#include "buzzer.h"
#include "board.h"
#include <string.h>

// 声明外部变量，用于 OLED 显示
extern int g_oled_battery_mv;

/* ========== Thread Configuration ========== */
#define MONITOR_THREAD_STACK_SIZE   1024
#define MONITOR_THREAD_PRIORITY     22
#define MONITOR_SAMPLE_INTERVAL_MS  50          /* 采样间隔 50ms */
#define FILTER_WINDOW_SIZE          10          /* 滤波窗口大小 */
#define PRINT_INTERVAL              10          /* 每10次滤波打印一次（约5秒一次） */

/* ========== Internal Reference Voltage Address (STM32F407) ========== */
//#define VREFINT_CAL_ADDR            (uint16_t*)0x1FFF7A2A   /* 3.3V 时校准值 */

// 定义通道索引，方便数组访问
enum monitor_channel {
    MONITOR_CH_BATTERY = 0,      // PC3, CH13
    MONITOR_CH_CHARGER_DET,      // PA5, CH5
    MONITOR_CH_HEATER_DET,       // PA6, CH6
    MONITOR_CH_CHARGER_SAMPLE,   // PA4, CH4
    MONITOR_CH_COUNT
};

// 比例系数数组（默认值）
static float s_scale[MONITOR_CH_COUNT] = {
    [MONITOR_CH_BATTERY]        = 222.0f / 22.0f,   // 10.0909
    [MONITOR_CH_CHARGER_DET]    = 222.0f / 22.0f,   // 10.0909
    [MONITOR_CH_HEATER_DET]     = 222.0f / 22.0f,   // 10.0909
    [MONITOR_CH_CHARGER_SAMPLE] = 222.0f / 22.0f,   // 10.0909
};

/* ========== Internal Variables ========== */
static rt_adc_device_t s_adc_dev = RT_NULL;
static rt_thread_t s_monitor_thread = RT_NULL;

/* 原始采样缓冲区（用于滤波） */
static struct {
    rt_uint32_t battery_raw[FILTER_WINDOW_SIZE];
    rt_uint32_t charger_raw[FILTER_WINDOW_SIZE];
    rt_uint32_t heater_raw[FILTER_WINDOW_SIZE];
		rt_uint32_t charger_sample_raw[FILTER_WINDOW_SIZE];
    rt_uint32_t vrefint_raw[FILTER_WINDOW_SIZE];
    rt_uint8_t index;
    rt_uint8_t ready;   /* 已采集满一个窗口 */
} s_filter_buf;

/* 滤波后的电压值（单位：mV） */
static rt_uint32_t s_battery_mv = 0;
static rt_uint32_t s_charger_mv = 0;
static rt_uint32_t s_heater_mv  = 0;
static rt_uint32_t s_charger_sample_mv = 0;   

/* VREFINT 校准值（在 3.3V VDDA 下的原始读数） */
static uint16_t s_vrefint_cal = 0;
static float    s_vdda_mv = 3300.0f;   /* 默认 3.3V，后续通过 VREFINT 校准 */

/* 打印计数器 */
static rt_uint32_t s_print_cnt = 0;
static rt_bool_t s_monitor_print_enabled = RT_FALSE;   // 默认关闭打印

// 低电量报警状态
static rt_bool_t s_low_battery_alarmed = RT_FALSE;
static rt_uint32_t s_alarm_count = 0;
static rt_tick_t s_last_alarm_tick = 0;
static rt_bool_t s_last_low_battery = RT_FALSE;

// 添加静态变量存储功率（毫瓦）
static rt_uint32_t s_charge_power_mw = 0;
static int32_t s_charge_diff_offset = 225;          // 充电压差静态偏移（mV），未充电时为0，开启充电有压差，可能是锂电池内部充电MOS管管压
static rt_bool_t s_offset_calibrated = RT_FALSE;  // 偏移是否已校准

/* ========== Utility Functions ========== */

/**
 * @brief 去极值平均滤波（去掉最大最小值）
 * @param arr 原始数据数组（长度为 FILTER_WINDOW_SIZE）
 * @return 滤波后的平均值（uint32_t）
 */
static rt_uint32_t filter_average(rt_uint32_t arr[FILTER_WINDOW_SIZE])
{
    rt_uint32_t sum = 0;
    rt_uint32_t min = arr[0], max = arr[0];

    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        sum += arr[i];
        if (arr[i] < min) min = arr[i];
        if (arr[i] > max) max = arr[i];
    }
    sum -= (min + max);                    /* 去掉最大值和最小值 */
    return sum / (FILTER_WINDOW_SIZE - 2); /* 剩余 8 个求平均 */
}

/**
 * @brief 通过内部参考电压校准 VDDA
 * @param vrefint_raw 当前读取的 VREFINT 原始值
 */
static void calibrate_vdda(uint32_t vrefint_raw)
{
    /* VREFINT_CAL 是在 3.3V VDDA 下测得的校准值 */
    if (s_vrefint_cal == 0) {
        s_vrefint_cal = *VREFINT_CAL_ADDR;
    }
    if (vrefint_raw != 0) {
        s_vdda_mv = 3300.0f * (float)s_vrefint_cal / (float)vrefint_raw;
    }
}

/**
 * @brief 根据原始 ADC 值计算实际电压（使用当前 VDDA）
 * @param raw ADC 原始值（0~4095）
 * @return 电压（mV）
 */
static rt_uint32_t adc_raw_to_mv(rt_uint32_t raw)
{
    return (rt_uint32_t)((raw * s_vdda_mv) / 4095.0f);
}

/**
 * @brief 电池电压换算（分压网络 R1=200k, R2=22k）
 * @param sample_mv 分压点电压（mV）
 * @return 电池电压（mV）
 */
static rt_uint32_t battery_mv_from_sample(rt_uint32_t sample_mv)
{
    return (sample_mv * 222) / 22;
}

/* ========== ADC 读取函数 ========== */

static void adc_read_all_channels(rt_uint32_t *battery_raw,
                                  rt_uint32_t *charger_raw,
                                  rt_uint32_t *heater_raw,
																	rt_uint32_t *charger_sample_raw,
                                  rt_uint32_t *vrefint_raw)
{
    // 电池通道 (PC3, ADC1_CH13)
    rt_adc_enable(s_adc_dev, ADC1_CH13);
    *battery_raw = rt_adc_read(s_adc_dev, ADC1_CH13);
    rt_adc_disable(s_adc_dev, ADC1_CH13);

    // 充电检测通道 (PA5, ADC1_CH5)
    rt_adc_enable(s_adc_dev, ADC1_CH5);
    *charger_raw = rt_adc_read(s_adc_dev, ADC1_CH5);
    rt_adc_disable(s_adc_dev, ADC1_CH5);
	    
		// 充电采样通道 (PA4, ADC1_CH4)
    rt_adc_enable(s_adc_dev, 4);
    *charger_sample_raw = rt_adc_read(s_adc_dev, ADC1_CH4);
    rt_adc_disable(s_adc_dev, 4);

    // 加热器检测通道 (PA6, ADC1_CH6)
    rt_adc_enable(s_adc_dev, ADC1_CH6);
    *heater_raw = rt_adc_read(s_adc_dev, ADC1_CH6);
    rt_adc_disable(s_adc_dev, ADC1_CH6);

    // 内部参考电压 (VREFINT, ADC1_CH17)
    rt_adc_enable(s_adc_dev, ADC1_CH17);
    *vrefint_raw = rt_adc_read(s_adc_dev, ADC1_CH17);
    rt_adc_disable(s_adc_dev, ADC1_CH17);
}

/* ========== Monitor Thread ========== */
static void monitor_thread_entry(void *parameter)
{
    rt_uint32_t battery_raw, charger_raw, heater_raw, charger_sample_raw, vrefint_raw;
    rt_uint32_t filtered_battery, filtered_charger, filtered_heater, filtered_charger_sample, filtered_vrefint;
		int32_t effective_diff = 0;
	
    s_filter_buf.index = 0;
    s_filter_buf.ready = 0;
		
		rt_thread_mdelay(5000);    // 等待系统启动稳定后开启

    while (1)
    {
        adc_read_all_channels(&battery_raw, &charger_raw, &heater_raw, &charger_sample_raw, &vrefint_raw);

        // 存入环形缓冲区
        s_filter_buf.battery_raw[s_filter_buf.index] = battery_raw;
        s_filter_buf.charger_raw[s_filter_buf.index] = charger_raw;
        s_filter_buf.heater_raw[s_filter_buf.index]  = heater_raw;
        s_filter_buf.charger_sample_raw[s_filter_buf.index] = charger_sample_raw;
        s_filter_buf.vrefint_raw[s_filter_buf.index] = vrefint_raw;

        s_filter_buf.index++;
        if (s_filter_buf.index >= FILTER_WINDOW_SIZE) {
            s_filter_buf.index = 0;
            s_filter_buf.ready = 1;
        }

        if (s_filter_buf.ready) {
            filtered_battery = filter_average(s_filter_buf.battery_raw);
            filtered_charger = filter_average(s_filter_buf.charger_raw);
            filtered_heater  = filter_average(s_filter_buf.heater_raw);
            filtered_charger_sample = filter_average(s_filter_buf.charger_sample_raw);
            filtered_vrefint = filter_average(s_filter_buf.vrefint_raw);

            calibrate_vdda(filtered_vrefint);

            // 转换为实际电压（mV）
            rt_uint32_t sample_battery_mv = adc_raw_to_mv(filtered_battery);
            rt_uint32_t sample_charger_mv = adc_raw_to_mv(filtered_charger);
            rt_uint32_t sample_heater_mv  = adc_raw_to_mv(filtered_heater);
            rt_uint32_t sample_charger_sample_mv = adc_raw_to_mv(filtered_charger_sample);

            // 应用校准系数
            s_battery_mv = (rt_uint32_t)(sample_battery_mv * s_scale[MONITOR_CH_BATTERY]);
            s_charger_mv = (rt_uint32_t)(sample_charger_mv * s_scale[MONITOR_CH_CHARGER_DET]);
            s_heater_mv  = (rt_uint32_t)(sample_heater_mv  * s_scale[MONITOR_CH_HEATER_DET]);
            s_charger_sample_mv = (rt_uint32_t)(sample_charger_sample_mv * s_scale[MONITOR_CH_CHARGER_SAMPLE]);

            s_filter_buf.ready = 0;
	
						// 更新 OLED 显示的全局电池电压
						g_oled_battery_mv = s_battery_mv;	
						
						// 满电指示 (≥ 25.2V)
						if (s_battery_mv >= BATTERY_FULL_VOLTAGE_MV) {
								led_set_battery_full(RT_TRUE);
						} else {
								led_set_battery_full(RT_FALSE);
						}
						
					  // 低电量报警
						if (s_battery_mv <= BATTERY_LOW_ALARM_MV) {
							    if (!s_last_low_battery) {
											led_set_battery_low(RT_TRUE);
											s_last_low_battery = RT_TRUE;
											rt_kprintf("[MONITOR]low battery\n");
									}
								// 未报警或报警次数未满 10 次
								if (s_alarm_count < 10) {
										rt_tick_t now = rt_tick_get();
										if (now - s_last_alarm_tick >= RT_TICK_PER_SECOND * 10) {  // 10 秒间隔
												buzzer_beep_once();   // 短促蜂鸣一次，可自定义更合适的警报声
												s_last_alarm_tick = now;
												s_alarm_count++;
												s_low_battery_alarmed = RT_TRUE;
										}
								}
							}else if (s_battery_mv >= BATTERY_LOW_ALARM_MV + BATTERY_LOW_HYSTERESIS_MV) {
							    if (s_last_low_battery) {
											led_set_battery_low(RT_FALSE);
											s_last_low_battery = RT_FALSE;
											rt_kprintf("[MONITOR] normal battery\n");
									}
								// 电压恢复正常，重置报警状态
								if (s_low_battery_alarmed) {
										s_alarm_count = 0;
										s_low_battery_alarmed = RT_FALSE;
								}
						}					
							
						// 检测MOSFET 关闭时，校准静态偏移
						if (!s_offset_calibrated && rt_pin_read(CHARGER_CONTROL_PIN) == PIN_LOW) {
								// 计算当前采样电压与电池电压的原始差值（未经 scale）
								int32_t diff_raw = (int32_t)sample_charger_sample_mv - (int32_t)sample_battery_mv;
								if (abs(diff_raw) > 10) {  // 偏差超过 10mV 才校准，避免微小抖动
										rt_kprintf("[MONITOR] diff_raw=%d mV\n", (int)diff_raw);
								}
								s_offset_calibrated = RT_TRUE;
						}
							
						// 计算充电功率：仅在 MOSFET 开启且有物理连接时
						// 充电电流 I = diff_mV / 10 （因为采样电阻 0.01Ω，压差 1mV 对应 0.1A，但公式推导后直接计算功率）
						// 充电功率 P = I * V_charge  = (diff_mV / 10) * (s_charger_sample_mV / 1000) = diff_mV * s_charger_sample_mV / 10000 （单位 W）
						// 转为毫瓦：P_mW = diff_mV * s_charger_sample_mV / 10
						if (rt_pin_read(CHARGER_CONTROL_PIN) == PIN_HIGH && s_charger_mv > 5000) {  
//								int32_t raw_diff = (int32_t)sample_charger_sample_mv - (int32_t)sample_battery_mv;
//								effective_diff = raw_diff - s_charge_diff_offset;
//								
//								if (effective_diff > 1) {  // 阈值可调，避免噪声
//										// 功率（毫瓦）= effective_diff * s_charger_sample_mv / 10
//										s_charge_power_mw = (rt_uint32_t)(((int64_t)(effective_diff  * s_charger_sample_mv)) / 10);
//								} else {
//										s_charge_power_mw = 0;
//								}			
									int32_t diff = (int32_t)s_charger_sample_mv - (int32_t)s_battery_mv - s_charge_diff_offset;
									effective_diff = diff;
									if (diff > 10) {
											s_charge_power_mw = (rt_uint32_t)(((int64_t)diff * s_charger_mv) / 10);
									} else {
											s_charge_power_mw = 0;
									}							
						} else {
								s_charge_power_mw = 0;
						}


            // 打印（调试）
            s_print_cnt++;
            if (s_monitor_print_enabled && s_print_cnt >= PRINT_INTERVAL) {
                s_print_cnt = 0;
                rt_kprintf("[MONITOR] VDDA=%d mV\n", (int)s_vdda_mv);
                rt_kprintf("[MONITOR] Bat=%d mV (raw=%lu, sample=%d mV) ",
                           s_battery_mv, filtered_battery, sample_battery_mv);
								rt_kprintf("scale = ");
								print_float(s_scale[MONITOR_CH_BATTERY], 6, 3, 1);
								rt_kprintf("\n");
                rt_kprintf("[MONITOR] ChargerDet=%d mV (raw=%lu, sample=%d mV) ",
                           s_charger_mv, filtered_charger, sample_charger_mv);
							  rt_kprintf("scale = ");
								print_float(s_scale[MONITOR_CH_CHARGER_DET], 6, 3, 1);
								rt_kprintf("\n");
                rt_kprintf("[MONITOR] HeaterDet=%d mV (raw=%lu, sample=%d mV) ",
                           s_heater_mv, filtered_heater, sample_heater_mv);
								rt_kprintf("scale = ");
								print_float(s_scale[MONITOR_CH_HEATER_DET], 6, 3, 1);
								rt_kprintf("\n");
                rt_kprintf("[MONITOR] ChargerSample=%d mV (raw=%lu, sample=%d mV) ",
                           s_charger_sample_mv, filtered_charger_sample, sample_charger_sample_mv);
							  rt_kprintf("scale = ");
								print_float(s_scale[MONITOR_CH_CHARGER_SAMPLE], 6, 3, 1);
								rt_kprintf("\n");
								rt_kprintf("[MONITOR] charge power=%d mW (eff diff=%d mV)\n", s_charge_power_mw, effective_diff);
            }
        }

        rt_thread_mdelay(MONITOR_SAMPLE_INTERVAL_MS);
    }
}

/* ========== Public API ========== */

rt_uint32_t monitor_get_battery_voltage(void)
{
    return s_battery_mv;
}

rt_uint32_t monitor_get_charger_voltage(void)
{
    return s_charger_mv;
}

rt_uint32_t monitor_get_heater_voltage(void)
{
    return s_heater_mv;
}

rt_uint32_t monitor_get_charger_sample_voltage(void)
{
    return s_charger_sample_mv;
}

void monitor_init(void)
{
//    rt_err_t ret;

    /* 查找 ADC 设备 */
    s_adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (s_adc_dev == RT_NULL) {
        rt_kprintf("[MONITOR] ERROR: ADC device '%s' not found!\n", ADC_DEV_NAME);
        return;
    }

    /* 使能所有需要的 ADC 通道 */
//    ret = rt_adc_enable(s_adc_dev, BATTERY_ADC_CHANNEL);
//    if (ret != RT_EOK) rt_kprintf("[MONITOR] Enable battery ADC failed\n");
//    ret = rt_adc_enable(s_adc_dev, CHARGER_DETECT_ADC);
//    if (ret != RT_EOK) rt_kprintf("[MONITOR] Enable charger ADC failed\n");
//    ret = rt_adc_enable(s_adc_dev, HEATER_DETECT_ADC);
//    if (ret != RT_EOK) rt_kprintf("[MONITOR] Enable heater ADC failed\n");
//    ret = rt_adc_enable(s_adc_dev, ADC1_CH17);  /* 内部参考电压通道 */
//    if (ret != RT_EOK) rt_kprintf("[MONITOR] Enable VREFINT ADC failed\n");

    /* 读取一次 VREFINT 校准值（出厂固化值） */
    s_vrefint_cal = *VREFINT_CAL_ADDR;
    rt_kprintf("[MONITOR] VREFINT_CAL = %d (3.3V reference)\n", s_vrefint_cal);

    /* 创建监控线程 */
    s_monitor_thread = rt_thread_create("monitor",
                                         monitor_thread_entry,
                                         RT_NULL,
                                         MONITOR_THREAD_STACK_SIZE,
                                         MONITOR_THREAD_PRIORITY,
                                         5);
    if (s_monitor_thread == RT_NULL) {
        rt_kprintf("[MONITOR] ERROR: Failed to create thread\n");
        return;
    }
    rt_thread_startup(s_monitor_thread);

    rt_kprintf("[MONITOR] Initialized (sample interval = %d ms, filter window = %d)\n",
               MONITOR_SAMPLE_INTERVAL_MS, FILTER_WINDOW_SIZE);
		
		rt_pin_mode(CHARGER_CONTROL_PIN, PIN_MODE_OUTPUT);
		rt_pin_write(CHARGER_CONTROL_PIN, PIN_LOW);
}

#ifdef RT_USING_MSH
#include <stdlib.h>

/* 通道名称映射（根据 STM32F407 引脚定义）*/
static const char* channel_name[] = {
    [4]  = "CH4 (PA4)",
    [5]  = "CH5 (PA5)",
    [6]  = "CH6 (PA6)",
    [7]  = "CH7 (PA7)",
    [10] = "CH10 (PC0)",
    [11] = "CH11 (PC1)",
    [12] = "CH12 (PC2)",
    [13] = "CH13 (PC3)",
    [14] = "CH14 (PC4)",
    [15] = "CH15 (PC5)",
    [17] = "CH17 (VREFINT)",
};

/* MSH 命令：扫描所有 ADC1 通道 */
static void adc_scan(int argc, char** argv)
{
    rt_adc_device_t adc;
    rt_uint32_t raw;
    uint16_t vrefint_cal = *(uint16_t*)0x1FFF7A2A;
    int vdda_mv = 3300;   /* 默认值，后面会修正 */

    adc = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc == RT_NULL) {
        rt_kprintf("ADC device not found!\n");
        return;
    }

    rt_kprintf("ADC1 Channel Scan\n");
    rt_kprintf("VREFINT_CAL = %d\n", vrefint_cal);

    /* 先读取 VREFINT 校准 VDDA */
    raw = rt_adc_read(adc, ADC1_CH17);
    if (raw > 0) {
        /* vdda = 3300 * vrefint_cal / raw  单位 mV */
        vdda_mv = (3300 * vrefint_cal) / raw;
        rt_kprintf("VREFINT raw=%lu => VDDA=%d mV\n", raw, vdda_mv);
    } else {
        rt_kprintf("VREFINT raw=0, use default VDDA=3300mV\n");
    }

    for (int ch = 4; ch <= 17; ch++) {
        if (ch == 8 || ch == 9 || ch == 16) continue;
        if (!channel_name[ch]) continue;

        rt_adc_enable(adc, ch);
        raw = rt_adc_read(adc, ch);
        rt_adc_disable(adc, ch);

        if (raw == 0 && (ch != 17)) {
            rt_kprintf("%-12s raw=0 (maybe not connected)\n", channel_name[ch]);
        } else {
            /* 电压(mV) = raw * vdda_mv / 4095 */
            int mv = (int)((uint64_t)raw * vdda_mv / 4095);
            rt_kprintf("%-12s raw=%-5lu voltage=%d mV\n", channel_name[ch], raw, mv);
        }
    }
}
// 根据通道号获取索引
static int channel_to_index(int ch)
{
    switch (ch) {
        case 13: return MONITOR_CH_BATTERY;
        case 5:  return MONITOR_CH_CHARGER_DET;
        case 6:  return MONITOR_CH_HEATER_DET;
        case 4:  return MONITOR_CH_CHARGER_SAMPLE;
        default: return -1;
    }
}

static void adc_calibrate(int argc, char** argv)
{
    if (argc < 4) {
        rt_kprintf("Usage: adc_calibrate <channel> <actual_mV> <adc_mV>\n");
        rt_kprintf("  channel: 13 (battery), 5 (charger detect), 6 (heater detect), 4 (charger sample)\n");
        return;
    }

    int ch = atoi(argv[1]);
    int actual_mv = atoi(argv[2]);
    int adc_mv = atoi(argv[3]);

    if (adc_mv == 0) {
        rt_kprintf("ADC mV cannot be zero!\n");
        return;
    }

    int idx = channel_to_index(ch);
    if (idx < 0) {
        rt_kprintf("Invalid channel: %d\n", ch);
        return;
    }

    float new_scale = (float)actual_mv / adc_mv;
    s_scale[idx] = new_scale;

    rt_kprintf("Channel %d calibrated: (actual=%d mV, adc=%d mV) ",
               ch, actual_mv, adc_mv);
		rt_kprintf("scale = ");
		print_float(new_scale, 6, 3, 1);
		rt_kprintf("\n");
}

static void adc_scale(int argc, char** argv)
{
    rt_kprintf("Current ADC scales:\n");
		rt_kprintf("Battery (CH13) : ");
		print_float(s_scale[MONITOR_CH_BATTERY], 6, 3, 1);
		rt_kprintf("\n");
		rt_kprintf("Charger detect (CH5) : ");
		print_float(s_scale[MONITOR_CH_CHARGER_DET], 6, 3, 1);
		rt_kprintf("\n");
		rt_kprintf("Heater detect (CH6) : ");
		print_float(s_scale[MONITOR_CH_HEATER_DET], 6, 3, 1);
		rt_kprintf("\n");
		rt_kprintf("Charger sample (CH4) : ");
		print_float(s_scale[MONITOR_CH_CHARGER_SAMPLE], 6, 3, 1);
		rt_kprintf("\n");	
}
static void monitor_print(int argc, char** argv)
{
    if (argc == 1) {
        rt_kprintf("Monitor print is %s\n", s_monitor_print_enabled ? "on" : "off");
        return;
    }
    if (rt_strcmp(argv[1], "on") == 0) {
        s_monitor_print_enabled = RT_TRUE;
        rt_kprintf("Monitor print enabled\n");
    } else if (rt_strcmp(argv[1], "off") == 0) {
        s_monitor_print_enabled = RT_FALSE;
        rt_kprintf("Monitor print disabled\n");
    } else {
        rt_kprintf("Usage: monitor_print [on|off]\n");
    }
}

static void charger_ctrl(int argc, char** argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: charger on/off\n");
        return;
    }

    if (strcmp(argv[1], "on") == 0) {
        rt_pin_write(CHARGER_CONTROL_PIN, PIN_HIGH);
        rt_kprintf("Charger enabled (MOSFET ON)\n");
    } else if (strcmp(argv[1], "off") == 0) {
        rt_pin_write(CHARGER_CONTROL_PIN, PIN_LOW);
        rt_kprintf("Charger disabled (MOSFET OFF)\n");
    } else {
        rt_kprintf("Invalid argument. Use 'on' or 'off'\n");
    }
}
MSH_CMD_EXPORT(charger_ctrl, "Control charger MOSFET (on/off)");
MSH_CMD_EXPORT(monitor_print, "on/off monitor thread printing");
MSH_CMD_EXPORT(adc_calibrate, Calibrate ADC channel: adc_calibrate <channel> <actual_mV> <adc_mV>);
MSH_CMD_EXPORT(adc_scale, Show current ADC scale factors);
MSH_CMD_EXPORT(adc_scan, Scan all ADC1 channels and print raw/voltage);

//需要出厂前校准scale比例系数
//adc_calibrate 13 22630 2228
//adc_calibrate 4 22620 2242
#endif /* RT_USING_MSH */

