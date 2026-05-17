#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>   // 用于 logf 计算
#include "monitor.h"
#include "global_conf.h"
#include "print_utils.h"
#include "led.h"
#include "buzzer.h"
#include "board.h"
#include <string.h>
#include <fal.h>
#include "ultrasonic_485.h"
#include "zltech_can_motor.h"
#include "uart_packet.h"
#include "packet_reports.h"
#include "wc_drv.h"
#include "user_action.h"
#include "irm_8601m2.h"

// 温度-ADC 查找表 (温度: -5°C ~ 50°C, 步长 1°C)
// ADC 值对应温度升高而递减
static const int16_t temp_table[] = {
	-50, -40, -30, -20, -10, 0, 10, 20, 30, 40,
	50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 
	150, 160, 170, 180, 190, 200, 210, 220, 230,240, 
	250, 260, 270, 280, 290, 300, 310, 320,330, 340, 
	350, 360, 370, 380, 390, 400, 410,420, 430, 440, 
	450, 460, 470, 480, 490, 500
};
static const uint16_t adc_table[] = {
	4095,4061,3984,3907,3829,3750,3672,3593,3514,3435,
	3356,3278,3200,3122,3045,2968,2892,2817,2743,2669,
	2597,2526,2455,2386,2319,2252,2187,2123,2060,1999,
	1939,1880,1823,1767,1713,1660,1609,1558,1510,1462,
	1416,1371,1328,1285,1245,1205,1166,1129,1093,1058,
	1024,991,959,929,899,870
};
#define ADC_TABLE_SIZE  (sizeof(adc_table) / sizeof(adc_table[0]))
	// NTC 校准参数：每个校准点对应一个理论温度（℃）和测量的实际温度（℃）
#define NTC_CALIB_POINTS 4
static const float calib_theory[NTC_CALIB_POINTS] = {30.0f, 35.0f, 40.0f, 45.0f}; // 校准点的理论温度
static float calib_actual[NTC_CALIB_POINTS] = {30.0f, 35.0f, 40.0f, 45.0f};       // 实际测量温度（初始等于理论）
static rt_bool_t calib_initialized = RT_FALSE;
	
static const char *DATA_PARTITION_NAME = "data";  // 与分区表名称一致
static uint32_t s_loaded_ultrasonic_baudrate = 0; 

// 声明外部变量，用于 OLED 显示
extern int g_oled_battery_mv;
extern int g_charge_power_mw;

/* ========== Thread Configuration ========== */
#define MONITOR_THREAD_STACK_SIZE   2048
#define MONITOR_THREAD_PRIORITY     18

#define MONITOR_SAMPLE_INTERVAL_MS  50          /* 采样间隔 50ms */
#define MONITOR_FILTER_WINDOW_SIZE  10          /* 滤波窗口大小 */
#define PRINT_INTERVAL              10          /* 每10次滤波打印一次（约5秒一次） */

/* ========== Internal Reference Voltage Address (STM32F407) ========== */
//#define VREFINT_CAL_ADDR            (uint16_t*)0x1FFF7A2A   /* 3.3V 时校准值 */

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
    rt_uint32_t battery_raw[MONITOR_FILTER_WINDOW_SIZE];
    rt_uint32_t charger_raw[MONITOR_FILTER_WINDOW_SIZE];
    rt_uint32_t heater_raw[MONITOR_FILTER_WINDOW_SIZE];
		rt_uint32_t charger_sample_raw[MONITOR_FILTER_WINDOW_SIZE];
    rt_uint32_t vrefint_raw[MONITOR_FILTER_WINDOW_SIZE];
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
static uint16_t battery_min_limit = BATTERY_LOW_VOLTAGE_MV; /* 低压报警值 */

// 添加静态变量存储功率（毫瓦）
static rt_uint32_t s_charge_power_mw = 0;
static int32_t s_charge_diff_offset = 220;          // 充电压差静态偏移（mV），未充电时为0，开启充电有压差，可能是锂电池内部充电MOS管管压
static rt_bool_t s_offset_calibrated = RT_FALSE;  // 偏移是否已校准

static const struct fal_partition * data_part = RT_NULL;

static rt_bool_t s_water_high_level = RT_FALSE;   // 高水位状态
static rt_bool_t s_water_low_level  = RT_FALSE;   // 低水位状态

static uint8_t last_low_battery = 0;

// 以下用于事件触发上报（变化时才发送）
static uint8_t last_charger_event = 0xFF;  // 0:断开,1:连接未充,2:充电中
static uint8_t last_heater_present = 0xFF; // 0:断开,1:连接
static uint8_t last_cliff_trigger = 0xFF;  // 0:无悬崖,1:有悬崖
static uint8_t last_ir_mask = 0xFF;        // 红外对射管状态掩码
static uint8_t last_water_high = 0xFF;     // 高水位状态
static uint8_t last_water_low = 0xFF;      // 低水位状态
static uint16_t last_battery_mv_report = 0;// 用于定时上报电池电压
static uint8_t charger_event = 0;
static uint8_t charger_connected = 0;

// 水温上报定时计数器（每5秒一次）
static uint32_t water_temp_report_cnt = 0;

static uint32_t ir_timeout_cnt = 0;  // 红外接收管超时未上报
static uint8_t last_left_match = 0, last_right_match = 0;

#define HEATER_POWER_STABLE_DELAY_MS  3000       // 等待3秒稳定
/**
 * @brief 保存校准数据到 Flash 分区
 */
void monitor_save_calibration(void)
{
    data_part = fal_partition_find(DATA_PARTITION_NAME);
    if (data_part == RT_NULL) {
        rt_kprintf("[MONITOR] Partition '%s' not found!\n", DATA_PARTITION_NAME);
        return;
    }

    MonitorCalibData_t calib_data;
    calib_data.magic = MONITOR_CALIB_MAGIC;
    calib_data.version = MONITOR_CALIB_VERSION;
    memcpy(calib_data.scale, s_scale, sizeof(s_scale));
    calib_data.charge_diff_offset = s_charge_diff_offset;
		calib_data.ultrasonic_baudrate = ultrasonic_485_get_baudrate();

    // 写入 Flash（注意：需要先擦除再写入，FAL 的 write 接口不会自动擦除）
    if (fal_partition_erase(data_part,0, sizeof(calib_data)) < 0) {
        rt_kprintf("[MONITOR] Erase partition failed!\n");
        return;
    }		
    if (fal_partition_write(data_part, 0, (uint8_t *)&calib_data, sizeof(calib_data)) < 0) {
        rt_kprintf("[MONITOR] Write calibration data failed!\n");
        return;
    }
    rt_kprintf("[MONITOR] Calibration data saved to Flash.\n");
}

/**
 * @brief 从 Flash 分区加载校准数据
 * @return RT_TRUE 加载成功，RT_FALSE 加载失败（使用默认值）
 */
static rt_bool_t monitor_load_calibration(void)
{
    data_part = fal_partition_find(DATA_PARTITION_NAME);
    if (data_part == RT_NULL) {
        rt_kprintf("[MONITOR] Partition '%s' not found, using default scales.\n", DATA_PARTITION_NAME);
        return RT_FALSE;
    }

    MonitorCalibData_t calib_data;
    if (fal_partition_read(data_part, 0, (uint8_t *)&calib_data, sizeof(calib_data)) < 0) {
        rt_kprintf("[MONITOR] Read calibration data failed, using defaults.\n");
        return RT_FALSE;
    }

    if (calib_data.magic != MONITOR_CALIB_MAGIC || calib_data.version != MONITOR_CALIB_VERSION) {
        rt_kprintf("[MONITOR] Calibration data invalid (magic=0x%08X, version=%d), using defaults.\n",
                   calib_data.magic, calib_data.version);
        return RT_FALSE;
    }

    // 加载数据到全局变量
    memcpy(s_scale, calib_data.scale, sizeof(s_scale));
    s_charge_diff_offset = calib_data.charge_diff_offset;
		
		if (calib_data.magic == MONITOR_CALIB_MAGIC && calib_data.version >= 2) {
        s_loaded_ultrasonic_baudrate = calib_data.ultrasonic_baudrate;
    } else {
        // 旧版本，使用默认值
        s_loaded_ultrasonic_baudrate = 115200;
    }

    rt_kprintf("[MONITOR] Calibration data loaded from Flash.\n");
    return RT_TRUE;
}

uint32_t monitor_get_ultrasonic_baudrate(void)
{
    return s_loaded_ultrasonic_baudrate;
}

/* ========== Utility Functions ========== */

/**
 * @brief 去极值平均滤波（去掉最大最小值）
 * @param arr 原始数据数组（长度为 MONITOR_FILTER_WINDOW_SIZE）
 * @return 滤波后的平均值（uint32_t）
 */
static rt_uint32_t filter_average(rt_uint32_t arr[MONITOR_FILTER_WINDOW_SIZE])
{
    rt_uint32_t sum = 0;
    rt_uint32_t min = arr[0], max = arr[0];

    for (int i = 0; i < MONITOR_FILTER_WINDOW_SIZE; i++) {
        sum += arr[i];
        if (arr[i] < min) min = arr[i];
        if (arr[i] > max) max = arr[i];
    }
    sum -= (min + max);                    /* 去掉最大值和最小值 */
    return sum / (MONITOR_FILTER_WINDOW_SIZE - 2); /* 剩余 8 个求平均 */
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
    rt_adc_enable(s_adc_dev, ADC1_CH4);
    *charger_sample_raw = rt_adc_read(s_adc_dev, ADC1_CH4);
    rt_adc_disable(s_adc_dev, ADC1_CH4);

    // 加热器检测通道 (PA6, ADC1_CH6)
    rt_adc_enable(s_adc_dev, ADC1_CH6);
    *heater_raw = rt_adc_read(s_adc_dev, ADC1_CH6);
    rt_adc_disable(s_adc_dev, ADC1_CH6);

    // 内部参考电压 (VREFINT, ADC1_CH17)
    rt_adc_enable(s_adc_dev, ADC1_CH17);
    *vrefint_raw = rt_adc_read(s_adc_dev, ADC1_CH17);
    rt_adc_disable(s_adc_dev, ADC1_CH17);
}

void read_cliff_sensor(uint16_t *front_mv, uint16_t *rear_mv, rt_bool_t *trigger)
{
    // 读取前悬崖电压
    rt_adc_enable(s_adc_dev, CLIFF_FRONT_ADC);
    rt_uint32_t raw_front = rt_adc_read(s_adc_dev, CLIFF_FRONT_ADC);
    rt_adc_disable(s_adc_dev, CLIFF_FRONT_ADC);
    // 粗略转换为 mV（假设 VDDA=3300mV，实际可用校准值）
    *front_mv = (uint16_t)adc_raw_to_mv(raw_front);

    // 读取后悬崖电压
    rt_adc_enable(s_adc_dev, CLIFF_REAR_ADC);
    rt_uint32_t raw_rear = rt_adc_read(s_adc_dev, CLIFF_REAR_ADC);
    rt_adc_disable(s_adc_dev, CLIFF_REAR_ADC);
    *rear_mv = (uint16_t)adc_raw_to_mv(raw_rear);

    // 阈值判断（与 car_action 中一致）
    *trigger = (*front_mv < CLIFF_VOLTAGE_THRESHOLD_MV || *rear_mv < CLIFF_VOLTAGE_THRESHOLD_MV);
}

static uint8_t read_ir_sensors(void)
{
    uint8_t mask = 0;
    if (rt_pin_read(IR_SENSOR1_PIN) == PIN_LOW) mask |= 0x01;
    if (rt_pin_read(IR_SENSOR2_PIN) == PIN_LOW) mask |= 0x02;
    if (rt_pin_read(IR_SENSOR3_PIN) == PIN_LOW) mask |= 0x04;
    return mask;
}

/**
 * @brief 通过二分查找和线性插值获取温度
 * @param adc_value ADC 原始读数
 * @return 温度值，单位 0.1°C
 */
static int16_t lookup_temperature(uint16_t adc_value)
{
    // 边界检查
    if (adc_value >= adc_table[0]) {
        // 温度最低，返回 -5°C
        return temp_table[0];
    }
    if (adc_value <= adc_table[ADC_TABLE_SIZE - 1]) {
        // 温度最高，返回 50°C
        return temp_table[ADC_TABLE_SIZE - 1];
    }

    // 二分查找找到所在区间
    uint8_t left = 0, right = ADC_TABLE_SIZE - 1;
    while (right - left > 1) {
        uint8_t mid = (left + right) / 2;
      // 关键：因为数组递减，若目标 adc 大于等于中间值，说明温度低于 mid 对应的温度，应在左半区间
        if (adc_value >= adc_table[mid])
            right = mid;      // 向左半区间移动
        else
            left = mid;       // 向右半区间移动
    }

    // 线性插值
    int16_t temp_left = temp_table[left];
    int16_t temp_right = temp_table[right];
    uint16_t adc_left = adc_table[left];
    uint16_t adc_right = adc_table[right];
    // ADC 值下降对应温度上升，插值公式：
    // temperature = temp_left + (temp_right - temp_left) * (adc_value - adc_left) / (adc_right - adc_left)
    int32_t diff = (int32_t)(temp_right - temp_left) * (adc_value - adc_left) / (adc_right - adc_left);
    return temp_left + diff;
}

int16_t wc_get_water_temperature_by_table(void)
{
    static rt_adc_device_t adc_dev = RT_NULL;
    rt_uint32_t raw;

    if (adc_dev == RT_NULL) {
        adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
        if (adc_dev == RT_NULL) {
            rt_kprintf("[WATER_TEMP] ADC device not found\n");
            return -32768;
        }
    }

    if (rt_adc_enable(adc_dev, WATER_TEMP_ADC) != RT_EOK) {
        rt_kprintf("[WATER_TEMP] Enable ADC channel failed\n");
        return -32768;
    }
    raw = rt_adc_read(adc_dev, WATER_TEMP_ADC);
    rt_adc_disable(adc_dev, WATER_TEMP_ADC);
		int mv = (int)((uint64_t)raw * 3300 / 4095);
//		rt_kprintf("water raw=%d voltage=%d mV\n",  raw,mv);
    // 查表得到温度（单位 0.1°C）
    return lookup_temperature((uint16_t)raw);
}

/**
 * @brief 读取 NTC 水温传感器的温度   开尔文计算法 运算量大
 * @return 温度值，单位 0.1°C；若读取失败返回 -32768
 */
int16_t wc_get_water_temperature_by_KELV(void)
{
    static rt_adc_device_t adc_dev = RT_NULL;
    rt_uint32_t raw;
    float v_ntc_mv, r_ntc;
    float temp_k, temp_c;

    // 获取 ADC 设备（只需查找一次）
    if (adc_dev == RT_NULL) {
        adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
        if (adc_dev == RT_NULL) {
            rt_kprintf("[WATER_TEMP] ADC device not found\n");
            return -32768;
        }
    }

    // 使能通道并读取原始值
    if (rt_adc_enable(adc_dev, WATER_TEMP_ADC) != RT_EOK) {
        rt_kprintf("[WATER_TEMP] Enable ADC channel failed\n");
        return -32768;
    }
    raw = rt_adc_read(adc_dev, WATER_TEMP_ADC);
    rt_adc_disable(adc_dev, WATER_TEMP_ADC);

    
    v_ntc_mv = adc_raw_to_mv(raw);

    // 计算 NTC 阻值 (欧姆)
    // 电路: VCC -- R_series -- NTC -- GND, ADC 测 NTC 两端电压
    // 公式: V_ntc = VCC * (R_ntc / (R_series + R_ntc))
    // 推导: R_ntc = (V_ntc * R_series) / (VCC - V_ntc)
    if (v_ntc_mv >= NTC_VCC_MV) {
        // 电压异常，可能断路或 ADC 饱和
        return -32768;
    }
    r_ntc = (v_ntc_mv * NTC_SERIES_RESISTOR_OHM) / (NTC_VCC_MV - v_ntc_mv);

    // 使用 B 值公式计算温度
    // 1/T = 1/T0 + (1/B) * ln(R/R0)
    // T 单位为开尔文
    float ln_r_ratio = logf(r_ntc / NTC_R25_OHM);
    temp_k = 1.0f / (1.0f / NTC_T0_KELVIN + ln_r_ratio / NTC_B_CONSTANT);
    temp_c = temp_k - 273.15f;

    // 限幅到合理范围（-20°C ~ 100°C），并转换为 0.1°C 整数返回
    if (temp_c < -20.0f) temp_c = -20.0f;
    if (temp_c > 100.0f) temp_c = 100.0f;
    return (int16_t)(temp_c * 10.0f);
}

int16_t wc_get_water_temperature_with_calib(void)
{
	 int16_t theory_temp = wc_get_water_temperature_by_table();
		if(theory_temp == temp_table[ADC_TABLE_SIZE - 1]){
			//如果超过50℃，不用查表法，直接用公式计算，重新赋值
			theory_temp = wc_get_water_temperature_by_KELV();
		}
 // 转换为浮点摄氏度
    float theory_c = theory_temp / 10.0f;
    float actual_c = theory_c;

    // 分段线性校准（基于理论温度所在的区间）
    if (theory_c >= calib_theory[0] && theory_c <= calib_theory[NTC_CALIB_POINTS-1]) {
        // 确认校准点已初始化（如果未初始化，则使用理论值）
        if (!calib_initialized) {
            // 首次使用，将 calib_actual 设为理论值（即无校准）
            for (int i = 0; i < NTC_CALIB_POINTS; i++) {
                calib_actual[i] = calib_theory[i];
            }
            calib_initialized = RT_TRUE;
        }
        // 找到理论温度所在的区间
        int idx = 0;
        while (idx < NTC_CALIB_POINTS-1 && theory_c > calib_theory[idx+1]) idx++;
        // 线性插值计算偏移
        float t0 = calib_theory[idx];
        float t1 = calib_theory[idx+1];
        float a0 = calib_actual[idx];
        float a1 = calib_actual[idx+1];
        // 实际温度 = 理论温度 + (区间内实际与理论的差值)
        // 更准确：直接线性插值实际温度
        if (t1 - t0 > 1e-6) {
            actual_c = a0 + (a1 - a0) * (theory_c - t0) / (t1 - t0);
        } else {
            actual_c = theory_c; // 防除零
        }
    }
    // 转换回 0.1°C 整数
    int16_t actual_temp = (int16_t)(actual_c * 10.0f);
    if (actual_temp < -200) actual_temp = -200;
    if (actual_temp > 1000) actual_temp = 1000;
    return actual_temp;	
}

int16_t wc_get_water_temperature(void)
{
//	 int16_t theory_temp = wc_get_water_temperature_by_table();
//		if(theory_temp == temp_table[ADC_TABLE_SIZE - 1]){
//			//如果超过50℃，不用查表法，直接用公式计算，重新赋值
//			theory_temp = wc_get_water_temperature_by_KELV();
//		}
//		return theory_temp;
	return wc_get_water_temperature_by_KELV();
}

/* ========== Monitor Thread ========== */
static void monitor_thread_entry(void *parameter)
{
	  uint32_t hb_check_counter = 0;   // 心跳检查计数器（每50ms加1，达到10次即500ms检查一次）
		uint32_t led_counter = 0;   // led灯刷新 20*60*60=72000 1小时刷新一次
    rt_uint32_t battery_raw, charger_raw, heater_raw, charger_sample_raw, vrefint_raw;
    rt_uint32_t filtered_battery, filtered_charger, filtered_heater, filtered_charger_sample, filtered_vrefint;
		int32_t effective_diff = 0;
    rt_bool_t high_level = RT_FALSE;
    rt_bool_t low_level  = RT_FALSE;	
    s_filter_buf.index = 0;
    s_filter_buf.ready = 0;

		static uint32_t pos_report_cnt = 0;
		static uint32_t temp_report_cnt = 0;
		static PacketReportMotorVelocityTypeDef vel_pkt = {.sub_cmd = MOTOR_SUB_VELOCITY_STATUS};
		static PacketReportMotorPositionTypeDef pos_pkt = {.sub_cmd = MOTOR_SUB_POSITION};
		static PacketReportMotorTemperatureTypeDef temp_pkt = {.sub_cmd = MOTOR_SUB_TEMPERATURE};	
		
		static uint32_t battery_report_cnt = 0;
		static PacketReportBatteryVoltageTypeDef bat_pkt = {.sub_cmd = SYS_SUB_BATTERY_EVENT};
		static PacketReportChargerEventTypeDef charge_pkt = {.sub_cmd = SYS_SUB_CHARGER_EVENT};
		static PacketReportHeaterEventTypeDef heat_pkt = {.sub_cmd = SYS_SUB_HEATER_EVENT};			
		static PacketReportCliffTypeDef cliff_pkt = {.sub_cmd = SYS_SUB_CLIFF};			
		static PacketReportIRSwitchTypeDef ir_pkt = {.sub_cmd = SYS_SUB_IR_SWITCH};	
		
		uint8_t heater_present;		
		uint16_t front_mv, rear_mv;
		rt_bool_t cliff_trigger;
		uint8_t trigger_val;
		uint8_t ir_mask;
		
		static uint32_t heater_power_on_tick = 0;        // 电源接入时刻（毫秒）
		static uint8_t heater_power_stable_wait = 0;     // 是否在等待稳定
		
		static uint16_t last_left_cnt = 0, last_right_cnt = 0;
 
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
        if (s_filter_buf.index >= MONITOR_FILTER_WINDOW_SIZE) {
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
                        g_charge_power_mw = s_charge_power_mw;  // 更新全局充电功率，OLED 显示使用
						
						// 满电指示 (≥ 25.2V)
						if (s_battery_mv >= BATTERY_FULL_VOLTAGE_MV) {
								led_set_battery_full(RT_TRUE);
						} else {
								led_set_battery_full(RT_FALSE);
						}
						
					  // 低电量报警
						if (s_battery_mv <= BATTERY_LOW_ALARM_MV || s_battery_mv < battery_min_limit) {
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
						// 变更采样电阻 0.01 调整为0.05 
							//新公式（0.05Ω）
							//电流 I = diff_mV / 50 （A）
							//功率 P_mW = (diff_mV / 50) × (V_charge / 1000) × 1000 = diff_mV × V_charge / 50		
						if (rt_pin_read(CHARGER_CONTROL_PIN) == PIN_HIGH && s_charger_mv > 5000) {  								
									int32_t diff = (int32_t)s_charger_sample_mv - (int32_t)s_battery_mv - s_charge_diff_offset;
									effective_diff = diff;
//									if (diff > 5) {
//											s_charge_power_mw = (rt_uint32_t)(((int64_t)diff * s_charger_mv) / 10);
							    // 阈值调整为 25mV（对应 0.5A 电流），可根据实际噪声调整
										if (diff > 15) {
												// 功率（毫瓦）= diff_mV * s_charger_mv / 50
												s_charge_power_mw = (rt_uint32_t)(((int64_t)diff * s_charger_mv) / 50);
									} else {
											s_charge_power_mw = 0;
									}							
						} else {
								int32_t diff = (int32_t)s_charger_sample_mv - (int32_t)s_battery_mv;
								effective_diff = 0;
								s_charge_power_mw = 0;
						}
						
						// 读取水位开关（假设低电平表示有水）
						high_level = wc_water_tank_high_level();
						low_level  = wc_water_tank_low_level();		
						
						
						// 检测状态变化，仅变化时更新 LED
						if (high_level != s_water_high_level) {
								led_set_water_full(high_level);
						}
						if (low_level != s_water_low_level) {
								led_set_water_low(low_level);
						}						
						// 更新全局变量
						s_water_high_level = high_level;
						s_water_low_level  = low_level;
						
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
								rt_kprintf("[MONITOR] charge power=%d W (eff diff=%d mV),current = %d mA\n", s_charge_power_mw/1000, effective_diff,effective_diff*20 );
            }
        }
				
				// zlac 心跳包检测 
				if (hb_check_counter >= 10) {  
						hb_check_counter = 0;
						zlac_check_heartbeat(); 
						zlac_refresh_status_cache();// 同时刷新电机状态缓存									
				}
				hb_check_counter++;
				
				if(led_counter >= 12000){   // 10分钟
					led_counter = 0;
					led_flush();	// led指示灯容易受到干扰，定期重新刷新	
				}
				led_counter++;
				// 电机位置上报（每 20 个周期上报一次，MONITOR_SAMPLE_INTERVAL_MS=50ms → 1s）
				pos_report_cnt++;
				if (pos_report_cnt >= 20) {
						pos_report_cnt = 0;
						zlac_get_position(&pos_pkt.left_pos_pulse, &pos_pkt.right_pos_pulse);
						uart_packet_send(PKT_FUNC_MOTOR, &pos_pkt, sizeof(pos_pkt));
				}

				// 电机温度上报（每 100 个周期上报一次 → 5s）
				temp_report_cnt++;
				if (temp_report_cnt >= 100) {
						temp_report_cnt = 0;
						temp_pkt.left_temp_c = zlac_get_motor_temp_left();
						temp_pkt.right_temp_c = zlac_get_motor_temp_right();
						temp_pkt.driver_temp_c = zlac_get_driver_temp();
						uart_packet_send(PKT_FUNC_MOTOR, &temp_pkt, sizeof(temp_pkt));
				}
				
				if (zlac_is_velocity_mode_ready() && zlac_is_left_enabled() && zlac_is_right_enabled())
				{
						// 1. 速度+状态（每 50ms 上报一次）
						zlac_get_velocity(&vel_pkt.left_vel_rpm, &vel_pkt.right_vel_rpm);
						vel_pkt.left_enabled = zlac_is_left_enabled();
						vel_pkt.right_enabled = zlac_is_right_enabled();
						vel_pkt.left_running = (zlac_get_motor_status_left() != 0);
						vel_pkt.right_running = (zlac_get_motor_status_right() != 0);
						uart_packet_send(PKT_FUNC_MOTOR, &vel_pkt, sizeof(vel_pkt));
				}	

				// 水箱水位状态（变化触发）	
				if (high_level != last_water_high || low_level != last_water_low) {
						last_water_high = high_level;
						last_water_low = low_level;
						PacketReportWaterLevelTypeDef water_pkt;
						water_pkt.sub_cmd = TOILET_SUB_WATER_LEVEL;
						water_pkt.high_level = high_level;
						water_pkt.low_level = low_level;
						uart_packet_send(PKT_FUNC_TOILET, &water_pkt, sizeof(water_pkt));
				}				

				// 红外对射管 变化触发  红外对射管在回充电桩过程中高频触发，平时不用读取
//				ir_mask = read_ir_sensors();
//				if (ir_mask != last_ir_mask) {
//						last_ir_mask = ir_mask;
//						ir_pkt.state_mask = ir_mask;
//						uart_packet_send(PKT_FUNC_SYS, &ir_pkt, sizeof(ir_pkt));
//				}	

//				// 电池电压上报（每10秒一次，避免频繁）
				battery_report_cnt++;
				if (battery_report_cnt >= 200) { // 50ms * 200 = 10秒
						battery_report_cnt = 0;
						bat_pkt.voltage = s_battery_mv;
						bat_pkt.event = (s_battery_mv <= BATTERY_LOW_ALARM_MV || s_battery_mv < battery_min_limit) ? 1 : 0;
						uart_packet_send(PKT_FUNC_SYS, &bat_pkt, sizeof(bat_pkt));
				}
//				// 充电上报 变化触发
				charger_connected = (s_charger_mv > 5000);
				if (charger_connected && rt_pin_read(CHARGER_CONTROL_PIN) == PIN_HIGH && s_charge_power_mw > 0) {
						charger_event = 2; // 充电中
				} else if (charger_connected) {
						charger_event = 1; // 连接未充电
				} else {
						charger_event = 0; // 断开
				}
				if (charger_event != last_charger_event) {
						last_charger_event = charger_event;
						charge_pkt.event = charger_event;
						charge_pkt.voltage_mv = s_charger_mv;
						charge_pkt.power_mw = s_charge_power_mw;
						uart_packet_send(PKT_FUNC_SYS, &charge_pkt, sizeof(charge_pkt));
				}
//				//悬崖传感器上报 周期检测并变化触发
				read_cliff_sensor(&front_mv, &rear_mv, &cliff_trigger);
//				rt_kprintf("cliff front %dmv,rear %dmv,trig %d\n",front_mv,rear_mv,cliff_trigger);
				trigger_val = cliff_trigger ? 1 : 0;
				if (trigger_val != last_cliff_trigger) {
						last_cliff_trigger = trigger_val;
						cliff_pkt.cliff_trigger = trigger_val;
						cliff_pkt.front_mv = front_mv;
						cliff_pkt.rear_mv = rear_mv;
						uart_packet_send(PKT_FUNC_SYS, &cliff_pkt, sizeof(cliff_pkt));
				}
				
				// 加热管电源对连接的判断  变化触发上报
				heater_present = (s_heater_mv > 5000) ? 1 : 0;
				if (heater_present != last_heater_present) {
						last_heater_present = heater_present;
						heat_pkt.event = heater_present;
						heat_pkt.voltage_mv = s_heater_mv;
						uart_packet_send(PKT_FUNC_SYS, &heat_pkt, sizeof(heat_pkt));
				}
				
				// 检测加热器电源是否刚接通
				if (s_heater_mv > 5000) {
						if (heater_power_stable_wait == 0) {
								// 电源刚刚接通，记录时间并进入等待稳定状态
								heater_power_on_tick = rt_tick_get_millisecond();
								heater_power_stable_wait = 1;
								rt_kprintf("[MONITOR] Heater power connected, waiting %d ms for stable\n", HEATER_POWER_STABLE_DELAY_MS);
						} else if (heater_power_stable_wait == 1) {
								// 检查是否已经达到稳定延时
								if (rt_tick_get_millisecond() - heater_power_on_tick >= HEATER_POWER_STABLE_DELAY_MS) {
										heater_power_stable_wait = 2;  // 稳定完成，可以正常控制
										rt_kprintf("[MONITOR] Heater power stable, enable control\n");
								}
						}
				} else {
						// 电源断开，重置状态
						heater_power_stable_wait = 0;
						heater_power_on_tick = 0;
				}				
				
				//水温加热以及（周期上报，每5秒）
				water_temp_report_cnt++;
				if (water_temp_report_cnt >= 100) { // 50ms*100 = 5s
						water_temp_report_cnt = 0;
										
						int16_t water_temp = wc_get_water_temperature();
					// 读取到水温，判断加热管供电是否连接，设定阀值39℃，低于阀值-2℃再次加热，避免频繁加热，可以保持在37-39℃。
	//					rt_kprintf("water T=%d \n",water_temp);
					if (water_temp != -32768) {
									// 上报数据
									PacketReportWaterTempTypeDef temp_pkt;
									temp_pkt.sub_cmd = TOILET_SUB_WATER_TEMP;
									temp_pkt.temperature_c = water_temp;
									uart_packet_send(PKT_FUNC_TOILET, &temp_pkt, sizeof(temp_pkt));

									// 获取当前水温目标值
									uint16_t target_temp = user_action_get_water_target_temp();
									uint16_t low_temp = target_temp - 20;   // 低于目标 2°C 开启加热
									uint16_t high_temp = target_temp;       // 达到目标即关闭
						 
									// 恒温控制：仅在加热器电源连接时执行  自动加热开启 + 电源稳定完成 + 水位正常
									if (g_auto_water_heater_enable&& heater_power_stable_wait == 2 && !wc_water_tank_low_level()) {   // 加热器电源已连接 且 不缺水
											if (water_temp < low_temp && !wc_water_heater_is_on()) {
													wc_water_heater_on();
											} else if (water_temp >= target_temp && wc_water_heater_is_on()) {
													wc_water_heater_off();
											}
									} else if ((!g_auto_water_heater_enable || wc_water_tank_low_level() || heater_power_stable_wait != 2) && wc_water_heater_is_on()) {
											// 如果缺水且加热器还在工作，强制关闭加热器（安全保护）
											wc_water_heater_off();
											rt_kprintf("[MONITOR] Water level low, forced heater off!\n");
									}
									
									
							}
				}

				// 红外对射管，上一次有数据，超时500ms,清空数据
				if (g_ir_alignment_enable) {
						/* 获取当前匹配计数 */
						uint16_t left_cnt = irm_get_left_match_cnt();
						uint16_t right_cnt = irm_get_right_match_cnt();						
						
						/* 检查是否有新匹配 */
						if (left_cnt != last_left_cnt || right_cnt != last_right_cnt) {
								/* 有数据更新，重置超时计数器 */
								ir_timeout_cnt = 0;
						} else {
								ir_timeout_cnt++;
								if (ir_timeout_cnt >= 10) {  // 10 * 50ms = 500ms 无新数据
										/* 超时，清除左右接收管的状态 */
										irm_clear_left_status();
										irm_clear_right_status();
										rt_kprintf("[MONITOR] IR alignment timeout, status cleared\n");
										ir_timeout_cnt = 0;  // 重置计数器，避免重复清除
								}
						}
						last_left_cnt = left_cnt;
						last_right_cnt = right_cnt;
				} else {
						/* 不在回充过程，不清零也不做超时检测，但可选择性清零一次（可选）*/
						/* 如果想退出回充后立即清空旧状态，可以加以下代码 */
						// static rt_bool_t was_enabled = RT_FALSE;
						// if (was_enabled) {
						//     irm_clear_left_status();
						//     irm_clear_right_status();
						//     was_enabled = RT_FALSE;
						// }
						// was_enabled = RT_FALSE;
				}
								
        rt_thread_mdelay(MONITOR_SAMPLE_INTERVAL_MS);
    }
}

/* ========== Public API ========== */

rt_bool_t monitor_get_water_high_level(void)
{
    return s_water_high_level;
}

rt_bool_t monitor_get_water_low_level(void)
{
    return s_water_low_level;
}

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

void change_battery_limit(uint16_t limit)
{
    battery_min_limit = limit;
}


void monitor_init(void)
{
    rt_err_t ret;

	rt_pin_mode(IR_SENSOR1_PIN, PIN_MODE_INPUT);
	rt_pin_mode(IR_SENSOR2_PIN, PIN_MODE_INPUT);
	rt_pin_mode(IR_SENSOR3_PIN, PIN_MODE_INPUT);
	
    /* 查找 ADC 设备 */
    s_adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (s_adc_dev == RT_NULL) {
        rt_kprintf("[MONITOR] ERROR: ADC device '%s' not found!\n", ADC_DEV_NAME);
        return;
    }

    /* 使能所有需要的 ADC 通道 */
    ret = rt_adc_enable(s_adc_dev, BATTERY_ADC_CHANNEL);
    if (ret != RT_EOK) rt_kprintf("[MONITOR] Enable battery ADC failed\n");
    ret = rt_adc_enable(s_adc_dev, CHARGER_DETECT_ADC);
    if (ret != RT_EOK) rt_kprintf("[MONITOR] Enable charger ADC failed\n");
		ret = rt_adc_enable(s_adc_dev, CHARGER_SAMPLE_ADC);
		if (ret != RT_EOK) rt_kprintf("[MONITOR] Enable charger sample ADC failed\n");
    ret = rt_adc_enable(s_adc_dev, HEATER_DETECT_ADC);
    if (ret != RT_EOK) rt_kprintf("[MONITOR] Enable heater ADC failed\n");
    ret = rt_adc_enable(s_adc_dev, ADC1_CH17);  /* 内部参考电压通道 */
    if (ret != RT_EOK) rt_kprintf("[MONITOR] Enable VREFINT ADC failed\n");

    /* 读取一次 VREFINT 校准值（出厂固化值） */
    s_vrefint_cal = *VREFINT_CAL_ADDR;
    rt_kprintf("[MONITOR] VREFINT_CAL = %d (3.3V reference)\n", s_vrefint_cal);

		   /* 尝试从 Flash 加载校准数据 */
    if (!monitor_load_calibration()) {
        // 加载失败，使用默认值（已在静态初始化中设置）
        rt_kprintf("[MONITOR] Using default scale factors.\n");
    }
		
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
               MONITOR_SAMPLE_INTERVAL_MS, MONITOR_FILTER_WINDOW_SIZE);
		
		rt_pin_mode(CHARGER_CONTROL_PIN, PIN_MODE_OUTPUT);
		rt_pin_write(CHARGER_CONTROL_PIN, PIN_LOW);
		
		// 初始化水位状态
    s_water_high_level = (rt_pin_read(WATER_LEVEL_HIGH_PIN) == PIN_LOW);
    s_water_low_level  = (rt_pin_read(WATER_LEVEL_LOW_PIN)  == PIN_HIGH);
    led_set_water_full(s_water_high_level);
    led_set_water_low(s_water_low_level);
}

#ifdef RT_USING_MSH
#include <stdlib.h>

static void set_water_temp(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: set_water_temp <temp_0p1c>\n");
        return;
    }
    uint16_t temp = atoi(argv[1]);
    user_action_set_water_target_temp(temp);
    rt_kprintf("Water target temp set to %d.%d°C\n", temp/10, temp%10);
}
MSH_CMD_EXPORT(set_water_temp, "set water target temperature (0.1°C)");

/* 通道名称映射（根据 STM32F407 引脚定义）*/
static const char* channel_name[] = {
    [4]  = "CH4 (PA4) CHARGER_SAMPLE_ADC ",
    [5]  = "CH5 (PA5) CHARGER_DETECT_ADC",
    [6]  = "CH6 (PA6) HEATER_DETECT_ADC",
    [7]  = "CH7 (PA7) CLIFF_FRONT_ADC",
    [10] = "CH10 (PC0)SEWAGE_MOTOR_POS_ADC",
    [11] = "CH11 (PC1)RING_MOTOR_POS_ADC",
    [12] = "CH12 (PC2)LID_MOTOR_POS_ADC",
    [13] = "CH13 (PC3)BATTERY_ADC_CHANNEL",
    [14] = "CH14 (PC4)CLIFF_REAR_ADC",
    [15] = "CH15 (PC5)WATER_TEMP_ADC",
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
    rt_adc_enable(adc, ADC1_CH17);
    raw = rt_adc_read(adc, ADC1_CH17);
    rt_adc_disable(adc, ADC1_CH17);		
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
            rt_kprintf("%-30s raw=%-5lu voltage=%d mV\n", channel_name[ch], raw, mv);
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
static void adc_save(int argc, char** argv)
{
    monitor_save_calibration();
}
MSH_CMD_EXPORT(adc_save, Save current ADC calibration data to Flash);
MSH_CMD_EXPORT(charger_ctrl, "Control charger MOSFET (on/off)");
MSH_CMD_EXPORT(monitor_print, "on/off monitor thread printing");
MSH_CMD_EXPORT(adc_calibrate, Calibrate ADC channel: adc_calibrate <channel> <actual_mV> <adc_mV>);
MSH_CMD_EXPORT(adc_scale, Show current ADC scale factors);
MSH_CMD_EXPORT(adc_scan, Scan all ADC1 channels and print raw/voltage);

static void adc_set_offset(int argc, char** argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: adc_set_offset <offset_mV>\n");
        rt_kprintf("Current offset: %d mV\n", s_charge_diff_offset);
        return;
    }
    int new_offset = atoi(argv[1]);
    s_charge_diff_offset = new_offset;
    rt_kprintf("Charge diff offset set to %d mV (not saved). Use 'adc_save' to store.\n", s_charge_diff_offset);
}
MSH_CMD_EXPORT(adc_set_offset, "Set charge diff offset (mV)");

static void adc_show_offset(int argc, char** argv)
{
    rt_kprintf("Current charge diff offset: %d mV\n", s_charge_diff_offset);
}
MSH_CMD_EXPORT(adc_show_offset, "Show current charge diff offset");
//需要出厂前校准scale比例系数
//adc_calibrate 13 22630 2228
//adc_calibrate 4 22620 2242
// adc_set_offset 200
static void ntc_calib(int argc, char **argv)
{
    if (argc != 3) {
        rt_kprintf("Usage: ntc_calib <theory_temp> <actual_temp>\n");
        rt_kprintf("Example: ntc_calib 35 34.5\n");
        rt_kprintf("Supported theory temps: 30, 35, 40, 45\n");
        return;
    }
    float theory = atof(argv[1]);
    float actual = atof(argv[2]);
    int idx = -1;
    for (int i = 0; i < NTC_CALIB_POINTS; i++) {
        if (fabs(theory - calib_theory[i]) < 0.1f) {
            idx = i;
            break;
        }
    }
    if (idx == -1) {
        rt_kprintf("Theory temperature not supported. Use 30, 35, 40 or 45.\n");
        return;
    }
    calib_actual[idx] = actual;
    rt_kprintf("Calibration set: theory %.1f°C -> actual %.1f°C\n", theory, actual);
    // 可选：保存到 Flash
    // monitor_save_calibration();  // 如果希望掉电保存，需要扩展 MonitorCalibData_t
}
MSH_CMD_EXPORT(ntc_calib, "Calibrate NTC temperature at specific points");
#endif /* RT_USING_MSH */

