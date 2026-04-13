/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 智能马桶底层驱动实现
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "wc_drv.h"
#include "global_conf.h"
#include <board.h>  
#include "user_action.h"

/* ========== 静态变量 ========== */
static struct rt_device_pwm *s_small_pump_pwm = RT_NULL;
static struct rt_adc_device *s_adc_dev = RT_NULL;
static rt_uint16_t s_clean_rod_pos = 0;
static rt_uint16_t s_pipe_pos = 0;

/* ========== 滤波缓冲区 ========== */
#define WC_FILTER_WINDOW_SIZE 5
static rt_uint16_t s_ring_adc_buf[WC_FILTER_WINDOW_SIZE] = {0};
static rt_uint8_t s_ring_buf_idx = 0;
static rt_uint16_t s_lid_adc_buf[WC_FILTER_WINDOW_SIZE] = {0};
static rt_uint8_t s_lid_buf_idx = 0;
static rt_bool_t s_ring_buf_full = RT_FALSE;
static rt_bool_t s_lid_buf_full = RT_FALSE;
static rt_bool_t s_warm_fan_state = RT_FALSE; 
/* ========== 内部函数 ========== */

/* 移动平均滤波函数 */
static rt_uint16_t moving_average(rt_uint16_t *buf, rt_uint8_t *idx, rt_bool_t *full, rt_uint16_t new_val)
{
    buf[*idx] = new_val;
    *idx = (*idx + 1) % WC_FILTER_WINDOW_SIZE;
    if (*idx == 0) *full = RT_TRUE;

    rt_uint32_t sum = 0;
    rt_uint8_t count = (*full ? WC_FILTER_WINDOW_SIZE : *idx);
    for (rt_uint8_t i = 0; i < count; i++) {
        sum += buf[i];
    }
    return (rt_uint16_t)(sum / count);
}


static void small_pump_pwm_init(void)
{
    s_small_pump_pwm = (struct rt_device_pwm *)rt_device_find("pwm5");
    if (s_small_pump_pwm == RT_NULL) {
        rt_kprintf("[WC] PWM device pwm5 not found!\n");
        return;
    }
		rt_pwm_set(s_small_pump_pwm, 1, PWM_TIMER, 0);  /* 8kHz, 周期125000ns, 初始占空比0 */
    rt_pwm_disable(s_small_pump_pwm, 1);
}

static void step_motor_pipe_phase(rt_uint8_t phase)
{
    switch (phase % 4) {
        case 0:
            rt_pin_write(TUBE_STEPPER_OUT5, PIN_HIGH);
            rt_pin_write(TUBE_STEPPER_OUT6, PIN_HIGH);
            rt_pin_write(TUBE_STEPPER_OUT7, PIN_LOW);
            rt_pin_write(TUBE_STEPPER_OUT8, PIN_LOW);
            break;
        case 1:
            rt_pin_write(TUBE_STEPPER_OUT5, PIN_LOW);
            rt_pin_write(TUBE_STEPPER_OUT6, PIN_HIGH);
            rt_pin_write(TUBE_STEPPER_OUT7, PIN_HIGH);
            rt_pin_write(TUBE_STEPPER_OUT8, PIN_LOW);
            break;
        case 2:
            rt_pin_write(TUBE_STEPPER_OUT5, PIN_LOW);
            rt_pin_write(TUBE_STEPPER_OUT6, PIN_LOW);
            rt_pin_write(TUBE_STEPPER_OUT7, PIN_HIGH);
            rt_pin_write(TUBE_STEPPER_OUT8, PIN_HIGH);
            break;
        case 3:
            rt_pin_write(TUBE_STEPPER_OUT5, PIN_HIGH);
            rt_pin_write(TUBE_STEPPER_OUT6, PIN_LOW);
            rt_pin_write(TUBE_STEPPER_OUT7, PIN_LOW);
            rt_pin_write(TUBE_STEPPER_OUT8, PIN_HIGH);
            break;
    }
}

static void step_motor_rod_phase(rt_uint8_t phase)
{
	   uint8_t rev = (7 - (phase % 8)) % 8;  
//    switch (phase % 8) {					// 换相顺序反了
		switch (rev) {
        case 0:
            rt_pin_write(CLEANING_STEPPER_OUT1, PIN_HIGH);
            rt_pin_write(CLEANING_STEPPER_OUT2, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT3, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT4, PIN_LOW);
            break;
        case 1:
            rt_pin_write(CLEANING_STEPPER_OUT1, PIN_HIGH);
            rt_pin_write(CLEANING_STEPPER_OUT2, PIN_HIGH);
            rt_pin_write(CLEANING_STEPPER_OUT3, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT4, PIN_LOW);
            break;
        case 2:
            rt_pin_write(CLEANING_STEPPER_OUT1, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT2, PIN_HIGH);
            rt_pin_write(CLEANING_STEPPER_OUT3, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT4, PIN_LOW);
            break;
        case 3:
            rt_pin_write(CLEANING_STEPPER_OUT1, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT2, PIN_HIGH);
            rt_pin_write(CLEANING_STEPPER_OUT3, PIN_HIGH);
            rt_pin_write(CLEANING_STEPPER_OUT4, PIN_LOW);
            break;
        case 4:
            rt_pin_write(CLEANING_STEPPER_OUT1, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT2, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT3, PIN_HIGH);
            rt_pin_write(CLEANING_STEPPER_OUT4, PIN_LOW);
            break;
        case 5:
            rt_pin_write(CLEANING_STEPPER_OUT1, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT2, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT3, PIN_HIGH);
            rt_pin_write(CLEANING_STEPPER_OUT4, PIN_HIGH);
            break;
        case 6:
            rt_pin_write(CLEANING_STEPPER_OUT1, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT2, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT3, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT4, PIN_HIGH);
            break;
        case 7:
            rt_pin_write(CLEANING_STEPPER_OUT1, PIN_HIGH);
            rt_pin_write(CLEANING_STEPPER_OUT2, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT3, PIN_LOW);
            rt_pin_write(CLEANING_STEPPER_OUT4, PIN_HIGH);
            break;
    }
}

/* ========== 公开 API 实现 ========== */

// 旋钮中断回调函数
static void knob_female_isr(void *args)
{
    // 下降沿触发，发送女士清洁命令
    user_action_send_cmd(ACTION_CLEAN_FEMALE, 0);
}

static void knob_rear_isr(void *args)
{
    // 下降沿触发，发送臀部清洁命令
    user_action_send_cmd(ACTION_CLEAN_REAR, 0);
}

static void knob_button_isr(void *args)
{
    // 下降沿触发，发送冲洗马桶命令
    user_action_send_cmd(ACTION_FLUSH_TOILET, 0);
}


void wc_pump_wash_on(void)
{
    rt_pin_write(LARGE_PUMP_PIN, PIN_HIGH);
}

void wc_pump_wash_off(void)
{
    rt_pin_write(LARGE_PUMP_PIN, PIN_LOW);
}

/* ========== 排污泵 ========== */
void wc_sewage_pump_on(void)
{
    rt_pin_write(SEWAGE_PUMP_PIN, PIN_HIGH);
}

void wc_sewage_pump_off(void)
{
    rt_pin_write(SEWAGE_PUMP_PIN, PIN_LOW);
}

/* 马桶圈电机 */
void wc_ring_motor_enable(void)
{
    rt_pin_write(RING_MOTOR_ENABLE_PIN, PIN_HIGH);
}
void wc_ring_motor_disable(void)
{
    rt_pin_write(RING_MOTOR_ENABLE_PIN, PIN_LOW);
}
void wc_ring_motor_dir_on(void)
{
    rt_pin_write(RING_MOTOR_DIR_PIN, PIN_HIGH);
}
void wc_ring_motor_dir_off(void)
{
    rt_pin_write(RING_MOTOR_DIR_PIN, PIN_LOW);
}

void wc_ring_motor_open(void)
{
    rt_pin_write(RING_MOTOR_DIR_PIN, PIN_HIGH);
    wc_ring_motor_enable();
}
void wc_ring_motor_close(void)
{
    rt_pin_write(RING_MOTOR_DIR_PIN, PIN_LOW);
    wc_ring_motor_enable();
}

/* 马桶盖电机 */
void wc_lid_motor_enable(void)
{
    rt_pin_write(LID_MOTOR_ENABLE_PIN, PIN_HIGH);
}
void wc_lid_motor_disable(void)
{
    rt_pin_write(LID_MOTOR_ENABLE_PIN, PIN_LOW);
}
void wc_lid_motor_dir_on(void)
{
    rt_pin_write(LID_MOTOR_DIR_PIN, PIN_HIGH);
}
void wc_lid_motor_dir_off(void)
{
    rt_pin_write(LID_MOTOR_DIR_PIN, PIN_LOW);
}

void wc_lid_motor_open(void)
{
	  rt_pin_write(LID_MOTOR_DIR_PIN, PIN_LOW);
    wc_lid_motor_enable();
}
void wc_lid_motor_close(void)
{
	  rt_pin_write(LID_MOTOR_DIR_PIN, PIN_HIGH);
    wc_lid_motor_enable();
}

/* 小水泵 PWM */
void wc_small_pump_set_duty(rt_uint8_t duty)
{
    if (s_small_pump_pwm == RT_NULL) return;
    if (duty > 100) duty = 100;
     rt_pwm_set(s_small_pump_pwm, 1, PWM_TIMER, (duty * PWM_DUTY));
}
void wc_small_pump_enable(rt_bool_t enable)
{
    if (s_small_pump_pwm == RT_NULL) return;
    if (enable)
        rt_pwm_enable(s_small_pump_pwm, 1);
    else
        rt_pwm_disable(s_small_pump_pwm, 1);
}

/* 清洁杆步进电机 */
void wc_clean_rod_set_position(rt_uint16_t position)
{
    if (position > CLEAN_ROD_MAX_STEPS) position = CLEAN_ROD_MAX_STEPS;
    rt_uint16_t diff, step;
    if (position == s_clean_rod_pos) return;

    if (position > s_clean_rod_pos) {
        diff = position - s_clean_rod_pos;
        for (step = 0; step < diff; step++) {
            step_motor_rod_phase(step);
            rt_thread_mdelay(2);   /* 速度可调 */
        }
    } else {
        diff = s_clean_rod_pos - position;
        for (step = 0; step < diff; step++) {
            step_motor_rod_phase(s_clean_rod_pos - step - 1);
            rt_thread_mdelay(2);
        }
    }
    s_clean_rod_pos = position;
    wc_clean_rod_stop();
}
void wc_clean_rod_stop(void)
{
    rt_pin_write(CLEANING_STEPPER_OUT1, PIN_LOW);
    rt_pin_write(CLEANING_STEPPER_OUT2, PIN_LOW);
    rt_pin_write(CLEANING_STEPPER_OUT3, PIN_LOW);
    rt_pin_write(CLEANING_STEPPER_OUT4, PIN_LOW);
}
rt_uint16_t wc_clean_rod_get_position(void)
{
    return s_clean_rod_pos;
}

/* 管路分配器步进电机 */
void wc_pipe_distributor_set_position(rt_uint16_t position)
{
    if (position > PIPE_POS_MAX) position = PIPE_POS_MAX;
    rt_uint16_t diff, step;
    if (position == s_pipe_pos) return;

    if (position > s_pipe_pos) {
        diff = position - s_pipe_pos;
        for (step = 0; step < diff; step++) {
            step_motor_pipe_phase(step);
            rt_thread_mdelay(3);
        }
    } else {
        diff = s_pipe_pos - position;
        for (step = diff; step > 0; step--) {
            step_motor_pipe_phase(step);
            rt_thread_mdelay(3);
        }
    }
    s_pipe_pos = position;
    wc_pipe_distributor_stop();
}
void wc_pipe_distributor_stop(void)
{
    rt_pin_write(TUBE_STEPPER_OUT5, PIN_LOW);
    rt_pin_write(TUBE_STEPPER_OUT6, PIN_LOW);
    rt_pin_write(TUBE_STEPPER_OUT7, PIN_LOW);
    rt_pin_write(TUBE_STEPPER_OUT8, PIN_LOW);
}
rt_uint16_t wc_pipe_distributor_get_position(void)
{
    return s_pipe_pos;
}

/* 紫外灯 */
void wc_uv_light_on(void)
{
    rt_pin_write(UV_LED_PIN, PIN_HIGH);
}
void wc_uv_light_off(void)
{
    rt_pin_write(UV_LED_PIN, PIN_LOW);
}

/* 红外灯 */
void wc_ir_light_on(void)
{
    rt_pin_write(IR_HEAT_PIN, PIN_HIGH);
}
void wc_ir_light_off(void)
{
    rt_pin_write(IR_HEAT_PIN, PIN_LOW);
}

/* 暖风风扇 */
void wc_warm_fan_on(void)
{
    rt_pin_write(FAN_PIN, PIN_HIGH);
		s_warm_fan_state = RT_TRUE;
}
void wc_warm_fan_off(void)
{
    rt_pin_write(FAN_PIN, PIN_LOW);
		s_warm_fan_state = RT_FALSE;
}
rt_bool_t wc_warm_fan_get_state(void)
{
    return s_warm_fan_state;
}
/* 暖风电热丝 */
void wc_warm_heater_on(void)
{
    rt_pin_write(DRYER_HEATER_PIN, PIN_HIGH);
}
void wc_warm_heater_off(void)
{
    rt_pin_write(DRYER_HEATER_PIN, PIN_LOW);
}
void wc_warm_heater_set_power(rt_uint8_t power)
{

}

/* 水箱加热管 */
void wc_water_heater_on(void)
{
    rt_pin_write(HEATER_CTRL_PIN, PIN_HIGH);
}
void wc_water_heater_off(void)
{
    rt_pin_write(HEATER_CTRL_PIN, PIN_LOW);
}


/* 马桶圈角度转换（PC1，ADC1_CH11）*/
static rt_uint8_t ring_adc_to_angle(rt_uint16_t adc)
{
    // 实测：0° 对应 0.550V → ADC = 0.550 * 4095 / 3.3 ≈ 683
    //       90°对应 1.906V → ADC = 1.906 * 4095 / 3.3 ≈ 2366
    const uint16_t adc_0 = 683;
    const uint16_t adc_90 = 2366;
    if (adc <= adc_0) return 0;
    if (adc >= adc_90) return 90;
    // 线性插值
    float tmp = (float)(adc - adc_0) * 90.0f / (adc_90 - adc_0);
    return (rt_uint8_t)(tmp + 0.5f);
}

/* 马桶盖角度转换（PC2，ADC1_CH12）*/
static rt_uint8_t lid_adc_to_angle(rt_uint16_t adc)
{
    // 实测：0° 对应 1.873V → ADC = 1.873 * 4095 / 3.3 ≈ 2325
    //       90°对应 0.533V → ADC = 0.533 * 4095 / 3.3 ≈ 661
	    const uint16_t adc_0 = 2325;
    const uint16_t adc_90 = 661;
    if (adc >= adc_0) return 0;
    if (adc <= adc_90) return 90;
    // 线性插值
    float tmp = (float)(adc - adc_0) * 90.0f / (adc_90 - adc_0);
    return (rt_uint8_t)(tmp + 0.5f);
}

/* 马桶圈电机位置（返回角度 0~90）*/
rt_uint16_t wc_ring_motor_get_position(void)
{
    if (!s_adc_dev) return 0;

    rt_uint32_t sum = 0;
    for (uint8_t i = 0; i < WC_FILTER_WINDOW_SIZE; i++) {
        rt_adc_enable(s_adc_dev, RING_MOTOR_POS_ADC);
        rt_uint32_t raw = rt_adc_read(s_adc_dev, RING_MOTOR_POS_ADC);
        rt_adc_disable(s_adc_dev, RING_MOTOR_POS_ADC);
        sum += raw;
    }
    rt_uint16_t avg_raw = (rt_uint16_t)(sum / WC_FILTER_WINDOW_SIZE);
    return (rt_uint16_t)ring_adc_to_angle(avg_raw);
}

/* 马桶盖电机位置（返回角度 0~90）*/
rt_uint16_t wc_lid_motor_get_position(void)
{
    if (!s_adc_dev) return 0;

    rt_uint32_t sum = 0;
    for (uint8_t i = 0; i < WC_FILTER_WINDOW_SIZE; i++) {
        rt_adc_enable(s_adc_dev, LID_MOTOR_POS_ADC);
        rt_uint32_t raw = rt_adc_read(s_adc_dev, LID_MOTOR_POS_ADC);
        rt_adc_disable(s_adc_dev, LID_MOTOR_POS_ADC);
        sum += raw;
    }
    rt_uint16_t avg_raw = (rt_uint16_t)(sum / WC_FILTER_WINDOW_SIZE);
    return (rt_uint16_t)lid_adc_to_angle(avg_raw);
}

/* 圆形旋钮  已改用中断*/
KnobEvent_t wc_knob_get_event(void)
{
    static rt_bool_t last_female = PIN_HIGH, last_rear = PIN_HIGH, last_button = PIN_HIGH;
    rt_bool_t female = rt_pin_read(KNOB_FEMALE_PIN);
    rt_bool_t rear   = rt_pin_read(KNOB_REAR_PIN);
    rt_bool_t button = rt_pin_read(KNOB_BUTTON_PIN);
    /* 电平下降沿触发 */
    if (button == PIN_LOW && last_button == PIN_HIGH) {
        last_button = button;
        return KNOB_BUTTON;
    }
    if (female == PIN_LOW && last_female == PIN_HIGH) {
        last_female = female;
        return KNOB_FEMALE;
    }
    if (rear == PIN_LOW && last_rear == PIN_HIGH) {
        last_rear = rear;
        return KNOB_REAR;
    }
    last_female = female;
    last_rear   = rear;
    last_button = button;
    return KNOB_NONE;
}

/* 水温传感器 */
rt_uint16_t wc_water_temp_get(void)
{
    if (s_adc_dev == RT_NULL) return 0;
	
	  rt_adc_enable(s_adc_dev, RING_MOTOR_POS_ADC);       // 配置并启动转换（内部已加锁）
		rt_uint32_t raw = rt_adc_read(s_adc_dev, WATER_TEMP_ADC);
    rt_adc_disable(s_adc_dev, RING_MOTOR_POS_ADC);      // 释放资源		
	
    /* 返回原始 ADC 值，上层根据 NTC 分压公式转换温度 */
    return (rt_uint16_t)raw;
}

/* 水位开关 */
rt_bool_t wc_water_tank_high_level(void)
{
    return rt_pin_read(WATER_LEVEL_HIGH_PIN) == PIN_HIGH ? RT_TRUE : RT_FALSE;
}
rt_bool_t wc_water_tank_low_level(void)
{
    return rt_pin_read(WATER_LEVEL_LOW_PIN) == PIN_HIGH ? RT_TRUE : RT_FALSE;
}

/* 旋钮中断初始化 */
static void knob_irq_init(void)
{
    // 配置引脚为输入上拉（默认为高电平）
    rt_pin_mode(KNOB_FEMALE_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(KNOB_REAR_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(KNOB_BUTTON_PIN, PIN_MODE_INPUT_PULLUP);

    // 绑定中断回调（下降沿触发）
    rt_pin_attach_irq(KNOB_FEMALE_PIN, PIN_IRQ_MODE_FALLING, knob_female_isr, RT_NULL);
    rt_pin_attach_irq(KNOB_REAR_PIN, PIN_IRQ_MODE_FALLING, knob_rear_isr, RT_NULL);
    rt_pin_attach_irq(KNOB_BUTTON_PIN, PIN_IRQ_MODE_FALLING, knob_button_isr, RT_NULL);

    // 使能中断
    rt_pin_irq_enable(KNOB_FEMALE_PIN, PIN_IRQ_ENABLE);
    rt_pin_irq_enable(KNOB_REAR_PIN, PIN_IRQ_ENABLE);
    rt_pin_irq_enable(KNOB_BUTTON_PIN, PIN_IRQ_ENABLE);
}

/* 初始化 */
void wc_drv_init(void)
{
    /* 1. GPIO 输出配置 */
		// 大水泵
		rt_pin_mode(LARGE_PUMP_PIN, PIN_MODE_OUTPUT);
		rt_pin_write(LARGE_PUMP_PIN, PIN_LOW);

		// 排污泵
		rt_pin_mode(SEWAGE_PUMP_PIN, PIN_MODE_OUTPUT);
		rt_pin_write(SEWAGE_PUMP_PIN, PIN_LOW);

    rt_pin_mode(RING_MOTOR_ENABLE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RING_MOTOR_ENABLE_PIN, PIN_LOW);
    rt_pin_mode(RING_MOTOR_DIR_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RING_MOTOR_DIR_PIN, PIN_LOW);

    rt_pin_mode(LID_MOTOR_ENABLE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LID_MOTOR_ENABLE_PIN, PIN_LOW);
    rt_pin_mode(LID_MOTOR_DIR_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LID_MOTOR_DIR_PIN, PIN_LOW);

    rt_pin_mode(CLEANING_STEPPER_OUT1, PIN_MODE_OUTPUT);
    rt_pin_mode(CLEANING_STEPPER_OUT2, PIN_MODE_OUTPUT);
    rt_pin_mode(CLEANING_STEPPER_OUT3, PIN_MODE_OUTPUT);
    rt_pin_mode(CLEANING_STEPPER_OUT4, PIN_MODE_OUTPUT);
    wc_clean_rod_stop();

    rt_pin_mode(TUBE_STEPPER_OUT5, PIN_MODE_OUTPUT);
    rt_pin_mode(TUBE_STEPPER_OUT6, PIN_MODE_OUTPUT);
    rt_pin_mode(TUBE_STEPPER_OUT7, PIN_MODE_OUTPUT);
    rt_pin_mode(TUBE_STEPPER_OUT8, PIN_MODE_OUTPUT);
    wc_pipe_distributor_stop();

    rt_pin_mode(UV_LED_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(UV_LED_PIN, PIN_LOW);  
    rt_pin_mode(IR_HEAT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(IR_HEAT_PIN, PIN_LOW);
    rt_pin_mode(FAN_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(FAN_PIN, PIN_LOW);
    rt_pin_mode(DRYER_HEATER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(DRYER_HEATER_PIN, PIN_LOW);
    rt_pin_mode(HEATER_CTRL_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(HEATER_CTRL_PIN, PIN_LOW);

    knob_irq_init();

    rt_pin_mode(WATER_LEVEL_HIGH_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(WATER_LEVEL_LOW_PIN, PIN_MODE_INPUT_PULLUP);

    /* 2. PWM 初始化 */
    small_pump_pwm_init();

    /* 3. ADC 初始化（用于位置反馈和水温） */
    s_adc_dev = (struct rt_adc_device *)rt_device_find("adc1");
    if (s_adc_dev) {
//        rt_adc_enable(s_adc_dev, RING_MOTOR_POS_ADC);
//        rt_adc_enable(s_adc_dev, LID_MOTOR_POS_ADC);
//        rt_adc_enable(s_adc_dev, WATER_TEMP_ADC);
    } else {
        rt_kprintf("[WC] ADC device not found!\n");
    }

    rt_kprintf("[WC] Smart Toilet driver initialized\n");
}

// 出厂前需要校准 1.温度传感器的ad采样 2. 马桶圈、马桶盖的ad采样和角度的关系lid_adc_to_angle

