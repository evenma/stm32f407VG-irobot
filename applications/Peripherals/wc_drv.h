/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 智能马桶底层驱动接口
 */

#ifndef __WC_DRV_H__
#define __WC_DRV_H__

#include <rtthread.h>

/* ========== 大水泵（冲洗马桶） ========== */
void wc_pump_wash_on(void);
void wc_pump_wash_off(void);

/* ========== 排污泵 ========== */
void wc_sewage_pump_on(void);
void wc_sewage_pump_off(void);

/* ========== 马桶圈电机（便门） ========== */
void wc_ring_motor_enable(void);
void wc_ring_motor_disable(void);
void wc_ring_motor_dir_on(void);
void wc_ring_motor_dir_off(void);
void wc_ring_motor_open(void);
void wc_ring_motor_close(void);
rt_uint16_t wc_ring_motor_get_position(void);   /* 返回位置反馈 ADC 原始值 0~4095 */

/* ========== 马桶盖电机（可选） ========== */
void wc_lid_motor_enable(void);
void wc_lid_motor_disable(void);
void wc_lid_motor_dir_on(void);
void wc_lid_motor_dir_off(void);
void wc_lid_motor_open(void);
void wc_lid_motor_close(void);
rt_uint16_t wc_lid_motor_get_position(void);

/* ========== 小水泵（PWM 调速） ========== */
/* duty: 0~100，对应占空比百分比 */
#define PWM_TIMER			125000UL//500000UL//125000UL    /* 8K 周期为125us，单位为纳秒ns */
#define PWM_DUTY    		1250//5000//1250		/* PWM_PERIOD/100;*/
void wc_small_pump_set_duty(rt_uint8_t duty);
void wc_small_pump_enable(rt_bool_t enable);

/* ========== 清洁杆步进电机（35型，8步） ========== */
/* position: 步数，范围 0 ~ CLEAN_ROD_MAX_STEPS，对应 0~70mm */
#define CLEAN_ROD_MAX_STEPS   1600  //实际测量100mm行程,// 1500   /* 总行程 70mm */
#define CLEAN_ROD_4MM_STEPS    86    /* 4mm 对应步数（约 1500/70*4） */
void wc_clean_rod_set_position(rt_uint16_t position);
void wc_clean_rod_stop(void);
rt_uint16_t wc_clean_rod_get_position(void);

/* ========== 管路分配器步进电机（28型，4步） ========== */
#define PIPE_POS_FEMALE       0     /* 女士清洁（小便） */
#define PIPE_POS_SELF_CLEAN   780   /* 自清洁 */
#define PIPE_POS_ANUS         1480  /* 肛门管路（大便） */
#define PIPE_POS_MAX          1750
void wc_pipe_distributor_set_position(rt_uint16_t position);
void wc_pipe_distributor_stop(void);
rt_uint16_t wc_pipe_distributor_get_position(void);

/* ========== 紫外灯 ========== */
void wc_uv_light_on(void);
void wc_uv_light_off(void);

/* ========== 红外灯 ========== */
void wc_ir_light_on(void);
void wc_ir_light_off(void);

/* ========== 暖风风扇 ========== */
void wc_warm_fan_on(void);
void wc_warm_fan_off(void);
rt_bool_t wc_warm_fan_get_state(void); 

/* ========== 暖风电热丝 ========== */
void wc_warm_heater_on(void);
void wc_warm_heater_off(void);
/* 功率控制（0~100），通过 PWM 或定时开关实现，简单实现可只开关 */
void wc_warm_heater_set_power(rt_uint8_t power);

/* ========== 水箱加热管 ========== */
void wc_water_heater_on(void);
void wc_water_heater_off(void);

/* ========== 圆形旋钮信号（非阻塞） ========== */
typedef enum {
    KNOB_NONE,
    KNOB_FEMALE,    // 女士清洁
    KNOB_REAR,      // 臀部清洁
    KNOB_BUTTON,    // 按钮按下（冲洗马桶）
} KnobEvent_t;
KnobEvent_t wc_knob_get_event(void);

/* ========== 水温传感器 ========== */
/* 返回温度，单位 0.1℃（如 253 表示 25.3℃）*/
rt_uint16_t wc_water_temp_get(void);

/* ========== 水箱水位开关 ========== */
rt_bool_t wc_water_tank_high_level(void);   /* RT_TRUE = 高水位 */
rt_bool_t wc_water_tank_low_level(void);    /* RT_TRUE = 低水位 */

/* 马桶圈电机位置（返回角度 0~90）*/
rt_uint16_t wc_ring_motor_get_position(void);
/* 马桶盖电机位置（返回角度 0~90）*/
rt_uint16_t wc_lid_motor_get_position(void);

/* ========== 初始化 ========== */
void wc_drv_init(void);

#endif /* __WC_DRV_H__ */

