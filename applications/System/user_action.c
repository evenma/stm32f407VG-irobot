/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 智能马桶任务管理实现
 */

#include <rtthread.h>
#include "user_action.h"
#include "wc_drv.h"
#include "buzzer.h"
#include "monitor.h"   // 用于获取水箱加热器供电状态
#include <stdio.h>

/* ========== 邮箱和事件 ========== */
#define ACTION_THREAD_STACK_SIZE     2048
#define ACTION_THREAD_PRIORITY       15

static struct rt_mailbox s_action_mb;
static rt_uint32_t s_action_mb_pool[4];
static struct rt_event s_action_evt;
static rt_thread_t s_action_thread;

/* ========== 全局配置 ========== */
volatile UserActionConfig_t g_action_cfg = {
//    .clean_rod_fixed_pos = 800,
	  .clean_rod_fixed_pos_anus = 1285,   // 60mm (1500步/70mm * 60 = 1285.7)
    .clean_rod_fixed_pos_female = CLEAN_ROD_MAX_STEPS, // 70mm
//    .clean_duration_sec = 40,
		.clean_duration_sec_anus = 60,
		.clean_duration_sec_female = 30,
    .dry_duration_sec = 60,
    .small_pump_duty = 70,
    .warm_heater_power = 50,
    .clean_mode = CLEAN_MODE_FIXED,
		.flush_duration_sec = 5,
    .flush_pump_delay_sec = 2,
    .flush_pump_stop_delay_sec = 1,
};
typedef enum {
    CLEAN_TYPE_NONE,
    CLEAN_TYPE_ANUS,
    CLEAN_TYPE_FEMALE
} CleanType_t;

static CleanType_t s_current_clean_type = CLEAN_TYPE_NONE;
static WorkStatus_t s_work_status = {0};          // 工作状态
static rt_timer_t s_report_timer;                 // 状态上报定时器

static rt_timer_t s_status_timer = RT_NULL;
static rt_bool_t s_status_timer_running = RT_FALSE;

/* ========== 动作执行辅助函数 ========== */
/* 状态上报函数（可替换为实际通信） */
static void report_work_status(void *param)
{
    // 通过串口发送到上位机
//		transfer_report
    rt_kprintf("[WORK_STATUS] Flush:%d Rear:%d Female:%d Dry:%d\n",
               s_work_status.flush_toilet,
               s_work_status.clean_rear,
               s_work_status.clean_female,
               s_work_status.dry);
}
const WorkStatus_t* user_action_get_work_status(void)
{
    return &s_work_status;
}

/* 等待事件（超时或手动停止） */
static int wait_stop_or_timeout(rt_int32_t timeout_ticks)
{
    rt_uint32_t recv_evt;
    rt_err_t ret = rt_event_recv(&s_action_evt, EVENT_STOP, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                 timeout_ticks, &recv_evt);
    if (ret == RT_EOK) {
        rt_kprintf("DEBUG: stop event received\n");
        return -1;
    }
    return 0;        // 超时
}

/* 安全停止所有动作 */
static void stop_all_actions(void)
{
    wc_pump_wash_off();
		wc_sewage_pump_off();   // 停止排污泵
    wc_small_pump_enable(RT_FALSE);
    wc_warm_fan_off();
    wc_warm_heater_off();
    wc_uv_light_off();
    wc_ir_light_off();
    wc_water_heater_off();
    /* 步进电机停止 */
    wc_clean_rod_stop();
    wc_pipe_distributor_stop();
	  wc_lid_motor_disable();
		wc_ring_motor_disable();
}

// 德西欧直流无刷水泵DXO-B01,最大流量40±15%L/min  最大扬程10±1m  0.66L/S   设定5S 3.3L
/** 冲洗马桶 水量=设定时间*0.33L/S 如9S=3L水，排水水泵比进水快1倍  用水量1.2L,用时10S **/
static void action_flush_toilet(void)
{
    rt_kprintf("[ACTION] Flush toilet start\n");
    /* 检查低水位，水位过低不能冲洗 */
    if (wc_water_tank_low_level()) {
        rt_kprintf("[ACTION] Water level low, cannot flush!\n");
        buzzer_beep_once();
        return;
    }

    /* 1. 开启大水泵，延时后开排污泵 */
		s_work_status.flush_toilet = RT_TRUE;  
    wc_pump_wash_on();
    rt_kprintf("[ACTION] Pump wash ON\n");

    /* 等待指定时间后再开启排污泵 */
    if (wait_stop_or_timeout(RT_TICK_PER_SECOND * g_action_cfg.flush_pump_delay_sec) != 0) {
        wc_pump_wash_off();
        return;  // 收到停止信号
    }

    wc_sewage_pump_on();
    rt_kprintf("[ACTION] Sewage pump ON\n");

    /* 继续运行剩余冲洗时间（总冲洗时间 - 延迟时间） */
    rt_int32_t remaining_ticks = RT_TICK_PER_SECOND * (g_action_cfg.flush_duration_sec - g_action_cfg.flush_pump_delay_sec);
    if (remaining_ticks > 0) {
        if (wait_stop_or_timeout(remaining_ticks) != 0) {
            // 收到停止信号，需先关排污泵再关大水泵
            wc_sewage_pump_off();
            wc_pump_wash_off();
					   s_work_status.flush_toilet = RT_FALSE;
            return;
        }
    }

    /* 3. 关闭：先关排污泵，延时后关大水泵 */
    wc_sewage_pump_off();
    rt_kprintf("[ACTION] Sewage pump OFF\n");

    if (wait_stop_or_timeout(RT_TICK_PER_SECOND * g_action_cfg.flush_pump_stop_delay_sec) != 0) {
        // 收到停止信号，直接关大水泵
        wc_pump_wash_off();
			   s_work_status.flush_toilet = RT_FALSE;
        return;
    }

    wc_pump_wash_off();
    rt_kprintf("[ACTION] Pump wash OFF\n");
		s_work_status.flush_toilet = RT_FALSE;
    rt_kprintf("[ACTION] Flush toilet done\n");
}

/* 执行清洁（通用） */
static void action_clean_common(CleanMode_t mode, uint16_t target_pos, uint8_t duration_sec, CleanType_t clean_type)
{
	    rt_tick_t start_tick = rt_tick_get();
    rt_tick_t end_tick = start_tick + RT_TICK_PER_SECOND * duration_sec;
    rt_bool_t stopped = RT_FALSE;
    /* 检查低水位 */
    if (wc_water_tank_low_level()) {
        rt_kprintf("[ACTION] Water level low, cannot clean!\n");
        buzzer_beep_once();
        return;
    }
		rt_kprintf("[ACTION] Clean start, mode=%d, target_pos=%d\n", mode, target_pos);
    // 根据类型设置状态
    if (clean_type == CLEAN_TYPE_ANUS)
        s_work_status.clean_rear = RT_TRUE;
    else if (clean_type == CLEAN_TYPE_FEMALE)
        s_work_status.clean_female = RT_TRUE;		

    /* 1. 清洁杆移动到设定位置 */
    wc_clean_rod_set_position(target_pos);
		if (wait_stop_or_timeout(500) != 0) {
				// 用户在中途停止了动作，需要清理并返回
				wc_clean_rod_set_position(0);		
				if (clean_type == CLEAN_TYPE_ANUS)
						s_work_status.clean_rear = RT_FALSE;
				else if (clean_type == CLEAN_TYPE_FEMALE)
						s_work_status.clean_female = RT_FALSE;
				rt_kprintf("[ACTION] Clean stopped during rod moving\n");
				wc_clean_rod_stop();
				return;
		}

    /* 2. 开启小水泵 */
		uint8_t duty = g_action_cfg.small_pump_duty;
    wc_small_pump_set_duty(duty);
    wc_small_pump_enable(RT_TRUE);

    /* 3. 根据模式执行清洁动作 */
		/* 按摩模式：往复运动，4mm 范围（约86步） */
		rt_uint16_t current_pos = target_pos;
		rt_bool_t forward = RT_TRUE;
		rt_uint16_t step_delta = CLEAN_ROD_4MM_STEPS;		
		
    while (rt_tick_get() < end_tick) {
        rt_uint32_t recv_evt;
			// 运行过程中调节水泵清洁力度
				if(duty != g_action_cfg.small_pump_duty){
					duty = g_action_cfg.small_pump_duty;
					wc_small_pump_set_duty(duty);
				}
				// 运行过程中修改清洁模式、增减清洁杆、停止
        if (rt_event_recv(&s_action_evt, EVENT_STOP | EVENT_CLEAN_MODE_SW|EVENT_CLEAN_ROD_INC|EVENT_CLEAN_ROD_DEC,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &recv_evt) == RT_EOK) {
            if (recv_evt & EVENT_STOP) {
                stopped = RT_TRUE;
                break;
            }
            if (recv_evt & EVENT_CLEAN_MODE_SW) {
                mode = g_action_cfg.clean_mode;
                rt_kprintf("[ACTION] Clean mode switched to %d\n", mode);
            }
						
						if (recv_evt & EVENT_CLEAN_ROD_INC) {
                // 根据清洁类型调整配置
                uint16_t new_pos;
                if (clean_type == CLEAN_TYPE_ANUS) {
                    new_pos = g_action_cfg.clean_rod_fixed_pos_anus + 10;
                    if (new_pos <= CLEAN_ROD_MAX_STEPS) {
                        g_action_cfg.clean_rod_fixed_pos_anus = new_pos;
                        target_pos = new_pos;           // 更新中心位置
                        if (mode == CLEAN_MODE_MASSAGE) {
                            current_pos = new_pos;     // 重置当前位置到新中心
														forward = RT_TRUE;
                        } 
												wc_clean_rod_set_position(new_pos); //立即移动清洁杆到新位置      
                        rt_kprintf("[ACTION] Anus clean rod position: %d\n", new_pos);
                    }
                } else if (clean_type == CLEAN_TYPE_FEMALE) {
                    new_pos = g_action_cfg.clean_rod_fixed_pos_female + 10;
                    if (new_pos <= CLEAN_ROD_MAX_STEPS) {
                        g_action_cfg.clean_rod_fixed_pos_female = new_pos;
                        target_pos = new_pos;
                        if (mode == CLEAN_MODE_MASSAGE) {
                            current_pos = new_pos;     // 重置当前位置到新中心
														forward = RT_TRUE;
                        } 
												wc_clean_rod_set_position(new_pos);
                        rt_kprintf("[ACTION] Female clean rod position: %d\n", new_pos);
                    }
                }
            }
            if (recv_evt & EVENT_CLEAN_ROD_DEC) {
                uint16_t new_pos;
                if (clean_type == CLEAN_TYPE_ANUS) {
                    if (g_action_cfg.clean_rod_fixed_pos_anus >= 10) {
                        new_pos = g_action_cfg.clean_rod_fixed_pos_anus - 10;
                        g_action_cfg.clean_rod_fixed_pos_anus = new_pos;
                        target_pos = new_pos;
                        if (mode == CLEAN_MODE_MASSAGE) {
                            current_pos = new_pos;
                        } 
												wc_clean_rod_set_position(new_pos);
                        rt_kprintf("[ACTION] Anus clean rod position: %d\n", new_pos);
                    }
                } else if (clean_type == CLEAN_TYPE_FEMALE) {
                    if (g_action_cfg.clean_rod_fixed_pos_female >= 10) {
                        new_pos = g_action_cfg.clean_rod_fixed_pos_female - 10;
                        g_action_cfg.clean_rod_fixed_pos_female = new_pos;
                        target_pos = new_pos;
                        if (mode == CLEAN_MODE_MASSAGE) {
                            current_pos = new_pos;
                        }
												wc_clean_rod_set_position(new_pos);
                        rt_kprintf("[ACTION] Female clean rod position: %d\n", new_pos);
                    }
                }
            }
        }
        if (mode == CLEAN_MODE_FIXED) {
            if (wait_stop_or_timeout(100) != 0) {
                stopped = RT_TRUE;
                break;
            }
        } else {
					int32_t new_pos;
					int32_t step = step_delta;  // 86步，对应4mm
					// 根据当前运动方向计算下一位置
					if (forward) {
							new_pos = current_pos + step;
							if (new_pos > target_pos + step) {
									// 超过上限，反向
									new_pos = target_pos + step - (new_pos - (target_pos + step));
									forward = RT_FALSE;
							}
					} else {
							new_pos = current_pos - step;
							if (new_pos < target_pos - step) {
									// 超过下限，反向
									new_pos = target_pos - step - (new_pos - (target_pos - step));
									forward = RT_TRUE;
							}
					}

					// 边界裁剪（防止超出硬件行程）
					if (new_pos > CLEAN_ROD_MAX_STEPS) new_pos = CLEAN_ROD_MAX_STEPS;
					if (new_pos < 0) new_pos = 0;
					current_pos = (rt_uint16_t)new_pos;
            wc_clean_rod_set_position(current_pos);
            if (wait_stop_or_timeout(500) != 0) {
                stopped = RT_TRUE;
                break;
            }
        }
    }

    /* 4. 停止小水泵和清洁杆 */
    wc_small_pump_enable(RT_FALSE);
		
    if (stopped) {
        rt_kprintf("[ACTION] Clean stopped by user\n");
    } else {
        rt_kprintf("[ACTION] Clean finished\n");
    }
		// 正常完成或者意外停止，清洁杆缩回0位置
    wc_clean_rod_set_position(0);
		
		if (clean_type == CLEAN_TYPE_ANUS)
        s_work_status.clean_rear = RT_FALSE;
    else if (clean_type == CLEAN_TYPE_FEMALE)
        s_work_status.clean_female = RT_FALSE;
		
		wc_clean_rod_stop();
}

/* 自清洁：清洁杆归零，管路分配器切换到自清洁位，小水泵低功率运行5秒 */
static void action_self_clean(void)
{
    rt_kprintf("[ACTION] Self clean start\n");
    if (wc_water_tank_low_level()) {
        rt_kprintf("[ACTION] Water level low, cannot self clean!\n");
        buzzer_beep_once();
        return;
    }

    // 1. 确保清洁杆在0位
    wc_clean_rod_set_position(0);
    // 2. 管路分配器切换到自清洁位置
    wc_pipe_distributor_set_position(PIPE_POS_SELF_CLEAN);
    // 3. 小水泵低功率运行（20%）
    uint8_t original_duty = g_action_cfg.small_pump_duty;
    wc_small_pump_set_duty(20);
    wc_small_pump_enable(RT_TRUE);

    // 4. 等待5秒或用户停止
    rt_tick_t start = rt_tick_get();
    rt_tick_t timeout = RT_TICK_PER_SECOND * 5;
    rt_bool_t stopped = RT_FALSE;
    while (rt_tick_get() - start < timeout) {
        if (wait_stop_or_timeout(100) != 0) {
            stopped = RT_TRUE;
            break;
        }
    }

    // 5. 关闭小水泵
    wc_small_pump_enable(RT_FALSE);
    // 恢复原功率值（可选）
    wc_small_pump_set_duty(original_duty);
    // 管路分配器回零位（可选）
    wc_pipe_distributor_set_position(0);

    if (stopped) {
        rt_kprintf("[ACTION] Self clean stopped by user\n");
    } else {
        rt_kprintf("[ACTION] Self clean finished\n");
    }
}

/* 暖风烘干动作 */
static void action_dry(void)
{
		rt_tick_t start_tick = rt_tick_get();
    rt_tick_t end_tick = start_tick + RT_TICK_PER_SECOND * g_action_cfg.dry_duration_sec;
    rt_kprintf("[ACTION] Dry start\n");
		s_work_status.dry = RT_TRUE;
		wc_warm_fan_on();                     // 风扇一直开

    while (rt_tick_get() < end_tick) {
		// 1s内对功率的调整 时间过长温度波动会比较大，不能稳定的恒温输出
			for (uint8_t i = 0; i < 50; i++) {
					if (i < (uint8_t)(g_action_cfg.warm_heater_power/2))
							wc_warm_heater_on();
					else
							wc_warm_heater_off();
					// 20ms一次等待
					if (wait_stop_or_timeout(20) != 0) {
							wc_warm_heater_off();
							wc_warm_fan_off();
							rt_kprintf("[ACTION] Dry done\n");
							s_work_status.dry = RT_FALSE;
							return;
					}
			}	
		}

    wc_warm_heater_off();
    wc_warm_fan_off();
    rt_kprintf("[ACTION] Dry done\n");
		s_work_status.dry = RT_FALSE;
}

/* 单独排污泵动作 */
//特别注意：在管路没水时不能开启，否则直接损坏排污泵，只有在马桶堵塞时可开启
static void action_sewage_pump(void)
{
    rt_kprintf("[ACTION] Sewage pump start (3s)\n");

    wc_sewage_pump_on();
    if (wait_stop_or_timeout(RT_TICK_PER_SECOND * 3) != 0) {
        rt_kprintf("[ACTION] Sewage pump stopped by user\n");
    }
    wc_sewage_pump_off();
    rt_kprintf("[ACTION] Sewage pump done\n");
}

/* 马桶圈控制 */
static void action_seat_open(void)
{
    rt_kprintf("[ACTION] Seat open start\n");
    wc_ring_motor_open();
    rt_tick_t start = rt_tick_get();
    rt_tick_t timeout = RT_TICK_PER_SECOND * 3;   // 3秒超时
    while (rt_tick_get() - start < timeout) {
        rt_uint16_t angle = wc_ring_motor_get_position();
        if (angle >= 85) {   // 接近 90°
            rt_kprintf("[ACTION] Seat opened, angle=%d\n", angle);
            break;
        }
        rt_thread_mdelay(50);
    }
    if (rt_tick_get() - start >= timeout) {
        rt_kprintf("[ACTION] Lid close timeout!\n");
    }else{
				rt_thread_mdelay(100); // 多动作一会确保到位
		}
		wc_ring_motor_disable();
		wc_ring_motor_dir_off();
}

static void action_seat_close(void)
{
    rt_kprintf("[ACTION] Seat close start\n");
    wc_ring_motor_close();
    rt_tick_t start = rt_tick_get();
    rt_tick_t timeout = RT_TICK_PER_SECOND * 3;
    while (rt_tick_get() - start < timeout) {
        rt_uint16_t angle = wc_ring_motor_get_position();
        if (angle <= 5) {   // 接近 0°
            rt_kprintf("[ACTION] Seat closed, angle=%d\n", angle);
            break;
        }
        rt_thread_mdelay(50);
    }
    if (rt_tick_get() - start >= timeout) {
        rt_kprintf("[ACTION] Lid close timeout!\n");
    }else{
				rt_thread_mdelay(100); // 多动作一会确保到位
		}
		wc_ring_motor_disable();
		wc_ring_motor_dir_off();
}

/* 马桶盖控制（类似） */
static void action_lid_open(void)
{
    rt_kprintf("[ACTION] Lid open start\n");
    wc_lid_motor_open();
    rt_tick_t start = rt_tick_get();
    rt_tick_t timeout = RT_TICK_PER_SECOND * 3;
    while (rt_tick_get() - start < timeout) {
        rt_uint16_t angle = wc_lid_motor_get_position();
        if (angle >= 85) {
            rt_kprintf("[ACTION] Lid opened, angle=%d\n", angle);
            break;
        }
        rt_thread_mdelay(50);
    }
    if (rt_tick_get() - start >= timeout) {
        rt_kprintf("[ACTION] Lid close timeout!\n");
    }else{
				rt_thread_mdelay(100); // 多动作一会确保到位
		}
		wc_lid_motor_disable();
		wc_lid_motor_dir_off();
}

static void action_lid_close(void)
{
    rt_kprintf("[ACTION] Lid close start\n");
    wc_lid_motor_close();
    rt_tick_t start = rt_tick_get();
    rt_tick_t timeout = RT_TICK_PER_SECOND * 3;
    while (rt_tick_get() - start < timeout) {
        rt_uint16_t angle = wc_lid_motor_get_position();
        if (angle <= 5) {
            rt_kprintf("[ACTION] Lid closed, angle=%d\n", angle);
            break;
        }
        rt_thread_mdelay(50);
    }

    if (rt_tick_get() - start >= timeout) {
        rt_kprintf("[ACTION] Lid close timeout!\n");
    }else{
				rt_thread_mdelay(100); // 多动作一会确保到位
		}
		wc_lid_motor_disable();
		wc_lid_motor_dir_off();
}

/* 清洁杆位置调整 */
static void action_clean_rod_inc(void)
{
    if (s_work_status.clean_rear || s_work_status.clean_female) {
        rt_event_send(&s_action_evt, EVENT_CLEAN_ROD_INC);
    } else {
        rt_kprintf("[ACTION] No active clean, cannot adjust position\n");
    }
}

static void action_clean_rod_dec(void)
{
    if (s_work_status.clean_rear || s_work_status.clean_female) {
        rt_event_send(&s_action_evt, EVENT_CLEAN_ROD_DEC);
    } else {
        rt_kprintf("[ACTION] No active clean, cannot adjust position\n");
    }
}

/* 水箱加热器控制（需检测供电）*/
static void action_set_water_heater(rt_bool_t on)
{
    if (on) {
        /* 检查外部供电是否正常（monitor 中已提供 s_heater_mv） */
        if (monitor_get_heater_voltage() < 1000) { // 假设电压低于 1V 表示未接入
            rt_kprintf("[ACTION] Heater power not connected!\n");
            buzzer_beep_once();
            return;
        }
        wc_water_heater_on();
    } else {
        wc_water_heater_off();
    }
}

/* ========== 主线程入口 ========== */
static void user_action_thread_entry(void *param)
{
    rt_ubase_t cmd;
    rt_uint32_t param_val;
    ActionCmd_t action;

    rt_kprintf("[ACTION] Thread started\n");

    while (1) {
        /* 等待邮箱命令 */
        if (rt_mb_recv(&s_action_mb, &cmd, RT_WAITING_FOREVER) != RT_EOK) {
            continue;
        }
        action = (ActionCmd_t)(cmd & 0xFF);
        param_val = (cmd >> 8);

        rt_kprintf("[ACTION] Received command: %d, param=%d\n", action, param_val);

        /* 清空事件（避免残留）*/
        rt_event_recv(&s_action_evt, 0xFFFFFFFF, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_NO, &param_val);

        switch (action) {
					/* 动作类命令 */
            case ACTION_FLUSH_TOILET:
                action_flush_toilet();
                break;
						case ACTION_CLEAN_REAR:
								s_current_clean_type = CLEAN_TYPE_ANUS;
								wc_pipe_distributor_set_position(PIPE_POS_ANUS);
								action_clean_common(g_action_cfg.clean_mode,
																		g_action_cfg.clean_rod_fixed_pos_anus,
																		g_action_cfg.clean_duration_sec_anus,
																		CLEAN_TYPE_ANUS);
								break;
						case ACTION_CLEAN_FEMALE:
								s_current_clean_type = CLEAN_TYPE_FEMALE;
								wc_pipe_distributor_set_position(PIPE_POS_FEMALE);
								action_clean_common(g_action_cfg.clean_mode,
																		g_action_cfg.clean_rod_fixed_pos_female,
																		g_action_cfg.clean_duration_sec_female,
																		CLEAN_TYPE_FEMALE);
								break;
            case ACTION_DRY:
                action_dry();
                break;
            case ACTION_STOP:
                rt_event_send(&s_action_evt, EVENT_STOP);				//第三方来调用event,此处无用
                break;
            case ACTION_CLEAN_ROD_INC:
                action_clean_rod_inc();													//第三方来调用event,此处无用
                break;
            case ACTION_CLEAN_ROD_DEC:
                action_clean_rod_dec();													//第三方来调用event,此处无用
                break;
						case ACTION_SELF_CLEAN:
								action_self_clean();
								break;
					  /* 调节类命令：模式 */
            case ACTION_TOGGLE_CLEAN_MODE:											//第三方来调用event,此处无用
                g_action_cfg.clean_mode = (g_action_cfg.clean_mode == CLEAN_MODE_FIXED) ?
                                            CLEAN_MODE_MASSAGE : CLEAN_MODE_FIXED;
                rt_kprintf("[ACTION] Clean mode switched to %s\n",
                           g_action_cfg.clean_mode == CLEAN_MODE_FIXED ? "Fixed" : "Massage");
                /* 如果清洁正在运行，发送模式切换事件 */
                rt_event_send(&s_action_evt, EVENT_CLEAN_MODE_SW);
                break;
            case ACTION_SET_CLEAN_ROD_POS:
								if (param_val <= CLEAN_ROD_MAX_STEPS) {
										wc_clean_rod_set_position(param_val);
										rt_kprintf("[ACTION] Set clean rod position: %d\n", param_val);									
//									if (s_current_clean_type == CLEAN_TYPE_ANUS) {
//											g_action_cfg.clean_rod_fixed_pos_anus = param_val;
//											wc_clean_rod_set_position(param_val);
//											rt_kprintf("[ACTION] Set anus clean rod position: %d\n", param_val);
//									} else if (s_current_clean_type == CLEAN_TYPE_FEMALE) {
//											g_action_cfg.clean_rod_fixed_pos_female = param_val;
//											wc_clean_rod_set_position(param_val);
//											rt_kprintf("[ACTION] Set female clean rod position: %d\n", param_val);
//									} else {
//											rt_kprintf("[ACTION] No active clean type, cannot set position\n");
//									}
								}
                break;
            case ACTION_SEAT_OPEN:
                action_seat_open();
                break;
            case ACTION_SEAT_CLOSE:
                action_seat_close();
                break;
            case ACTION_LID_OPEN:
                action_lid_open();
                break;
            case ACTION_LID_CLOSE:
                action_lid_close();
                break;
            case ACTION_SET_WATER_HEATER:									//第三方来调用event,此处无用
                action_set_water_heater(param_val != 0);
                break;
            case ACTION_SET_WARM_FAN:
								if (param_val) {
										wc_warm_fan_on();
								} else {
										// 关闭风扇时，必须同时关闭电热丝，防止过热
										wc_warm_fan_off();
										wc_warm_heater_off();                     // 直接关硬件
										g_action_cfg.warm_heater_power = 0;       // 同步功率配置
								}
                break;
            case ACTION_SET_WARM_HEATER:													// 第三方调用设置，此处无用
							// 设置功率值（0~100）
								if (param_val > 100) param_val = 100;
								g_action_cfg.warm_heater_power = (rt_uint8_t)param_val;

								// 安全联动：功率非零时，若风扇未开则自动开启风扇
								if (param_val > 0 && !wc_warm_fan_get_state()) {
										wc_warm_fan_on();
										rt_kprintf("[ACTION] Auto turn on fan due to heater power set\n");
								} else if (param_val == 0) {
										// 功率为0时，关闭电热丝硬件（但风扇保持，由上层决定）
										wc_warm_heater_off();
								}
                break;
            case ACTION_SET_SMALL_PUMP_DUTY:												// 第三方调用设置，此处无用
                if (param_val <= 100) {
                    g_action_cfg.small_pump_duty = (rt_uint8_t)param_val;
                    wc_small_pump_set_duty(g_action_cfg.small_pump_duty);
                }
                break;
						case ACTION_SEWAGE_PUMP:
								action_sewage_pump();
								break;
						case ACTION_UV_LIGHT_ON:
								wc_uv_light_on();
								rt_kprintf("[ACTION] UV light ON\n");
								break;
						case ACTION_UV_LIGHT_OFF:
								wc_uv_light_off();
								rt_kprintf("[ACTION] UV light OFF\n");
								break;
						case ACTION_IR_LIGHT_ON:
								wc_ir_light_on();
								rt_kprintf("[ACTION] IR light ON\n");
								break;
						case ACTION_IR_LIGHT_OFF:
								wc_ir_light_off();
								rt_kprintf("[ACTION] IR light OFF\n");
								break;
            default:
                rt_kprintf("[ACTION] Unknown command\n");
                break;
        }
    }
}

/* ========== 公共函数 ========== */
void user_action_init(void)
{
    /* 初始化邮箱 */
    rt_mb_init(&s_action_mb, "action_mb", s_action_mb_pool,
               sizeof(s_action_mb_pool) / sizeof(rt_uint32_t),
               RT_IPC_FLAG_FIFO);
    /* 初始化事件 */
    rt_event_init(&s_action_evt, "action_evt", RT_IPC_FLAG_FIFO);

    /* 创建线程 */
    s_action_thread = rt_thread_create("user_action",
                                        user_action_thread_entry,
                                        RT_NULL,
                                        ACTION_THREAD_STACK_SIZE,
                                        ACTION_THREAD_PRIORITY,
                                        5);
		wc_drv_init(); 
    /* 归零清洁杆和管路分配器：先走到最大行程，再回到零点，确保机械归零 */
//		 rt_kprintf("[ACTION] Reset clean rod and pipe distributor to zero\n");
//    wc_clean_rod_set_position(CLEAN_ROD_MAX_STEPS);
//    rt_thread_mdelay(500);   // 等待到达最大位置
//    wc_clean_rod_set_position(0);
//    rt_thread_mdelay(500);

//    wc_pipe_distributor_set_position(PIPE_POS_MAX);
//    rt_thread_mdelay(500);
//    wc_pipe_distributor_set_position(0);
//    rt_thread_mdelay(500);
		
		if (s_action_thread != RT_NULL) {
        rt_thread_startup(s_action_thread);
    } else {
        rt_kprintf("[ACTION] Failed to create thread\n");
    }

		stop_all_actions();   //停止所有动作
}

void user_action_send_cmd(ActionCmd_t cmd, uint32_t param)
{
    rt_uint32_t msg = (uint32_t)cmd | (param << 8);
    rt_mb_send(&s_action_mb, msg);
}

void user_action_stop(void)
{
    user_action_send_cmd(ACTION_STOP, 0);
}

void user_action_set_clean_rod_position(uint16_t pos)
{
    user_action_send_cmd(ACTION_SET_CLEAN_ROD_POS, pos);
}

void user_action_clean_rod_inc(void)
{
    user_action_send_cmd(ACTION_CLEAN_ROD_INC, 0);
}

void user_action_clean_rod_dec(void)
{
    user_action_send_cmd(ACTION_CLEAN_ROD_DEC, 0);
}

void user_action_toggle_clean_mode(void)
{
    user_action_send_cmd(ACTION_TOGGLE_CLEAN_MODE, 0);
}

void user_action_sewage_pump(void)
{
    user_action_send_cmd(ACTION_SEWAGE_PUMP, 0);
}

#ifdef RT_USING_MSH
#include <stdlib.h>
#include <string.h>

/* 辅助函数：打印工作状态和传感器信息 */
static void print_toilet_status(void)
{
    const WorkStatus_t* work = user_action_get_work_status();
    rt_kprintf("\n========== Toilet Status ==========\n");
    rt_kprintf("Work Status:\n");
    rt_kprintf("  Flush:      %s\n", work->flush_toilet ? "Working" : "Idle");
    rt_kprintf("  Rear Clean: %s\n", work->clean_rear ? "Working" : "Idle");
    rt_kprintf("  Female Clean:%s\n", work->clean_female ? "Working" : "Idle");
    rt_kprintf("  Dry:        %s\n", work->dry ? "Working" : "Idle");

    // 水位状态
    rt_kprintf("\nWater Level:\n");
    rt_kprintf("  High: %s\n", monitor_get_water_high_level() ? "Yes" : "No");
    rt_kprintf("  Low:  %s\n", monitor_get_water_low_level() ? "Yes (Water shortage)" : "OK");

    // 电池信息
    rt_kprintf("\nPower:\n");
    rt_kprintf("  Battery: %d mV\n", monitor_get_battery_voltage());

	  // 马桶圈和马桶盖角度
    rt_kprintf("\nPosition:\n");
    rt_kprintf("  Seat angle: %d°\n", wc_ring_motor_get_position());
    rt_kprintf("  Lid angle:  %d°\n", wc_lid_motor_get_position());
		// 清洁杆当前步数（可选）
    rt_kprintf("  Clean rod steps: %d\n", wc_clean_rod_get_position());
		rt_kprintf("  pipe distributor steps: %d\n", wc_pipe_distributor_get_position());
    rt_kprintf("==================================\n");
}

static void status_timer_callback(void *param)
{
    print_toilet_status();
}


/* 命令: toilet flush 冲洗马桶*/
static void cmd_flush(void)
{
    user_action_send_cmd(ACTION_FLUSH_TOILET, 0);
    rt_kprintf("[CMD] Flush toilet command sent\n");
}

/* 命令: toilet rear 清洁臀部*/
static void cmd_rear(void)
{
    user_action_send_cmd(ACTION_CLEAN_REAR, 0);
    rt_kprintf("[CMD] Rear clean command sent\n");
}

/* 命令: toilet female 女士清洁*/
static void cmd_female(void)
{
    user_action_send_cmd(ACTION_CLEAN_FEMALE, 0);
    rt_kprintf("[CMD] Female clean command sent\n");
}

/* 命令: toilet dry 暖风烘干*/
static void cmd_dry(void)
{
    user_action_send_cmd(ACTION_DRY, 0);
    rt_kprintf("[CMD] Dry command sent\n");
}

/* 命令: toilet stop 全部停止*/
static void cmd_stop(void)
{
    user_action_send_cmd(ACTION_STOP, 0);
    rt_kprintf("[CMD] Stop command sent\n");
}

/* 命令: toilet seat open/close 马桶圈开启和放下*/
static void cmd_seat_open(void)
{
    user_action_send_cmd(ACTION_SEAT_OPEN, 0);
    rt_kprintf("[CMD] Seat open command sent\n");
}
static void cmd_seat_close(void)
{
    user_action_send_cmd(ACTION_SEAT_CLOSE, 0);
    rt_kprintf("[CMD] Seat close command sent\n");
}

/* 命令: toilet lid open/close  马桶盖开启和放下*/
static void cmd_lid_open(void)
{
    user_action_send_cmd(ACTION_LID_OPEN, 0);
    rt_kprintf("[CMD] Lid open command sent\n");
}
static void cmd_lid_close(void)
{
    user_action_send_cmd(ACTION_LID_CLOSE, 0);
    rt_kprintf("[CMD] Lid close command sent\n");
}

/* 命令: toilet sewage 排污泵*/
/* 单独排污泵动作（带用户确认） */
static void cmd_sewage(void)
{
    rt_kprintf("\n!!! WARNING !!!\n");
    rt_kprintf("Sewage pump may be damaged if there is no water in the pipeline.\n");
    rt_kprintf("Are you sure to start sewage pump? Press Enter to confirm, any other key to cancel.\n");

    // 等待用户输入
    int c = getchar();
    if (c == '\r' || c == '\n') {
        user_action_send_cmd(ACTION_SEWAGE_PUMP, 0);
        rt_kprintf("[CMD] Sewage pump command sent\n");
    } else {
        rt_kprintf("[CMD] Sewage pump command canceled\n");
    }
}

/* 命令: toilet set_pump_duty <0-100> */
static void cmd_set_pump_duty(int duty)
{
    if (duty < 0) duty = 0;
    if (duty > 100) duty = 100;
		g_action_cfg.small_pump_duty = (rt_uint8_t)duty;
		wc_small_pump_set_duty(g_action_cfg.small_pump_duty);
		if(duty > 0){
			wc_small_pump_enable(RT_TRUE);
		}else{
			wc_small_pump_enable(RT_FALSE);
		}
    rt_kprintf("[CMD] Set small pump duty to %d\n", duty);
}

/* 命令: toilet set_heater_power <0-100> */
static void cmd_set_heater_power(int power)
{
    if (power < 0) power = 0;
    if (power > 100) power = 100;
    user_action_send_cmd(ACTION_SET_WARM_HEATER, power);
    rt_kprintf("[CMD] Set warm heater power to %d\n", power);
}

/* 主命令解析函数 */
static void toilet_cmd(int argc, char **argv)
{
     if (argc < 2) {
        rt_kprintf("Usage: toilet <command> [param]\n");
        rt_kprintf("Commands:\n");
        rt_kprintf("  flush\n  rear\n  female\n  dry\n  stop\n");
        rt_kprintf("  seat_open / seat_close\n  lid_open / lid_close\n");
        rt_kprintf("  sewage\n  set_pump_duty <0-100>\n  set_heater_power <0-100>\n");
        rt_kprintf("  rod_pos <steps> (0-%d)\n  rod_inc\n  rod_dec\n", CLEAN_ROD_MAX_STEPS);
				rt_kprintf("  uv_on / uv_off\n  ir_on / ir_off\n");
			  rt_kprintf("  self_clean\n  pipe_pos\n");
        rt_kprintf("  clean_mode [fixed|massage]\n  status [on|off]\n");
        return;
    }
		 
    if (strcmp(argv[1], "flush") == 0) {								// 冲洗马桶  大水泵 和排污泵
        cmd_flush();
    } else if (strcmp(argv[1], "rear") == 0) {					// 清洁臀部
        cmd_rear();
    } else if (strcmp(argv[1], "female") == 0) {    		// 女性清洁
        cmd_female();
    } else if (strcmp(argv[1], "dry") == 0) {       		// 暖风烘干
        cmd_dry();
    } else if (strcmp(argv[1], "stop") == 0) {			 		// 停止
				rt_event_send(&s_action_evt, EVENT_STOP);
				rt_kprintf("[CMD] Stop event sent\n");
    } else if (strcmp(argv[1], "seat_open") == 0) {   	// 坐便圈打开
        cmd_seat_open();
    } else if (strcmp(argv[1], "seat_close") == 0) {   	// 坐便圈关闭
        cmd_seat_close();
    } else if (strcmp(argv[1], "lid_open") == 0) {     	// 马桶盖打开
        cmd_lid_open();
    } else if (strcmp(argv[1], "lid_close") == 0) {     // 马桶盖关闭
        cmd_lid_close();
    } else if (strcmp(argv[1], "sewage") == 0) {        // 排污泵开启
        cmd_sewage();
    } else if (strcmp(argv[1], "set_pump_duty") == 0) {	// 调节小水泵pwm
        if (argc < 3) {
            rt_kprintf("Missing duty value\n");
            return;
        }
        int duty = atoi(argv[2]);
        cmd_set_pump_duty(duty);				
    } else if (strcmp(argv[1], "set_heater_power") == 0) { // 调节暖风温度
        if (argc < 3) {
            rt_kprintf("Missing power value\n");
            return;
        }
        int power = atoi(argv[2]);
				if (power < 0) power = 0;
				if (power > 100) power = 100;
				g_action_cfg.warm_heater_power = (uint8_t)power;
				rt_kprintf("[CMD] Heater power set to %d (will take effect immediately)\n", power);
    } else if (strcmp(argv[1], "rod_pos") == 0) {						// 设置清洁杆位置 0 - 1600步
        if (argc < 3) {
            rt_kprintf("Usage: toilet rod_pos <steps> (0-%d)\n", CLEAN_ROD_MAX_STEPS);
            return;
        }
        int pos = atoi(argv[2]);
        if (pos < 0 || pos > CLEAN_ROD_MAX_STEPS) {
            rt_kprintf("Invalid position, must be 0-%d\n", CLEAN_ROD_MAX_STEPS);
            return;
        }
        user_action_send_cmd(ACTION_SET_CLEAN_ROD_POS, pos);
        rt_kprintf("[CMD] Set clean rod position to %d\n", pos);
    }else if (strcmp(argv[1], "rod_inc") == 0) {								// 调整清洁杆位置 增1mm
//        user_action_send_cmd(ACTION_CLEAN_ROD_INC, 0);
				if (s_work_status.clean_rear || s_work_status.clean_female) {
						rt_event_send(&s_action_evt, EVENT_CLEAN_ROD_INC);
				} else {
						rt_kprintf("[ACTION] No active clean, cannot adjust position\n");
				}					
        rt_kprintf("[CMD] Clean rod increased\n");
    }else if (strcmp(argv[1], "rod_dec") == 0) {								// 调整清洁杆位置  减1mm
//        user_action_send_cmd(ACTION_CLEAN_ROD_DEC, 0);
				if (s_work_status.clean_rear || s_work_status.clean_female) {
						rt_event_send(&s_action_evt, ACTION_CLEAN_ROD_DEC);
				} else {
						rt_kprintf("[ACTION] No active clean, cannot adjust position\n");
				}				
        rt_kprintf("[CMD] Clean rod decreased\n");
    }else if (strcmp(argv[1], "clean_mode") == 0) {							// 设置清洁过程中的模式 固定模式还是按摩来回伸缩4mm
        if (argc < 3) {
            rt_kprintf("Current clean mode: %s\n", g_action_cfg.clean_mode == CLEAN_MODE_FIXED ? "Fixed" : "Massage");
            rt_kprintf("Usage: toilet clean_mode [fixed|massage]\n");
            return;
        }
        if (strcmp(argv[2], "fixed") == 0) {
            if (g_action_cfg.clean_mode != CLEAN_MODE_FIXED) {
 //               user_action_send_cmd(ACTION_TOGGLE_CLEAN_MODE, 0);
								g_action_cfg.clean_mode = CLEAN_MODE_FIXED;
                rt_kprintf("[ACTION] Clean mode switched to %s\n",
                           g_action_cfg.clean_mode == CLEAN_MODE_FIXED ? "Fixed" : "Massage");
                /* 如果清洁正在运行，发送模式切换事件 */
                rt_event_send(&s_action_evt, EVENT_CLEAN_MODE_SW);
            }
            rt_kprintf("[CMD] Clean mode set to Fixed\n");
        } else if (strcmp(argv[2], "massage") == 0) {
            if (g_action_cfg.clean_mode != CLEAN_MODE_MASSAGE) {							
//                user_action_send_cmd(ACTION_TOGGLE_CLEAN_MODE, 0);
								g_action_cfg.clean_mode = CLEAN_MODE_MASSAGE;
                rt_kprintf("[ACTION] Clean mode switched to %s\n",
                           g_action_cfg.clean_mode == CLEAN_MODE_FIXED ? "Fixed" : "Massage");
                /* 如果清洁正在运行，发送模式切换事件 */
                rt_event_send(&s_action_evt, EVENT_CLEAN_MODE_SW);
            }
            rt_kprintf("[CMD] Clean mode set to Massage\n");
        } else {
            rt_kprintf("Invalid mode, use 'fixed' or 'massage'\n");
        }
    }	else if (strcmp(argv[1], "uv_on") == 0) {											// 紫外灯开启
			user_action_send_cmd(ACTION_UV_LIGHT_ON, 0);
			rt_kprintf("[CMD] UV light ON command sent\n");
		}else if (strcmp(argv[1], "uv_off") == 0) {											// 紫外灯关闭
				user_action_send_cmd(ACTION_UV_LIGHT_OFF, 0);
				rt_kprintf("[CMD] UV light OFF command sent\n");
		}else if (strcmp(argv[1], "ir_on") == 0) {											// 红外灯开启
				user_action_send_cmd(ACTION_IR_LIGHT_ON, 0);
				rt_kprintf("[CMD] IR light ON command sent\n");
		}else if (strcmp(argv[1], "ir_off") == 0) {											// 红外灯关闭
				user_action_send_cmd(ACTION_IR_LIGHT_OFF, 0);
				rt_kprintf("[CMD] IR light OFF command sent\n");
		}else if (strcmp(argv[1], "self_clean") == 0) {									// 自清洁
				user_action_send_cmd(ACTION_SELF_CLEAN, 0);
				rt_kprintf("[CMD] Self clean command sent\n");
		}else if (strcmp(argv[1], "pipe_pos") == 0) {										// 管路分配器位置设置0 780 1480 1750
				if (argc < 3) {
						rt_kprintf("Usage: toilet pipe_pos <position> (0-%d)\n", PIPE_POS_MAX);
						return;
				}
				int pos = atoi(argv[2]);
				if (pos < 0 || pos > PIPE_POS_MAX) {
						rt_kprintf("Invalid position, must be 0-%d\n", PIPE_POS_MAX);
						return;
				}
				wc_pipe_distributor_set_position((rt_uint16_t)pos);
				rt_kprintf("[CMD] Pipe distributor set to %d\n", pos);
		}
		else if (strcmp(argv[1], "status") == 0) {
				if (argc == 2) {
						// 无参数，打印一次状态
						print_toilet_status();
				} else if (argc == 3 && strcmp(argv[2], "on") == 0) {
						// 开启定时刷新
						if (s_status_timer == RT_NULL) {
								s_status_timer = rt_timer_create("sts_tmr",
																								 status_timer_callback,
																								 RT_NULL,
																								 RT_TICK_PER_SECOND * 2,
																								 RT_TIMER_FLAG_PERIODIC);
								if (s_status_timer == RT_NULL) {
										rt_kprintf("Failed to create status timer\n");
										return;
								}
						}
						if (!s_status_timer_running) {
								rt_timer_start(s_status_timer);
								s_status_timer_running = RT_TRUE;
								rt_kprintf("Status auto-refresh started (interval 2s)\n");
						} else {
								rt_kprintf("Status auto-refresh already running\n");
						}
				} else if (argc == 3 && strcmp(argv[2], "off") == 0) {
						// 关闭定时刷新
						if (s_status_timer_running) {
								rt_timer_stop(s_status_timer);
								s_status_timer_running = RT_FALSE;
								rt_kprintf("Status auto-refresh stopped\n");
						} else {
								rt_kprintf("Status auto-refresh not running\n");
						}
				} else {
						rt_kprintf("Usage: toilet status [on|off]\n");
				}
		}else {
        rt_kprintf("Unknown command: %s\n", argv[1]);
    }
}

MSH_CMD_EXPORT(toilet_cmd, toilet control commands);
#endif /* RT_USING_MSH */

