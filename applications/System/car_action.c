/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 小车运动控制实现
 */

#include "car_action.h"
#include "zltech_can_motor.h"
#include "global_conf.h"
#include "monitor.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

/* 线程配置 */
#define CAR_THREAD_STACK_SIZE  1024
#define CAR_THREAD_PRIORITY    16
#define CAR_SAFETY_CHECK_MS    50      /* 安全检测周期（ms）*/

/* 辅助宏：将两个16位值组合成32位参数 */
#define CAR_PACK_PARAM(low16, high16)  (((uint32_t)(high16) << 16) | ((uint32_t)(low16) & 0xFFFF))

/* 内部变量 */
static struct rt_mailbox s_car_mb;
static rt_uint32_t s_car_mb_pool[8];
static rt_thread_t s_car_thread;

static int32_t s_setpos_left = 0;
static int32_t s_setpos_right = 0;
static struct rt_mutex s_setpos_mutex;

/* 运行时状态（用于安全检测）*/
static volatile rt_bool_t s_emergency_stop = RT_FALSE;
// 速度指令超时保护配置
#define SAFETY_VEL_TIMEOUT_MS       1000   // 无速度指令后零速等待时间 (ms)
#define SAFETY_VEL_FREE_TIMEOUT_MS  10000  // 完全停车并抱闸超时 (ms)

static uint32_t s_last_vel_cmd_tick = 0;
static rt_bool_t s_vel_in_safe_stop = RT_FALSE;  // 是否已执行零速锁定
static rt_bool_t s_vel_in_free_stop = RT_FALSE;  // 是否已执行完全停车

/* 前向声明 */
static void safety_check(void);
static void process_cmd(CarCmd_t cmd, uint32_t param);

rt_bool_t g_ir_alignment_enable = RT_FALSE;    // 电机运动时开启充电座红外接收管检测

static uint32_t s_cmd_vel_param = 0;


/* ==================== 辅助设备控制 ==================== */

static void car_light_on(void)
{
    rt_pin_write(HEADLIGHT_LEFT_PIN, PIN_HIGH);
    rt_pin_write(HEADLIGHT_RIGHT_PIN, PIN_HIGH);
    rt_kprintf("[CAR] Headlights ON\n");
}

static void car_light_off(void)
{
    rt_pin_write(HEADLIGHT_LEFT_PIN, PIN_LOW);
    rt_pin_write(HEADLIGHT_RIGHT_PIN, PIN_LOW);
    rt_kprintf("[CAR] Headlights OFF\n");
}

static void car_charger_on(void)
{
    rt_pin_write(CHARGER_CONTROL_PIN, PIN_HIGH);
    rt_kprintf("[CAR] Charger enabled\n");		
}

static void car_charger_off(void)
{
    rt_pin_write(CHARGER_CONTROL_PIN, PIN_LOW);
    rt_kprintf("[CAR] Charger disabled\n");
}

/* ==================== 安全检测 ==================== */
static void reset_safety_monitor(void)
{
    s_last_vel_cmd_tick = 0;
    s_vel_in_safe_stop = RT_FALSE;
    s_vel_in_free_stop = RT_FALSE;
}
/* 悬崖传感器检测（前、后）*/
static rt_bool_t is_cliff_detected(void)
{
    rt_adc_device_t adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL) {
        return RT_FALSE;
    }
    /* 读取前悬崖电压 */
    rt_adc_enable(adc_dev, CLIFF_FRONT_ADC);
    rt_uint32_t raw_front = rt_adc_read(adc_dev, CLIFF_FRONT_ADC);
    rt_adc_disable(adc_dev, CLIFF_FRONT_ADC);
    int front_mv = (int)(raw_front * 3300 / 4095);  /* 粗略计算，实际可能需校准 */

    /* 读取后悬崖电压 */
    rt_adc_enable(adc_dev, CLIFF_REAR_ADC);
    rt_uint32_t raw_rear = rt_adc_read(adc_dev, CLIFF_REAR_ADC);
    rt_adc_disable(adc_dev, CLIFF_REAR_ADC);
    int rear_mv = (int)(raw_rear * 3300 / 4095);

    if (front_mv < CLIFF_VOLTAGE_THRESHOLD_MV || rear_mv < CLIFF_VOLTAGE_THRESHOLD_MV) {
        rt_kprintf("[CAR] Cliff detected! Front=%dmV, Rear=%dmV\n", front_mv, rear_mv);
        return RT_TRUE;
    }
    return RT_FALSE;
}

/* 低电量检测 */
static rt_bool_t is_low_battery(void)
{
    uint32_t bat_mv = monitor_get_battery_voltage();
    if (bat_mv < BATTERY_LOW_VOLTAGE_MV) {
        rt_kprintf("[CAR] Low battery: %d mV\n", bat_mv);
        return RT_TRUE;
    }
    return RT_FALSE;
}

/* 安全停车（由安全检测触发）*/
static void safety_stop(void)
{
    if (!s_emergency_stop) {
        s_emergency_stop = RT_TRUE;
        rt_kprintf("[CAR] Safety stop triggered!\n");
        zlac_control_free();      // 完全解轴
        zlac_brake_engage();      // 锁紧抱闸
        car_light_off();          // 可选：关闭大灯       
    }
}

/* 定期安全检测（在线程循环中调用）*/
static void safety_check(void)
{
    /* 如果已经紧急停车，不再重复检测（恢复需外部命令）*/
    if (s_emergency_stop) {
        return;
    }
    if (is_cliff_detected() || is_low_battery()) {
//        safety_stop();
			 // 可向上位机发送报警信息（通过串口等）
    }
}

/* ==================== 命令处理 ==================== */

static void process_cmd(CarCmd_t cmd, uint32_t param)
{
    rt_err_t ret;

    switch (cmd) {
				case  CAR_CMD_CLEAR_FAULT:
					zlac_clear_fault();
					break;
        case CAR_CMD_VEL_MODE:
            ret = zlac_init_velocity_mode();
            if (ret == RT_EOK) {
								reset_safety_monitor();   // 重置安全监控
                rt_kprintf("[CAR] Switched to velocity mode\n");
            } else {
                rt_kprintf("[CAR] Failed to switch to velocity mode (err=%d)\n", ret);
            }
            break;

        case CAR_CMD_POS_MODE_ABS:
            ret = zlac_init_position_mode(ZLAC_POS_MODE_ABSOLUTE);
            if (ret == RT_EOK) {
                rt_kprintf("[CAR] Switched to absolute position mode\n");
            } else {
                rt_kprintf("[CAR] Failed to switch to absolute position mode (err=%d)\n", ret);
            }
            break;

				case CAR_CMD_ZERO_POS:
            /* 设置当前位置为零点（绝对位置模式原点）*/
            ret = zlac_set_position_abs_home();
            if (ret == RT_EOK) {
                rt_kprintf("[CAR] Home position set\n");
            } else {
                rt_kprintf("[CAR] Set home failed (err=%d)\n", ret);
            }
            break;
						
        case CAR_CMD_POS_MODE_REL:
            ret = zlac_init_position_mode(ZLAC_POS_MODE_RELATIVE);
            if (ret == RT_EOK) {
                rt_kprintf("[CAR] Switched to relative position mode\n");
            } else {
                rt_kprintf("[CAR] Failed to switch to relative position mode (err=%d)\n", ret);
            }
            break;

        case CAR_CMD_SET_VEL:
            {
							// zlac_set_velocity(0, 0);  //下发零速 保存力矩的停，停并且带阻尼或者锁定位姿的停							
//                int16_t left = (int16_t)(param >> 16);
//                int16_t right = (int16_t)(param & 0xFFFF);
							    int16_t left = (int16_t)(s_cmd_vel_param >> 16);
									int16_t right = (int16_t)(s_cmd_vel_param & 0xFFFF);
//							rt_kprintf("[car]speed left=%d,right=%d,parm=%d\n",left,right,param);
                ret = zlac_set_velocity(left, right);
                if (ret != RT_EOK) {
                    rt_kprintf("[CAR] Set velocity failed (err=%d)\n", ret);
                }else{
									// 记录最后收到速度指令的时刻，并清除安全停止标志
									s_last_vel_cmd_tick = rt_tick_get_millisecond();
									s_vel_in_safe_stop = RT_FALSE;
									s_vel_in_free_stop = RT_FALSE;							
								}
            }
            break;

        case CAR_CMD_SET_POS:
            {
							int32_t left, right;
							rt_mutex_take(&s_setpos_mutex, RT_WAITING_FOREVER);
							left = s_setpos_left;
							right = s_setpos_right;
							rt_mutex_release(&s_setpos_mutex);
							ret = zlac_set_position_by_mode(left, right);
							if (ret == RT_EOK) {
									rt_kprintf("[CAR] Set position L=%ld R=%ld pulses\n", left, right);
							} else {
									rt_kprintf("[CAR] Set position failed (err=%d)\n", ret);
							}
            }
            break;

        case CAR_CMD_ENABLE:
            ret = zlac_control_enable();
            if (ret == RT_EOK) {
								reset_safety_monitor();
                rt_kprintf("[CAR] Motor enabled\n");
            } else {
                rt_kprintf("[CAR] Enable failed (err=%d)\n", ret);
            }
            break;

        case CAR_CMD_DISABLE:
            ret = zlac_control_disable();
            if (ret == RT_EOK) {
                rt_kprintf("[CAR] Motor disabled\n");
            } else {
                rt_kprintf("[CAR] Disable failed (err=%d)\n", ret);
            }
            break;

        case CAR_CMD_FREE:
            ret = zlac_control_free();
            rt_kprintf("[CAR] Motor free (no torque), ret=%d\n", ret);
            break;

        case CAR_CMD_BRAKE_ON:
						g_ir_alignment_enable = RT_FALSE;  // 停止运动，关闭检测
            ret = zlac_brake_engage();
            rt_kprintf("[CAR] Brake engaged, ret=%d\n", ret);
            break;

        case CAR_CMD_BRAKE_OFF:
						g_ir_alignment_enable = RT_TRUE;   // 开始运动，启用红外对准检测
            ret = zlac_brake_release();
            rt_kprintf("[CAR] Brake released, ret=%d\n", ret);
            break;


				case CAR_CMD_QUICK_STOP:
					// 发送急停命令 急停后需要重新使能电机 zlac_control_enable--> zlac_set_velocity(0, 0)待机
					  zlac_control_quickstop();
            rt_kprintf("[CAR] Quick Stop command executed\n");
            break;

// 以下是组合命令				
        case CAR_CMD_STOP:  // 超过10s的停，彻底停。驻车模式 不发热
					  zlac_set_velocity(0, 0);  //运动的停，下发零速 保存力矩的停，停并且带阻尼或者锁定位姿的停 零速锁定（有保持力矩）电机会发热					
					//	zlac_control_quickstop();
						// 等待速度降到接近零
						int16_t left = 0, right = 0;
						uint32_t start = rt_tick_get_millisecond();
						while (rt_tick_get_millisecond() - start < 3000) {  // 最长等待 3 秒
								zlac_get_velocity(&left, &right);
								if (abs(left) < 50 && abs(right) < 50) break;  // 5 rpm 以下
								rt_thread_mdelay(20);
						}
            /* 直接停止并抱闸 */
            zlac_control_free();
            zlac_brake_engage();
						g_ir_alignment_enable = RT_FALSE;  // 停止运动，关闭检测
            rt_kprintf("[CAR]WORK STOP! Normal Stop command executed\n");
            break;					

				case CAR_CMD_MOVE_START:
					    g_ir_alignment_enable = RT_TRUE;   // 开始运动，启用红外对准检测
						ret = zlac_brake_release();
						if(ret != RT_EOK){
							rt_kprintf("[CAR] Failed to brake release (err=%d)\n", ret);
								// 返回错误应答
								break;
						} 
						ret = zlac_init_velocity_mode();
            if (ret != RT_EOK) {
                rt_kprintf("[CAR] Failed to switch to velocity mode (err=%d)\n", ret);
							// 返回错误应答
								break;
            }
						rt_thread_mdelay(100); // 释放抱闸时不能马上开始运动
						// 返回正确应答
						ret = zlac_set_velocity(0, 0);  //下发零速
						if (ret != RT_EOK) {
								rt_kprintf("[CAR] Set velocity failed (err=%d)\n", ret);
						}	
						reset_safety_monitor();   // 运动启动准备完成，重置监控						
						break;
				
//其他				
        case CAR_CMD_LIGHT_ON:
            car_light_on();
            break;

        case CAR_CMD_LIGHT_OFF:
            car_light_off();
            break;

        case CAR_CMD_CHARGER_ON:
            car_charger_on();
            break;

        case CAR_CMD_CHARGER_OFF:
            car_charger_off();
            break;

        default:
            rt_kprintf("[CAR] Unknown command: %d\n", cmd);
            break;
    }
}

/* ==================== 主线程 ==================== */

static void car_action_thread_entry(void *param)
{
    rt_ubase_t cmd;
    CarCmd_t action;
    uint32_t param_val;
	  uint32_t now;

    rt_kprintf("[CAR] Thread started\n");

    while (1) {
        /* 等待邮箱消息，超时 100ms 以定期执行安全检查 */
        if (rt_mb_recv(&s_car_mb, &cmd, rt_tick_from_millisecond(100)) == RT_EOK) {
            action = (CarCmd_t)(cmd & 0xFF);
            param_val = (cmd >> 8);
            process_cmd(action, param_val);
        }
        /* 定期安全检测（非阻塞）*/
        //safety_check();

				        // ========== 速度指令超时保护 ==========
        // 仅在速度模式、电机使能且抱闸释放时才检查
        if (zlac_is_velocity_mode_ready() && zlac_is_left_enabled() && zlac_is_right_enabled() && zlac_is_brake_released()) {
            now = rt_tick_get_millisecond();
            // 如果从未收到过速度指令，忽略
            if (s_last_vel_cmd_tick != 0) {
                uint32_t elapsed = now - s_last_vel_cmd_tick;
                if (elapsed >= SAFETY_VEL_FREE_TIMEOUT_MS && !s_vel_in_free_stop) {
                    rt_kprintf("[CAR] Safety: No speed command for %d ms, free and brake!\n", elapsed);
                    zlac_control_free();
                    zlac_brake_engage();
                    s_vel_in_free_stop = RT_TRUE;
                    s_vel_in_safe_stop = RT_TRUE;
                } 
                else if (elapsed >= SAFETY_VEL_TIMEOUT_MS && !s_vel_in_safe_stop) {
                    rt_kprintf("[CAR] Safety: No speed command for %d ms, set speed to 0\n", elapsed);
                    zlac_set_velocity(0, 0);
                    s_vel_in_safe_stop = RT_TRUE;
                }
            }
        } else {
            // 非速度模式或未使能/抱闸状态，重置安全停止标志（以备下次启用时重新监控）
            s_vel_in_safe_stop = RT_FALSE;
            s_vel_in_free_stop = RT_FALSE;
        }
				
    }
}

/* ==================== 公共接口 ==================== */

void car_action_init(void)
{
    /* 初始化邮箱 */
    rt_mb_init(&s_car_mb, "car_mb", s_car_mb_pool,
               sizeof(s_car_mb_pool) / sizeof(rt_uint32_t),
               RT_IPC_FLAG_FIFO);
		
		rt_mutex_init(&s_setpos_mutex, "setpos", RT_IPC_FLAG_PRIO);							 

    /* 配置大灯引脚为输出 */
    rt_pin_mode(HEADLIGHT_LEFT_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(HEADLIGHT_RIGHT_PIN, PIN_MODE_OUTPUT);
    /* 初始关闭大灯 */
    rt_pin_write(HEADLIGHT_LEFT_PIN, PIN_LOW);
    rt_pin_write(HEADLIGHT_RIGHT_PIN, PIN_LOW);

    /* 充电控制引脚初始为低电平（关闭）*/
    rt_pin_mode(CHARGER_CONTROL_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(CHARGER_CONTROL_PIN, PIN_LOW);

    /* 创建线程 */
    s_car_thread = rt_thread_create("car_action",
                                    car_action_thread_entry,
                                    RT_NULL,
                                    CAR_THREAD_STACK_SIZE,
                                    CAR_THREAD_PRIORITY,
                                    5);
    if (s_car_thread != RT_NULL) {
        rt_thread_startup(s_car_thread);
        rt_kprintf("[CAR] Action module initialized\n");
    } else {
        rt_kprintf("[CAR] Failed to create thread\n");
    }
		s_last_vel_cmd_tick = 0;
}

void car_action_set_position_global(int32_t left, int32_t right)
{
    rt_mutex_take(&s_setpos_mutex, RT_WAITING_FOREVER);
    s_setpos_left = left;
    s_setpos_right = right;
    rt_mutex_release(&s_setpos_mutex);
}

void car_action_send_cmd(CarCmd_t cmd, uint32_t param)
{
//    rt_uint32_t msg = (uint32_t)cmd | (param << 8);
//    rt_mb_send(&s_car_mb, msg);
	    // 根据命令类型存储参数
    switch (cmd) {
        case CAR_CMD_SET_VEL:
            s_cmd_vel_param = param;
            break;
        default:
            break;
    }
    rt_mb_send(&s_car_mb, (rt_uint32_t)cmd);
}

/* ==================== MSH 测试命令 ==================== */
#ifdef RT_USING_MSH
#include <stdlib.h>

static void car_cmd(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage:\n");
        rt_kprintf("  car vel <left> <right>           - set velocity (rpm)\n");
        rt_kprintf("  car pos_abs <left> <right>       - absolute position (pulses)\n");
        rt_kprintf("  car pos_rel <left> <right>       - relative position (pulses)\n");
        rt_kprintf("  car mode vel                     - switch to velocity mode\n");
        rt_kprintf("  car mode abs_pos                 - switch to absolute position mode\n");
        rt_kprintf("  car mode rel_pos                 - switch to relative position mode\n");
        rt_kprintf("  car enable                       - enable motor\n");
        rt_kprintf("  car disable                      - disable motor (lock)\n");
        rt_kprintf("  car free                         - free motor\n");
        rt_kprintf("  car brake on/off                 - engage/release brake\n");
        rt_kprintf("  car light on/off                 - headlights on/off\n");
        rt_kprintf("  car charger on/off               - charger on/off\n");
        rt_kprintf("  car zero                         - set current position as home\n");
        rt_kprintf("  car quick_stop                   - suspend pause\n");
        rt_kprintf("  car stop                         - stop\n");			
        return;
    }

    if (rt_strcmp(argv[1], "mode") == 0) {
        if (argc < 3) {
            rt_kprintf("Need mode: vel, abs_pos, rel_pos\n");
            return;
        }
        if (rt_strcmp(argv[2], "vel") == 0) {
            car_action_send_cmd(CAR_CMD_VEL_MODE, 0);
        } else if (rt_strcmp(argv[2], "abs_pos") == 0) {
            car_action_send_cmd(CAR_CMD_POS_MODE_ABS, 0);
        } else if (rt_strcmp(argv[2], "rel_pos") == 0) {
            car_action_send_cmd(CAR_CMD_POS_MODE_REL, 0);
        } else {
            rt_kprintf("Invalid mode\n");
        }
    } else if (rt_strcmp(argv[1], "vel") == 0) {
        if (argc < 4) {
            rt_kprintf("Need left and right rpm\n");
            return;
        }
        int16_t left = atoi(argv[2]);
        int16_t right = atoi(argv[3]);
        uint32_t param = CAR_PACK_VEL(right, left);
        car_action_send_cmd(CAR_CMD_SET_VEL, param);
    } else if (rt_strcmp(argv[1], "pos_abs") == 0) {
        if (argc < 4) {
            rt_kprintf("Need left and right pulses\n");
            return;
        }
        int32_t left_pos = atoi(argv[2]);
        int32_t right_pos = atoi(argv[3]);
				car_action_set_position_global(left_pos, right_pos);
        car_action_send_cmd(CAR_CMD_SET_POS, 0);
    } else if (rt_strcmp(argv[1], "pos_rel") == 0) {
        if (argc < 4) {
            rt_kprintf("Need left and right pulses\n");
            return;
        }
        int32_t left_pos = atoi(argv[2]);
        int32_t right_pos = atoi(argv[3]);
				car_action_set_position_global(left_pos, right_pos);
        car_action_send_cmd(CAR_CMD_SET_POS, 0);
    } else if (rt_strcmp(argv[1], "enable") == 0) {
        car_action_send_cmd(CAR_CMD_ENABLE, 0);
    } else if (rt_strcmp(argv[1], "disable") == 0) {
        car_action_send_cmd(CAR_CMD_DISABLE, 0);
    } else if (rt_strcmp(argv[1], "free") == 0) {
        car_action_send_cmd(CAR_CMD_FREE, 0);
    } else if (rt_strcmp(argv[1], "brake") == 0) {
        if (argc < 3) {
            rt_kprintf("Need on/off\n");
            return;
        }
        if (rt_strcmp(argv[2], "on") == 0) {
            car_action_send_cmd(CAR_CMD_BRAKE_ON, 0);
        } else if (rt_strcmp(argv[2], "off") == 0) {
            car_action_send_cmd(CAR_CMD_BRAKE_OFF, 0);
        } else {
            rt_kprintf("Invalid argument\n");
        }
    } else if (rt_strcmp(argv[1], "light") == 0) {
        if (argc < 3) {
            rt_kprintf("Need on/off\n");
            return;
        }
        if (rt_strcmp(argv[2], "on") == 0) {
            car_action_send_cmd(CAR_CMD_LIGHT_ON, 0);
        } else if (rt_strcmp(argv[2], "off") == 0) {
            car_action_send_cmd(CAR_CMD_LIGHT_OFF, 0);
        } else {
            rt_kprintf("Invalid argument\n");
        }
    } else if (rt_strcmp(argv[1], "charger") == 0) {
        if (argc < 3) {
            rt_kprintf("Need on/off\n");
            return;
        }
        if (rt_strcmp(argv[2], "on") == 0) {
            car_action_send_cmd(CAR_CMD_CHARGER_ON, 0);
        } else if (rt_strcmp(argv[2], "off") == 0) {
            car_action_send_cmd(CAR_CMD_CHARGER_OFF, 0);
        } else {
            rt_kprintf("Invalid argument\n");
        }
    } else if (rt_strcmp(argv[1], "zero") == 0) {
        car_action_send_cmd(CAR_CMD_ZERO_POS, 0);
    } else if (rt_strcmp(argv[1], "quick_stop") == 0) {
        car_action_send_cmd(CAR_CMD_QUICK_STOP, 0);
    } else if (rt_strcmp(argv[1], "stop") == 0) {
        car_action_send_cmd(CAR_CMD_STOP, 0);
    } else if (rt_strcmp(argv[1], "start") == 0) {
        car_action_send_cmd(CAR_CMD_MOVE_START, 0);
    } else {
        rt_kprintf("Unknown command\n");
    }
}
MSH_CMD_EXPORT(car_cmd, "car control commands");
#endif /* RT_USING_MSH */

