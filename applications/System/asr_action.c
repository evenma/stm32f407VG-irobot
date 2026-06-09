/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 语音识别动作线程
 *
 * 功能：
 *   1. 初始化 WonderEcho Pro 语音模块
 *   2. 轮询识别结果，根据命令词 ID 执行对应动作或播报
 *   3. 可主动播报系统状态（如启动完成、低电量等）
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "asr_wonderecho.h"
#include "global_conf.h"
#include "car_action.h"
#include "user_action.h"
#include "monitor.h"
#include "led.h"
#include "buzzer.h"
#include "uart_packet.h"
#include "packet_reports.h"
#include "zltech_can_motor.h"
#include "math.h"
#include "ir_receiver.h"

#ifdef ULTRASONIC_GPIO
    #include "ultrasonic_hc_sr04.h"
#elif defined(ULTRASONIC_485)
    #include "ultrasonic_485.h"
#endif


/* 线程栈大小和优先级 */
#define ASR_ACTION_THREAD_STACK_SIZE   2048
#define ASR_ACTION_THREAD_PRIORITY     10      /* 中等优先级 */
#define ASR_ACTION_THREAD_TICK         100     /* 轮询间隔 100ms */

/* 单机模式标志（全局，可被其他模块修改）*/
static rt_bool_t g_single_mode = RT_FALSE;

/* 安全移动参数 */
#define SAFE_DISTANCE_MM       100     /* 前方障碍物小于10cm时停止 */
#define MOVE_DURATION_MS       5000     /* 单次移动动作持续时间（ms）*/

#define SAFT_SPEED   30   /* 单机模式下安全移动的速度设定*/

#define VOICE_I2C_BUS_NAME   "hwi2c2"

static rt_bool_t sewage_confirm_pending = RT_FALSE;
static rt_tick_t s_last_confirm_tick = 0;

/* 旋转参数 */
#define PULSES_PER_REV           16384   // 电机一圈脉冲数
#define WHEEL_DIAMETER_MM        173     // 车轮直径（mm）
#define WHEEL_BASE_MM            500     // 轴距（mm）
//#define TURN_90_PULSES           (int32_t)((3.14159265f * (WHEEL_BASE_MM / 2.0f) / 2.0f) / (3.14159265f * WHEEL_DIAMETER_MM) * PULSES_PER_REV)
#define TURN_90_PULSES			(WHEEL_BASE_MM * PULSES_PER_REV) / (4 * WHEEL_DIAMETER_MM)
// 简化计算：TURN_90_PULSES = (WHEEL_BASE_MM * PULSES_PER_REV) / (4 * WHEEL_DIAMETER_MM) ≈ 11841
#define TURN_TIMEOUT_MS          5000    // 旋转超时时间（ms）

typedef enum {
    MOVE_DIR_FORWARD,   // 前进（检测前方悬崖 + 前方超声波）
    MOVE_DIR_BACKWARD,  // 后退（检测后方悬崖 + 后方超声波）
    MOVE_DIR_TURN,      // 旋转（检测前后悬崖 + 全部超声波）
} MoveDirection_t;

#define BASE_SPEED      10      // 基础直行速度 (rpm)
#define MAX_SPEED       15      // 最大速度限制

/* 线程入口函数声明 */
static void asr_action_thread_entry(void *parameter);

/* 内部函数 */
static void robot_simple_move_forward(void);
static void robot_simple_move_backward(void);
static void robot_simple_turn_left(void);
static void robot_simple_turn_right(void);
static void robot_simple_stop(void);


/* ========== 主动播报接口（供其他模块调用）========== */
void asr_action_speak_state(uint8_t talk_id)
{
    asr_speak(ASR_TYPE_ANNOUNCER, talk_id);
}
/* ========== 设置单机模式 ========== */
void asr_action_set_single_mode(rt_bool_t enable)
{
    g_single_mode = enable;
    rt_kprintf("[ASR] Single mode: %s\n", enable ? "ON" : "OFF");
}

rt_bool_t asr_action_is_single_mode(void)
{
    return g_single_mode;
}

/* ========== 内部函数 ========== */
static void alert_once(rt_bool_t condition, uint8_t talk_id)
{
    static rt_tick_t last_tick = 0;
    static rt_bool_t last_state = RT_FALSE;
    rt_tick_t now = rt_tick_get_millisecond();
    if (condition) {
        if (!last_state || (now - last_tick) >= 5000) {
            asr_speak(ASR_TYPE_ANNOUNCER, talk_id);
            last_tick = now;
        }
        last_state = RT_TRUE;
    } else {
        last_state = RT_FALSE;
    }
}

/* 检查左右接收管的上管是否都有效 */
static rt_bool_t is_ir_valid(void)
{
    uint8_t left = ir_get_left_status();
    uint8_t right = ir_get_right_status();
    return ((left & 0x04) && (right & 0x04));  // bit2 为 1 表示上管收到
}

static void calculate_charge_speed(int16_t *left_speed, int16_t *right_speed)
{
    uint8_t left = ir_get_left_status();
    uint8_t right = ir_get_right_status();
    
    // 默认直行速度
    *left_speed = BASE_SPEED;
    *right_speed = BASE_SPEED;
    
    // 提取左右管状态（bit1:左管, bit0:右管）
    uint8_t left_hor = left & 0x03;
    uint8_t right_hor = right & 0x03;
    
    // 偏移方向与程度判断
    int8_t error = 0;  // 正值表示偏右（需左转），负值表示偏左（需右转）
    
    if (left_hor == 0x03 && right_hor == 0x03) {
        // 完全对准，不调整
        error = 0;
    } 
    else if (left_hor == 0x03 && right_hor == 0x02) {
        // 左全收，右只收到左管（偏右轻微）
        error = 2;
    }
    else if (left_hor == 0x03 && right_hor == 0x00) {
        // 左全收，右无（偏右严重）
        error = 5;
    }
    else if (left_hor == 0x02 && right_hor == 0x03) {
        // 左只收到左管，右全收（偏左轻微）
        error = -2;
    }
    else if (left_hor == 0x00 && right_hor == 0x03) {
        // 左无，右全收（偏左严重）
        error = -5;
    }
    else if (left_hor == 0x01 || right_hor == 0x01) {
        // 有右管信号（即外侧管）属于较大偏移，可单独处理
        if (left_hor == 0x01 && right_hor == 0x03) error = -3;  // 左仅右管，右全收
        else if (left_hor == 0x03 && right_hor == 0x01) error = 3;
        else if (left_hor == 0x01 && right_hor == 0x01) error = 0; // 两外侧，可能平行
        else error = 0;
    }
    else {
        error = 0;
    }
    
    // 根据误差调整速度差
    if (error > 0) {  // 偏右，需要左转：左轮慢，右轮快
        *left_speed = BASE_SPEED - error;
        *right_speed = BASE_SPEED + error;
    } else if (error < 0) {  // 偏左，需要右转：左轮快，右轮慢
        *left_speed = BASE_SPEED + (-error);
        *right_speed = BASE_SPEED - (-error);
    }
    
    // 限幅
    if (*left_speed > MAX_SPEED) *left_speed = MAX_SPEED;
    if (*left_speed < -MAX_SPEED) *left_speed = -MAX_SPEED;
    if (*right_speed > MAX_SPEED) *right_speed = MAX_SPEED;
    if (*right_speed < -MAX_SPEED) *right_speed = -MAX_SPEED;
}

static rt_bool_t is_safe_to_move(MoveDirection_t dir)
{
    uint16_t front_mv, rear_mv;
    rt_bool_t cliff_trigger;
    read_cliff_sensor(&front_mv, &rear_mv, &cliff_trigger);
    
    // 1. 悬崖检测
    if (dir == MOVE_DIR_FORWARD && front_mv < CLIFF_VOLTAGE_THRESHOLD_MV) {
        rt_kprintf("[ASR] Front cliff detected!\n");
        alert_once(RT_TRUE, TALK_DETECT_CLIFF);
        return RT_FALSE;
    }
    if (dir == MOVE_DIR_BACKWARD && rear_mv < CLIFF_VOLTAGE_THRESHOLD_MV) {
        rt_kprintf("[ASR] Rear cliff detected!\n");
        alert_once(RT_TRUE, TALK_DETECT_CLIFF);
        return RT_FALSE;
    }
    if (dir == MOVE_DIR_TURN && cliff_trigger) {
        rt_kprintf("[ASR] Cliff detected (front or rear)!\n");
        alert_once(RT_TRUE, TALK_DETECT_CLIFF);
        return RT_FALSE;
    }
    
    // 2. 超声波检测
    uint32_t dist[8] = {0};
    int sensor_count = 0;
#ifdef ULTRASONIC_GPIO
    hc_sr04_get_distances(dist, 8);
    sensor_count = 8;
#else
    ultrasonic_485_get_distances(dist, 7);
    sensor_count = 7;
#endif
    
    switch (dir) {
        case MOVE_DIR_FORWARD:
            // 检查前左(0)、前右(1)
            if ((dist[0] > 0 && dist[0] < SAFE_DISTANCE_MM) ||
                (dist[1] > 0 && dist[1] < SAFE_DISTANCE_MM)) {
                rt_kprintf("[ASR] Obstacle ahead!\n");
                alert_once(RT_TRUE, TALK_DETECT_DISTANCE);
                return RT_FALSE;
            }
            break;
        case MOVE_DIR_BACKWARD:
            // 检查后左(6)、后右(7)（注意传感器数量可能只有7，需判断）
            if (sensor_count >= 7) {
                if ((dist[6] > 0 && dist[6] < SAFE_DISTANCE_MM) ||
                    (sensor_count == 8 && dist[7] > 0 && dist[7] < SAFE_DISTANCE_MM)) {
                    rt_kprintf("[ASR] Obstacle behind!\n");
                    alert_once(RT_TRUE, TALK_DETECT_DISTANCE);
                    return RT_FALSE;
                }
            }
            break;
        case MOVE_DIR_TURN:
            // 检查所有有效传感器
            for (int i = 0; i < sensor_count; i++) {
                if (dist[i] > 0 && dist[i] < SAFE_DISTANCE_MM) {
                    rt_kprintf("[ASR] Obstacle[%d] = %d mm, stop!\n", i, dist[i]);
                    alert_once(RT_TRUE, TALK_DETECT_DISTANCE);
                    return RT_FALSE;
                }
            }
            break;
    }
		alert_once(RT_FALSE, 0);
    return RT_TRUE;
}


/**
 * @brief 基于电机码盘的原地旋转（相对位置模式）
 * @param angle_deg 旋转角度，正数为左转，负数为右转
 * @return RT_EOK 成功，否则失败
 */
static rt_err_t robot_turn_with_encoder(float angle_deg)
{
    if (!zlac_is_online()) {
        rt_kprintf("[ASR] Motor offline, cannot turn\n");
        return -RT_ERROR;
    }

    // 计算目标脉冲数（按比例）
    int32_t pulses = (int32_t)(TURN_90_PULSES * (fabsf(angle_deg) / 90.0f));
    if (pulses == 0) return RT_EOK;

    int32_t left_pulses, right_pulses;
    if (angle_deg > 0) {        // 左转
        left_pulses = pulses;   // 左轮正向（向前）
        right_pulses = -pulses; // 右轮反向
    } else {                    // 右转
        left_pulses = -pulses;
        right_pulses = pulses;
    }

    rt_kprintf("[ASR] Turning %d deg: L=%ld, R=%ld pulses\n", (int)angle_deg, left_pulses, right_pulses);

    /* 1. 切换至相对位置模式（使用 car_action）*/
    car_action_send_cmd(CAR_CMD_POS_MODE_REL, 0);
    rt_thread_mdelay(100);      // 等待模式切换生效

		    /* 2. 临时降低位置模式最大速度 */
    set_position_mode_max_speed(15);
		
    /* 2. 释放抱闸 */
    car_action_send_cmd(CAR_CMD_BRAKE_OFF, 0);
    rt_thread_mdelay(50);

    /* 3. 使能电机 */
    car_action_send_cmd(CAR_CMD_ENABLE, 0);
    rt_thread_mdelay(100);

    /* 4. 设置目标脉冲数并启动运动 */
    car_action_set_position_global(left_pulses, right_pulses);
    car_action_send_cmd(CAR_CMD_SET_POS, 0);

    rt_thread_mdelay(2000);
    /* 5. 等待运动完成或超时 */
    rt_tick_t start = rt_tick_get_millisecond();
    while (1) {
				if(!is_safe_to_move(MOVE_DIR_TURN)){
					  car_action_send_cmd(CAR_CMD_STOP, 0);
					    set_position_mode_max_speed(ZLAC_MOTOR_NORMAL_RPM);   // 55 rpm
						return -RT_ERROR;
				}
        if (zlac_is_left_target_reached() && zlac_is_right_target_reached()) {
            rt_kprintf("[ASR] Turn completed\n");
            break;
        }
        if (rt_tick_get_millisecond() - start > TURN_TIMEOUT_MS) {
            rt_kprintf("[ASR] Turn timeout\n");
            car_action_send_cmd(CAR_CMD_STOP, 0);
					    set_position_mode_max_speed(ZLAC_MOTOR_NORMAL_RPM);   // 55 rpm
            return -RT_ETIMEOUT;
        }
        rt_thread_mdelay(20);
    }
		    /* 6. 恢复最大速度（可选）*/
    set_position_mode_max_speed(ZLAC_MOTOR_NORMAL_RPM);   // 55 rpm

    /* 6. 可选：保持抱闸或自由（根据需求）*/
    // car_action_send_cmd(CAR_CMD_BRAKE_ON, 0);
	car_action_send_cmd(CAR_CMD_STOP, 0);

    return RT_EOK;
}

/**
 * @brief 上报语音识别结果到上位机（JSON 格式）
 * @param cmd_id 命令词 ID
 * @param cmd_str 命令词字符串（可选，若为 NULL 则根据 ID 生成）
 */
static void report_voice_command(uint8_t cmd_id)
{
	    // 单机模式下不上报
    if (g_single_mode) {
        return;
    }
    PacketReportVoiceTypeDef pkt;
    pkt.command_id = cmd_id;

    uart_packet_send(PKT_FUNC_VOICE, &pkt, sizeof(pkt));
}

/* ========== 单机模式下安全移动（带悬崖和超声波检测）========== */
static void safe_move_start(void)
{
    /* 移动前先释放抱闸、使能电机（如果尚未使能）*/
    car_action_send_cmd(CAR_CMD_MOVE_START, 0);
    rt_thread_mdelay(100);
}

static void robot_simple_stop(void)
{
    car_action_send_cmd(CAR_CMD_STOP, 0);
}

static void robot_simple_move_forward(void)
{
    safe_move_start();
	uint8_t cnt = MOVE_DURATION_MS/500;
	for(int i=0;i<cnt;i++){
		if (!is_safe_to_move(MOVE_DIR_FORWARD)) {
			robot_simple_stop();
			return;
		}
		car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(30, 30));  // 速度 30 rpm
		rt_thread_mdelay(500);
	}    
//    rt_thread_mdelay(MOVE_DURATION_MS);

    robot_simple_stop();
}

static void robot_simple_move_backward(void)
{
    safe_move_start();
	uint8_t cnt = MOVE_DURATION_MS/500;
	for(int i=0;i<cnt;i++){
		if (!is_safe_to_move(MOVE_DIR_BACKWARD)) {
			robot_simple_stop();
			return;
		}
//		int16_t left_rpm = (int16_t)(-15);
//    int16_t right_rpm = (int16_t)(-15);
//		uint32_t param = CAR_PACK_VEL(left_rpm, right_rpm);  // 左轮在高16位，右轮在低16位	
//		rt_kprintf("[asr] speed left=%d,right=%d,parm=%d\n",left_rpm,right_rpm,param);
    car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(-15, -15));	

		rt_thread_mdelay(500);
	}  
    robot_simple_stop();
}

static void robot_simple_turn_left(void)
{
		// car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(-15, 15));
		// rt_thread_mdelay(780); // 假设轴距0.5m ,轮子一圈0.5m ，旋转90° 采用imu读取实现
		// car_action_send_cmd(CAR_CMD_STOP, 0);
	
    if (robot_turn_with_encoder(90.0f)) {
        rt_kprintf("[ASR] Left turn failed\n");
       asr_speak(ASR_TYPE_ANNOUNCER, TALK_RUN_ERR);
    }	
	
}

static void robot_simple_turn_right(void)
{
		// car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(15, -15));
		// rt_thread_mdelay(780); // 假设轴距0.5m ,轮子一圈0.5m ，旋转90° 采用imu读取实现
		// car_action_send_cmd(CAR_CMD_STOP, 0);
	
    if (robot_turn_with_encoder(-90.0f)) {
        rt_kprintf("[ASR] Right turn failed\n");
        asr_speak(ASR_TYPE_ANNOUNCER, TALK_RUN_ERR);
    }	
}

static void robot_auto_charge(void)
{
    rt_kprintf("[ASR] Auto charge start\n");
    
    /* 1. 释放抱闸，使能电机，进入速度模式 */
    car_action_send_cmd(CAR_CMD_MOVE_START, 0);
    rt_thread_mdelay(100);
    
    rt_tick_t start_tick = rt_tick_get_millisecond();
    rt_bool_t aligned = RT_FALSE;
    rt_bool_t timeout = RT_FALSE;
    rt_tick_t aligned_start = 0;
    
    while (1) {
        /* 超时检测（60秒）*/
        if (rt_tick_get_millisecond() - start_tick > 60000) {
            rt_kprintf("[ASR] Auto charge timeout\n");
            timeout = RT_TRUE;
            break;
        }
        
        /* 安全检测（前方障碍物）*/
        if (!is_safe_to_move(MOVE_DIR_FORWARD)) {
            car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(0, 0));
            rt_thread_mdelay(200);
            continue;
        }

        /* 优先检测充电电压（物理接触）*/
        if (monitor_is_charging() || monitor_get_charger_voltage() > 5000) {
            rt_kprintf("[ASR] Charging voltage detected, docking successful\n");
            aligned = RT_TRUE;
            break;
        }				
        
        /* 红外数据有效性检查 */
        if (!is_ir_valid()) {
            rt_kprintf("[ASR] IR invalid (no upper beam), stop and retry\n");
            car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(0, 0));
            rt_thread_mdelay(200);
            aligned_start = 0;
            continue;
        }
        
        /* 获取左右接收管水平状态（低2位）*/
        uint8_t left_hor = ir_get_left_status() & 0x03;
        uint8_t right_hor = ir_get_right_status() & 0x03;
        
        /* 判断是否完全对准（左右都收到两个水平管）*/
        if (left_hor == 0x03 && right_hor == 0x03) {
            if (aligned_start == 0) {
                aligned_start = rt_tick_get_millisecond();
            }
            if (rt_tick_get_millisecond() - aligned_start >= 2000) {
                rt_kprintf("[ASR] Aligned for 2 seconds, waiting for voltage\n");
                // 继续以低速度前进，同时保持检测充电电压
                car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(5, 5));
                rt_thread_mdelay(100);
                continue;  // 回到循环顶部检测电压
            }
            // 未达到持续时间，继续直行
            car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(BASE_SPEED, BASE_SPEED));
            rt_thread_mdelay(50);
            continue;
        } else {
            aligned_start = 0;
        }
        
        /* 根据偏移计算左右速度 */
        int16_t left_spd, right_spd;
        calculate_charge_speed(&left_spd, &right_spd);
        
        rt_kprintf("[ASR] IR: L=0x%02X R=0x%02X -> speed L=%d R=%d\n",
                   ir_get_left_status(), ir_get_right_status(), left_spd, right_spd);
        
        car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(left_spd, right_spd));
        rt_thread_mdelay(100);
    }
    
    /* 停止运动 */
    car_action_send_cmd(CAR_CMD_STOP, 0);
    
    /* 判断结果 */
    if (aligned) {
        rt_kprintf("[ASR] Docking successful, starting charging\n");
        asr_speak(ASR_TYPE_ANNOUNCER, TALK_CHARGE_START);
        car_action_send_cmd(CAR_CMD_CHARGER_ON, 0);
    } else if (timeout) {
        rt_kprintf("[ASR] Auto charge timeout\n");
        asr_speak(ASR_TYPE_ANNOUNCER, TALK_CANNOT_CHARGE);
    } else {
        rt_kprintf("[ASR] Auto charge failed\n");
        asr_speak(ASR_TYPE_ANNOUNCER, TALK_CANNOT_CHARGE);
    }
}

/* ========== 命令执行分发 ========== */
static void execute_action(uint8_t cmd_id)
{
	    // 上报识别结果
    report_voice_command(cmd_id);
	
    switch (cmd_id) {
 
    /* ----- 小车运动（需判断模式）----- */		
        case CMD_FORWARD:
            if (g_single_mode) {
                robot_simple_move_forward();
            } 
            break;
        case CMD_BACKWARD:
            if (g_single_mode) {
                robot_simple_move_backward();
            } 
            break;
        case CMD_LEFT:
            if (g_single_mode) {
                robot_simple_turn_left();
            } 
            break;
        case CMD_RIGHT:
            if (g_single_mode) {
                robot_simple_turn_right();
            } 
            break;
        case CMD_STOP:
		    robot_simple_stop();   // 如果电机在运行 
			user_action_stop();   //如果马桶功能在使用
            break;
        case CMD_SPEED_UP:
            /* 速度调节，单机模式不支持速度调节*/
				    if (g_single_mode) {
							rt_thread_mdelay(500);
                asr_speak(ASR_TYPE_ANNOUNCER, TALK_ACC_ERR);
								rt_kprintf("[ASR] Not suport\n");
            } 										
            break;
        case CMD_SPEED_DOWN:
						if (g_single_mode) {
								rt_thread_mdelay(500);
                asr_speak(ASR_TYPE_ANNOUNCER, TALK_ACC_ERR);
								rt_kprintf("[ASR] Not suport\n");
            } 	
            break;
		/* 上位机自动导航到指定点 */	
        case CMD_BACK_CHARGE:
					 if (g_single_mode) {
							robot_auto_charge();					 
					 }
            break;
        case CMD_GOTO_BEDROOM:
            break;
        case CMD_GOTO_TOILET:
            break;	
		case CMD_FREE_ON:
            if (g_single_mode) {
				car_action_send_cmd(CAR_CMD_BRAKE_OFF, 0);
				
                car_action_send_cmd(CAR_CMD_FREE, 0);				
            } 
            break;	
		case CMD_FREE_OFF:
            if (g_single_mode) {
                car_action_send_cmd(CAR_CMD_BRAKE_ON, 0);
            } 
            break;				

        /* ----- 小车其他功能 ----- */	
        case CMD_LIGHT_ON:
		case CMD_CAR_LIGHT_ON:
            // 开灯
            car_action_send_cmd(CAR_CMD_LIGHT_ON, 0);
            rt_kprintf("[LIGHT] ON\n");
            break;
        case CMD_LIGHT_OFF:
		case CMD_CAR_LIGHT_OFF:
            // 关灯
            car_action_send_cmd(CAR_CMD_LIGHT_OFF, 0);
            rt_kprintf("[LIGHT] OFF\n");
            break;			
			
        case CMD_CHARGE_ON:
            car_action_send_cmd(CAR_CMD_CHARGER_ON, 0);
            rt_kprintf("[CHARGER] ON\n");
            break;
        case CMD_CHARGE_OFF:
            car_action_send_cmd(CAR_CMD_CHARGER_OFF, 0);
            rt_kprintf("[CHARGER] OFF\n");
            break;			

        /* ----- 智能马桶功能（直接执行）----- */
        case CMD_FLUSH:
            user_action_send_cmd(ACTION_FLUSH_TOILET, 0);
            break;
        case CMD_CLEAN_BUTT:
            user_action_send_cmd(ACTION_CLEAN_REAR, 0);
            break;
        case CMD_FEMALE_CLEAN:
            user_action_send_cmd(ACTION_CLEAN_FEMALE, 0);
            break;
        case CMD_DRY:
            user_action_send_cmd(ACTION_DRY, 0);
            break;
				case CMD_CLEAN:
						user_action_set_clean_mode(CLEAN_MODE_MASSAGE);
						user_action_send_cmd(ACTION_CLEAN_REAR, 0);
						break;
			 case CMD_MODE_MASSAGE:
						user_action_set_clean_mode(CLEAN_MODE_MASSAGE);
						break;
			 case CMD_MODE_FIXED:
						user_action_set_clean_mode(CLEAN_MODE_FIXED);
						break;
        case CMD_SEWAGE_PUMP:
		// 排污泵的开启，需要二次确认，收到确认信号才能开启
 //           user_action_send_cmd(ACTION_SEWAGE_PUMP, 0);
			sewage_confirm_pending = RT_TRUE;
			s_last_confirm_tick = rt_tick_get();
            break;
		case CMD_CONFIRM:
			{
					rt_tick_t now = rt_tick_get();
					if (sewage_confirm_pending && (now - s_last_confirm_tick) < RT_TICK_PER_SECOND * 10) {
							user_action_send_cmd(ACTION_SEWAGE_PUMP, 0);
							sewage_confirm_pending = RT_FALSE;
					} else {
							sewage_confirm_pending = RT_FALSE;
					}
					break;
			}
			break;			
        case CMD_SELF_CLEAN:
            user_action_send_cmd(ACTION_SELF_CLEAN, 0);
            break;
        case CMD_SEAT_OPEN:
            user_action_send_cmd(ACTION_SEAT_OPEN, 0);
            rt_kprintf("[TOILET] Seat open\n");
            break;
        case CMD_SEAT_CLOSE:
            user_action_send_cmd(ACTION_SEAT_CLOSE, 0);
            rt_kprintf("[TOILET] Seat close\n");
            break;
        case CMD_LID_OPEN:
            user_action_send_cmd(ACTION_LID_OPEN, 0);
            rt_kprintf("[TOILET] Lid open\n");
            break;
        case CMD_LID_CLOSE:
            user_action_send_cmd(ACTION_LID_CLOSE, 0);
            rt_kprintf("[TOILET] Lid close\n");
            break;
        case CMD_UV_LIGHT_ON:
            user_action_send_cmd(ACTION_UV_LIGHT_ON, 0);
            rt_kprintf("[TOILET] UV on\n");
            break;
        case CMD_UV_LIGHT_OFF:
            user_action_send_cmd(ACTION_UV_LIGHT_OFF, 0);
            rt_kprintf("[TOILET] UV off\n");
            break;
        case CMD_IR_LIGHT_ON:
            user_action_send_cmd(ACTION_IR_LIGHT_ON, 0);
            rt_kprintf("[TOILET] IR on\n");
            break;
        case CMD_IR_LIGHT_OFF:
            user_action_send_cmd(ACTION_IR_LIGHT_OFF, 0);
            rt_kprintf("[TOILET] IR off\n");
            break;


        /* ----- 单机模式切换（隐藏命令）----- */
        case CMD_SINGLE_MODE:
            asr_action_set_single_mode(RT_TRUE);
            break;
        case CMD_NORMAL_MODE:
            asr_action_set_single_mode(RT_FALSE);
            break;

        default:
            rt_kprintf("[ASR] Unhandled cmd ID: 0x%02X\n", cmd_id);
            break;
    }
}

/* 线程入口函数 */
static void asr_action_thread_entry(void *parameter)
{
    /* 1. 初始化语音模块 */
    if (asr_wonderecho_init(VOICE_I2C_BUS_NAME) != RT_EOK)
    {
        rt_kprintf("ASR action: voice module init failed!\n");
        return;
    }

    /* 2. 启动成功，播报欢迎语（模块已包含欢迎语，此处可选额外播报）*/
    asr_speak(ASR_TYPE_ANNOUNCER, TALK_WELCOME);

    /* 3. 主循环：轮询识别结果 */
    while (1)
    {
        uint8_t cmd_id = asr_get_result();
        if (cmd_id != 0)
        {
            rt_kprintf("ASR recognized: 0x%02X\n", cmd_id);
            execute_action(cmd_id);
        }
        rt_thread_mdelay(ASR_ACTION_THREAD_TICK);
    }
}
 
int asr_action_init(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("asr_act",
                           asr_action_thread_entry,
                           RT_NULL,
                           ASR_ACTION_THREAD_STACK_SIZE,
                           ASR_ACTION_THREAD_PRIORITY,
                           10);
    if (tid == RT_NULL)
    {
        rt_kprintf("Failed to create ASR action thread\n");
        return -RT_ERROR;
    }
    rt_thread_startup(tid);
    return RT_EOK;
}


