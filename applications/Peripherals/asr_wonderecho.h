/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 幻尔语音模块 WonderEcho Pro 驱动头文件
 *
 * 硬件 I2C2: PB10(SCL), PB11(SDA)
 * 设备地址: 0x34 (7-bit)
 */

#ifndef __ASR_WONDERECHO_H__
#define __ASR_WONDERECHO_H__

#include <rtthread.h>
#include <rtdevice.h>

/* ========== 模块信息 ========== */
#define ASR_DEV_ADDR                0x34        /* 7-bit I2C 从机地址 */
#define ASR_RESULT_REG              0x64        /* 识别结果寄存器（读1字节ID） */
#define ASR_SPEAK_REG               0x6E        /* 主动播报寄存器（写2字节：类型+ID） */

/* ========== 播报类型 ========== */
#define ASR_TYPE_CMD_WORD           0x00        /* 命令词播报语（如“正在前进”） */
#define ASR_TYPE_ANNOUNCER          0xFF        /* 普通播报语（如“可回收物”） */

/* ========== 命令词 ID（功能类型 0x00）========== */
/* 基础运动 */
#define CMD_FORWARD                 0x01
#define CMD_BACKWARD                0x02
#define CMD_LEFT                    0x03
#define CMD_RIGHT                   0x04
#define CMD_TILT_FWD                0x05        /* 前倾 */
#define CMD_TILT_BACK               0x06        /* 后仰 */
#define CMD_TILT_LEFT               0x07        /* 左倾 */
#define CMD_TILT_RIGHT              0x08        /* 右倾 */
#define CMD_STOP                    0x09
#define CMD_STAND                   0x0A        /* 立正 */
#define CMD_DOWN                    0x0B        /* 趴下 */
#define CMD_SIT                     0x0C        /* 坐下 */
#define CMD_SPEED_UP                0x0D        /* 加速 */
#define CMD_SPEED_DOWN              0x0E        /* 减速 */
#define CMD_BODY_UP                 0x0F        /* 抬高身体 */
#define CMD_BODY_DOWN               0x10        /* 降低身体 */
#define CMD_BARK                    0x11        /* 叫一声 */

/* 设备控制 */
#define CMD_LIGHT_ON                0x12
#define CMD_LIGHT_OFF               0x13
#define CMD_DOOR_OPEN               0x14
#define CMD_DOOR_CLOSE              0x15
#define CMD_PUMP_ON                 0x16        /* 打开水泵 */
#define CMD_PUMP_OFF                0x17        /* 关闭水泵 */
#define CMD_RACK_EXTEND             0x18        /* 伸出晾衣架 */
#define CMD_RACK_RETRACT            0x19        /* 收回晾衣架 */

/* 交互 */
#define CMD_HELLO                   0x1A
#define CMD_INTRO                   0x1B        /* 介绍自己 */
#define CMD_SHOW_OFF                0x1C        /* 露一手 */
#define CMD_WALK                    0x1D        /* 走两步 */
#define CMD_SHAKE_HEAD              0x1E        /* 摇头 */
#define CMD_LEAP_FWD                0x1F        /* 向前扑 */
#define CMD_LEAP_BACK               0x20        /* 向后扑 */
#define CMD_FIGHT_MODE              0x21
#define CMD_SQUAT                   0x22        /* 蹲下来 */
#define CMD_SHAKE                   0x23        /* 抖一抖 */

/* 家电 */
#define CMD_FAN_ON                  0x24
#define CMD_FAN_OFF                 0x25
#define CMD_SERVO_MOVE              0x26        /* 转动舵机 */
#define CMD_LED_RED                 0x27
#define CMD_LED_GREEN               0x28
#define CMD_LED_BLUE                0x29
#define CMD_DISPLAY_SMILE           0x2A
#define CMD_DISPLAY_CRY             0x2B

/* 视觉玩法（0x62-0x7C）*/
#define CMD_FACE_RECOG              0x62
#define CMD_TAG_RECOG               0x63
#define CMD_GARBAGE_CLASSIFY        0x64
#define CMD_COLOR_RECOG             0x65
#define CMD_LINE_FOLLOW             0x66
#define CMD_COLOR_TRACK             0x67
#define CMD_AVOID                   0x68
#define CMD_SORT_RED                0x69
#define CMD_SORT_GREEN              0x6A
#define CMD_SORT_BLUE               0x6B
#define CMD_DANCE                   0x6C
#define CMD_PUSH_UP                 0x6D
#define CMD_LEFT_KICK               0x6E
#define CMD_RIGHT_KICK              0x6F
#define CMD_TWIST                   0x71
#define CMD_SIT_UP                  0x72
#define CMD_BOW                     0x73
#define CMD_EAGLE                   0x74        /* 大鹏展翅 */
#define CMD_WAVE                    0x75        /* 招招手 */
#define CMD_MARCH                   0x76        /* 原地踏步 */
#define CMD_GET_UP                  0x77        /* 跌倒起立 */
#define CMD_SWAGGER                 0x78        /* 大摇大摆 */
#define CMD_LEFT_HOOK               0x79        /* 左勾拳 */
#define CMD_RIGHT_HOOK              0x7A
#define CMD_EXIT_GAME               0x7B        /* 关闭玩法 */
#define CMD_SORT_YELLOW             0x7C

/* 导航/充电 */
#define CMD_SINGLE_MODE             0x80
#define CMD_NORMAL_MODE             0x81
#define CMD_GOTO_BEDROOM            0x82
#define CMD_GOTO_TOILET             0x83
#define CMD_BACK_CHARGE             0x84        /* 回充电座 */
#define CMD_CHARGE_ON               0x85	    /* 开始充电 */
#define CMD_CHARGE_OFF              0x86		/* 关闭充电 */
#define CMD_CAR_LIGHT_ON            0x87		/* 打开车灯 */
#define CMD_CAR_LIGHT_OFF           0x88		/* 关闭车灯 */
#define CMD_FREE_ON               	0x89        /* 打开手动推车 */
#define CMD_FREE_OFF               	0x8A        /* 关闭手动推车 */

/* 马桶专用 */
#define CMD_FLUSH                   0xA1        /* 冲洗马桶 */
#define CMD_CLEAN_BUTT              0xA2        /* 清洁臀部 */
#define CMD_FEMALE_CLEAN            0xA3        /* 女性清洁 */
#define CMD_SEWAGE_PUMP             0xA4        /* 开启排污泵 */
#define CMD_CONFIRM                 0xA5        /* 好的或者是的（确认） */
#define CMD_SELF_CLEAN              0xA6		/* 清洁杆自清洁 */
#define CMD_SEAT_OPEN               0xA7		/* 抬起坐便圈 */
#define CMD_SEAT_CLOSE              0xA8		/* 放下坐便圈 */
#define CMD_LID_OPEN                0xA9		/* 抬起马桶盖 */
#define CMD_LID_CLOSE               0xAA		/* 放下马桶盖 */
#define CMD_UV_LIGHT_ON             0xAB        /* 打开紫外灯 */
#define CMD_UV_LIGHT_OFF            0xAC        /* 关闭紫外灯 */
#define CMD_IR_LIGHT_ON             0xAD        /* 打开红外灯 */
#define CMD_IR_LIGHT_OFF            0xAE        /* 关闭红外灯 */
#define CMD_DRY               		  0xAF			/* 暖风烘干 */
#define CMD_CLEAN	                  0xB0			/* 开启按摩模式清洁臀部 */
#define CMD_MODE_FIXED              0xB1			/* 清洁杆固定模式 */
#define CMD_MODE_MASSAGE            0xB2			/* 清洁杆来回按摩模式 */
#define CMD_ACTION_29               0xB3
#define CMD_ACTION_30               0xB4

/* ========== 播报语 ID（功能类型 0xFF）========== */
#define TALK_RECYCLABLE             0x01        /* 可回收物 */
#define TALK_KITCHEN                0x02        /* 厨余垃圾 */
#define TALK_HARMFUL                0x03        /* 有害垃圾 */
#define TALK_OTHER                  0x04        /* 其他垃圾 */
#define TALK_OBSTACLE               0x05        /* 前方有障碍物 */
#define TALK_PARK_DONE              0x06        /* 完成泊车 */
#define TALK_RESTART                0x07        /* 重新出发 */
#define TALK_LEFT_DETECT            0x08        /* 识别到左转 */
#define TALK_RIGHT_DETECT           0x09        /* 识别到右转 */
#define TALK_STOP_DETECT            0x0A        /* 识别到停车 */
#define TALK_RED_DETECT             0x0B        /* 识别到红灯 */
#define TALK_GREEN_DETECT           0x0C        /* 识别到绿灯 */
#define TALK_UTURN_DETECT           0x0D        /* 识别到掉头 */
#define TALK_TURN_DETECT            0x0E        /* 识别到转弯 */
#define TALK_WELCOME                0x0F        /* 欢迎光临 */
#define TALK_GRAB_START             0x10        /* 发现物品正在抓取 */
#define TALK_GRAB_FAIL              0x11        /* 距离过远无法抓取 */
#define TALK_GRAB_END               0x12        /* 抓取结束 */

/* 数字（0x20-0x29）*/
#define TALK_NUM_0                  0x20
#define TALK_NUM_1                  0x21
#define TALK_NUM_2                  0x22
#define TALK_NUM_3                  0x23
#define TALK_NUM_4                  0x24
#define TALK_NUM_5                  0x25
#define TALK_NUM_6                  0x26
#define TALK_NUM_7                  0x27
#define TALK_NUM_8                  0x28
#define TALK_NUM_9                  0x29

/* 颜色（0x2A-0x2E）*/
#define TALK_COLOR_RED              0x2A
#define TALK_COLOR_GREEN            0x2B
#define TALK_COLOR_BLUE             0x2C
#define TALK_COLOR_YELLOW           0x2D
#define TALK_COLOR_PURPLE           0x2E

#define TALK_BATT_LOW           	0x30
#define TALK_CHARGE_START           0x31
#define TALK_WATER_LEVEL_LOW      	0x32
#define TALK_CONNECT_OK      		0x33
#define TALK_DETECT_DISTANCE  		0x34
#define TALK_DETECT_CLIFF      		0x35
#define TALK_RUN_ERR      				0x36
#define TALK_ACC_ERR      				0x37
#define TALK_CANNOT_CHARGE 				0x38


/* ========== 公共 API ========== */

/**
 * @brief 初始化语音模块驱动
 * @param i2c_bus_name I2C 总线名称，例如 "i2c2"
 * @return RT_EOK 成功，否则失败
 */
int asr_wonderecho_init(const char *i2c_bus_name);

/**
 * @brief 读取识别结果（非阻塞，立即返回）
 * @return 识别到的命令词 ID，0 表示未识别到任何词条
 */
uint8_t asr_get_result(void);

/**
 * @brief 主动播报语音
 * @param type 播报类型：ASR_TYPE_CMD_WORD 或 ASR_TYPE_ANNOUNCER
 * @param id   播报内容 ID（见上面的 TALK_* 宏或 CMD_* 宏对应的播报语）
 */
void asr_speak(uint8_t type, uint8_t id);

/**
 * @brief 发送 I2C 停止条件（清空总线，通常在初始化后调用一次）
 */
void asr_send_stop_condition(void);

/*“防重复播报”机制，仅用于警告类*/
void asr_speak_cooldown(uint8_t talk_id, uint32_t cooldown_ms);

#endif /* __ASR_WONDERECHO_H__ */

