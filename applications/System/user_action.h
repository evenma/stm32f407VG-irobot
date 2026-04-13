/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 智能马桶任务管理接口
 */

#ifndef __USER_ACTION_H__
#define __USER_ACTION_H__

#include <rtthread.h>

/* 动作命令（用于邮箱） */
typedef enum {
    ACTION_NONE = 0,
    ACTION_FLUSH_TOILET,          // 冲洗马桶
    ACTION_CLEAN_REAR,            // 清洁臀部
    ACTION_CLEAN_FEMALE,          // 清洁女士
    ACTION_DRY,                   // 暖风烘干
    ACTION_STOP,                  // 停止所有动作（事件）
    ACTION_SET_CLEAN_ROD_POS,     // 设置清洁杆位置（参数：位置步数）
    ACTION_CLEAN_ROD_INC,         // 清洁杆往外升（增加步数）
    ACTION_CLEAN_ROD_DEC,         // 清洁杆往回缩（减少步数）
    ACTION_TOGGLE_CLEAN_MODE,     // 切换清洁模式（定点/按摩）
    ACTION_SEAT_OPEN,             // 打开马桶圈
    ACTION_SEAT_CLOSE,            // 关闭马桶圈
    ACTION_LID_OPEN,              // 打开马桶盖
    ACTION_LID_CLOSE,             // 关闭马桶盖
    ACTION_SET_WATER_HEATER,      // 设置水箱加热（参数：开/关）
    ACTION_SET_WARM_FAN,          // 设置暖风风扇（参数：开/关）
    ACTION_SET_WARM_HEATER,       // 设置暖风电热丝（参数：功率 0~100）
    ACTION_SET_SMALL_PUMP_DUTY,   // 设置小水泵占空比（0~100）
		ACTION_SEWAGE_PUMP,           // 单独开启排污泵  注意：在管路没水时不能开启，否则直接损坏排污泵，只有在马桶堵塞时可开启
    ACTION_UV_LIGHT_ON,           // 紫外灯开
    ACTION_UV_LIGHT_OFF,          // 紫外灯关
    ACTION_IR_LIGHT_ON,           // 红外灯开
    ACTION_IR_LIGHT_OFF,          // 红外灯关
		ACTION_SELF_CLEAN,            // 自清洁
} ActionCmd_t;

/* 动作执行过程中的事件（用于停止或超时） */
#define EVENT_STOP           (1 << 0)   // 停止当前动作
#define EVENT_TIMEOUT        (1 << 1)   // 超时（内部使用）
#define EVENT_CLEAN_MODE_SW  (1 << 2)   // 清洁模式切换
#define EVENT_CLEAN_ROD_INC  (1 << 3)   // 清洁杆增加
#define EVENT_CLEAN_ROD_DEC  (1 << 4)   // 清洁杆减少

/* 清洁模式 */
typedef enum {
    CLEAN_MODE_FIXED,    // 定点清洁
    CLEAN_MODE_MASSAGE   // 按摩模式（往复运动）
} CleanMode_t;

/* 全局配置参数（可从 Flash 加载） */
typedef struct {
	  uint16_t clean_rod_fixed_pos_anus;      // 肛门清洁位置（步数，默认 60mm -> 约1285步）
    uint16_t clean_rod_fixed_pos_female;    // 女性清洁位置（步数，默认 70mm -> 1500步
//    uint16_t clean_rod_fixed_pos;      // 定点清洁位置（步数）
//    uint8_t  clean_duration_sec;       // 清洁时长（秒）
    uint8_t  clean_duration_sec_anus;       // 肛门清洁时长（秒）
    uint8_t  clean_duration_sec_female;       // 女性清洁时长（秒）	
    uint8_t  dry_duration_sec;         // 烘干时长（秒）
    uint8_t  small_pump_duty;          // 小水泵默认占空比（0~100）
    uint8_t  warm_heater_power;        // 暖风电热丝功率（0~100）
    CleanMode_t clean_mode;            // 当前清洁模式
		uint8_t flush_duration_sec;        // 冲洗马桶需要的总时长(默认5-7S)
	  uint8_t flush_pump_delay_sec;      // 大水泵开启后延迟多少秒启动排污泵（默认 2）
    uint8_t flush_pump_stop_delay_sec; // 排污泵关闭后延迟多少秒关大水泵（默认 1）
} UserActionConfig_t;

/* 外部变量声明 */
extern volatile UserActionConfig_t g_action_cfg;

/* 工作状态结构体 */
typedef struct {
    rt_bool_t flush_toilet;      // 冲洗马桶是否进行中
    rt_bool_t clean_rear;        // 肛门清洁是否进行中
    rt_bool_t clean_female;      // 女性清洁是否进行中
    rt_bool_t dry;               // 暖风烘干是否进行中
} WorkStatus_t;


/* 公共函数 */
void user_action_init(void);
void user_action_send_cmd(ActionCmd_t cmd, uint32_t param);
void user_action_stop(void);
void user_action_set_clean_rod_position(uint16_t pos);
void user_action_clean_rod_inc(void);
void user_action_clean_rod_dec(void);
void user_action_toggle_clean_mode(void);
/* 获取当前工作状态 */
const WorkStatus_t* user_action_get_work_status(void);
void user_action_sewage_pump(void);
#endif /* __USER_ACTION_H__ */

