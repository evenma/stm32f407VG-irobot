#ifndef __IR_RECEIVER_H__
#define __IR_RECEIVER_H__

#include <rtthread.h>

/* 初始化红外接收模块（左右两个通道）*/
void ir_receiver_init(void);

/* 获取左接收管最新状态（bit2:上管, bit1:左管, bit0:右管）*/
uint8_t ir_get_left_status(void);

/* 获取右接收管最新状态（bit2:上管, bit1:左管, bit0:右管）*/
uint8_t ir_get_right_status(void);

/* 获取左接收管匹配计数器（每次成功匹配自增）*/
uint16_t ir_get_left_match_cnt(void);

/* 获取右接收管匹配计数器 */
uint16_t ir_get_right_match_cnt(void);

/* 清除左接收管状态和计数器（由 monitor 超时调用）*/
void ir_clear_left_status(void);

/* 清除右接收管状态和计数器 */
void ir_clear_right_status(void);

/* 外部控制红外对准使能标志（由 car_action 设置）*/
//extern volatile rt_bool_t g_ir_alignment_enable;

#endif /* __IR_RECEIVER_H__ */
