#ifndef __IRM_8601M2_H__
#define __IRM_8601M2_H__

#include <rtthread.h>


void irm_8601m2_init(void);   // 初始化两个接收管，启动解码线程

/* 获取左接收管最新状态（bit0:右, bit1:左, bit2:上）*/
uint8_t irm_get_left_status(void);

/* 获取右接收管最新状态（bit0:右, bit1:左, bit2:上）*/
uint8_t irm_get_right_status(void);

/* 获取左接收管的匹配计数器（每次成功匹配自增）*/
uint16_t irm_get_left_match_cnt(void);

/* 获取右接收管的匹配计数器 */
uint16_t irm_get_right_match_cnt(void);

/* 清除左接收管的状态和计数器（由 monitor 超时调用）*/
void irm_clear_left_status(void);

/* 清除右接收管的状态和计数器 */
void irm_clear_right_status(void);


#endif

