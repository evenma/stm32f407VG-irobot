#ifndef __PACKET_HANDLE
#define __PACKET_HANDLE


#include "uart_packet.h"   // 引用 uart_packet 的枚举定义

/**
* @brief 串口命令回调处理
* @param frame 数据帧
*/
static void packet_led_handle(pkt_frame_t *frame);
static void packet_buzzer_handle(pkt_frame_t *frame);
static void packet_motor_handle(pkt_frame_t *frame);
static void packet_battery_limit_handle(pkt_frame_t *frame);
static void packet_light_handle(pkt_frame_t *frame);
static void packet_charger_handle(pkt_frame_t *frame);
static void packet_toilet_handle(pkt_frame_t *frame);

int packet_handle_init(void);

#endif


