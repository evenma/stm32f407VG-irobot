#ifndef __UART_PACKET_H__
#define __UART_PACKET_H__

#include <rtthread.h>

/* 协议常量 */
#define PKT_START_BYTE1         0xAA
#define PKT_START_BYTE2         0x55
#define PKT_TX_QUEUE_SIZE       64          /* 发送队列数量 */
#define PKT_RX_FIFO_SIZE        2048        /* 接收环形缓冲区大小 */
#define PKT_TX_FIFO_SIZE        256			/* 发送环形缓冲区大小 */
#define PKT_DMA_PING_SIZE       512          /* Ping缓冲区大小 */
// RT-Thread Serial V2底层策略 “生产者（DMA）→ Ping 缓冲区 → 消费者（CPU）→ 主环形缓冲区” 的双缓存协作模式

/* 功能号枚举 */
typedef enum {
    PKT_FUNC_SYS        = 0,    //电池电压，充电状态，加热管电源,悬崖传感器，红外对射管
    PKT_FUNC_LED,
    PKT_FUNC_BUZZER,
		PKT_FUNC_MOTOR,			//小车电机控制 以及上报 电机离线，故障，工作状态，实际速度，位置脉冲，温度
    PKT_FUNC_PWM_SERVO,
    PKT_FUNC_BUS_SERVO,
		PKT_FUNC_KEY,					// 按键值上报 包括手控器按键和home键
		PKT_FUNC_IMU,					// 7 姿态传感器上报
		PKT_FUNC_GAMEPAD,     // 摇杆不在下位机，暂不支持
    PKT_FUNC_SBUS,
    PKT_FUNC_OLED,
    PKT_FUNC_RGB,
	  PKT_FUNC_LIGHT,      //灯光控制
    PKT_FUNC_CHARGER,    //充电控制
		PKT_FUNC_TOILET,    		 //e智能马桶控制 以及上报  水箱高低水位状态，水箱水温
    PKT_FUNC_ULTRASONIC,   //f 超声波距离上报 
		PKT_FUNC_VOICE,        // 语音模块信息上报 
    PKT_FUNC_NONE,
} pkt_func_t;

/* 帧结构（与上位机协议一致）*/
#pragma pack(1)
typedef struct {
    uint8_t start_byte1;                /* 0xAA */
    uint8_t start_byte2;                /* 0x55 */
    uint8_t func;                       /* 功能号 */
    uint8_t data_len;                   /* 数据段长度 */
    uint8_t data_and_checksum[257];     /* 数据 + 校验和(最后一个字节) */
} pkt_frame_t;    // 1帧数据最大长度 4+257=261字节
#pragma pack()

/* 命令回调函数类型 */
typedef void (*pkt_callback_t)(pkt_frame_t *frame);

/* 初始化协议栈（创建线程、FIFO、消息队列）*/
int uart_packet_init(void);

/* 注册命令回调 */
void uart_packet_register_callback(pkt_func_t func, pkt_callback_t callback);

/* 发送数据帧（阻塞或非阻塞，由队列实现）*/
int uart_packet_send(uint8_t func, void *data, size_t data_len);

#endif /* __UART_PACKET_H__ */
