#include "uart_packet.h"
#include "checksum.h"
#include <rtdevice.h>
#include <string.h>
#include "packet_reports.h"

/* ---------------------------- 硬件配置 ---------------------------- */
#define UART_DEV_NAME       "uart2"         /* 与上位机通信的串口设备名称 */
#define UART_BAUDRATE       1000000  // BAUD_RATE_921600/* 波特率需与上位机一致 */

/* ---------------------------- 内部数据结构 ---------------------------- */

/* 发送消息队列（存储待发送的帧指针）*/
static rt_mq_t  s_tx_mq;
//static char s_tx_mq_pool[sizeof(pkt_frame_t *) * PKT_TX_QUEUE_SIZE];
#define PKT_TX_MQ_MSG_SIZE     sizeof(pkt_frame_t *)

/* 回调函数表 */
static pkt_callback_t s_callbacks[PKT_FUNC_NONE];

/* 解析器状态机状态 */
typedef enum {
    STATE_STARTBYTE1,
    STATE_STARTBYTE2,
    STATE_FUNCTION,
    STATE_LENGTH,
    STATE_DATA,
    STATE_CHECKSUM,
} pkt_parse_state_t;

static pkt_parse_state_t s_state = STATE_STARTBYTE1;
static pkt_frame_t s_frame;
static uint8_t s_data_idx = 0;

/* 发送线程相关 */
static rt_thread_t s_tx_thread = RT_NULL;

/* 接收线程相关 */
static rt_thread_t s_rx_thread = RT_NULL;
static rt_sem_t s_rx_sem = RT_NULL;         /* 有数据可读信号量 */
static rt_device_t s_uart_dev = RT_NULL;
static rt_mp_t s_frame_mp = RT_NULL;   /* 帧内存池 */

/* ---------------------------- 协议辅助函数 ---------------------------- */
static void send_command_ack(uint8_t func, uint8_t status)
{
    uint8_t data[3];
    data[0] = SYS_SUB_CMD_ACK;   // 子命令：ACK
    data[1] = func;              // 原功能号
    data[2] = status;            // 0=成功, 非0=错误码
    uart_packet_send(PKT_FUNC_SYS, data, sizeof(data));
}
/* ---------------------------- 串口接收回调 ---------------------------- */
/* 串口接收完成中断回调（非 DMA 模式下可用，这里使用 DMA 时需要特殊处理）*/
/* 由于我们在 Kconfig 中启用了 UART2 RX DMA，RT-Thread 串口驱动会在 DMA 收到数据后
   调用 rx_indicate，我们只需在回调中释放信号量，通知接收线程读取数据。*/
static rt_err_t uart_rx_indicate(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(s_rx_sem);
    return RT_EOK;
}

/* ---------------------------- 接收线程 ---------------------------- */
static void rx_thread_entry(void *param)
{
	 rt_kprintf("[UART_PKT]rx_thread started\n");
    uint8_t buf[256];
    rt_size_t len;
    uint8_t crc;

    while (1) {
        /* 等待数据到达信号量 */
        rt_sem_take(s_rx_sem, RT_WAITING_FOREVER);
        uint8_t byte;
        while (1) {
			len = rt_device_read(s_uart_dev, 0, buf, sizeof(buf));
			if (len == 0) break;
			for (size_t i = 0; i < len; i++) {
				byte = buf[i];
//				rt_kprintf("[%02X ", byte);
//				rt_kprintf("%d] ", s_state);
				/*根据状态机解析1帧数据 从任意字节流中自动提取符合协议格式的帧*/
			   switch (s_state) {
					case STATE_STARTBYTE1:
						if (byte == PKT_START_BYTE1)
							s_state = STATE_STARTBYTE2;
						break;
					case STATE_STARTBYTE2:
						if (byte == PKT_START_BYTE2)
							s_state = STATE_FUNCTION;
						else
							s_state = STATE_STARTBYTE1;
						break;
					case STATE_FUNCTION:
						if (byte < PKT_FUNC_NONE) {
							s_frame.func = byte;
							s_state = STATE_LENGTH;
						} else {
							s_state = STATE_STARTBYTE1;
						}
						break;
					case STATE_LENGTH:
						s_frame.data_len = byte;
						s_data_idx = 0;
						if (s_frame.data_len == 0)
							s_state = STATE_CHECKSUM;
						else
							s_state = STATE_DATA;
						break;
					case STATE_DATA:
						s_frame.data_and_checksum[s_data_idx++] = byte;
						if (s_data_idx >= s_frame.data_len)
							s_state = STATE_CHECKSUM;
						break;
					case STATE_CHECKSUM:
						s_frame.data_and_checksum[s_frame.data_len] = byte;
						/* 计算校验和 */
						crc = checksum_crc8((uint8_t *)&s_frame.func, s_frame.data_len + 2);  //从第三位字节开始计算
//						rt_kprintf("calc_crc=%02X, recv_crc=%02X\n", crc, byte);  // 添加这行
						if (crc == s_frame.data_and_checksum[s_frame.data_len]) {
							rt_kprintf("[UART_PKT] Recv frame: func=%d, len=%d, data: ", s_frame.func, s_frame.data_len);
							for (int i = 0; i < s_frame.data_len; i++) {
									rt_kprintf("%02X ", s_frame.data_and_checksum[i]);
							}
							rt_kprintf("\n");
							/* 校验成功，执行回调 */
							if (s_frame.func < PKT_FUNC_NONE && s_callbacks[s_frame.func] != RT_NULL) {
								s_callbacks[s_frame.func](&s_frame);
							}
							
							 if (s_frame.func != PKT_FUNC_OTA) {
										send_command_ack(s_frame.func, 0);   // 0 表示成功
								}
						}
						/* 重置状态机 */
						memset(&s_frame, 0, sizeof(s_frame));
						s_state = STATE_STARTBYTE1;
						break;
					default:
						s_state = STATE_STARTBYTE1;
						break;
				}			
			}
        }
    }
}

/* ---------------------------- 发送线程 ---------------------------- */
static void tx_thread_entry(void *param)
{
	 rt_kprintf("[UART_PKT]tx_thread started\n");
    pkt_frame_t *frame;
rt_kprintf("tx_thread: s_tx_mq = %p\n", s_tx_mq);
    while (1) {
			/* 从消息队列中取待发送帧 */
			  // 注意：rt_mq_recv 返回实际接收到的字节数，成功时等于 sizeof(frame)
        if (rt_mq_recv(s_tx_mq, &frame, sizeof(frame), RT_WAITING_FOREVER) == sizeof(frame)) {
//            rt_kprintf("tx_thread_entry: sending frame, len=%d\n", frame->data_len + 5);
					  /* 发送数据（使用阻塞发送）*/
            rt_device_write(s_uart_dev, 0, (uint8_t *)frame, frame->data_len + 5);
					  /* 释放动态分配的帧内存（如果帧是动态申请的）*/
            //rt_free(frame);
            rt_mp_free(frame);
        } else {
            // 接收失败（理论上不会发生，因为超时为永久等待）
            rt_kprintf("tx_thread: recv error\n");
        }      
    }
}

/* ---------------------------- 发送接口 ---------------------------- */
int uart_packet_send(uint8_t func, void *data, size_t data_len)
{
		if (s_frame_mp == RT_NULL) {
			rt_kprintf("[UART_PKT] Memory pool not ready, drop packet\n");
			return -RT_ERROR;
	}
    if (data_len > 250) return -RT_EINVAL;  /* 最多 250 字节数据 + 校验和 */
//    pkt_frame_t *frame = rt_malloc(sizeof(pkt_frame_t));
	pkt_frame_t *frame = rt_mp_alloc(s_frame_mp, RT_WAITING_NO);  // 非阻塞分配 内存池
    if (frame == RT_NULL) return -RT_ENOMEM;

    frame->start_byte1 = PKT_START_BYTE1;
    frame->start_byte2 = PKT_START_BYTE2;
    frame->func = func;
    frame->data_len = data_len;
    if (data_len > 0) {
        memcpy(frame->data_and_checksum, data, data_len);
    }
    uint8_t crc = checksum_crc8((uint8_t *)&frame->func, data_len + 2);
    frame->data_and_checksum[data_len] = crc;

    /* 将帧指针发送到消息队列 */
//		rt_kprintf("[UART_PKT] frame = %p\n",frame);
//		rt_kprintf("[UART_PKT] sizeof(frame) = %d\n",sizeof(frame));
//		rt_kprintf("[UART_PKT]send: s_tx_mq = %p\n", s_tx_mq);
		int ret = rt_mq_send(s_tx_mq, &frame, sizeof(frame));
//		rt_kprintf("mq_send ret=%d\n", ret);
		if (ret != RT_EOK) {
				//rt_free(frame);
				rt_mp_free(frame);
				return -RT_EBUSY;
		}
    return RT_EOK;
}

/* ---------------------------- 命令注册 ---------------------------- */
void uart_packet_register_callback(pkt_func_t func, pkt_callback_t callback)
{
    if (func < PKT_FUNC_NONE) {
        s_callbacks[func] = callback;
    }
}

/* ---------------------------- 初始化 ---------------------------- */
int uart_packet_init(void)
{

    /* 1. 打开串口设备 */
    s_uart_dev = rt_device_find(UART_DEV_NAME);
    if (s_uart_dev == RT_NULL) {
        rt_kprintf("[UART_PKT] Failed to find %s\n", UART_DEV_NAME);
        return -RT_ERROR;
    }

    /* 配置串口参数：修改波特率 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = UART_BAUDRATE;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity = PARITY_NONE;
    rt_device_control(s_uart_dev, RT_DEVICE_CTRL_CONFIG, &config);

    /* 3. 初始化信号量、消息队列 */
    s_rx_sem = rt_sem_create("pkt_rx", 0, RT_IPC_FLAG_FIFO);
//    rt_mq_init(s_tx_mq, "pkt_tx_mq", s_tx_mq_pool, sizeof(pkt_frame_t *), 
//               sizeof(s_tx_mq_pool), RT_IPC_FLAG_FIFO);
	  rt_kprintf("[UART_PKT] PKT_TX_MQ_MSG_SIZE = %d\n",PKT_TX_MQ_MSG_SIZE);
		s_tx_mq = rt_mq_create("pkt_tx_mq", PKT_TX_MQ_MSG_SIZE, PKT_TX_QUEUE_SIZE, RT_IPC_FLAG_FIFO);
		if (s_tx_mq == RT_NULL) {
				rt_kprintf("[UART_PKT] Failed to create tx mq\n");
				return -RT_ERROR;
		}
	    /* 创建帧内存池：每块大小 = sizeof(pkt_frame_t)，共 PKT_TX_QUEUE_SIZE 块 */
		rt_kprintf("[UART_PKT] sizeof(pkt_frame_t) = %d\n",sizeof(pkt_frame_t));
    s_frame_mp = rt_mp_create("pkt_mp", PKT_TX_QUEUE_SIZE, sizeof(pkt_frame_t));
    if (s_frame_mp == RT_NULL) {
        rt_kprintf("[UART_PKT] Failed to create memory pool\n");
        return -RT_ERROR;
    }		   

    /* 4. 创建接收线程 */
    s_rx_thread = rt_thread_create("pkt_rx", rx_thread_entry, NULL,
                                   2048, 12, 10);
    if (s_rx_thread) rt_thread_startup(s_rx_thread);

    /* 5. 创建发送线程 */
    s_tx_thread = rt_thread_create("pkt_tx", tx_thread_entry, NULL,
                                   2048, 13, 10);
    if (s_tx_thread) rt_thread_startup(s_tx_thread);
	
	/* 打开设备（中断方式，DMA 模式由底层决定）*/
	//应用层操作模式 DMA tx_bufsz>0 
	rt_err_t ret = rt_device_open(s_uart_dev, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING); // 串口设备使用模式为 (发送阻塞 接收非阻塞) 模式
	// 硬件层工作模式 模式优先级为：DMA 模式 > 中断模式 > 轮询模式。即当有 DMA 配置时，默认使用 DMA 模式，以此类推。且非必要条件，不选择使用轮询模式
    // rt_err_t ret = rt_device_open(s_uart_dev, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);
    if (ret != RT_EOK) {
        rt_kprintf("[UART_PKT] Failed to open %s\n", UART_DEV_NAME);
        return -RT_ERROR;
    }
    /* 设置接收回调 */
    rt_device_set_rx_indicate(s_uart_dev, uart_rx_indicate);
    rt_kprintf("[UART_PKT] Packet protocol initialized on %s\n", UART_DEV_NAME);
				 /* 可选：读取配置验证波特率是否设置成功 */
    struct serial_configure verify_config;
    rt_device_control(s_uart_dev, RT_SERIAL_CTRL_GET_CONFIG, &verify_config);
    rt_kprintf("[UART_PKT] UART configured: baud=%d, data=%d, stop=%d, parity=%d\n",
               verify_config.baud_rate, verify_config.data_bits,
               verify_config.stop_bits, verify_config.parity);
		return RT_EOK;
}
//INIT_BOARD_EXPORT(uart_packet_init);

#ifdef RT_USING_FINSH
#include <finsh.h>

static void uart2_pkt_test(int argc, char **argv)
{
    uint8_t imu_data[] = {0x11, 0x22, 0x33, 0x44};
    uart_packet_send(PKT_FUNC_IMU, imu_data, sizeof(imu_data));
    rt_kprintf("send imu frame\n");
}
MSH_CMD_EXPORT(uart2_pkt_test, send imu test frame);


static rt_thread_t s_simple_tx_thread = RT_NULL;

static void simple_tx_thread_entry(void *param)
{
    const char *test_str = "UART2 Test\n";

    while (1) {
        rt_device_write(s_uart_dev, 0, test_str, strlen(test_str));
        rt_thread_mdelay(1000);  // 延时1秒
    }
}

static void uart2_simple_loop(void)
{
//     s_simple_dev = rt_device_find("uart2");
//    if (s_simple_dev == RT_NULL) {
//        rt_kprintf("uart2 not found\n");
//        return;
//    }
//    /* 打开设备，阻塞发送模式 */
//    rt_device_open(s_simple_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_TX_BLOCKING);

    if (s_simple_tx_thread == RT_NULL) {
        s_simple_tx_thread = rt_thread_create("tx_simple",
                                              simple_tx_thread_entry,
                                              NULL,
                                              1024, 20, 10);
        if (s_simple_tx_thread) {
            rt_thread_startup(s_simple_tx_thread);
            rt_kprintf("Simple loop started, sending every 1s\n");
        } else {
            rt_kprintf("Failed to create thread\n");
        }
    } else {
        rt_kprintf("Thread already running\n");
    }
}
MSH_CMD_EXPORT(uart2_simple_loop, start a simple loop tx test without timer/semaphore);

static void uart2_simple_stop(void)
{
    if (s_simple_tx_thread != RT_NULL) {
        rt_thread_delete(s_simple_tx_thread);
        s_simple_tx_thread = RT_NULL;
    }
//    if (s_simple_dev != RT_NULL) {
//        rt_device_close(s_simple_dev);
//        s_simple_dev = RT_NULL;
//    }
    rt_kprintf("Simple loop stopped\n");
}
MSH_CMD_EXPORT(uart2_simple_stop, stop the simple loop tx test);

#endif


