#include <rtthread.h>
#include <rtdevice.h>
#include "ble_serial.h"

static rt_device_t ble_uart = RT_NULL;
static rt_sem_t   rx_sem = RT_NULL;
static int        rx_timeout = 100;   // 默认100ms
static volatile int rx_remain = 0;    // 剩余可读字节数

/* 串口接收中断回调（由 DMA 或中断触发）*/
static rt_err_t ble_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
		rx_remain += size;                // 累加新收到的字节数
    rt_sem_release(rx_sem);
    return RT_EOK;
}

void ble_serial_init(const char *uart_name, int timeout)
{
    rx_timeout = timeout;
    ble_uart = rt_device_find(uart_name);
    if (!ble_uart) {
        rt_kprintf("BLE UART %s not found\n", uart_name);
        return;
    }

    /* 配置串口参数（波特率 115200，8N1）*/
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = 115200;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity   = PARITY_NONE;
    rt_device_control(ble_uart, RT_DEVICE_CTRL_CONFIG, &config);

    /* 以非阻塞接收 + 阻塞发送方式打开 */
    rt_device_open(ble_uart, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    rt_device_set_rx_indicate(ble_uart, ble_uart_rx_ind);

    rx_sem = rt_sem_create("ble_rx", 0, RT_IPC_FLAG_FIFO);
}

/* 阻塞读取一个字节（超时单位：ms）*/
int ble_serial_getc_timeout(int timeout_ms)
{
    uint8_t ch;
		
	  // 如果还有剩余字节，直接读取，不等待信号量
    if (rx_remain > 0) {
        if (rt_device_read(ble_uart, 0, &ch, 1) == 1) {
            rx_remain--;              // 消耗一个字节
						rt_sem_trytake(rx_sem);    // 有数据，已处理，不需要此信号量
//						rt_kprintf("nb:%c=0x%02X\n", ch,ch);
            return ch;
        } else {
            // 理论上不应发生，但若读取失败则清零计数并返回错误
            rx_remain = 0;
            return -1;
        }
    }
		    // 无剩余数据，等待信号量（有新数据到达）
    if (rt_sem_take(rx_sem, timeout_ms) != RT_EOK) {
        return -1;
    }
		// 信号量到达，再次尝试读取（此时 rx_remain 应至少为1）
    if (rt_device_read(ble_uart, 0, &ch, 1) == 1) {
        rx_remain--;
//				rt_kprintf("%c=0x%02X\n", ch,ch);
        return ch;
    } else {
        // 异常情况：有信号量但读不到数据，清零计数
        rx_remain = 0;
        return -1;
    }
    return -1;
}

//int ble_serial_getc_timeout(int timeout_ms)
//{
//    uint8_t ch;
//    if (rx_remain == 0) {
//        if (rt_sem_take(rx_sem, timeout_ms) != RT_EOK)
//            return -1;
//        // 此时 rx_remain 应该 > 0
//    }
//    if (rt_device_read(ble_uart, 0, &ch, 1) != 1)
//        return -1;
//    rx_remain--;
//    if (rx_remain == 0) {
//        // 读空了，消耗掉那个“有数据”信号量
//        rt_sem_trytake(rx_sem);
//    }
//    return ch;
//}

/* 非阻塞读取一个字节（立即返回）*/
int ble_serial_getc_nowait(void)
{
    uint8_t ch;
		if (rt_device_read(ble_uart, 0, &ch, 1) == 1) {
				if(rx_remain){
						rx_remain--;              // 消耗一个字节
						if (rx_remain == 0){
                rt_sem_trytake(rx_sem);								
						}							
				}
				return ch;
		} 
    return -1;
}

/* 发送一个字节（阻塞）*/
int ble_serial_putc(char c)
{
    return (rt_device_write(ble_uart, 0, &c, 1) == 1) ? 0 : -1;
}

