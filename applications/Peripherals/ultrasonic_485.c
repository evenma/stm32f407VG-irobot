#include "ultrasonic_485.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "global_conf.h"
#include <fal.h>
#include "monitor.h"

/* ----------------------------- 硬件配置 ----------------------------- */
#define RS485_UART_NAME         "uart4"      /* 串口设备名称 */

static uint32_t current_baudrate = BAUD_RATE_115200;   // 当前 UART 波特率，默认 115200

/* ----------------------------- Modbus 参数 ----------------------------- */
#define MODBUS_FUNC_READ        0x03
#define MODBUS_FUNC_WRITE       0x06
#define MODBUS_REG_LV_DIST    	0x0100       /* 经算法处理后，输出距离值，单位：mm，响应时间约 190～750ms(量程不同而有差异) */
#define MODBUS_REG_REAL_DIST    0x0101       /* 实时距离寄存器 单位：mm，响应时间约 15～140ms(量程不同而有差异)*/
/* 测距量程等级 1～5(默认为 5)，量程范围:5-约 500cm，实时值响应时间 40～140ms，处理值响应时间 320～750ms。 */
#define MODBUS_READ_LEN         4            /* 读取响应数据长度（不含CRC）: 地址+功能+字节数+2字节数据 */
#define MODBUS_FRAME_MAX_LEN    8            /* 请求帧最大长度：地址(1)+功能(1)+寄存器地址(2)+寄存器数量(2)+CRC(2)=8 */
#define MODBUS_RESP_MAX_LEN     8            /* 响应帧最大长度：地址(1)+功能(1)+字节数(1)+数据(2)+CRC(2)=7，预留8 */
/*注意：设置类响应的是发送的数据帧，读取类响应的是地址(1)+功能(1)+字节数(1)+数据(2)+CRC(2)*/

#define RT_ENOOBJECT   (-RT_ERROR - 1)   // 无物体
#define RT_EINTERFERE  (-RT_ERROR - 2)   // 同频干扰

/* ----------------------------- 静态变量 ----------------------------- */
/* 耗时统计变量 */
static struct {
    uint32_t count;          // 成功读取次数
    uint32_t total_ms;       // 总耗时（毫秒）
    uint32_t min_ms;         // 最小耗时
    uint32_t max_ms;         // 最大耗时
} read_stats;
static uint8_t print_each_time = 0;   // 是否每次读取都打印耗时（默认关闭）
/* 发送模式 */
typedef enum {
    SEND_MODE_PER_SENSOR,
    SEND_MODE_PER_ROUND
} send_mode_t;
static send_mode_t current_send_mode = SEND_MODE_PER_ROUND;   // 默认一轮读取到了发送
static uint8_t poll_print_enabled = 0;                         // 默认开启打印

static rt_device_t serial = RT_NULL;
static struct rt_mutex rs485_mutex;          /* 互斥锁，避免并发操作串口 */
static uint8_t poll_running = 0;
static rt_thread_t poll_thread = RT_NULL;

static uint32_t s_last_distances[ULTRASONIC_485_NUM] = {0};
static struct rt_mutex s_dist_mutex;   // 保护距离数组的互斥锁（可选，若轮询线程独占写，读时简单复制可不加锁，但为安全建议加锁）

static int32_t rt_tick_to_millisecond(rt_tick_t tick_diff)
{
    // 返回 tick_diff * 1000 / RT_TICK_PER_SECOND
    return (int32_t)((tick_diff * 1000) / RT_TICK_PER_SECOND);
}

uint32_t ultrasonic_485_get_baudrate(void)
{
    return current_baudrate;
}

/* ----------------------------- Modbus CRC16 计算 ----------------------------- */
static uint16_t modbus_crc16(uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = crc >> 1;
        }
    }
    return crc;
}

/* ----------------------------- RS485 方向控制 ----------------------------- */
static void rs485_set_dir(uint8_t tx_enable)
{
    rt_pin_write(RS485_DIR_PIN, tx_enable ? PIN_HIGH : PIN_LOW);
    /* 方向切换后延时，确保收发器稳定 */
    if (tx_enable)
        rt_hw_us_delay(10);
    else
        rt_hw_us_delay(5);
}

/* ----------------------------- 发送 Modbus 请求并等待响应 ----------------------------- */
static rt_err_t modbus_send_recv(uint8_t *req, uint16_t req_len,
                                 uint8_t *resp, uint16_t *resp_len,
                                 uint32_t timeout_ms)
{
    rt_err_t ret = RT_EOK;
    rt_size_t len;
    uint32_t start_tick;

    /* 加锁，防止多线程同时使用串口 */
    rt_mutex_take(&rs485_mutex, RT_WAITING_FOREVER);

    /* 清空串口缓冲区（可能残留的旧数据）*/
//    while (rt_device_read(serial, 0, resp, MODBUS_RESP_MAX_LEN) > 0);
		rt_device_control(serial, RT_SERIAL_CTRL_RX_FLUSH, NULL);

    /* 发送请求 */
    rs485_set_dir(1);   /* 发送模式 */
    len = rt_device_write(serial, 0, req, req_len);
    if (len != req_len)
    {
        ret = -RT_ERROR;
        goto exit;
    }
    /* 等待发送完成（简单延时，亦可使用TX完成中断，此处延时1ms足够）*/
    rt_thread_mdelay(1);
    rs485_set_dir(0);   /* 切换为接收模式 */

    /* 接收响应 */
    start_tick = rt_tick_get();
    *resp_len = 0;
    while (rt_tick_get() - start_tick < rt_tick_from_millisecond(timeout_ms))
    {
        len = rt_device_read(serial, 0, resp + *resp_len, MODBUS_RESP_MAX_LEN - *resp_len);
        if (len > 0)
        {
					*resp_len += len;
					// 判断是否收到完整帧：至少需要地址+功能+字节数（3字节）
					if (*resp_len >= 3)
					{
							uint8_t data_len = resp[2];          // 数据字节数
							uint16_t expected_len = 3 + data_len + 2; // 地址+功能+数据长度+数据+CRC
							if (*resp_len >= expected_len)
							{
									// 收齐完整帧，退出
									break;
							}
					}
					// 防止溢出
					if (*resp_len >= MODBUS_RESP_MAX_LEN)
							break;					
        }
        rt_thread_mdelay(1);
    }

    if (*resp_len < 5)   /* 至少地址+功能+字节数+2字节数据=5，再加CRC=7，此处保守 */
    {
        ret = -RT_ETIMEOUT;
        goto exit;
    }

exit:
    rt_mutex_release(&rs485_mutex);
    return ret;
}

/* ----------------------------- 读取指定地址传感器的实时距离（mm） ----------------------------- */
rt_err_t ultrasonic_485_read_distance(uint8_t addr, uint32_t *distance_mm, uint32_t timeout_ms)
{
    if (addr < 1 || addr > 247)      /* Modbus 有效地址范围 1~247 */
        return -RT_EINVAL;
    if (distance_mm == NULL)
        return -RT_EINVAL;
    if (timeout_ms == 0)
        timeout_ms = RS485_TIMEOUT_MS;

    uint8_t req[MODBUS_FRAME_MAX_LEN];
    uint8_t resp[MODBUS_RESP_MAX_LEN];
    uint16_t resp_len;
    rt_err_t ret;
		rt_tick_t start_tick, end_tick;
    uint32_t elapsed_ms;

    /* 构建读取实时距离寄存器请求（地址 0x0101）*/
    req[0] = addr;
    req[1] = MODBUS_FUNC_READ;
    req[2] = (MODBUS_REG_REAL_DIST >> 8) & 0xFF;
    req[3] = MODBUS_REG_REAL_DIST & 0xFF;
    req[4] = 0x00;   /* 寄存器数量高字节 */
    req[5] = 0x01;   /* 读取1个寄存器（2字节）*/
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

		  start_tick = rt_tick_get();
    ret = modbus_send_recv(req, 8, resp, &resp_len, timeout_ms);
    if (ret != RT_EOK)
        return ret;

    /* 校验响应帧长度和内容 */
    if (resp_len < 7)   /* 最小有效长度：地址+功能+字节数+数据(2)+CRC(2)=7 */
        return -RT_ERROR;
    if (resp[0] != addr)
        return -RT_ERROR;
    if (resp[1] != MODBUS_FUNC_READ)
        return -RT_ERROR;
    if (resp[2] != 0x02)   /* 数据字节数应为2 */
        return -RT_ERROR;

    /* 校验 CRC */
    uint16_t recv_crc = resp[resp_len-2] | (resp[resp_len-1] << 8);
    uint16_t calc_crc = modbus_crc16(resp, resp_len-2);
    if (recv_crc != calc_crc)
        return -RT_ERROR;

    /* 提取距离值（大端）*/
    uint16_t dist_raw = (resp[3] << 8) | resp[4];
		if (dist_raw == 0xFFFD)   /* 无物体 */
				return -RT_ENOOBJECT;   // 或者返回 -RT_ETIMEOUT 但上层区分
		if (dist_raw == 0xFFFE)   /* 同频干扰 */
				return -RT_EINTERFERE;

		*distance_mm = dist_raw;
		
		    // 结束计时
    end_tick = rt_tick_get();
    if (print_each_time) {
			elapsed_ms = (end_tick - start_tick) * 1000 / RT_TICK_PER_SECOND;
			// 更新统计
			if (read_stats.count == 0) {
					read_stats.min_ms = elapsed_ms;
					read_stats.max_ms = elapsed_ms;
			} else {
					if (elapsed_ms < read_stats.min_ms) read_stats.min_ms = elapsed_ms;
					if (elapsed_ms > read_stats.max_ms) read_stats.max_ms = elapsed_ms;
			}
			read_stats.total_ms += elapsed_ms;
			read_stats.count++; 
      
			rt_kprintf("[485] read addr %d, distance=%d mm, time=%d ms\n", addr, *distance_mm, elapsed_ms);
    }		
    return RT_EOK;
}

/* ----------------------------- 轮询线程（每 500ms 读取所有传感器） ----------------------------- */
static void poll_entry(void *parameter)
{
    uint32_t dist;
    rt_err_t ret;
    uint8_t i;
    char single_buf[64];
    char round_buf[256];
    int len;

    while (poll_running)
    {
				rt_tick_t start_tick = rt_tick_get();
        if (current_send_mode == SEND_MODE_PER_ROUND)
            len = rt_snprintf(round_buf, sizeof(round_buf), "[Poll] ");

        for (i = 0; i < ULTRASONIC_485_NUM; i++)   // 地址 1~7 对应 S0~S6
        {
            ret = ultrasonic_485_read_distance(i+1, &dist, RS485_TIMEOUT_MS);
						if (ret == RT_EOK)
						{
								rt_mutex_take(&s_dist_mutex, RT_WAITING_FOREVER);
								s_last_distances[i] = dist;   // i 对应地址 i+1
								rt_mutex_release(&s_dist_mutex);
						}
						else
						{
								rt_mutex_take(&s_dist_mutex, RT_WAITING_FOREVER);
								s_last_distances[i] = 0;       // 0 表示无效（因为传感器最小距离 20mm）
								rt_mutex_release(&s_dist_mutex);
						}
            if (current_send_mode == SEND_MODE_PER_SENSOR)
            {
							  /* 立即发送单个传感器数据 */
                if (ret == RT_EOK)
                {
                    rt_snprintf(single_buf, sizeof(single_buf), "S%d=%dmm\n", i, dist);
                }
                else
                {
                    rt_snprintf(single_buf, sizeof(single_buf), "S%d=ERR\n", i);
                }
								if (poll_print_enabled)
									rt_kprintf("%s", single_buf);  /* 可改为串口直接输出，或使用回调发送至上位机 */
            }
            else if (current_send_mode == SEND_MODE_PER_ROUND)
            {
							  if (ret == RT_EOK)
                {
                    len += rt_snprintf(round_buf + len, sizeof(round_buf) - len,
                                       "S%d=%dmm ", i, dist);
                }
                else
                {
                    len += rt_snprintf(round_buf + len, sizeof(round_buf) - len,
                                       "S%d=ERR ", i);
                }
            }
            rt_thread_mdelay(10);   // 间隔
        }
				if (current_send_mode == SEND_MODE_PER_ROUND && poll_print_enabled)
        {
            rt_kprintf("%s\n", round_buf);
        }
				if (current_send_mode == SEND_MODE_PER_SENSOR && poll_print_enabled)
        {
            rt_kprintf("\n-------\n");
        }
				rt_tick_t elapsed = rt_tick_get() - start_tick;
        int32_t delay_ms = POLL_INTERVAL_485_MS - rt_tick_to_millisecond(elapsed);
        if (delay_ms > 0)
            rt_thread_mdelay(delay_ms);
				else 
					rt_thread_mdelay(10);    // 超时，下一次轮询过渡时间10ms
    }
}

void ultrasonic_485_start_poll(void)
{
    if (poll_running)
        return;
    poll_running = 1;
    poll_thread = rt_thread_create("ultr_485",
                                    poll_entry,
                                    RT_NULL,
                                    2048,
                                    17,
                                    10);
    if (poll_thread)
        rt_thread_startup(poll_thread);
    else
        rt_kprintf("[485] Failed to create poll thread\n");
}

void ultrasonic_485_stop_poll(void)
{
    if (poll_running)
    {
        poll_running = 0;
			  // 等待线程自然退出（最长等待 1 秒）
        for (int i = 0; i < 20; i++) {  // 20 * 50ms = 1000ms
            rt_thread_mdelay(50);
            if (poll_thread == RT_NULL) // 线程已被删除，退出
                break;
        }
        if (poll_thread)
        {
            rt_thread_delete(poll_thread);
            poll_thread = RT_NULL;
        }
    }
}

/* ----------------------------- 获取距离 ----------------------------- */
void ultrasonic_485_get_distances(uint32_t *dist_array, uint8_t len)
{
    uint8_t i;
    uint8_t copy_len = len < ULTRASONIC_485_NUM ? len : ULTRASONIC_485_NUM;
    rt_mutex_take(&s_dist_mutex, RT_WAITING_FOREVER);
    for (i = 0; i < copy_len; i++) {
        dist_array[i] = s_last_distances[i];
    }
    rt_mutex_release(&s_dist_mutex);
}

/* ----------------------------- 设置传感器 Modbus 地址 ----------------------------- */
rt_err_t ultrasonic_485_set_addr(uint8_t old_addr, uint8_t new_addr, uint32_t timeout_ms)
{
    if (old_addr < 1 || old_addr > 247 || new_addr < 1 || new_addr > 247)
        return -RT_EINVAL;
    if (timeout_ms == 0)
        timeout_ms = RS485_TIMEOUT_MS;

    uint8_t req[MODBUS_FRAME_MAX_LEN];
    uint8_t resp[MODBUS_RESP_MAX_LEN];
    uint16_t resp_len;
    rt_err_t ret;

    /* 构建写寄存器请求（功能码 0x06，寄存器地址 0x0200）*/
    req[0] = old_addr;
    req[1] = 0x06;                      /* 写单个寄存器 */
    req[2] = 0x02;                      /* 寄存器地址高字节 0x02 */
    req[3] = 0x00;                      /* 寄存器地址低字节 0x00 */
    req[4] = 0x00;                      /* 数据高字节（固定为0）*/
    req[5] = new_addr;                  /* 数据低字节 = 新地址 */
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    ret = modbus_send_recv(req, 8, resp, &resp_len, timeout_ms);
    if (ret != RT_EOK)
        return ret;

    /* 校验响应：应原样返回写请求（地址、功能码、寄存器地址、数据）*/
    if (resp_len < 8)
        return -RT_ERROR;
    if (resp[0] != old_addr || resp[1] != 0x06)
        return -RT_ERROR;
    if (resp[2] != 0x02 || resp[3] != 0x00)
        return -RT_ERROR;
    if (resp[4] != 0x00 || resp[5] != new_addr)
        return -RT_ERROR;

    /* 校验 CRC */
    uint16_t recv_crc = resp[resp_len-2] | (resp[resp_len-1] << 8);
    uint16_t calc_crc = modbus_crc16(resp, resp_len-2);
    if (recv_crc != calc_crc)
        return -RT_ERROR;

    return RT_EOK;
}

/* 修改传感器波特率（写寄存器 0x0201）*/
rt_err_t ultrasonic_485_set_baudrate(uint8_t addr, uint32_t baudrate, uint32_t timeout_ms)
{
	  uint8_t req[MODBUS_FRAME_MAX_LEN];
    uint8_t resp[MODBUS_RESP_MAX_LEN];
    uint16_t resp_len;
    rt_err_t ret;
    uint16_t baud_code;
    switch (baudrate)
    {
        case 4800:  baud_code = 0x0002; break;
        case 9600:  baud_code = 0x0003; break;
        case 14400: baud_code = 0x0004; break;
        case 19200: baud_code = 0x0005; break;
        case 38400: baud_code = 0x0006; break;
        case 57600: baud_code = 0x0007; break;
        case 76800: baud_code = 0x0008; break;
        case 115200: baud_code = 0x0009; break;
        default: return -RT_EINVAL;
    }
    req[0] = addr;
    req[1] = 0x06;               // 写单个寄存器
    req[2] = 0x02;               // 寄存器地址高字节 0x02
    req[3] = 0x01;               // 寄存器地址低字节 0x01
    req[4] = (baud_code >> 8) & 0xFF;
    req[5] = baud_code & 0xFF;
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    // 发送命令并等待响应（响应与请求相同）
		ret = modbus_send_recv(req, 8, resp, &resp_len, timeout_ms);
		if (ret != RT_EOK) {
				rt_kprintf("[485] set baudrate: send_recv failed, ret=%d\n", ret);
				return ret;
		}

		/* 打印收到的原始数据 */
		rt_kprintf("[485] set baudrate recv %d bytes: ", resp_len);
		for (uint16_t i = 0; i < resp_len; i++) {
				rt_kprintf("%02X ", resp[i]);
		}
		rt_kprintf("\n");

		/* 校验响应：应原样返回写请求（地址、功能码、寄存器地址、数据）*/
		if (resp_len < 8) {
				rt_kprintf("[485] resp_len < 8\n");
				return -RT_ERROR;
		}
		if (resp[0] != addr || resp[1] != 0x06) {
				rt_kprintf("[485] addr or func mismatch: %02X %02X\n", resp[0], resp[1]);
				return -RT_ERROR;
		}
		if (resp[2] != 0x02 || resp[3] != 0x01) {
				rt_kprintf("[485] register addr mismatch: %02X %02X\n", resp[2], resp[3]);
				return -RT_ERROR;
		}
		if (resp[4] != ((baud_code >> 8) & 0xFF) || resp[5] != (baud_code & 0xFF)) {
				rt_kprintf("[485] data mismatch: %02X %02X\n", resp[4], resp[5]);
				return -RT_ERROR;
		}

		/* 校验 CRC */
		uint16_t recv_crc = resp[resp_len-2] | (resp[resp_len-1] << 8);
		uint16_t calc_crc = modbus_crc16(resp, resp_len-2);
		if (recv_crc != calc_crc) {
				rt_kprintf("[485] CRC mismatch: recv=%04X calc=%04X\n", recv_crc, calc_crc);
				return -RT_ERROR;
		}

		rt_kprintf("[485] baudrate set successfully\n");
		return RT_EOK;
}

/* 设置传感器量程等级（1~5）*/
rt_err_t ultrasonic_485_set_range(uint8_t addr, uint8_t level, uint32_t timeout_ms)
{
    if (level < 1 || level > 5) return -RT_EINVAL;
    uint16_t range_code = level;  // 寄存器值直接对应等级
	  uint8_t req[MODBUS_FRAME_MAX_LEN];
    uint8_t resp[MODBUS_RESP_MAX_LEN];
    uint16_t resp_len;
    rt_err_t ret;
    req[0] = addr;
    req[1] = 0x06;
    req[2] = 0x02;
    req[3] = 0x1F;          // 量程等级寄存器 0x021F
    req[4] = 0x00;
    req[5] = range_code;
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;
    // ... 发送并校验响应 ...
	    // 发送命令并等待响应（响应与请求相同）
		ret = modbus_send_recv(req, 8, resp, &resp_len, timeout_ms);
		if (ret != RT_EOK) {
				rt_kprintf("[485] set range: send_recv failed, ret=%d\n", ret);
				return ret;
		}

		/* 打印收到的原始数据 */
		rt_kprintf("[485] set range recv %d bytes: ", resp_len);
		for (uint16_t i = 0; i < resp_len; i++) {
				rt_kprintf("%02X ", resp[i]);
		}
		rt_kprintf("\n");

		/* 校验响应：应原样返回写请求（地址、功能码、寄存器地址、数据）*/
		if (resp_len < 8) {
				rt_kprintf("[485] resp_len < 8\n");
				return -RT_ERROR;
		}
		if (resp[0] != addr || resp[1] != 0x06) {
				rt_kprintf("[485] addr or func mismatch: %02X %02X\n", resp[0], resp[1]);
				return -RT_ERROR;
		}
		if (resp[2] != 0x02 || resp[3] != 0x1F) {
				rt_kprintf("[485] register addr mismatch: %02X %02X\n", resp[2], resp[3]);
				return -RT_ERROR;
		}
		if (resp[4] != 0x00 || resp[5] != range_code) {
				rt_kprintf("[485] data mismatch: %02X %02X\n", resp[4], resp[5]);
				return -RT_ERROR;
		}

		/* 校验 CRC */
		uint16_t recv_crc = resp[resp_len-2] | (resp[resp_len-1] << 8);
		uint16_t calc_crc = modbus_crc16(resp, resp_len-2);
		if (recv_crc != calc_crc) {
				rt_kprintf("[485] CRC mismatch: recv=%04X calc=%04X\n", recv_crc, calc_crc);
				return -RT_ERROR;
		}

		rt_kprintf("[485] range set successfully\n");
		return RT_EOK;
}


/* ----------------------------- 驱动初始化 ----------------------------- */
rt_err_t ultrasonic_485_init(void)
{
    /* 配置方向控制引脚 */
    rt_pin_mode(RS485_DIR_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RS485_DIR_PIN, PIN_LOW);   /* 默认接收模式 */

    /* 查找串口设备 */
    serial = rt_device_find(RS485_UART_NAME);
    if (serial == RT_NULL)
    {
        rt_kprintf("[485] Cannot find %s\n", RS485_UART_NAME);
        return -RT_ERROR;
    }
		
		 // 从 Flash 加载保存的波特率
     uint32_t saved_baud = monitor_get_ultrasonic_baudrate();
    if (saved_baud >= 4800 && saved_baud <= 115200) {
        current_baudrate = saved_baud;
        rt_kprintf("[485] Loaded baudrate from monitor config: %d\n", current_baudrate);
    } else {
        current_baudrate = BAUD_RATE_115200;
        rt_kprintf("[485] Using default baudrate: %d\n", current_baudrate);
    }		

    /* 配置串口参数：115200, 8N1 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = current_baudrate;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity = PARITY_NONE;
		config.rx_bufsz = 64;   // 接收缓冲区大小
		config.tx_bufsz = 64;   // 发送缓冲区大小（若不需要大缓冲区，也可设为较小值）
		config.parity    = PARITY_NONE; 
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* 打开串口（阻塞读写，非 DMA）*/
    rt_err_t ret = rt_device_open(serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_RX_NON_BLOCKING);
    if (ret != RT_EOK)
    {
        rt_kprintf("[485] Open %s failed\n", RS485_UART_NAME);
        return ret;
    }		
		 /* 可选：读取配置验证波特率是否设置成功 */
    struct serial_configure verify_config;
    rt_device_control(serial, RT_SERIAL_CTRL_GET_CONFIG, &verify_config);
    rt_kprintf("[485] UART configured: baud=%d, data=%d, stop=%d, parity=%d\n",
               verify_config.baud_rate, verify_config.data_bits,
               verify_config.stop_bits, verify_config.parity);

    /* 初始化互斥锁 */
    rt_mutex_init(&rs485_mutex, "rs485_mtx", RT_IPC_FLAG_PRIO);
		rt_mutex_init(&s_dist_mutex, "ult_dist", RT_IPC_FLAG_PRIO);
		
		ultrasonic_485_start_poll();

    rt_kprintf("[485] RS485 ultrasonic driver initialized\n");
    return RT_EOK;
}
/* 重新配置 UART 波特率（不重新初始化互斥锁等）*/
static rt_err_t ultrasonic_485_reconfig_uart(uint32_t new_baudrate)
{
    if (serial == RT_NULL) {
        rt_kprintf("[485] Serial device not initialized\n");
        return -RT_ERROR;
    }

    /* 停止轮询（如果正在运行）*/
    uint8_t was_running = poll_running;
		if (was_running) {
				ultrasonic_485_stop_poll();
		}

    /* 关闭串口设备 */
    rt_device_close(serial);

    /* 配置新波特率 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = new_baudrate;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity = PARITY_NONE;
    config.rx_bufsz = 64;
    config.tx_bufsz = 64;
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* 重新打开串口 */
    rt_err_t ret = rt_device_open(serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_RX_NON_BLOCKING);
    if (ret != RT_EOK) {
        rt_kprintf("[485] Reopen UART failed\n");
        return ret;
    }

    /* 更新当前波特率 */
    current_baudrate = new_baudrate;

    /* 如果之前轮询在运行，则重新启动 */
    if (was_running) {
        ultrasonic_485_start_poll();
    }

    rt_kprintf("[485] UART baudrate switched to %d\n", new_baudrate);
    return RT_EOK;
}

/* ----------------------------- MSH 命令（手动测试、轮询控制） ----------------------------- */
#ifdef RT_USING_MSH
#include <stdlib.h>
static void ult_485_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage:\n");
        rt_kprintf("  ult_485_cmd <addr>          - read one sensor (addr 1-7)\n");
        rt_kprintf("  ult_485_cmd poll start      - start polling all sensors\n");
        rt_kprintf("  ult_485_cmd poll stop       - stop polling\n");
        return;
    }

    if (rt_strcmp(argv[1], "poll") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("Usage: ult_485_cmd poll start/stop\n");
            return;
        }
        if (rt_strcmp(argv[2], "start") == 0)
            ultrasonic_485_start_poll();
        else if (rt_strcmp(argv[2], "stop") == 0)
            ultrasonic_485_stop_poll();
        else
            rt_kprintf("Invalid poll command\n");
        return;
    }

    /* 读取单个传感器 */
    uint8_t addr = (uint8_t)atoi(argv[1]);
    if (addr < 1 || addr > ULTRASONIC_485_NUM)
    {
        rt_kprintf("Address must be 1~%d\n", ULTRASONIC_485_NUM);
        return;
    }
    uint32_t dist;
    rt_err_t ret = ultrasonic_485_read_distance(addr, &dist, RS485_TIMEOUT_MS);
		if (ret == RT_EOK) {
				rt_kprintf("Sensor %d distance: %d mm\n", addr, dist);
		} else if (ret == -RT_ENOOBJECT) {
				rt_kprintf("Sensor %d: no object detected (out of range)\n", addr);
		} else if (ret == -RT_EINTERFERE) {
				rt_kprintf("Sensor %d: interference\n", addr);
		} else if (ret == -RT_ETIMEOUT) {
				rt_kprintf("Sensor %d: communication timeout\n", addr);
		} else {
				rt_kprintf("Sensor %d read failed (err=%d)\n", addr, ret);
		}
}
MSH_CMD_EXPORT(ult_485_cmd, "ult_485_cmd [addr] or ult_485_cmd poll start/stop");

static void ult_485_setaddr(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("Usage: ult_485_setaddr <old_addr> <new_addr>\n");
        rt_kprintf("  old_addr : current address of the sensor (1-247)\n");
        rt_kprintf("  new_addr : new address to set (1-247, unique in bus)\n");
        rt_kprintf("Example: ult_485_setaddr 1 2   (change address from 1 to 2)\n");
        return;
    }

    uint8_t old_addr = (uint8_t)atoi(argv[1]);
    uint8_t new_addr = (uint8_t)atoi(argv[2]);

    if (old_addr < 1 || old_addr > 247 || new_addr < 1 || new_addr > 247)
    {
        rt_kprintf("Address must be between 1 and 247\n");
        return;
    }

    rt_err_t ret = ultrasonic_485_set_addr(old_addr, new_addr, RS485_TIMEOUT_MS);
    if (ret == RT_EOK)
        rt_kprintf("Success: sensor address changed from %d to %d\n", old_addr, new_addr);
    else
        rt_kprintf("Failed to set address (err=%d). Check connection and old_addr.\n", ret);
}
MSH_CMD_EXPORT(ult_485_setaddr, "ult_485_setaddr <old_addr> <new_addr> - change Modbus address of a sensor");

static void ult_485_setbaud(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("Usage: ult_485_setbaud <addr> <baudrate>\n");
        rt_kprintf("  addr     : sensor address (1-247)\n");
        rt_kprintf("  baudrate : 4800, 9600, 14400, 19200, 38400, 57600, 76800, 115200\n");
        rt_kprintf("Example: ult_485_setbaud 1 9600\n");
        return;
    }

    uint8_t addr = (uint8_t)atoi(argv[1]);
    uint32_t baudrate = atoi(argv[2]);

    if (addr < 1 || addr > 247)
    {
        rt_kprintf("Invalid address (1-247)\n");
        return;
    }

    rt_err_t ret = ultrasonic_485_set_baudrate(addr, baudrate, RS485_TIMEOUT_MS);
    if (ret == RT_EOK)
        rt_kprintf("Sensor %d baudrate set to %d successfully\n", addr, baudrate);
    else
        rt_kprintf("Failed to set baudrate (err=%d)\n", ret);
}
MSH_CMD_EXPORT(ult_485_setbaud, "ult_485_setbaud <addr> <baudrate> - change Modbus baudrate of a sensor");

static void ult_485_setrange(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("Usage: ult_485_setrange <addr> <level>\n");
        rt_kprintf("  addr   : sensor address (1-247)\n");
        rt_kprintf("  level  : range level (1-5)\n");
        rt_kprintf("    1: about 50cm, response time 15~80ms (real) / 190~500ms (processed)\n");
        rt_kprintf("    2: about 150cm\n");
        rt_kprintf("    3: about 250cm\n");
        rt_kprintf("    4: about 350cm\n");
        rt_kprintf("    5: about 500cm (default)\n");
        rt_kprintf("Example: ult_485_setrange 1 2\n");
        return;
    }

    uint8_t addr = (uint8_t)atoi(argv[1]);
    uint8_t level = (uint8_t)atoi(argv[2]);

    if (addr < 1 || addr > 247)
    {
        rt_kprintf("Invalid address (1-247)\n");
        return;
    }
    if (level < 1 || level > 5)
    {
        rt_kprintf("Invalid level (1-5)\n");
        return;
    }

    rt_err_t ret = ultrasonic_485_set_range(addr, level, RS485_TIMEOUT_MS);
    if (ret == RT_EOK)
        rt_kprintf("Sensor %d range level set to %d successfully\n", addr, level);
    else
        rt_kprintf("Failed to set range (err=%d)\n", ret);
}
MSH_CMD_EXPORT(ult_485_setrange, "ult_485_setrange <addr> <level> - set range level (1-5)");

static void ult_485_switch_baud(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: ult_485_switch_baud <baudrate>\n");
        rt_kprintf("  baudrate: 4800, 9600, 14400, 19200, 38400, 57600, 76800, 115200\n");
        rt_kprintf("Example: ult_485_switch_baud 57600\n");
        return;
    }
    uint32_t new_baud = atoi(argv[1]);
    /* 支持所有标准 Modbus 波特率 */
    if (new_baud != 4800 && new_baud != 9600 && new_baud != 14400 && 
        new_baud != 19200 && new_baud != 38400 && new_baud != 57600 && 
        new_baud != 76800 && new_baud != 115200) {
        rt_kprintf("Unsupported baudrate. Use: 4800, 9600, 14400, 19200, 38400, 57600, 76800, 115200\n");
        return;
    }
    if (new_baud == current_baudrate) {
        rt_kprintf("Already at %d baud\n", new_baud);
        return;
    }
    rt_err_t ret = ultrasonic_485_reconfig_uart(new_baud);
    if (ret != RT_EOK) {
        rt_kprintf("Failed to switch baudrate\n");
    }
		rt_kprintf("Baudrate switched to %d. Use 'ult_485_save_baud' to store permanently.\n", new_baud);
}
MSH_CMD_EXPORT(ult_485_switch_baud, "switch UART4 baudrate dynamically");

static void ult_485_stats(int argc, char **argv)
{
    if (read_stats.count == 0) {
        rt_kprintf("No successful reads yet.\n");
        return;
    }
    uint32_t avg_ms = read_stats.total_ms / read_stats.count;
    rt_kprintf("=== RS485 Ultrasonic Read Statistics ===\n");
    rt_kprintf("Successful reads : %u\n", read_stats.count);
    rt_kprintf("Total time       : %u ms\n", read_stats.total_ms);
    rt_kprintf("Average time     : %u ms\n", avg_ms);
    rt_kprintf("Min time         : %u ms\n", read_stats.min_ms);
    rt_kprintf("Max time         : %u ms\n", read_stats.max_ms);
}
MSH_CMD_EXPORT(ult_485_stats, "display read statistics");

static void ult_485_reset_stats(int argc, char **argv)
{
    memset(&read_stats, 0, sizeof(read_stats));
    rt_kprintf("Statistics reset.\n");
}
MSH_CMD_EXPORT(ult_485_reset_stats, "reset read statistics");

static void ult_485_print_time(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: ult_485_print_time on/off\n");
        return;
    }
    if (rt_strcmp(argv[1], "on") == 0) {
        print_each_time = 1;
        rt_kprintf("Print each read time enabled.\n");
    } else if (rt_strcmp(argv[1], "off") == 0) {
        print_each_time = 0;
        rt_kprintf("Print each read time disabled.\n");
    } else {
        rt_kprintf("Invalid argument. Use 'on' or 'off'.\n");
    }
}
MSH_CMD_EXPORT(ult_485_print_time, "ult_485_print_time on/off - enable/disable per-read time printing");

/* 开启/关闭打印 */
static void ult_485_print(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Current print status: %s\n", poll_print_enabled ? "on" : "off");
        rt_kprintf("Usage: ult_485_print on/off\n");
        return;
    }
    if (rt_strcmp(argv[1], "on") == 0) {
        poll_print_enabled = 1;
        rt_kprintf("Poll printing enabled\n");
    } else if (rt_strcmp(argv[1], "off") == 0) {
        poll_print_enabled = 0;
        rt_kprintf("Poll printing disabled\n");
    } else {
        rt_kprintf("Invalid argument. Use 'on' or 'off'.\n");
    }
}
MSH_CMD_EXPORT(ult_485_print, "enable/disable poll data printing");

/* 切换发送模式 */
static void ult_485_sendmode(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Current send mode: %s\n",
                   current_send_mode == SEND_MODE_PER_SENSOR ? "per_sensor" : "per_round");
        rt_kprintf("Usage: ult_485_sendmode [per_sensor|per_round]\n");
        return;
    }
    if (rt_strcmp(argv[1], "per_sensor") == 0) {
        current_send_mode = SEND_MODE_PER_SENSOR;
        rt_kprintf("Send mode changed to: per sensor (immediate)\n");
    } else if (rt_strcmp(argv[1], "per_round") == 0) {
        current_send_mode = SEND_MODE_PER_ROUND;
        rt_kprintf("Send mode changed to: per round (batch)\n");
    } else {
        rt_kprintf("Invalid mode. Choose 'per_sensor' or 'per_round'.\n");
    }
}
MSH_CMD_EXPORT(ult_485_sendmode, "set data send mode");

static void ult_485_save_baud(int argc, char **argv)
{
    extern void monitor_save_calibration(void);
    monitor_save_calibration();
    rt_kprintf("Current ultrasonic baudrate saved to Flash (via monitor system).\n");
}
MSH_CMD_EXPORT(ult_485_save_baud, "save current baudrate to flash (using monitor partition)");
#endif /* RT_USING_MSH */

