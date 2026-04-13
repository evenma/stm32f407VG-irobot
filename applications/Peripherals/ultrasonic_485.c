#include "ultrasonic_485.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

/* ----------------------------- 硬件配置 ----------------------------- */
#define RS485_UART_NAME         "uart4"      /* 串口设备名称 */
#define RS485_DIR_PIN           GET_PIN(D, 0) /* PD0 方向控制引脚，高电平发送，低电平接收 */

/* ----------------------------- Modbus 参数 ----------------------------- */
#define MODBUS_FUNC_READ        0x03
#define MODBUS_REG_REAL_DIST    0x0101       /* 实时距离寄存器 */
#define MODBUS_READ_LEN         4            /* 读取响应数据长度（不含CRC）: 地址+功能+字节数+2字节数据 */
#define MODBUS_FRAME_MAX_LEN    8            /* 请求帧最大长度：地址(1)+功能(1)+寄存器地址(2)+寄存器数量(2)+CRC(2)=8 */
#define MODBUS_RESP_MAX_LEN     8            /* 响应帧最大长度：地址(1)+功能(1)+字节数(1)+数据(2)+CRC(2)=7，预留8 */

/* 默认超时时间（ms），需大于传感器最大响应时间（约140ms） */
#define DEFAULT_TIMEOUT_MS      200

/* ----------------------------- 静态变量 ----------------------------- */
static rt_device_t serial = RT_NULL;
static struct rt_mutex rs485_mutex;          /* 互斥锁，避免并发操作串口 */
static uint8_t poll_running = 0;
static rt_thread_t poll_thread = RT_NULL;

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
    while (rt_device_read(serial, 0, resp, MODBUS_RESP_MAX_LEN) > 0);

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
        timeout_ms = DEFAULT_TIMEOUT_MS;

    uint8_t req[MODBUS_FRAME_MAX_LEN];
    uint8_t resp[MODBUS_RESP_MAX_LEN];
    uint16_t resp_len;
    rt_err_t ret;

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
        return -RT_ETIMEOUT;
    if (dist_raw == 0xFFFE)   /* 同频干扰 */
        return -RT_ERROR;

    *distance_mm = dist_raw;
    return RT_EOK;
}

/* ----------------------------- 轮询线程（每 500ms 读取所有传感器） ----------------------------- */
static void poll_entry(void *parameter)
{
    uint32_t dist;
    rt_err_t ret;
    uint8_t i;
    char buf[128];
    int len;

    while (poll_running)
    {
        len = 0;
        for (i = 1; i <= ULTRASONIC_485_NUM; i++)
        {
            ret = ultrasonic_485_read_distance(i, &dist, DEFAULT_TIMEOUT_MS);
            if (ret == RT_EOK)
                len += rt_snprintf(buf + len, sizeof(buf) - len, "S%d=%dmm ", i, dist);
            else
                len += rt_snprintf(buf + len, sizeof(buf) - len, "S%d=ERR ", i);
            /* 传感器间隔延时，避免总线冲突（实际 RS485 是轮询，无需额外延时，但留一点处理时间）*/
            rt_thread_mdelay(10);
        }
        rt_kprintf("[485] %s\n", buf);
        /* 控制轮询周期约 500ms（扣除测量耗时后动态延时）*/
        rt_thread_mdelay(500);
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
                                    12,
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
        if (poll_thread)
        {
            rt_thread_delete(poll_thread);
            poll_thread = RT_NULL;
        }
    }
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

    /* 打开串口（阻塞读写，非 DMA）*/
    rt_err_t ret = rt_device_open(serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    if (ret != RT_EOK)
    {
        rt_kprintf("[485] Open %s failed\n", RS485_UART_NAME);
        return ret;
    }

    /* 配置串口参数：115200, 8N1 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_115200;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity = PARITY_NONE;
    config.bufsz = 64;   /* 接收缓冲区大小 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* 初始化互斥锁 */
    rt_mutex_init(&rs485_mutex, "rs485_mtx", RT_IPC_FLAG_PRIO);

    rt_kprintf("[485] RS485 ultrasonic driver initialized\n");
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
    rt_err_t ret = ultrasonic_485_read_distance(addr, &dist, 300);
    if (ret == RT_EOK)
        rt_kprintf("Sensor %d distance: %d mm\n", addr, dist);
    else
        rt_kprintf("Sensor %d read failed (err=%d)\n", addr, ret);
}
MSH_CMD_EXPORT(ult_485_cmd, "ult_485_cmd [addr] or ult_485_cmd poll start/stop");
#endif /* RT_USING_MSH */

