/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief RS485 超声波传感器驱动 - Modbus RTU 协议实现
 */

#include <rtthread.h>
#include "rs485_ultrasonic.h"
#include "global_conf.h"


/**
 * @brief RS485 对象实例
 */
static Rs485UltrasonicObject_t s_us_obj;

/**
 * @brief UART RX 接收线程句柄
 */
static rt_thread_t s_rx_thread = RT_NULL;

/**
 * @brief 计算 Modbus CRC16 校验码
 */
static uint16_t modbus_crc16(const rt_uint8_t* data, rt_uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (rt_uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;  // Modbus polynomial
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}


/**
 * @brief 构建 Modbus 读保持寄存器请求帧
 */
static void build_read_request(uint8_t slave_addr, uint16_t reg_addr, 
                               uint16_t reg_count, rt_uint8_t* tx_frame)
{
    uint16_t crc;
    
    tx_frame[0] = slave_addr;           // 从机地址
    tx_frame[1] = 0x03;                 // 功能码 03(读保持寄存器)
    tx_frame[2] = (reg_addr >> 8) & 0xFF;  // 起始地址高字节
    tx_frame[3] = reg_addr & 0xFF;      // 起始地址低字节
    tx_frame[4] = (reg_count >> 8) & 0xFF; // 寄存器数量高字节
    tx_frame[5] = reg_count & 0xFF;     // 寄存器数量低字节
    
    crc = modbus_crc16(tx_frame, 6);
    tx_frame[6] = crc & 0xFF;           // CRC 低字节
    tx_frame[7] = (crc >> 8) & 0xFF;    // CRC 高字节
}


/**
 * @brief 解析 Modbus 响应帧
 */
static int parse_response(uint8_t slave_addr, const rt_uint8_t* rx_frame, 
                          uint16_t rx_len, UltrasonicChannel_t* channels_out)
{
    /* 最小有效帧长：1(地址)+1(功能)+2(数据长度)+2(距离数据)*N+2(CRC) = 5+N*2 */
    if (rx_len < 5)
    {
        return -RT_ERROR;
    }
    
    /* 验证从机地址 */
    if (rx_frame[0] != slave_addr)
    {
        return -RT_ERROR;
    }
    
    /* 验证功能码 */
    if (rx_frame[1] != 0x03)
    {
        return -RT_ERROR;
    }
    
    /* 验证数据长度字段 */
    rt_uint8_t byte_count = rx_frame[2];
    if (byte_count + 4 != rx_len)  /* +4: 地址 + 功能码 + 字节数 + CRC */
    {
        return -RT_ERROR;
    }
    
    /* 计算预期数据点数 */
    rt_uint8_t num_registers = byte_count / 2;
    
    /* 提取距离数据（假设每个传感器有 7 路通道）*/
    rt_uint8_t data_idx = 3;
    for (uint8_t ch = 0; ch < MAX_US_CHANNELS && ch < num_registers; ch++)
    {
        uint16_t distance_raw = (rx_frame[data_idx] << 8) | rx_frame[data_idx + 1];
        
        if (channels_out)
        {
            channels_out[ch].distance_mm = distance_raw;
            channels_out[ch].valid = (distance_raw < 0xFFF0);  /* 0xFFF0 以上视为无效 */
            channels_out[ch].overload = (distance_raw == 0x0000);
            channels_out[ch].invalid = (distance_raw > 0xFFF0);
        }
        
        data_idx += 2;
    }
    
    return 0;
}


/**
 * @brief UART TX 发送函数
 */
static int us_uart_send(rt_uint8_t* data, uint16_t length)
{
    if (s_us_obj.uart_dev == NULL || !rt_device_is_available(s_us_obj.uart_dev))
    {
        return -RT_ERROR;
    }
    
    rt_err_t err = rt_device_write(s_us_obj.uart_dev, 0, data, length);
    return (err == RT_EOK) ? 0 : -RT_ERROR;
}


/**
 * @brief UART RX 中断回调
 */
static void us_uart_rx_int_cb(struct rt_device* dev, rt_size_t size)
{
    /* 信号量释放通知接收完成 */
    rt_sem_release(&s_us_obj.rx_sem);
}


/**
 * @brief UART RX 处理线程
 */
static void us_rx_thread_entry(void* parameter)
{
    while (1)
    {
        /* 等待数据到达信号量（带超时）*/
        if (rt_sem_take(&s_us_obj.rx_sem, READ_TIMEOUT_MS) == RT_EOK)
        {
            /* 这里可以扩展为环形缓冲区自动接收 */
        }
        
        rt_thread_mdelay(10);  // 防止信号量丢失
    }
}


/* ========== 公共 API 实现 ========== */

int rs485_us_init(void)
{
    rt_err_t err;
    
    /* 1. 查找 UART 设备 */
    s_us_obj.uart_dev = rt_device_find(ULTRASONIC_UART_BUS);
    if (s_us_obj.uart_dev == NULL)
    {
        rt_kprintf("[RS485] UART device not found: %s\n", ULTRASONIC_UART_BUS);
        return -RT_ERROR;
    }
    
    /* 2. 配置 UART 参数 */
    struct rt_serial_config config = 
    {
        .baud_rate = MODBUS_BAUDRATE,
        .data_bits = MODBUS_DATAWIDTH,
        .stop_bits = MODBUS_STOPBITS,
        .parity = MODBUS_PARITY,
        .flow_ctrl = RT_FLOW_CONTROL_NONE,
    };
    
    err = rt_device_control(s_us_obj.uart_dev, RT_DEVICE_CTRL_SERIAL_CFG, &config);
    if (err != RT_EOK)
    {
        rt_kprintf("[RS485] UART configure failed!\n");
        return -RT_ERROR;
    }
    
    /* 3. 打开设备 */
    err = rt_device_open(s_us_obj.uart_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_STREAM);
    if (err != RT_EOK)
    {
        rt_kprintf("[RS485] UART open failed!\n");
        return -RT_ERROR;
    }
    
    /* 4. 设置 RX 中断回调 */
    rt_device_set_rx_indicate(s_us_obj.uart_dev, us_uart_rx_int_cb);
    
    /* 5. 初始化数据结构 */
    memset(&s_us_obj, 0, sizeof(Rs485UltrasonicObject_t));
    s_us_obj.initialized = RT_TRUE;
    
    /* 6. 创建 RX 线程 */
    s_us_obj.rx_sem = RT_NULL;
    s_us_obj.rx_sem = rt_sem_create("us_rx", 0, RT_IPC_FLAG_PRIO);
    
    s_rx_thread = rt_thread_create("us_rx",
                                   us_rx_thread_entry,
                                   RT_NULL,
                                   1024,
                                   RT_THREAD_PRIORITY_MAX - 5,
                                   50);
    
    if (s_rx_thread != RT_NULL)
    {
        rt_thread_startup(s_rx_thread);
    }
    
    /* 7. 等待传感器就绪 */
    rt_thread_mdelay(SENSOR_READY_DELAY_MS);
    
    rt_kprintf("[RS485] RS485 ultrasonic system initialized on %s\n", ULTRASONIC_UART_BUS);
    
    return 0;
}


int rs485_us_read_all(rt_uint8_t* sensor_addr_list, rt_uint8_t count)
{
    rt_uint8_t success_count = 0;
    
    if (!s_us_obj.initialized)
    {
        return -RT_ERROR;
    }
    
    /* 确定扫描范围 */
    rt_uint8_t start_addr = 1;
    rt_uint8_t end_addr = 16;
    
    if (sensor_addr_list != NULL)
    {
        start_addr = sensor_addr_list[0];
        end_addr = start_addr + count - 1;
    }
    
    /* 逐个从机轮询 */
    for (uint8_t addr = start_addr; addr <= end_addr; addr++)
    {
        if (rs485_us_read_sensor(addr, s_us_obj.sensors[addr - 1].channels) == 0)
        {
            success_count++;
            s_us_obj.sensors[addr - 1].read_count++;
            
            rt_kprintf("[RS485] Slave %d OK: ", addr);
            for (int i = 0; i < 3; i++)
            {
                rt_kprintf("%dmm ", s_us_obj.sensors[addr - 1].channels[i].distance_mm);
            }
            rt_kprintf("\n");
        }
        else
        {
            s_us_obj.sensors[addr - 1].error_count++;
            s_us_obj.total_errors++;
            rt_kprintf("[RS485] Slave %d failed\n", addr);
        }
    }
    
    return success_count;
}


int rs485_us_read_sensor(uint8_t slave_addr, UltrasonicChannel_t* channels_out)
{
    if (!s_us_obj.initialized || slave_addr < US_SLAVE_ADDR_MIN || slave_addr > US_SLAVE_ADDR_MAX)
    {
        return -RT_ERROR;
    }
    
    /* 1. 构建 Modbus 请求帧 */
    rt_uint8_t tx_frame[8];
    build_read_request(slave_addr, REG_DISTANCE_1, MAX_US_CHANNELS, tx_frame);
    
    /* 2. 发送请求（RS485 半双工需要延时）*/
    us_uart_send(tx_frame, 8);
    
    /* 3. 等待响应（简单轮询方式，实际可用信号量）*/
    rt_uint16_t timeout = 100;
    while (timeout-- > 0)
    {
        /* 检查是否有数据可接收（简化版）*/
        rt_msleep(1);
        
        /* TODO: 添加 RX 缓冲区和接收逻辑 */
        /* 这里仅做演示，实际需要完整 Modbus RTU 接收状态机 */
        
        break;  /* 第一次执行时跳过 */
    }
    
    /* 
     * 注意：这里是一个简化实现！
     * 完整实现需要：
     * 1. UART RX 中断接收数据到环形缓冲区
     * 2. 状态机解析 Modbus 帧（地址、功能码、数据、CRC）
     * 3. 超时检测和重试机制
     * 4. RS485 DIR 引脚控制（TX 时拉高，RX 时拉低）
     */
    
    /* 模拟响应（实际应替换为真实接收逻辑）*/
    rt_uint8_t mock_rx[19];  /* 7 个寄存器×2 字节 + 头尾 */
    
    /* 填充模拟数据（仅测试用）*/
    mock_rx[0] = slave_addr;        /* 从机地址 */
    mock_rx[1] = 0x03;              /* 功能码 */
    mock_rx[2] = 0x0E;              /* 14 字节数据 */
    
    for (int i = 0; i < MAX_US_CHANNELS; i++)
    {
        mock_rx[3 + i*2] = 0xFF;
        mock_rx[4 + i*2] = 0xF0;
    }
    
    /* CRC 计算 */
    uint16_t crc = modbus_crc16(mock_rx, 17);
    mock_rx[17] = crc & 0xFF;
    mock_rx[18] = (crc >> 8) & 0xFF;
    
    /* 解析响应 */
    if (parse_response(slave_addr, mock_rx, sizeof(mock_rx), channels_out) == 0)
    {
        s_us_obj.total_reads++;
        return 0;
    }
    
    return -RT_ERROR;
}


uint16_t rs485_us_read_distance(uint8_t slave_addr, uint8_t channel_idx)
{
    if (channel_idx >= MAX_US_CHANNELS)
    {
        return 0xFFFF;
    }
    
    UltrasonicChannel_t channels[MAX_US_CHANNELS];
    
    if (rs485_us_read_sensor(slave_addr, channels) != 0)
    {
        return 0xFFFF;
    }
    
    return channels[channel_idx].distance_mm;
}


void rs485_us_get_latest(uint8_t slave_addr, uint8_t channel_idx, 
                         UltrasonicChannel_t* data_out)
{
    if (data_out && slave_addr >= 1 && slave_addr <= 16 && channel_idx < MAX_US_CHANNELS)
    {
        *data_out = s_us_obj.sensors[slave_addr - 1].channels[channel_idx];
    }
}


rt_bool_t rs485_us_ping(uint8_t slave_addr)
{
    if (slave_addr < US_SLAVE_ADDR_MIN || slave_addr > US_SLAVE_ADDR_MAX)
    {
        return RT_FALSE;
    }
    
    return (s_us_obj.sensors[slave_addr - 1].read_count > 0) ? RT_TRUE : RT_FALSE;
}


uint16_t rs485_us_get_online_status(void)
{
    uint16_t status = 0;
    
    for (uint8_t i = 0; i < 16; i++)
    {
        if (s_us_obj.sensors[i].read_count > 0)
        {
            status |= (1 << i);
        }
    }
    
    return status;
}


void rs485_us_reset_stats(void)
{
    memset(&s_us_obj.sensors, 0, sizeof(s_us_obj.sensors));
    s_us_obj.total_reads = 0;
    s_us_obj.total_errors = 0;
}


/* ========== MSH 调试命令 ========== */

#ifdef RT_USING_MSH

#include <msh.h>

MSH_CMD_DEFINE(us_show,   "us show",          "Show all ultrasonic sensor data")
MSH_CMD_DEFINE(us_test,   "us test [addr]",   "Test single sensor read")
MSH_CMD_DEFINE(us_scan,   "us scan",          "Scan for online sensors")

#endif /* RT_USING_MSH */

#ifdef RT_USING_MSH

static void msh_rs485_us_show(int argc, char** argv)
{
    rt_kprintf("\n========== RS485 Ultrasonic Data ==========\n");
    
    uint16_t online = rs485_us_get_online_status();
    
    if (online == 0)
    {
        rt_kprintf("No sensors detected! Check RS485 wiring.\n");
        return;
    }
    
    for (uint8_t addr = 1; addr <= 16; addr++)
    {
        if ((online >> (addr - 1)) & 0x01)
        {
            rt_kprintf("\nSlave %d (Online): ", addr);
            for (int i = 0; i < 7; i++)
            {
                uint16_t dist = s_us_obj.sensors[addr - 1].channels[i].distance_mm;
                if (dist < 0xFFF0)
                {
                    rt_kprintf("%dmm ", dist);
                }
                else
                {
                    rt_kprintf("--   ");
                }
            }
            rt_kprintf("\n");
            rt_kprintf("  Status: Reads=%lu, Errors=%lu\n",
                       s_us_obj.sensors[addr - 1].read_count,
                       s_us_obj.sensors[addr - 1].error_count);
        }
    }
    
    rt_kprintf("============================================\n\n");
}


static void msh_rs485_us_test(int argc, char** argv)
{
    uint8_t addr = 1;
    
    if (argc > 1)
    {
        addr = atoi(argv[1]);
    }
    
    if (addr < 1 || addr > 16)
    {
        rt_kprintf("[RS485] Invalid address: %d (must be 1~16)\n", addr);
        return;
    }
    
    rt_kprintf("[RS485] Testing slave %d...\n", addr);
    
    UltrasonicChannel_t channels[MAX_US_CHANNELS];
    
    if (rs485_us_read_sensor(addr, channels) == 0)
    {
        rt_kprintf("[RS485] Success!\n");
        for (int i = 0; i < 7; i++)
        {
            if (channels[i].valid)
            {
                rt_kprintf("  CH%d: %dmm (%s)\n", 
                           i + 1, 
                           channels[i].distance_mm,
                           channels[i].overload ? "OVERLOAD" : "OK");
            }
        }
    }
    else
    {
        rt_kprintf("[RS485] Failed to communicate with slave %d\n", addr);
        rt_kprintf("[RS485] Possible issues:\n");
        rt_kprintf("  1. RS485 wiring incorrect\n");
        rt_kprintf("  2. Power supply unstable\n");
        rt_kprintf("  3. Wrong Modbus address\n");
        rt_kprintf("  4. Baudrate mismatch (expected 9600)\n");
    }
}


static void msh_rs485_us_scan(int argc, char** argv)
{
    rt_kprintf("[RS485] Scanning for online sensors...\n");
    
    rt_uint8_t addrs[] = {1, 2, 3, 4, 5, 6, 7};
    int count = rs485_us_read_all(addrs, 7);
    
    rt_kprintf("[RS485] Found %d online sensors\n", count);
}

#endif /* RT_USING_MSH */
