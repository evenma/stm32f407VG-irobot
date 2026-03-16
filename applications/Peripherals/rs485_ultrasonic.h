/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief RS485 超声波传感器驱动（Modbus RTU 协议）
 * 
 * 功能：
 *   - 通过 UART + RS485 收发模块通信
 *   - Modbus RTU 协议解析
 *   - 支持多路超声波测距（最多 16 路从机）
 */

#ifndef PERIPHERALS_RS485_ULTRASONIC_H__
#define PERIPHERALS_RS485_ULTRASONIC_H__

#include <rtthread.h>


/**
 * @brief RS485 接口配置 (根据 global_conf.h)
 */
#ifndef ULTRASONIC_UART_BUS
#define ULTRASONIC_UART_BUS    "uart3"  // PA10(TX), PA11(RX)
#endif

/**
 * @brief Modbus RTU 参数
 */
#define MODBUS_BAUDRATE        9600     // 波特率
#define MODBUS_PARITY          'N'      // 无校验 (N/O/E)
#define MODBUS_STOPBITS        1        // 停止位
#define MODBUS_DATAWIDTH       8        // 数据位

/**
 * @brief 超声波从机地址范围
 */
#define US_SLAVE_ADDR_MIN      1
#define US_SLAVE_ADDR_MAX      16


/**
 * @brief Modbus 寄存器地址定义（假设每个传感器有这些寄存器）
 */
#define REG_DISTANCE_1         0x00     // 第 1 路距离 (mm)
#define REG_DISTANCE_2         0x01     // 第 2 路距离 (mm)
#define REG_DISTANCE_N         0x00     // 起始地址，动态计算
#define REG_SENSOR_STATUS      0x10     // 传感器状态（温度、错误标志）

/**
 * @brief 最大支持的通道数
 */
#define MAX_US_CHANNELS        7        // iBed-body 使用 7 路超声波

/**
 * @brief 超时配置（单位：ms）
 */
#define READ_TIMEOUT_MS        50       // 读取单次响应超时
#define SENSOR_READY_DELAY_MS  100      // 上电后等待传感器就绪


/* ========== 数据结构 ========== */

/**
 * @brief 单路超声波数据
 */
typedef struct
{
    uint16_t distance_mm;   // 距离值 (单位：毫米)
    rt_bool_t valid;        // 数据有效性标记
    rt_bool_t overload;     // 超量程标志
    rt_bool_t invalid;      // 无效测量标志
} UltrasonicChannel_t;


/**
 * @brief 传感器数组
 */
typedef struct
{
    rt_uint8_t slave_addr;                      // 从机地址（Modbus ID）
    UltrasonicChannel_t channels[MAX_US_CHANNELS];
    uint32_t read_count;                        // 累计成功读取次数
    uint32_t error_count;                       // 累计错误次数
} UltrasonicSensor_t;


/**
 * @brief RS485 对象结构体
 */
typedef struct
{
    rt_bool_t initialized;          // 是否已初始化
    struct rt_device* uart_dev;     // UART 设备句柄
    struct rt_semaphore rx_sem;     // RX 接收完成信号量
    UltrasonicSensor_t sensors[16]; // 最多支持 16 个从机
    
    /* 内部缓冲区 */
    rt_uint8_t tx_buffer[256];      // 发送缓冲区
    rt_uint8_t rx_buffer[256];      // 接收缓冲区
    
    /* 统计信息 */
    uint32_t total_reads;
    uint32_t total_errors;
} Rs485UltrasonicObject_t;


/* ========== 公共 API ========== */

/**
 * @brief 初始化 RS485 超声波系统
 * @return 0 成功，负值失败
 */
int rs485_us_init(void);

/**
 * @brief 一次性读取所有传感器的距离数据
 * @param sensor_addr_list 从机地址列表，NULL 表示全部扫描
 * @param count 数量
 * @return 成功读取的从机数量
 */
int rs485_us_read_all(rt_uint8_t* sensor_addr_list, rt_uint8_t count);

/**
 * @brief 读取指定从机的所有通道数据
 * @param slave_addr 从机地址（1~16）
 * @param channels_out 输出指针，用于保存结果
 * @return 0 成功，负值失败
 */
int rs485_us_read_sensor(uint8_t slave_addr, UltrasonicChannel_t* channels_out);

/**
 * @brief 读取单个通道的距离值
 * @param slave_addr 从机地址
 * @param channel_idx 通道索引（0~6）
 * @return 距离值（单位：mm），无效时返回 0xFFFF
 */
uint16_t rs485_us_read_distance(uint8_t slave_addr, uint8_t channel_idx);

/**
 * @brief 获取指定通道最新数据（线程安全）
 * @param slave_addr 从机地址
 * @param channel_idx 通道索引
 * @param data_out 输出指针
 */
void rs485_us_get_latest(uint8_t slave_addr, uint8_t channel_idx, 
                         UltrasonicChannel_t* data_out);

/**
 * @brief 检查指定从机是否存在（ping）
 * @param slave_addr 从机地址
 * @return RT_TRUE 存在，RT_FALSE 不存在
 */
rt_bool_t rs485_us_ping(uint8_t slave_addr);

/**
 * @brief 获取传感器在线状态
 * @return 位掩码，bit[n] = 1 表示从机 n+1 在线
 */
uint16_t rs485_us_get_online_status(void);

/**
 * @brief 重置传感器错误计数
 */
void rs485_us_reset_stats(void);


/* ========== 调试工具 ========== */

#ifdef RT_USING_MSH

/**
 * @brief MSH 命令：显示所有传感器数据
 */
void msh_rs485_us_show(int argc, char** argv);

/**
 * @brief MSH 命令：测试读取单个从机
 */
void msh_rs485_us_test(int argc, char** argv);

/**
 * @brief MSH 命令：批量扫描（发现所有在线从机）
 */
void msh_rs485_us_scan(int argc, char** argv);

#endif /* RT_USING_MSH */


#endif /* PERIPHERALS_RS485_ULTRASONIC_H__ */
