/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief QMI8658C 6 轴 IMU 姿态传感器驱动
 * 
 * 功能：
 *   - 读取 3 轴加速度计 (+/- 4g/8g/16g)
 *   - 读取 3 轴陀螺仪 (+/- 2000°/s)
 *   - 计算俯仰角 (Pitch)、横滚角 (Roll)、航向角 (Yaw)
 */

#ifndef PERIPHERALS_QMI8658_H__
#define PERIPHERALS_QMI8658_H__

#include <rtthread.h>


/**
 * @brief I2C 接口定义 (根据 global_conf.h)
 */
#ifndef QMI8658_I2C_BUS
#define QMI8658_I2C_BUS    "i2c1"  // PB6(SCL), PB7(SDA)
#endif

/**
 * @brief QMI8658C 设备地址 (固定为 0x6B)
 */
#define QMI8658_SLAVE_ADDR     0x6B

/**
 * @brief 寄存器地址定义
 */
#define QMI8658_REG_WHOAMI     0x00
#define QMI8658_REG_DATA_CTRL1 0x01
#define QMI8658_REG_DATA_CTRL2 0x02
#define QMI8658_REG_GYRO_OUT_L 0x09
#define QMI8658_REG_ACC_OUT_L  0x0D
#define QMI8658_REG_STATUS     0x18

/**
 * @brief WHOAMI 返回值验证
 */
#define QMI8658_WHOAMI_VALUE   0x86

/**
 * @brief 加速度计量程配置
 *   0 = +/- 4g, 1 = +/- 8g, 2 = +/- 16g
 */
#define ACCEL_RANGE          2  // 16g (最大量程)

/**
 * @brief 陀螺仪量程配置
 *   0 = +/- 2000 deg/s (默认值，无法修改)
 */
#define GYRO_RANGE           2000  // 2000 deg/s

/**
 * @brief 输出数据速率配置
 *   0 = 100Hz, 1 = 200Hz, 2 = 500Hz, 3 = 1000Hz
 */
#define ODR_CONFIG           3  // 1000Hz 高速采样


/**
 * @brief 6 轴原始数据
 */
typedef struct
{
    int16_t acc_x;      // X 轴加速度
    int16_t acc_y;      // Y 轴加速度
    int16_t acc_z;      // Z 轴加速度
    
    int16_t gyro_x;     // X 轴角速度
    int16_t gyro_y;     // Y 轴角速度
    int16_t gyro_z;     // Z 轴角速度
} QmiDataRaw_t;


/**
 * @brief 解算后的姿态数据
 */
typedef struct
{
    float acc_x_g;      // X 轴加速度 (单位：g)
    float acc_y_g;      // Y 轴加速度 (单位：g)
    float acc_z_g;      // Z 轴加速度 (单位：g)
    
    float gyro_x_deg;   // X 轴角速度 (单位：度/秒)
    float gyro_y_deg;   // Y 轴角速度 (单位：度/秒)
    float gyro_z_deg;   // Z 轴角速度 (单位：度/秒)
    
    float pitch;        // 俯仰角 (-90 ~ 90 度)
    float roll;         // 横滚角 (-180 ~ 180 度)
    float yaw;          // 航向角 (未校准，漂移严重)
} QmiDataDecoded_t;


/**
 * @brief IMU 对象结构体
 */
typedef struct
{
    rt_bool_t initialized;    // 是否已初始化
    rt_uint8_t sample_count;  // 累计采样次数
    
    QmiDataRaw_t raw_data;    // 原始数据
    QmiDataDecoded_t decoded; // 解码数据
} Qmi8658Object_t;


/* ========== 公共 API ========== */

/**
 * @brief 初始化 QMI8658 传感器
 * @return 0 成功，负值失败
 */
int qmi8658_init(void);

/**
 * @brief 读取一次 6 轴数据并解算
 * @param raw_out 原始数据输出指针（可选）
 * @param decoded_out 解码数据输出指针（可选）
 * @return 0 成功，负值失败
 */
int qmi8658_read_once(QmiDataRaw_t* raw_out, QmiDataDecoded_t* decoded_out);

/**
 * @brief 连续读取 N 次数据并取平均（提高精度）
 * @param samples 采样次数
 * @param decoded_out 解码数据输出指针
 * @return 0 成功，负值失败
 */
int qmi8658_read_average(rt_uint8_t samples, QmiDataDecoded_t* decoded_out);

/**
 * @brief 获取最新姿态数据（线程安全）
 */
void qmi8658_get_latest(QmiDataDecoded_t* decoded_out);

/**
 * @brief 设置自动采样任务（后台运行）
 * @param freq_hz 采样频率 (100~1000 Hz)
 */
void qmi8658_start_continuous(uint16_t freq_hz);

/**
 * @brief 停止自动采样
 */
void qmi8658_stop_continuous(void);

/**
 * @brief 检查传感器是否就绪
 */
rt_bool_t qmi8658_is_ready(void);


/* ========== 调试工具 ========== */

#ifdef RT_USING_MSH

/**
 * @brief MSH 命令：手动读取一次数据
 */
void msh_qmi8658_read(int argc, char** argv);

/**
 * @brief MSH 命令：连续读取并打印 (stop 停止)
 */
void msh_qmi8658_stream(int argc, char** argv);

#endif /* RT_USING_MSH */


#endif /* PERIPHERALS_QMI8658_H__ */
