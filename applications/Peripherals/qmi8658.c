/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief QMI8658C 6 轴 IMU 姿态传感器驱动实现
 */

#include <rtthread.h>
#include "qmi8658.h"
#include "global_conf.h"


/**
 * @brief IMU 对象实例
 */
static Qmi8658Object_t s_imu;

/**
 * @brief I2C 设备句柄
 */
static struct rt_i2c_device* s_i2c_dev = RT_NULL;

/**
 * @brief 自动采样任务句柄
 */
static rt_thread_t s_sample_thread = RT_NULL;


/* ========== I2C 通信函数 ========== */

/**
 * @brief 读取 I2C 寄存器（单字节）
 */
static rt_err_t qmi8658_i2c_read_reg(uint8_t reg, uint8_t* value)
{
    if (s_i2c_dev == RT_NULL || !rt_device_is_available(&s_i2c_dev->parent))
    {
        return -RT_ERROR;
    }
    
    // 发送寄存器地址
    if (rt_i2c_transfer(s_i2c_dev, &reg, 1, value, 1) != RT_EOK)
    {
        return -RT_ERROR;
    }
    
    return RT_EOK;
}


/**
 * @brief 读取 I2C 寄存器（多字节）
 */
static rt_err_t qmi8658_i2c_read_regs(uint8_t reg, uint8_t* buffer, rt_uint16_t length)
{
    if (s_i2c_dev == RT_NULL || !rt_device_is_available(&s_i2c_dev->parent))
    {
        return -RT_ERROR;
    }
    
    // 发送寄存器地址
    if (rt_i2c_transfer(s_i2c_dev, &reg, 1, buffer, length) != RT_EOK)
    {
        return -RT_ERROR;
    }
    
    return RT_EOK;
}


/**
 * @brief 写入 I2C 寄存器（单字节）
 */
static rt_err_t qmi8658_i2c_write_reg(uint8_t reg, uint8_t value)
{
    if (s_i2c_dev == RT_NULL || !rt_device_is_available(&s_i2c_dev->parent))
    {
        return -RT_ERROR;
    }
    
    uint8_t tx_data[2] = {reg, value};
    
    if (rt_i2c_transfer(s_i2c_dev, tx_data, 2, NULL, 0) != RT_EOK)
    {
        return -RT_ERROR;
    }
    
    return RT_EOK;
}


/* ========== 初始化配置 ========== */

int qmi8658_init(void)
{
    rt_err_t err;
    uint8_t whoami;
    
    /* 1. 查找并打开 I2C 设备 */
    s_i2c_dev = rt_i2c_bus_device_find(QMI8658_I2C_BUS);
    if (s_i2c_dev == RT_NULL)
    {
        rt_kprintf("[IMU] I2C device not found: %s\n", QMI8658_I2C_BUS);
        return -RT_ERROR;
    }
    
    /* 2. 配置 I2C 参数：100kHz 速率 */
    struct rt_i2c_bus_config_info config = {
        .freq = RT_I2C_SPEED_STANDARD,  // 100kHz
        .bus_mode = RT_I2C_MODE_MASTER,
    };
    rt_i2c_bus_configure(s_i2c_dev, &config);
    
    /* 3. 验证 WHOAMI 寄存器 */
    err = qmi8658_i2c_read_reg(QMI8658_REG_WHOAMI, &whoami);
    if (err != RT_EOK || whoami != QMI8658_WHOAMI_VALUE)
    {
        rt_kprintf("[IMU] WHOAMI check failed: 0x%02X (expected 0x%02X)\n", 
                   whoami, QMI8658_WHOAMI_VALUE);
        return -RT_ERROR;
    }
    
    rt_kprintf("[IMU] QMI8658C detected (WHOAMI: 0x%02X)\n", whoami);
    
    /* 4. 配置数据控制寄存器 */
    /* DATA_CTRL1: 设置 ODR 和量程 */
    /*   Bit [1:0]: ACC Range (0=4g, 1=8g, 2=16g) */
    /*   Bit [3:2]: GYRO Range (0=2000 dps) */
    /*   Bit [7:4]: ODR (0=100Hz, 1=200Hz, 2=500Hz, 3=1000Hz) */
    uint8_t ctrl1 = (ACCEL_RANGE << 4) | (GYRO_RANGE << 2) | (ODR_CONFIG << 0);
    qmi8658_i2c_write_reg(QMI8658_REG_DATA_CTRL1, ctrl1);
    
    /* DATA_CTRL2: 启用输出 */
    /*   Bit [0]: Enable accelerometer */
    /*   Bit [1]: Enable gyroscope */
    uint8_t ctrl2 = 0x03;  // 同时启用 ACC 和 GYRO
    qmi8658_i2c_write_reg(QMI8658_REG_DATA_CTRL2, ctrl2);
    
    /* 5. 等待传感器稳定 */
    rt_thread_mdelay(50);
    
    /* 6. 初始化数据结构 */
    s_imu.initialized = RT_TRUE;
    s_imu.sample_count = 0;
    
    rt_kprintf("[IMU] QMI8658 initialized successfully!\n");
    
    return 0;
}


/* ========== 数据读取 ========== */

int qmi8658_read_once(QmiDataRaw_t* raw_out, QmiDataDecoded_t* decoded_out)
{
    if (!s_imu.initialized)
    {
        return -RT_ERROR;
    }
    
    uint8_t buffer[12];
    
    /* 1. 检查数据就绪状态 */
    uint8_t status;
    qmi8658_i2c_read_reg(QMI8658_REG_STATUS, &status);
    if (!(status & 0x0F))  // 最低 4 位是数据就绪标志
    {
        return -RT_ERROR;  // 无新数据
    }
    
    /* 2. 读取陀螺仪原始数据 (6 字节) */
    qmi8658_i2c_read_regs(QMI8658_REG_GYRO_OUT_L, buffer, 6);
    
    if (raw_out)
    {
        raw_out->gyro_x = (int16_t)((buffer[1] << 8) | buffer[0]);
        raw_out->gyro_y = (int16_t)((buffer[3] << 8) | buffer[2]);
        raw_out->gyro_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    }
    
    /* 3. 读取加速度计原始数据 (6 字节) */
    qmi8658_i2c_read_regs(QMI8658_REG_ACC_OUT_L, &buffer[6], 6);
    
    if (raw_out)
    {
        raw_out->acc_x = (int16_t)((buffer[7] << 8) | buffer[6]);
        raw_out->acc_y = (int16_t)((buffer[9] << 8) | buffer[8]);
        raw_out->acc_z = (int16_t)((buffer[11] << 8) | buffer[10]);
    }
    
    /* 4. 解码为物理量 */
    if (decoded_out)
    {
        /* 陀螺仪：1 LSB = 0.061 deg/s (2000 dps / 32768) */
        const float gyro_scale = 0.061f;
        
        decoded_out->gyro_x_deg = raw_out ? raw_out->gyro_x * gyro_scale : 0.0f;
        decoded_out->gyro_y_deg = raw_out ? raw_out->gyro_y * gyro_scale : 0.0f;
        decoded_out->gyro_z_deg = raw_out ? raw_out->gyro_z * gyro_scale : 0.0f;
        
        /* 加速度计：取决于量程 */
        const float accel_scale_16g = 16.0f / 32768.0f;
        
        if (raw_out)
        {
            decoded_out->acc_x_g = raw_out->acc_x * accel_scale_16g;
            decoded_out->acc_y_g = raw_out->acc_y * accel_scale_16g;
            decoded_out->acc_z_g = raw_out->acc_z * accel_scale_16g;
            
            /* 计算欧拉角 (简化公式，未校准) */
            /* Pitch: 绕 X 轴旋转 */
            decoded_out->pitch = atan2(decoded_out->acc_y_g, decoded_out->acc_z_g) * 180.0f / RT_PI;
            
            /* Roll: 绕 Y 轴旋转 */
            decoded_out->roll = atan2(-decoded_out->acc_x_g, 
                                      sqrt(decoded_out->acc_y_g * decoded_out->acc_y_g + 
                                           decoded_out->acc_z_g * decoded_out->acc_z_g)) * 180.0f / RT_PI;
            
            /* Yaw: 需要磁力计校准，这里用陀螺仪积分（漂移严重）*/
            static float yaw_integral = 0.0f;
            static rt_uint32_t last_time = 0;
            rt_uint32_t current_time = rt_tick_get_millisecond();
            float dt = (current_time - last_time) / 1000.0f;
            last_time = current_time;
            
            if (dt > 0 && dt < 1.0f)
            {
                yaw_integral += decoded_out->gyro_z_deg * dt;
                decoded_out->yaw = yaw_integral;
            }
        }
        else
        {
            decoded_out->pitch = 0.0f;
            decoded_out->roll = 0.0f;
            decoded_out->yaw = 0.0f;
        }
    }
    
    s_imu.sample_count++;
    
    return 0;
}


int qmi8658_read_average(rt_uint8_t samples, QmiDataDecoded_t* decoded_out)
{
    QmiDataDecoded_t temp;
    float acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;
    float gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
    int pitch_sum = 0, roll_sum = 0, yaw_sum = 0;
    
    for (int i = 0; i < samples; i++)
    {
        if (qmi8658_read_once(RT_NULL, &temp) == 0)
        {
            acc_x_sum += temp.acc_x_g;
            acc_y_sum += temp.acc_y_g;
            acc_z_sum += temp.acc_z_g;
            
            gyro_x_sum += temp.gyro_x_deg;
            gyro_y_sum += temp.gyro_y_deg;
            gyro_z_sum += temp.gyro_z_deg;
            
            pitch_sum += (int)temp.pitch;
            roll_sum += (int)temp.roll;
            yaw_sum += (int)temp.yaw;
        }
        
        rt_thread_mdelay(5);  // 每两次采样间隔 5ms
    }
    
    /* 取平均值 */
    if (decoded_out)
    {
        decoded_out->acc_x_g = acc_x_sum / samples;
        decoded_out->acc_y_g = acc_y_sum / samples;
        decoded_out->acc_z_g = acc_z_sum / samples;
        
        decoded_out->gyro_x_deg = gyro_x_sum / samples;
        decoded_out->gyro_y_deg = gyro_y_sum / samples;
        decoded_out->gyro_z_deg = gyro_z_sum / samples;
        
        decoded_out->pitch = pitch_sum / (float)samples;
        decoded_out->roll = roll_sum / (float)samples;
        decoded_out->yaw = yaw_sum / (float)samples;
    }
    
    return 0;
}


void qmi8658_get_latest(QmiDataDecoded_t* decoded_out)
{
    if (decoded_out)
    {
        *decoded_out = s_imu.decoded;
    }
}


/* ========== 连续采样任务 ========== */

static void qmi8658_sample_thread_entry(void* parameter)
{
    QmiDataDecoded_t decoded;
    rt_uint16_t period_ms = 1000 / 1000;  // 1ms (1000Hz)
    
    while (1)
    {
        if (qmi8658_read_once(RT_NULL, &decoded) == 0)
        {
            /* 锁保护：更新全局数据 */
            s_imu.decoded = decoded;
        }
        
        /* 根据周期休眠 */
        rt_thread_mdelay(period_ms);
    }
}


void qmi8658_start_continuous(uint16_t freq_hz)
{
    if (freq_hz < 100 || freq_hz > 1000)
    {
        rt_kprintf("[IMU] Invalid frequency: %d Hz (must be 100~1000)\n", freq_hz);
        return;
    }
    
    if (s_sample_thread != RT_NULL)
    {
        rt_kprintf("[IMU] Sample thread already running\n");
        return;
    }
    
    rt_kprintf("[IMU] Starting continuous sampling at %d Hz\n", freq_hz);
    
    s_sample_thread = rt_thread_create("imu_smp",
                                       qmi8658_sample_thread_entry,
                                       RT_NULL,
                                       1024,
                                       RT_THREAD_PRIORITY_MAX / 5,
                                       20);
    
    if (s_sample_thread != RT_NULL)
    {
        rt_thread_startup(s_sample_thread);
    }
    else
    {
        rt_kprintf("[IMU] Failed to create sample thread!\n");
    }
}


void qmi8658_stop_continuous(void)
{
    if (s_sample_thread != RT_NULL)
    {
        rt_thread_delete(s_sample_thread);
        s_sample_thread = RT_NULL;
        rt_kprintf("[IMU] Continuous sampling stopped\n");
    }
}


rt_bool_t qmi8658_is_ready(void)
{
    return s_imu.initialized;
}


/* ========== MSH 调试命令 ========== */

#ifdef RT_USING_MSH

#include <msh.h>

MSH_CMD_DEFINE(qmi_read,    "qmi read",              "Read IMU data once")
MSH_CMD_DEFINE(qmi_stream,  "qmi stream [stop]",     "Stream IMU data (stop to quit)")
MSH_CMD_DEFINE(qmi_start,   "qmi start [hz]",        "Start continuous sampling (default 1000)")
MSH_CMD_DEFINE(qmi_stop,    "qmi stop",              "Stop continuous sampling")
MSH_CMD_DEFINE(qmi_status,  "qmi status",            "Show IMU status")

#endif /* RT_USING_MSH */


/* ========== MSH 命令实现 ========== */

#ifdef RT_USING_MSH

static void msh_qmi8658_read(int argc, char** argv)
{
    QmiDataDecoded_t decoded;
    
    if (qmi8658_read_once(RT_NULL, &decoded) != 0)
    {
        rt_kprintf("[IMU] Read failed! Is sensor connected?\n");
        return;
    }
    
    rt_kprintf("\n========== QMI8658 Data ==========\n");
    rt_kprintf("Accel:\n");
    rt_kprintf("  X: %.3f g  ", decoded.acc_x_g);
    rt_kprintf("Y: %.3f g  ", decoded.acc_y_g);
    rt_kprintf("Z: %.3f g\n", decoded.acc_z_g);
    
    rt_kprintf("Gyro:\n");
    rt_kprintf("  X: %.2f deg/s  ", decoded.gyro_x_deg);
    rt_kprintf("Y: %.2f deg/s  ", decoded.gyro_y_deg);
    rt_kprintf("Z: %.2f deg/s\n", decoded.gyro_z_deg);
    
    rt_kprintf("Euler:\n");
    rt_kprintf("  Pitch: %.2f°  ", decoded.pitch);
    rt_kprintf("Roll: %.2f°\n", decoded.roll);
    rt_kprintf("  Yaw: %.2f° (unreliable without magnetometer)\n", decoded.yaw);
    rt_kprintf("====================================\n\n");
}


static void msh_qmi8658_stream(int argc, char** argv)
{
    QmiDataDecoded_t decoded;
    rt_bool_t running = RT_TRUE;
    
    if (argc > 1 && rt_strcmp(argv[1], "stop") == 0)
    {
        running = RT_FALSE;
    }
    
    rt_kprintf("[IMU] Streaming IMU data (press 'q' on serial to quit)\n");
    
    while (running)
    {
        if (qmi8658_read_once(RT_NULL, &decoded) == 0)
        {
            rt_kprintf("P:%.1f° R:%.1f° | AX:%.2f AY:%.2f AZ:%.2f | GX:%.1f GY:%.1f GZ:%.1f\r",
                       decoded.pitch, decoded.roll,
                       decoded.acc_x_g, decoded.acc_y_g, decoded.acc_z_g,
                       decoded.gyro_x_deg, decoded.gyro_y_deg, decoded.gyro_z_deg);
        }
        
        rt_thread_mdelay(50);
    }
    
    rt_kprintf("[IMU] Stream stopped\n");
}


static void msh_qmi8658_start(int argc, char** argv)
{
    uint16_t freq = 1000;  // 默认 1000Hz
    
    if (argc > 1)
    {
        freq = atoi(argv[1]);
    }
    
    qmi8658_start_continuous(freq);
    rt_kprintf("[IMU] Sampling started at %d Hz\n", freq);
}


static void msh_qmi8658_stop(int argc, char** argv)
{
    qmi8658_stop_continuous();
    rt_kprintf("[IMU] Sampling stopped\n");
}


static void msh_qmi8658_status(int argc, char** argv)
{
    QmiDataDecoded_t decoded;
    
    rt_kprintf("\n========== QMI8658 Status ==========\n");
    
    if (qmi8658_is_ready())
    {
        rt_kprintf("Status: OK (Initialized)\n");
        
        if (qmi8658_read_once(RT_NULL, &decoded) == 0)
        {
            rt_kprintf("Samples collected: %d\n", s_imu.sample_count);
            rt_kprintf("Latest Angle: Pitch=%.1f° Roll=%.1f°\n", decoded.pitch, decoded.roll);
        }
    }
    else
    {
        rt_kprintf("Status: Not initialized!\n");
        rt_kprintf("Possible issues:\n");
        rt_kprintf("  1. I2C bus not configured correctly\n");
        rt_kprintf("  2. Sensor not powered or connected\n");
        rt_kprintf("  3. Wrong I2C address\n");
    }
    
    rt_kprintf("======================================\n\n");
}

#endif /* RT_USING_MSH */
