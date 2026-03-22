/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief QMI8658C 6-axis IMU driver v8.0 - FOCUS ON DATA ACQUISITION
 * 
 * Architecture Changes:
 *   ✓ 下位机专注数据采集、通信和实时控制
 *   ✓ 姿态解算和校准任务移到上位机（树莓派 + ROS2）
 *   ✓ 默认不打印数据，通过 mSH 控制台开关定时打印
 *   ✓ 支持线程停止/重新开启
 * 
 * Critical Register Address Corrections per Official Datasheet:
 *   ✓ WHOAMI = 0x00 (QMI8658A = 0x05)
 *   ✓ REVISION_ID = 0x01
 *   ✓ CTRL1 = 0x02  ← ODR configuration
 *   ✓ CTRL2 = 0x03  ← ACC range/LPF/cal
 *   ✓ CTRL3 = 0x04  ← GYRO range/LPF/cal
 *   ✓ CTRL5 = 0x06  ← LPF modes
 *   ✓ CTRL7 = 0x08  ← Interrupt enable
 *   ✓ CTRL9 = 0x0A  ← Commands
 *   ✓ STATUS = 0x18 ← Data ready flags [3:0]
 *   ✓ DATA_READY = 0x2D ← Sensor Data Available Register
 *   ✓ TEMP_OUT = 0x33 ← Temperature sensor output
 *   ✓ ACC_OUT_L = 0x35 ← Accelerometer X LSB (burst start)
 *   ✓ GYR_OUT_L = 0x3B ← Gyroscope X LSB (burst start)
 *   ✓ RESET = 0x60 ← Reset register
 *
 * Hardware connections:
 *   - I2C1 (Hardware): PB6(SCL), PB7(SDA) @ 400kHz
 *   - INT2 (EXTI): PB5 (falling edge trigger)
 *   - Sensor address: 0x6B (ADR_PIN = VCC)
 */

#include <rtthread.h>
#include <rtdevice.h>      // 包含 rt_pin 相关函数和 GET_PIN 宏
#include <board.h>          // 提供 GPIO 端口枚举和 HAL 库引脚定义
#include "qmi8658.h"
#include "global_conf.h"
#include <math.h>

/**
 * @brief Internal QMI8658 object instance
 */
static struct QMI8658Object s_imu;

/**
 * @brief Hardware I2C device handle
 */
static struct rt_i2c_bus_device* s_i2c_dev = RT_NULL;

/**
 * @brief EXTI initialization flag
 */
static rt_bool_t s_exti_initialized = RT_FALSE;

/**
 * @brief Semaphore for interrupt-notification mechanism
 */
static struct rt_semaphore s_data_ready_sem;

/**
 * @brief IMU thread control
 */
static struct rt_thread s_imu_thread;
#define IMU_THREAD_STACK_SIZE     1024
#define IMU_THREAD_PRIORITY       20
static rt_uint8_t s_imu_thread_stack[IMU_THREAD_STACK_SIZE];

/**
 * @brief Thread control flag (支持停止/重新开启)
 */
static rt_bool_t s_thread_running = RT_FALSE;

/**
 * @brief Print control flag (打印控制开关)
 */
static rt_bool_t s_print_enabled = RT_FALSE;

/**
 * @brief Print interval (打印间隔，单位：毫秒)
 */
#define PRINT_INTERVAL_MS 100  // 100ms 打印一次

/**
 * @brief Print counter (打印计数器)
 */
static rt_uint32_t s_print_counter = 0;

/**
 * @brief Scale factors (dynamically calculated)
 */
//static float s_accel_scale = 0.000122f;   /* ±4g / 32768 = 0.000122 g/LSB */
//static float s_gyro_scale = 0.0625f;      /* ±2048 dps / 32768 = 0.0625 dps/LSB */

// 加速度 ODR 映射表（索引对应 aODR[3:0] 的值）
static const uint16_t acc_odr_hz[] = {
    8000, 4000, 2000, 1000, 500, 250, 125, 62, 31, 0, 0, 0, 128, 21, 11, 3
};

// 陀螺仪 ODR 映射表（索引对应 gODR[3:0] 的值，需根据手册确认）
static const uint16_t gyr_odr_hz[] = {
    7174, 3587, 1793, 896, 448, 224, 112, 56, 28, 0, 0, 0, 0, 0, 0, 0
};

/**
 * @brief 将浮点数格式化为字符串并打印
 * @param value 要打印的浮点数
 * @param width 总宽度（包含符号、整数、小数点和精度），若实际宽度不足则用空格填充（右对齐）
 * @param precision 小数位数（0-6）
 * @param sign 是否显示正号（1: 显示+号，0: 负数自动显示-号，正数不显示+）
 */
static void print_float(float value, int width, int precision, int sign)
{
    char buf[32];
    int integer_part, decimal_part;
    int negative = (value < 0);
    if (negative) value = -value;

    integer_part = (int)value;
    // 处理小数部分，防止浮点误差
    decimal_part = (int)((value - integer_part) * powf(10, precision) + 0.5f);
    if (decimal_part >= (int)powf(10, precision)) {
        decimal_part -= (int)powf(10, precision);
        integer_part++;
    }

    // 格式化为字符串
    int len = rt_snprintf(buf, sizeof(buf), "%s%d.%0*d",
                          (negative ? "-" : (sign ? "+" : "")),
                          integer_part,
                          precision,
                          decimal_part);
    // 右对齐输出（宽度控制）
    if (width > len) {
        for (int i = 0; i < width - len; i++)
            rt_kprintf(" ");
    }
    rt_kprintf("%s", buf);
}

/* ========== Hardware I2C Communication Functions ========== */

static rt_err_t imu_i2c_read_reg(uint8_t reg, uint8_t* value)
{
    struct rt_i2c_msg msgs[2];
    uint8_t buf = reg;

    msgs[0].addr  = QMI8658_SLAVE_ADDR_H;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &buf;
    msgs[0].len   = 1;

    msgs[1].addr  = QMI8658_SLAVE_ADDR_H;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = value;
    msgs[1].len   = 1;

		int ret = rt_i2c_transfer(s_i2c_dev, msgs, 2);
		if (ret != 2) {			
				return -RT_ERROR;
		}
		return RT_EOK;   
}

static rt_err_t imu_i2c_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    struct rt_i2c_msg msg;

    msg.addr  = QMI8658_SLAVE_ADDR_H;
    msg.flags = RT_I2C_WR;
    msg.buf   = buf;
    msg.len   = 2;

    if (rt_i2c_transfer(s_i2c_dev, &msg, 1) != 1) {
        return -RT_ERROR;
    }
    return RT_EOK;
}


static rt_err_t imu_i2c_read_regs(uint8_t reg, uint8_t* buffer, uint8_t length)
{
    struct rt_i2c_msg msgs[2];
    uint8_t buf = reg;

    msgs[0].addr  = QMI8658_SLAVE_ADDR_H;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &buf;
    msgs[0].len   = 1;

    msgs[1].addr  = QMI8658_SLAVE_ADDR_H;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = buffer;
    msgs[1].len   = length;

    if (rt_i2c_transfer(s_i2c_dev, msgs, 2) != 2) {
        return -RT_ERROR;
    }
    return RT_EOK;
}


static int16_t imu_i2c_read_word(uint8_t reg)
{
    uint8_t buffer[2];
    
    if(imu_i2c_read_regs(reg, buffer, 2) != RT_EOK)
    {
        return 0;
    }
    
    return (int16_t)((buffer[1] << 8) | buffer[0]);
}


/* ========== EXTI Interrupt Configuration ========== */

static void qmi8658_exti_callback(void *args)
{
    if(s_imu.initialized == RT_TRUE && s_thread_running == RT_TRUE)
    {
        rt_sem_release(&s_data_ready_sem);
    }
}


static rt_err_t imu_init_exti(void)
{
    rt_err_t err;
	
    if(s_exti_initialized == RT_TRUE)
        return RT_EOK;

		rt_pin_mode(IMU_INT_PIN, PIN_MODE_INPUT_PULLDOWN);
    err = rt_pin_attach_irq(IMU_INT_PIN, PIN_IRQ_MODE_RISING, qmi8658_exti_callback, (void *)"qmi8658_exti_callback");
    if(err != RT_EOK) {
        rt_kprintf("[QMI8658] attach irq failed\n");
        return err;
    }
    err = rt_pin_irq_enable(IMU_INT_PIN, PIN_IRQ_ENABLE);
    if(err != RT_EOK) {
        rt_kprintf("[QMI8658] enable irq failed\n");
        return err;
    }

    s_exti_initialized = RT_TRUE;
    rt_kprintf("[QMI8658] EXTI on PB5 initialized\n");
    return RT_EOK;
}


/* ========== Validation Functions ========== */

rt_err_t qmi8658_validate_connection(int max_attempts)
{
    uint8_t whoami = 0x00;
    int attempt = 0;
    
    rt_kprintf("[QMI8658] Validating connection (%d attempts)...\n", max_attempts);
    
    while(attempt < max_attempts)
    {
        imu_i2c_read_reg(QMI8658_REG_WHOAMI, &whoami);
        
        if(whoami == QMI8658_WHOAMI)
        {
            rt_kprintf("[QMI8658] Connection validated: WHOAMI=0x%02X (attempt %d/%d)\n", 
                      whoami, attempt + 1, max_attempts);
            return RT_EOK;
        }
        
        rt_kprintf("[QMI8658] Retry %d/%d: WHOAMI=0x%02X (expected 0x%02X)\n", 
                   attempt + 1, max_attempts, whoami, QMI8658_WHOAMI);
				imu_i2c_write_reg(QMI8658_REG_RESET, 0xB0);
        rt_thread_mdelay(10);
        attempt++;
    }
    
    rt_kprintf("[QMI8658] ERROR: Connection validation failed after %d attempts\n", max_attempts);
    return -RT_ERROR;
}


uint8_t qmi8658_read_revision(void)
{
    uint8_t rev_id = 0xFF;
    
    if(imu_i2c_read_reg(QMI8658_REG_REVISION, &rev_id) != RT_EOK)
    {
        rt_kprintf("[QMI8658] Failed to read REVISION_ID\n");
        return 0xFF;
    }
    
    return rev_id;
}


uint8_t imu_get_id(void)
{
    uint8_t whoami = 0x00;
    uint8_t retry = 0;
    
    while(retry++ < 5)
    {
        imu_i2c_read_reg(QMI8658_REG_WHOAMI, &whoami);
        if(whoami == QMI8658_WHOAMI)
            return whoami;
        rt_thread_mdelay(10);
    }
    
    return 0x00;
}

static const char* acc_range_str(enum qmi8658_AccRange range)
{
    switch(range)
    {
        case Qmi8658AccRange_2g:  return "±2g";
        case Qmi8658AccRange_4g:  return "±4g";
        case Qmi8658AccRange_8g:  return "±8g";
        case Qmi8658AccRange_16g: return "±16g";
        default:                   return "Unknown";
    }
}

void imu_config_acc(enum qmi8658_AccRange range, enum qmi8658_AccOdr odr,
                    enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    uint8_t ctrl_data;
	uint16_t acc_odr = acc_odr_hz[odr & 0x0F];
    
    switch(range)
    {
        case Qmi8658AccRange_2g:
            s_imu.ssvt_a = (1<<14);
            break;
        case Qmi8658AccRange_4g:
            s_imu.ssvt_a = (1<<13);
            break;
        case Qmi8658AccRange_8g:
            s_imu.ssvt_a = (1<<12);
            break;
        default:
            s_imu.ssvt_a = (1<<11);
            break;
    }
    
    ctrl_data = (uint8_t)range | (uint8_t)odr;
    if(stEnable == Qmi8658St_Enable)
        ctrl_data |= QMI8658_aST;
    
    /* Write to CTRL2 = 0x03 per official datasheet! */
    imu_i2c_write_reg(QMI8658_REG_CTRL2, ctrl_data);
    
    /* Configure LPF in CTRL5 = 0x06 */
    uint8_t ctrl5_val;
		imu_i2c_read_reg(QMI8658_REG_CTRL5, &ctrl5_val);
		ctrl_data = ctrl5_val;
		
    ctrl_data &= 0xf0;
    if(lpfEnable == Qmi8658Lpf_Enable)
    {
        ctrl_data |= A_LSP_MODE_3;
        ctrl_data |= 0x01;
    }	else	{
			ctrl_data &= ~0x01;
		}
    imu_i2c_write_reg(QMI8658_REG_CTRL5, ctrl_data);
    
    rt_kprintf("[QMI8658] ACC configured: %s @ %dHz, scale=%d/1000000 \n",
               acc_range_str(range),
               acc_odr,
               (int)(1000000/s_imu.ssvt_a));
}

static const char* gyro_range_str(enum qmi8658_GyrRange range)
{
    switch(range)
    {
        case Qmi8658GyrRange_16dps:   return "±16dps";
        case Qmi8658GyrRange_32dps:   return "±32dps";
        case Qmi8658GyrRange_64dps:   return "±64dps";
        case Qmi8658GyrRange_128dps:  return "±128dps";
        case Qmi8658GyrRange_256dps:  return "±256dps";
        case Qmi8658GyrRange_512dps:  return "±512dps";
        case Qmi8658GyrRange_1024dps: return "±1024dps";
        case Qmi8658GyrRange_2048dps: return "±2048dps";
        default:                       return "Unknown";
    }
}

void imu_config_gyro(enum qmi8658_GyrRange range, enum qmi8658_GyrOdr odr,
                     enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    uint8_t ctrl_data;
    uint16_t gyr_odr = gyr_odr_hz[odr & 0x0F];
	
    switch(range)
    {
        case Qmi8658GyrRange_16dps:
            s_imu.ssvt_g = 2048;
            break;
        case Qmi8658GyrRange_32dps:
            s_imu.ssvt_g = 1024;
            break;
        case Qmi8658GyrRange_64dps:
            s_imu.ssvt_g = 512;
            break;
        case Qmi8658GyrRange_128dps:
            s_imu.ssvt_g = 256;
            break;
        case Qmi8658GyrRange_256dps:
            s_imu.ssvt_g = 128;
            break;
        case Qmi8658GyrRange_512dps:
            s_imu.ssvt_g = 64;
            break;
        case Qmi8658GyrRange_1024dps:
            s_imu.ssvt_g = 32;
            break;
        default:
            s_imu.ssvt_g = 16;
            break;
    }

    ctrl_data = (uint8_t)range | (uint8_t)odr;
    if(stEnable == Qmi8658St_Enable)
        ctrl_data |= QMI8658_gST;
    
    /* Write to CTRL3 = 0x04 per official datasheet! */
    imu_i2c_write_reg(QMI8658_REG_CTRL3, ctrl_data);
    
    /* Configure LPF in CTRL5 = 0x06 */
    uint8_t ctrl5_val;
		imu_i2c_read_reg(QMI8658_REG_CTRL5, &ctrl5_val);
		ctrl_data = ctrl5_val;
    ctrl_data &= 0x0f;
    if(lpfEnable == Qmi8658Lpf_Enable)
    {
        ctrl_data |= G_LSP_MODE_3;
        ctrl_data |= 0x10;
    }
    imu_i2c_write_reg(QMI8658_REG_CTRL5, ctrl_data);
    
    rt_kprintf("[QMI8658] GYRO configured: %s @ %dHz, scale=%d/10000\n",
               gyro_range_str(range),
               gyr_odr ,
               (int)(10000/s_imu.ssvt_g));
}


static void imu_enable_sensors(uint8_t enable_flags)
{
#if defined(QMI8658_SYNC_SAMPLE_MODE)
	imu_i2c_write_reg(Qmi8658Register_Ctrl7, enableFlags | 0x80);
#elif defined(QMI8658_USE_FIFO)
	imu_i2c_write_reg(QMI8658_REG_CTRL7, enable_flags);
#else
	imu_i2c_write_reg(Qmi8658Register_Ctrl7, enableFlags);
#endif    
    s_imu.enSensors = enable_flags & 0x03;
    rt_thread_mdelay(10);
}


/* ========== Data Reading Functions ========== */

static void imu_axis_convert(float acc[3], float gyro[3], uint8_t layout)
{
    float raw_acc[3], raw_gyro[3];
    
    raw_acc[0] = acc[0];
    raw_acc[1] = acc[1];
    raw_gyro[0] = gyro[0];
    raw_gyro[1] = gyro[1];
    
    if(layout >= 4 && layout <= 7)
    {
        acc[2] = -acc[2];
        gyro[2] = -gyro[2];
    }
    
    if(layout % 2)
    {
        acc[0] = raw_acc[1];
        acc[1] = raw_acc[0];
        gyro[0] = raw_gyro[1];
        gyro[1] = raw_gyro[0];
    }
    
    if(layout == 1 || layout == 2 || layout == 4 || layout == 7)
    {
        acc[0] = -acc[0];
        gyro[0] = -gyro[0];
    }
    
    if(layout == 2 || layout == 3 || layout == 6 || layout == 7)
    {
        acc[1] = -acc[1];
        gyro[1] = -gyro[1];
    }
}


static void imu_read_sensor_data(float acc[3], float gyro[3])
{
    int16_t raw_acc_xyz[3];
    int16_t raw_gyro_xyz[3];
    
    /* Read accelerometer - NORMAL MODE addresses from official datasheet! */
   
    /* ACC data: 6 bytes starting at 0x35 */
    raw_acc_xyz[0] = imu_i2c_read_word(QMI8658_REG_ACC_OUT_L);       /* X: 0x35-0x36 */
    raw_acc_xyz[1] = imu_i2c_read_word(QMI8658_REG_ACC_OUT_L + 2);   /* Y: 0x37-0x38 */
    raw_acc_xyz[2] = imu_i2c_read_word(QMI8658_REG_ACC_OUT_L + 4);   /* Z: 0x39-0x3A */
    
    /* Read gyroscope - NORMAL MODE addresses from official datasheet! */
    /* GYRO data: 6 bytes starting at 0x3B */
    raw_gyro_xyz[0] = imu_i2c_read_word(QMI8658_REG_GYR_OUT_L);     /* X: 0x3B-0x3C */
    raw_gyro_xyz[1] = imu_i2c_read_word(QMI8658_REG_GYR_OUT_L + 2); /* Y: 0x3D-0x3E */
    raw_gyro_xyz[2] = imu_i2c_read_word(QMI8658_REG_GYR_OUT_L + 4); /* Z: 0x3F-0x40 */

    acc[0] = accel_to_g(raw_acc_xyz[0]);
    acc[1] = accel_to_g(raw_acc_xyz[1]);
    acc[2] = accel_to_g(raw_acc_xyz[2]);
    
    gyro[0] = gyro_to_dps(raw_gyro_xyz[0]);
    gyro[1] = gyro_to_dps(raw_gyro_xyz[1]);
    gyro[2] = gyro_to_dps(raw_gyro_xyz[2]);
}


static void imu_read_xyz(float acc[3], float gyro[3], uint8_t layout)
{
   
    /* Check STATUS register */
    /* Data Ready Flag check first */
    uint8_t data_ready;
		int cnt = 5;
    while (cnt--) {
			imu_i2c_read_reg(QMI8658_REG_DATA_READY_STATUS0, &data_ready);
			if((data_ready & 0x03) == 0)  /* Bit 0: ACC data not ready */
			{
				rt_hw_us_delay(10);
				if(cnt < 1){
					rt_kprintf("[QMI8658] Warning: ACC data not ready (0x%02X)\n", data_ready);
					acc[0] = s_imu.last_acc[0];
					acc[1] = s_imu.last_acc[1];
					acc[2] = s_imu.last_acc[2];
					gyro[0] = s_imu.last_gyro[0];
					gyro[1] = s_imu.last_gyro[1];
					gyro[2] = s_imu.last_gyro[2];
					return;			
				}
			}	else{
				break;
			}	
		}

    
    imu_read_sensor_data(acc, gyro);
    imu_axis_convert(acc, gyro, layout);
    
    s_imu.last_acc[0] = acc[0];
    s_imu.last_acc[1] = acc[1];
    s_imu.last_acc[2] = acc[2];
    s_imu.last_gyro[0] = gyro[0];
    s_imu.last_gyro[1] = gyro[1];
    s_imu.last_gyro[2] = gyro[2];
    
    s_imu.sample_count++;
}


/* ========== IMU Thread Function (for continuous sampling) ========== */

void qmi8658_imu_thread_entry(void* parameter)
{
    QmiDataRaw_t raw;
    
    rt_kprintf("[QMI8658] IMU thread started (v8.0 - data acquisition only)\n");
    
    while(1)
    {
			/* Wait for interrupt signal (250Hz trigger) */
        rt_sem_take(&s_data_ready_sem, RT_WAITING_FOREVER);

        
        /* Read sensor data immediately after interrupt */
        qmi8658_read_once(&raw, RT_NULL);
        
        s_imu.interrupt_count++;
        
        /* Print every PRINT_INTERVAL_MS if print enabled */
        if(s_print_enabled)
        {
            s_print_counter++;
            if(s_print_counter >= (PRINT_INTERVAL_MS / (1000/250)))  // 250Hz sampling, convert to counter
            {
                s_print_counter = 0;
                
                rt_kprintf("\n========== QMI8658 Data @ %lu ms ==========\n", 
                           rt_tick_get() / RT_TICK_PER_SECOND * 1000 + 
                           (rt_tick_get() % RT_TICK_PER_SECOND) * 1000 / RT_TICK_PER_SECOND);
                
								rt_kprintf("ACC (raw LSB):   X: %d   Y: %d   Z: %d\n", 
                                 raw.acc_x, raw.acc_y, raw.acc_z);
								
								rt_kprintf("GYRO (raw LSB):  X: %d   Y: %d   Z: %d\n", 
                                 raw.gyro_x, raw.gyro_y, raw.gyro_z);
								
								rt_kprintf("Samples: %lu | Interrupts: %lu\n", 
                                 s_imu.sample_count, s_imu.interrupt_count);
                rt_kprintf("==========================================\n\n");
            }
        }
    }
}


rt_uint32_t qmi8658_create_imu_thread(void)
{
    rt_err_t err;
    
    if(!s_imu.initialized)
    {
        rt_kprintf("[QMI8658] ERROR: Cannot create thread - device not initialized\n");
        return 0;
    }
    
    err = rt_thread_init(&s_imu_thread, "imu", 
                         qmi8658_imu_thread_entry, RT_NULL,
                         s_imu_thread_stack,          // 静态栈地址
                         sizeof(s_imu_thread_stack),  // 栈大小
                         IMU_THREAD_PRIORITY, 
                         5);                           // 时间片
    
    if(err != RT_EOK)
    {
        rt_kprintf("[QMI8658] ERROR: Failed to create thread (err=%d)\n", err);
        return 0;
    }
    
    rt_thread_startup(&s_imu_thread);
    s_thread_running = RT_TRUE;
    rt_kprintf("[QMI8658] IMU thread created and started (priority=%d)\n", IMU_THREAD_PRIORITY);
    
    return (rt_uint32_t)&s_imu_thread;
}


/* ========== Thread Control Functions ========== */

int qmi8658_stop_thread(void)
{
    if(!s_thread_running)
    {
        rt_kprintf("[QMI8658] Thread is already stopped\n");
        return -RT_ERROR;
    }
    
    s_thread_running = RT_FALSE;
    rt_kprintf("[QMI8658] IMU thread stopped successfully\n");
    
    return RT_EOK;
}


int qmi8658_start_thread(void)
{
    if(s_thread_running)
    {
        rt_kprintf("[QMI8658] Thread is already running\n");
        return -RT_ERROR;
    }
    
    if(!s_imu.initialized)
    {
        rt_kprintf("[QMI8658] ERROR: Cannot start thread - device not initialized\n");
        return -RT_ERROR;
    }
    
    s_thread_running = RT_TRUE;
    rt_kprintf("[QMI8658] IMU thread started successfully\n");
    
    return RT_EOK;
}


int qmi8658_toggle_print(int argc, char** argv)
{
    if(argc > 1)
    {
        if(strcmp(argv[1], "on") == 0 || strcmp(argv[1], "1") == 0)
        {
            s_print_enabled = RT_TRUE;
            rt_kprintf("[QMI8658] Print enabled (interval: %d ms)\n", PRINT_INTERVAL_MS);
        }
        else if(strcmp(argv[1], "off") == 0 || strcmp(argv[1], "0") == 0)
        {
            s_print_enabled = RT_FALSE;
            rt_kprintf("[QMI8658] Print disabled\n");
        }
        else
        {
            rt_kprintf("[QMI8658] Usage: imu_print [on|off|1|0]\n");
            return -RT_ERROR;
        }
    }
    else
    {
        s_print_enabled = !s_print_enabled;
        if(s_print_enabled)
        {
            rt_kprintf("[QMI8658] Print enabled (interval: %d ms)\n", PRINT_INTERVAL_MS);
        }
        else
        {
            rt_kprintf("[QMI8658] Print disabled\n");
        }
    }
    
    return RT_EOK;
}


/* ========== Main Initialization (v8.0 - Focus on Data Acquisition) ========== */

int qmi8658_init(void)
{
    rt_err_t err;
    uint8_t whoami;
    uint8_t rev_id;
    
    rt_kprintf("\n========== QMI8658 v8.0 Init (Official Registers) ========== \n");
    
		s_i2c_dev = (struct rt_i2c_bus_device*)rt_device_find(QMI8658_I2C_BUS);
		if (s_i2c_dev == RT_NULL) {
				rt_kprintf("[QMI8658] ERROR: I2C bus '%s' not found!\n", QMI8658_I2C_BUS);
				return -RT_ERROR;
		} else {
				rt_kprintf("[QMI8658] I2C bus '%s' found.\n", QMI8658_I2C_BUS);
		}
		imu_i2c_write_reg(QMI8658_REG_RESET, 0xB0);
    rt_thread_mdelay(10);
        
    if(qmi8658_validate_connection(5) != RT_EOK)
    {
        rt_kprintf("[QMI8658] ERROR: Connection validation failed!\n");
        return -RT_ERROR;
    }
    
    rev_id = qmi8658_read_revision();
    if(rev_id != 0xFF)
    {
        rt_kprintf("[QMI8658] Chip REVISION_ID: 0x%02X\n", rev_id);
        s_imu.revision_id = rev_id;
    }
    else
    {
        rt_kprintf("[QMI8658] WARNING: Could not read REVISION_ID\n");
    }
		
		imu_i2c_write_reg(QMI8658_REG_CTRL7, 0x00);
		imu_i2c_write_reg(QMI8658_REG_CTRL8, 0xc0);
        
    s_imu.initialized = RT_TRUE;
    s_imu.layout = 0;
    s_imu.sample_count = 0;
    s_imu.interrupt_count = 0;
    s_imu.task_handle = RT_NULL;
    
		imu_enable_sensors(QMI8658_DISABLE_ALL);
		
    /* Configure with corrected register map! */
    /* ACC: CTRL2 = 0x03 */
    imu_config_acc(QMI8658_ACCEL_RANGE, QMI8658_ACCEL_ODR,
                   QMI8658_ACCEL_LPF, QMI8658_ACCEL_ST);
    
    /* GYRO: CTRL3 = 0x04 */
    imu_config_gyro(QMI8658_GYRO_RANGE, QMI8658_GYRO_ODR,
                    QMI8658_GYRO_LPF, QMI8658_GYRO_ST);
    
    /* Interrupt: CTRL7 = 0x08 */
//    uint8_t ctrl7 = QMI8658_DRDY_EN | QMI8658_INT2_SELECT;
//    imu_i2c_write_reg(QMI8658_REG_CTRL7, ctrl7);
//    rt_kprintf("[QMI8658] INT2 enabled on PB5\n");

		
    imu_enable_sensors(QMI8658_ACCGYR_ENABLE);
    
		rt_sem_init(&s_data_ready_sem, "qmi_sem", 0, RT_IPC_FLAG_FIFO);
    
    err = imu_init_exti();
    if(err != RT_EOK)
    {
        rt_kprintf("[QMI8658] WARNING: EXTI failed, polling mode\n");
    }
		
    rt_thread_mdelay(20);
    
    /* Create and start IMU thread for continuous sampling */
    qmi8658_create_imu_thread();
    
    rt_kprintf("========== QMI8658 v8.0 Initialized Successfully ==========\n");
    rt_kprintf("  - WHOAMI: 0x%02X (QMI8658A)\n", whoami);
    rt_kprintf("  - REVISION: 0x%02X\n", rev_id);
    rt_kprintf("  - CTRL1: 0x%02X (ODR 250Hz)\n", QMI8658_ADDR_AI|QMI8658_BE_BIG_ENDIAN|QMI8658_INT2_ENABLE|QMI8658_INT1_ENABLE);
    rt_kprintf("  - ACC: ±4g @ 250Hz via CTRL2(0x03)\n");
    rt_kprintf("  - GYRO: ±128dps @ 250Hz via CTRL3(0x04)\n");
    rt_kprintf("  - INT2: Enabled on PB5\n");
    rt_kprintf("  - Architecture: Data acquisition only (pose estimation on PC)\n");
    rt_kprintf("  - Print: Disabled by default (use 'imu_print on' to enable)\n");
    rt_kprintf("  - Thread: Use 'imu_stop'/'imu_start' to control\n");
    rt_kprintf("===========================================================\n\n");
    
    return RT_EOK;
}


/* ========== Public API Functions ========== */
// raw->acc_x 为原始寄存器数值，decoded 仅用于兼容性（实际姿态解算需在上位机完成）
int qmi8658_read_once(QmiDataRaw_t* raw, QmiDataDecoded_t* decoded)
{
    float acc[3], gyro[3];
    
    if(!s_imu.initialized)
    {
        return -RT_ERROR;
    }
    
    imu_read_xyz(acc, gyro, s_imu.layout);
    
    if(raw)
    {
        raw->acc_x = (int16_t)(acc[0] * s_imu.ssvt_a);
        raw->acc_y = (int16_t)(acc[1] * s_imu.ssvt_a);
        raw->acc_z = (int16_t)(acc[2] * s_imu.ssvt_a);
        raw->gyro_x = (int16_t)(gyro[0] * s_imu.ssvt_g);
        raw->gyro_y = (int16_t)(gyro[1] * s_imu.ssvt_g);
        raw->gyro_z = (int16_t)(gyro[2] * s_imu.ssvt_g);
    }
    
    if(decoded)
    {
        // 姿态解算已移到上位机，下位机仅返回原始数据
        // 如果需要兼容性，可以在这里调用 Mahony 算法
        decoded->acc_x_g = acc[0];
        decoded->acc_y_g = acc[1];
        decoded->acc_z_g = acc[2];
        decoded->gyro_x_deg = gyro[0];
        decoded->gyro_y_deg = gyro[1];
        decoded->gyro_z_deg = gyro[2];
        decoded->pitch = 0;
        decoded->roll = 0;
        decoded->yaw = 0;
    }
    
    s_imu.interrupt_count++;
    
    return RT_EOK;
}


int qmi8658_read_average(rt_uint8_t samples, QmiDataDecoded_t* decoded)
{
    QmiDataRaw_t raw;
    QmiDataDecoded_t temp;
    float sums[9] = {0};
    int count = 0;
    
    for(uint8_t i = 0; i < samples; i++)
    {
				if (rt_sem_take(&s_data_ready_sem, rt_tick_from_millisecond(50)) == RT_EOK)
        {
					if(qmi8658_read_once(&raw, &temp) == RT_EOK)
					{
							sums[0] += raw.acc_x;
							sums[1] += raw.acc_y;
							sums[2] += raw.acc_z;
							sums[3] += raw.gyro_x;
							sums[4] += raw.gyro_y;
							sums[5] += raw.gyro_z;
							count++;
					}
				}else{
            rt_kprintf("[QMI8658] average read timeout\n");
        }	
					
				rt_thread_mdelay(5);
    }
    
    if(count > 0 && decoded)
    {
        decoded->acc_x_g = sums[0] / count;
        decoded->acc_y_g = sums[1] / count;
        decoded->acc_z_g = sums[2] / count;
        decoded->gyro_x_deg = sums[3] / count;
        decoded->gyro_y_deg = sums[4] / count;
        decoded->gyro_z_deg = sums[5] / count;
        decoded->pitch = 0;
        decoded->roll = 0;
        decoded->yaw = 0;
    }
    
    return (count > 0) ? RT_EOK : -RT_ERROR;
}


void qmi8658_get_latest(QmiDataDecoded_t* decoded)
{
    if(decoded && s_imu.initialized)
    {
        // 返回最后一次读取的原始数据（无姿态解算）
    }
}


void qmi8658_start_continuous(uint16_t freq_hz)
{
    (void)freq_hz;
    rt_kprintf("[QMI8658] Continuous sampling not yet implemented\n");
}


void qmi8658_stop_continuous(void)
{
    rt_kprintf("[QMI8658] No continuous sampling running\n");
}


rt_bool_t qmi8658_is_ready(void)
{
    return s_imu.initialized;
}


rt_uint32_t qmi8658_get_interrupt_count(void)
{
    return s_imu.interrupt_count;
}


void qmi8658_self_calibrate(void)
{
    rt_kprintf("[QMI8658] Calibration has been moved to PC (ROS2).\n");
    rt_kprintf("[QMI8658] Use ROS2 IMU driver for on-demand calibration.\n");
}

void qmi8658_set_layout(uint8_t layout)
{
    if(layout <= 7)
    {
        s_imu.layout = layout;
        rt_kprintf("[QMI8658] Layout set to: %d\n", layout);
    }
}


#ifdef RT_USING_MSH

static void imu_read(int argc, char** argv)
{
    QmiDataRaw_t raw;
    QmiDataDecoded_t decoded;
    
    if(qmi8658_read_once(&raw, &decoded) != RT_EOK)
    {
        rt_kprintf("[QMI8658] Read failed!\n");
        return;
    }
    
    rt_kprintf("\n========== QMI8658 v8.0 Data Report ==========\n");
    rt_kprintf("Time: +%lus.%03us\n", 
               rt_tick_get() / RT_TICK_PER_SECOND,
               (rt_tick_get() % RT_TICK_PER_SECOND) * 1000 / RT_TICK_PER_SECOND);
    
    rt_kprintf("\nAccelerometer (g):\n");
    rt_kprintf("  X: ");
    print_float(decoded.acc_x_g, 8, 3, 1);
    rt_kprintf("    Y: ");
    print_float(decoded.acc_y_g, 8, 3, 1);
    rt_kprintf("    Z: ");
    print_float(decoded.acc_z_g, 8, 3, 1);
    rt_kprintf("\n");
    
    rt_kprintf("\nGyroscope (deg/s):\n");
    rt_kprintf("  X: ");
    print_float(decoded.gyro_x_deg, 9, 2, 1);
    rt_kprintf("    Y: ");
    print_float(decoded.gyro_y_deg, 9, 2, 1);
    rt_kprintf("    Z: ");
    print_float(decoded.gyro_z_deg, 9, 2, 1);
    rt_kprintf("\n");
    
    rt_kprintf("\nNote: Pose estimation is done on PC (ROS2).\n");
    rt_kprintf("Euler angles (pitch/roll/yaw) are 0 (not calculated on MCU).\n");
    
    rt_kprintf("\nStatistics:\n");
    rt_kprintf("  Samples: %lu    ", s_imu.sample_count);
    rt_kprintf("Interrupts: %lu\n", s_imu.interrupt_count);
    
    rt_kprintf("============================================\n\n");
}


static void imu_status(int argc, char** argv)
{
    rt_kprintf("\n========== QMI8658 v8.0 Status ==========\n");
    
    if(qmi8658_is_ready())
    {
        rt_kprintf("Status: OK (Initialized)\n");
        rt_kprintf("Model: QMI8658A\n");
        rt_kprintf("WHOAMI: 0x%02X\n", QMI8658_WHOAMI);
        rt_kprintf("REVISION_ID: 0x%02X\n", s_imu.revision_id);
        rt_kprintf("I2C Bus: %s (Hardware)\n", QMI8658_I2C_BUS);
        rt_kprintf("Scale Factors:\n");
        rt_kprintf("  ACC: %d/1000000 g/LSB\n",
                   (int)(1000000/s_imu.ssvt_a));
        rt_kprintf("  GYRO: %d/10000 dps/LSB \n",
                   (int)(10000/s_imu.ssvt_g));
        rt_kprintf("Mounting: Layout %d\n", s_imu.layout);
        rt_kprintf("Samples collected: %lu\n", s_imu.sample_count);
        rt_kprintf("INT2 interrupts: %lu\n", s_imu.interrupt_count);
        rt_kprintf("Thread running: %s\n", s_thread_running ? "Yes" : "No");
        rt_kprintf("Print enabled: %s\n", s_print_enabled ? "Yes" : "No");
        rt_kprintf("EXTI: %s\n", s_exti_initialized ? "Enabled" : "Disabled");
    }
    else
    {
        rt_kprintf("Status: NOT initialized!\n");
    }
    
    rt_kprintf("==========================================\n\n");
}

static void imu_rev(int argc, char** argv)
{
    rt_kprintf("\n========== QMI8658 Chip Info ==========\n");
    rt_kprintf("WHOAMI:       0x%02X (QMI8658A)\n", QMI8658_WHOAMI);
    rt_kprintf("REVISION_ID:  0x%02X\n", s_imu.revision_id);
    rt_kprintf("I2C Address:  0x%02X\n", QMI8658_SLAVE_ADDR_H);
    rt_kprintf("\nComplete Register Map:\n");
    rt_kprintf("  0x00 WHOAMI\n");
    rt_kprintf("  0x01 REVISION_ID\n");
    rt_kprintf("  0x02 CTRL1 (ODR config)\n");
    rt_kprintf("  0x03 CTRL2 (ACC config)\n");
    rt_kprintf("  0x04 CTRL3 (GYRO config)\n");
    rt_kprintf("  0x06 CTRL5 (LPF)\n");
    rt_kprintf("  0x08 CTRL7 (Interrupt)\n");
    rt_kprintf("  0x0A CTRL9 (Command)\n");
    rt_kprintf("  0x2E STATUS0 DATA_READY (Sensor available)\n");
    rt_kprintf("  0x33 TEMP_OUT (Temperature)\n");
    rt_kprintf("  0x35 ACC_OUT_L (Start of ACC data)\n");
    rt_kprintf("  0x3B GYR_OUT_L (Start of GYRO data)\n");
    rt_kprintf("  0x60 RESET\n");
    rt_kprintf("========================================\n\n");
}

static void imu_selfcal(int argc, char** argv)
{
    rt_kprintf("\n========== Calibration ==========\n");
    rt_kprintf("Calibration has been moved to PC (ROS2).\n");
    rt_kprintf("Use ROS2 IMU driver for on-demand calibration.\n");
    rt_kprintf("====================================\n\n");
}

static void imu_stop(int argc, char** argv)
{
    if(qmi8658_stop_thread() == RT_EOK)
    {
        rt_kprintf("[QMI8658] Thread stopped. Now you can use imu_read, imu_selfcal, etc.\n");
    }
}

static void imu_start(int argc, char** argv)
{
    if(qmi8658_start_thread() == RT_EOK)
    {
        rt_kprintf("[QMI8658] Thread started. Data acquisition resumed.\n");
    }
}

static void imu_print(int argc, char** argv)
{
    qmi8658_toggle_print(argc, argv);
}

MSH_CMD_EXPORT(imu_read, "Read QMI8658 v8.0 data (raw + note: pose on PC)");
MSH_CMD_EXPORT(imu_status, "Show QMI8658 v8.0 status");
MSH_CMD_EXPORT(imu_rev, "Show chip revision info");
MSH_CMD_EXPORT(imu_selfcal, "Self calibration (moved to PC/ROS2)");
MSH_CMD_EXPORT(imu_stop, "Stop IMU thread");
MSH_CMD_EXPORT(imu_start, "Start IMU thread");
MSH_CMD_EXPORT(imu_print, "Toggle print [on|off]");

#endif /* RT_USING_MSH */
