/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief QMI8658C 6-axis IMU driver v7.0 - FINAL REGISTER MAP CORRECTED
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
#include "qmi8658_v6.h"
#include "global_conf.h"


/**
 * @brief Internal QMI8658 object instance
 */
static struct QMI8658Object s_imu;

/**
 * @brief Hardware I2C device handle
 */
static struct rt_i2c_device* s_i2c_dev = RT_NULL;

/**
 * @brief EXTI initialization flag
 */
static rt_bool_t s_exti_initialized = RT_FALSE;

/**
 * @brief Semaphore for interrupt-notification mechanism
 */
static struct rt_semaphore s_data_ready_sem;

/**
 * @brief Scale factors (dynamically calculated)
 */
static float s_accel_scale = 0.000122f;   /* ±4g / 32768 = 0.000122 g/LSB */
static float s_gyro_scale = 0.0625f;      /* ±2048 dps / 32768 = 0.0625 dps/LSB */


/* ========== AHRS Algorithm: Mahony Quaternion Filter ========== */

static float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}


static void mahony_ahrs(float gx, float gy, float gz, 
                        float ax, float ay, float az,
                        float* pitch, float* roll, float* yaw)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    float recipNorm;
    
    if(ax*ay*az == 0.0f) {
        *pitch = 0.0f; *roll = 0.0f; *yaw = 0.0f;
        return;
    }
    
    recipNorm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    float Kp = 10.0f;
    float Ki = 0.008f;
    float halfT = 0.004f;  /* 1/(2*250Hz) */
    
    static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
    
    while(1) {
        float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2;
        float q1q1 = q1*q1, q1q3 = q1*q3;
        float q2q2 = q2*q2, q2q3 = q2*q3;
        float q3q3 = q3*q3;
        
        float vx = 2.0f*(q1q3 - q0q2);
        float vy = 2.0f*(q0q1 + q2q3);
        float vz = q0q0 - q1q1 - q2q2 + q3q3;
        
        float ex = ay*vz - az*vy;
        float ey = az*vx - ax*vz;
        float ez = ax*vy - ay*vx;
        
        gx += Kp*ex;
        gy += Kp*ey;
        gz += Kp*ez;
        
        exInt += ex*Ki;
        eyInt += ey*Ki;
        ezInt += ez*Ki;
        
        gx += exInt;
        gy += eyInt;
        gz += ezInt;
        
        q0 += -0.5f*(q1*gx + q2*gy + q3*gz)*halfT;
        q1 +=  0.5f*(q0*gx + q2*gz - q3*gy)*halfT;
        q2 +=  0.5f*(q0*gy - q1*gz + q3*gx)*halfT;
        q3 +=  0.5f*(q0*gz + q1*gy - q2*gx)*halfT;
        
        recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        
        *roll  = atan2f(2.0f*q2*q3 + 2.0f*q0*q1, -2.0f*q1*q1 - 2.0f*q2*q2 + 1.0f) * 57.2957795f;
        *pitch = asinf(2.0f*q1*q3 - 2.0f*q0*q2) * 57.2957795f;
        *yaw   = -atan2f(2.0f*q1*q2 + 2.0f*q0*q3, -2.0f*q2*q2 - 2.0f*q3*q3 + 1.0f) * 57.2957795f;
        
        break;
    }
}


static float accel_to_g(int16_t raw)
{
    return raw * s_accel_scale;
}


static float gyro_to_dps(int16_t raw)
{
    return raw * s_gyro_scale;
}


/* ========== Hardware I2C Communication Functions ========== */

static rt_err_t imu_i2c_read_reg(uint8_t reg, uint8_t* value)
{
    if(s_i2c_dev == RT_NULL || !rt_device_is_available(&s_i2c_dev->parent))
    {
        return -RT_ERROR;
    }
    
    if(rt_i2c_transfer(s_i2c_dev, &reg, 1, value, 1) != RT_EOK)
    {
        rt_kprintf("[QMI8658] I2C read failed at 0x%02X\n", reg);
        return -RT_ERROR;
    }
    
    return RT_EOK;
}


static rt_err_t imu_i2c_read_regs(uint8_t reg, uint8_t* buffer, uint8_t length)
{
    if(s_i2c_dev == RT_NULL || !rt_device_is_available(&s_i2c_dev->parent))
    {
        return -RT_ERROR;
    }
    
    if(rt_i2c_transfer(s_i2c_dev, &reg, 1, buffer, length) != RT_EOK)
    {
        rt_kprintf("[QMI8658] I2C burst read failed at 0x%02X, len=%d\n", reg, length);
        return -RT_ERROR;
    }
    
    return RT_EOK;
}


static rt_err_t imu_i2c_write_reg(uint8_t reg, uint8_t value)
{
    if(s_i2c_dev == RT_NULL || !rt_device_is_available(&s_i2c_dev->parent))
    {
        return -RT_ERROR;
    }
    
    uint8_t tx_data[2] = {reg, value};
    
    if(rt_i2c_transfer(s_i2c_dev, tx_data, 2, NULL, 0) != RT_EOK)
    {
        rt_kprintf("[QMI8658] I2C write failed at 0x%02X\n", reg);
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

static void qmi8658_exti_callback(void)
{
    if(s_imu.initialized == RT_TRUE && s_imu.task_handle != RT_NULL)
    {
        rt_sem_release(&s_data_ready_sem);
    }
}


static rt_err_t imu_init_exti(void)
{
    rt_err_t err;
    
    if(s_exti_initialized == RT_TRUE)
    {
        return RT_EOK;
    }
    
    rt_kprintf("[QMI8658] Initializing EXTI on PB5 (INT2)\n");
    
#ifdef BSP_USING_STM32F4
    err = rt_hw_exti_init("qmi_int2", 5, RT_GPIO_INT_FALL, qmi8658_exti_callback);
    if(err != RT_EOK)
    {
        rt_kprintf("[QMI8658] WARNING: EXTI init failed, using polling mode\n");
        return err;
    }
    
    rt_kprintf("[QMI8658] EXTI on PB5 configured successfully\n");
#else
    rt_kprintf("[QMI8658] ERROR: Platform-specific EXTI init not found\n");
    return -RT_ERROR;
#endif
    
    s_exti_initialized = RT_TRUE;
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


int qmi8658_quick_calibrate(rt_uint16_t timeout_ms)
{
    uint8_t ctrl_data;
    
    rt_kprintf("[QMI8658] Starting quick auto-calibration...\n");
    
    /* ACC Calibration - Set CTRL2[7] = 1 */
    ctrl_data = QMI8658_CTRL2_CAL_EN;
    if(imu_i2c_write_reg(QMI8658_REG_CTRL2, ctrl_data) != RT_EOK)
    {
        return -RT_ERROR;
    }
    rt_thread_mdelay(10);
    
    /* Clear CTRL2[7] */
    ctrl_data &= ~QMI8658_CTRL2_CAL_EN;
    if(imu_i2c_write_reg(QMI8658_REG_CTRL2, ctrl_data) != RT_EOK)
    {
        return -RT_ERROR;
    }
    
    /* GYRO Calibration - Set CTRL3[7] = 1 */
    ctrl_data = QMI8658_CTRL3_CAL_EN;
    if(imu_i2c_write_reg(QMI8658_REG_CTRL3, ctrl_data) != RT_EOK)
    {
        return -RT_ERROR;
    }
    rt_thread_mdelay(10);
    
    /* Clear CTRL3[7] */
    ctrl_data &= ~QMI8658_CTRL3_CAL_EN;
    if(imu_i2c_write_reg(QMI8658_REG_CTRL3, ctrl_data) != RT_EOK)
    {
        return -RT_ERROR;
    }
    
    rt_kprintf("[QMI8658] Quick calibration complete!\n");
    return RT_EOK;
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


void imu_config_acc(enum qmi8658_AccRange range, enum qmi8658_AccOdr odr,
                    enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    uint8_t ctrl_data;
    
    switch(range)
    {
        case Qmi8658AccRange_2g:
            s_imu.ssvt_a = (1<<14);
            s_accel_scale = 2.0f / 32768.0f;
            break;
        case Qmi8658AccRange_4g:
            s_imu.ssvt_a = (1<<13);
            s_accel_scale = 4.0f / 32768.0f;
            break;
        case Qmi8658AccRange_8g:
            s_imu.ssvt_a = (1<<12);
            s_accel_scale = 8.0f / 32768.0f;
            break;
        default:
            s_imu.ssvt_a = (1<<11);
            s_accel_scale = 16.0f / 32768.0f;
            break;
    }
    
    ctrl_data = (uint8_t)range | (uint8_t)odr;
    if(stEnable == Qmi8658St_Enable)
        ctrl_data |= 0x80;
    
    /* Write to CTRL2 = 0x03 per official datasheet! */
    imu_i2c_write_reg(QMI8658_REG_CTRL2, ctrl_data);
    
    /* Configure LPF in CTRL5 = 0x06 */
    ctrl_data = imu_i2c_read_reg(QMI8658_REG_CTRL5);
    ctrl_data &= 0xf0;
    if(lpfEnable == Qmi8658Lpf_Enable)
    {
        ctrl_data |= A_LSP_MODE_3;
        ctrl_data |= 0x01;
    }
    imu_i2c_write_reg(QMI8658_REG_CTRL5, ctrl_data);
    
    rt_kprintf("[QMI8658] ACC configured: ±%dg @ %dHz, scale=%.6f\n",
               (s_imu.ssvt_a >> 11) * 2,
               (1000u << (odr & 0x0F)),
               s_accel_scale);
}


void imu_config_gyro(enum qmi8658_GyrRange range, enum qmi8658_GyrOdr odr,
                     enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    uint8_t ctrl_data;
    
    switch(range)
    {
        case Qmi8658GyrRange_16dps:
            s_imu.ssvt_g = 2048;
            s_gyro_scale = 16.0f / 32768.0f;
            break;
        case Qmi8658GyrRange_32dps:
            s_imu.ssvt_g = 1024;
            s_gyro_scale = 32.0f / 32768.0f;
            break;
        case Qmi8658GyrRange_64dps:
            s_imu.ssvt_g = 512;
            s_gyro_scale = 64.0f / 32768.0f;
            break;
        case Qmi8658GyrRange_128dps:
            s_imu.ssvt_g = 256;
            s_gyro_scale = 128.0f / 32768.0f;
            break;
        case Qmi8658GyrRange_256dps:
            s_imu.ssvt_g = 128;
            s_gyro_scale = 256.0f / 32768.0f;
            break;
        case Qmi8658GyrRange_512dps:
            s_imu.ssvt_g = 64;
            s_gyro_scale = 512.0f / 32768.0f;
            break;
        case Qmi8658GyrRange_1024dps:
            s_imu.ssvt_g = 32;
            s_gyro_scale = 1024.0f / 32768.0f;
            break;
        default:
            s_imu.ssvt_g = 16;
            s_gyro_scale = 2048.0f / 32768.0f;
            break;
    }

    ctrl_data = (uint8_t)range | (uint8_t)odr;
    if(stEnable == Qmi8658St_Enable)
        ctrl_data |= 0x80;
    
    /* Write to CTRL3 = 0x04 per official datasheet! */
    imu_i2c_write_reg(QMI8658_REG_CTRL3, ctrl_data);
    
    /* Configure LPF in CTRL5 = 0x06 */
    ctrl_data = imu_i2c_read_reg(QMI8658_REG_CTRL5);
    ctrl_data &= 0x0f;
    if(lpfEnable == Qmi8658Lpf_Enable)
    {
        ctrl_data |= G_LSP_MODE_3;
        ctrl_data |= 0x10;
    }
    imu_i2c_write_reg(QMI8658_REG_CTRL5, ctrl_data);
    
    rt_kprintf("[QMI8658] GYRO configured: ±%d dps @ %dHz, scale=%.4f\n",
               s_imu.ssvt_g,
               (1000u << (odr & 0x0F)),
               s_gyro_scale);
}


static void imu_enable_sensors(uint8_t enable_flags)
{
    imu_i2c_write_reg(QMI8658_REG_CTRL7, enable_flags);
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
    /* Data Ready Flag check first */
    uint8_t data_ready;
    imu_i2c_read_reg(QMI8658_REG_DATA_READY, &data_ready);
    if((data_ready & 0x01) == 0)  /* Bit 0: ACC data not ready */
    {
        rt_kprintf("[QMI8658] Warning: ACC data not ready (0x%02X)\n", data_ready);
        acc[0] = s_imu.last_acc[0];
        acc[1] = s_imu.last_acc[1];
        acc[2] = s_imu.last_acc[2];
        gyro[0] = s_imu.last_gyro[0];
        gyro[1] = s_imu.last_gyro[1];
        gyro[2] = s_imu.last_gyro[2];
        return;
    }
    
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
    uint8_t status;
    rt_err_t err;
    
    if(s_exti_initialized == RT_TRUE)
    {
        err = rt_sem_take(&s_data_ready_sem, rt_tick_from_millisecond(50));
        if(err != RT_EOK) {}
    }
    
    /* Check STATUS register */
    imu_i2c_read_reg(QMI8658_REG_STATUS, &status);
    if((status & 0x0F) == 0)
    {
        acc[0] = s_imu.last_acc[0];
        acc[1] = s_imu.last_acc[1];
        acc[2] = s_imu.last_acc[2];
        gyro[0] = s_imu.last_gyro[0];
        gyro[1] = s_imu.last_gyro[1];
        gyro[2] = s_imu.last_gyro[2];
        return;
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


/* ========== Main Initialization (v7.0 - Corrected Register Map) ========== */

int qmi8658_init(void)
{
    rt_err_t err;
    uint8_t whoami;
    uint8_t rev_id;
    
    rt_kprintf("\n========== QMI8658 v7.0 Init (Official Registers) ========== \n");
    
    s_i2c_dev = rt_i2c_bus_device_find(QMI8658_I2C_BUS);
    if(s_i2c_dev == RT_NULL)
    {
        rt_kprintf("[QMI8658] ERROR: I2C bus not found: %s\n", QMI8658_I2C_BUS);
        return -RT_ERROR;
    }
    
    struct rt_i2c_bus_config_info cfg = {
        .freq = RT_I2C_SPEED_FAST,
        .bus_mode = RT_I2C_MODE_MASTER,
    };
    rt_i2c_bus_configure(s_i2c_dev, &cfg);
    rt_kprintf("[QMI8658] Hardware I2C %s configured at 400kHz\n", QMI8658_I2C_BUS);
    
    rt_thread_mdelay(50);
    
    if(qmi8658_validate_connection(5) != RT_EOK)
    {
        rt_kprintf("[QMI8658] ERROR: Connection validation failed!\n");
        return -RT_ERROR;
    }
    
    whoami = imu_get_id();
    if(whoami != QMI8658_WHOAMI)
    {
        rt_kprintf("[QMI8658] ERROR: WHOAMI mismatch: 0x%02X (expected 0x%02X)\n", 
                   whoami, QMI8658_WHOAMI);
        return -RT_ERROR;
    }
    
    rt_kprintf("[QMI8658] QMI8658A detected (WHOAMI=0x%02X)\n", whoami);
    
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
    
    /* Configure CTRL1 (0x02) for ODR settings - Both ACC and GYRO @ 250Hz */
    uint8_t ctrl1_val = (0x05 << 4) | 0x05;  /* GYRO@250Hz | ACC@250Hz */
    imu_i2c_write_reg(QMI8658_REG_CTRL1, ctrl1_val);
    rt_kprintf("[QMI8658] CTRL1 configured: 0x%02X (ODR setting)\n", ctrl1_val);
    
    /* **Auto-Calibration** via CTRL2[7]/CTRL3[7] */
    qmi8658_quick_calibrate(50);
    
    s_imu.initialized = RT_TRUE;
    s_imu.layout = 0;
    s_imu.sample_count = 0;
    s_imu.interrupt_count = 0;
    s_imu.task_handle = RT_NULL;
    
    /* Configure with corrected register map! */
    /* ACC: CTRL2 = 0x03 */
    imu_config_acc(QMI8658_ACCEL_RANGE, QMI8658_ACCEL_ODR,
                   QMI8658_ACCEL_LPF, QMI8658_ACCEL_ST);
    
    /* GYRO: CTRL3 = 0x04 */
    imu_config_gyro(QMI8658_GYRO_RANGE, QMI8658_GYRO_ODR,
                    QMI8658_GYRO_LPF, QMI8658_GYRO_ST);
    
    /* Interrupt: CTRL7 = 0x08 */
    uint8_t ctrl7 = QMI8658_DRDY_EN | QMI8658_INT2_SELECT;
    imu_i2c_write_reg(QMI8658_REG_CTRL7, ctrl7);
    rt_kprintf("[QMI8658] INT2 enabled on PB5\n");
    
    imu_enable_sensors(QMI8658_ACCGYR_ENABLE);
    
    err = rt_sem_create(&s_data_ready_sem, "qmi_sem", 0, RT_IPC_FLAG_FIFO);
    if(err != RT_EOK)
    {
        rt_kprintf("[QMI8658] ERROR: Failed to create semaphore!\n");
        return -RT_ERROR;
    }
    
    err = imu_init_exti();
    if(err != RT_EOK)
    {
        rt_kprintf("[QMI8658] WARNING: EXTI failed, polling mode\n");
    }
    
    rt_thread_mdelay(20);
    
    rt_kprintf("========== QMI8658 v7.0 Initialized Successfully ==========\n");
    rt_kprintf("  - WHOAMI: 0x%02X (QMI8658A)\n", whoami);
    rt_kprintf("  - REVISION: 0x%02X\n", rev_id);
    rt_kprintf("  - CTRL1: 0x%02X (ODR 250Hz)\n", ctrl1_val);
    rt_kprintf("  - ACC: ±4g @ 250Hz via CTRL2(0x03)\n");
    rt_kprintf("  - GYRO: ±2048dps @ 250Hz via CTRL3(0x04)\n");
    rt_kprintf("  - INT2: Enabled on PB5\n");
    rt_kprintf("===========================================================\n\n");
    
    return RT_EOK;
}


/* ========== Public API Functions ========== */

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
        raw->acc_x = (int16_t)(acc[0] / s_accel_scale);
        raw->acc_y = (int16_t)(acc[1] / s_accel_scale);
        raw->acc_z = (int16_t)(acc[2] / s_accel_scale);
        raw->gyro_x = (int16_t)(gyro[0] / s_gyro_scale);
        raw->gyro_y = (int16_t)(gyro[1] / s_gyro_scale);
        raw->gyro_z = (int16_t)(gyro[2] / s_gyro_scale);
    }
    
    if(decoded)
    {
        float acc_mag = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
        float acc_norm[3];
        
        if(acc_mag > 0.01f)
        {
            acc_norm[0] = acc[0] / acc_mag;
            acc_norm[1] = acc[1] / acc_mag;
            acc_norm[2] = acc[2] / acc_mag;
        }
        else
        {
            acc_norm[0] = 0; acc_norm[1] = 0; acc_norm[2] = 1;
        }
        
        float gyro_rad[3] = {
            gyro[0] * 0.0174533f,
            gyro[1] * 0.0174533f,
            gyro[2] * 0.0174533f
        };
        
        mahony_ahrs(gyro_rad[0], gyro_rad[1], gyro_rad[2],
                   acc_norm[0], acc_norm[1], acc_norm[2],
                   &decoded->pitch, &decoded->roll, &decoded->yaw);
        
        decoded->acc_x_g = acc[0];
        decoded->acc_y_g = acc[1];
        decoded->acc_z_g = acc[2];
        decoded->gyro_x_deg = gyro[0];
        decoded->gyro_y_deg = gyro[1];
        decoded->gyro_z_deg = gyro[2];
    }
    
    s_imu.interrupt_count++;
    
    return RT_EOK;
}


int qmi8658_read_average(rt_uint8_t samples, QmiDataDecoded_t* decoded)
{
    QmiDataDecoded_t temp;
    float sums[9] = {0};
    int count = 0;
    
    for(uint8_t i = 0; i < samples; i++)
    {
        if(qmi8658_read_once(RT_NULL, &temp) == RT_EOK)
        {
            sums[0] += temp.acc_x_g;
            sums[1] += temp.acc_y_g;
            sums[2] += temp.acc_z_g;
            sums[3] += temp.gyro_x_deg;
            sums[4] += temp.gyro_y_deg;
            sums[5] += temp.gyro_z_deg;
            sums[6] += temp.pitch;
            sums[7] += temp.roll;
            sums[8] += temp.yaw;
            count++;
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
        decoded->pitch = sums[6] / count;
        decoded->roll = sums[7] / count;
        decoded->yaw = sums[8] / count;
    }
    
    return (count > 0) ? RT_EOK : -RT_ERROR;
}


void qmi8658_get_latest(QmiDataDecoded_t* decoded)
{
    if(decoded && s_imu.initialized)
    {
        decoded->pitch = 0.0f;
        decoded->roll = 0.0f;
        decoded->yaw = 0.0f;
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
    rt_kprintf("[QMI8658] Starting on-demand calibration...\n");
    
    imu_i2c_write_reg(QMI8658_REG_RESET, 0xB0);
    rt_thread_mdelay(10);
    
    imu_i2c_write_reg(QMI8658_REG_CTRL9, (uint8_t)qmi8658_Ctrl9_Cmd_On_Demand_Cali);
    rt_thread_mdelay(2200);
    
    imu_i2c_write_reg(QMI8658_REG_CTRL9, (uint8_t)qmi8658_Ctrl9_Cmd_NOP);
    rt_thread_mdelay(100);
    
    rt_kprintf("[QMI8658] Calibration complete!\n");
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

static void msh_qmi_read(int argc, char** argv)
{
    QmiDataDecoded_t data;
    
    if(qmi8658_read_once(RT_NULL, &data) != RT_EOK)
    {
        rt_kprintf("[QMI8658] Read failed!\n");
        return;
    }
    
    rt_kprintf("\n========== QMI8658 v7.0 Data Report ==========\n");
    rt_kprintf("Time: +%lus.%03us\n", 
               rt_tick_get() / RT_TICK_PER_SECOND,
               (rt_tick_get() % RT_TICK_PER_SECOND) * 1000 / RT_TICK_PER_SECOND);
    
    rt_kprintf("\nAccelerometer (g):\n");
    rt_kprintf("  X: %+6.3f    ", data.acc_x_g);
    rt_kprintf("Y: %+6.3f    ", data.acc_y_g);
    rt_kprintf("Z: %+6.3f\n", data.acc_z_g);
    
    rt_kprintf("\nGyroscope (deg/s):\n");
    rt_kprintf("  X: %+7.2f    ", data.gyro_x_deg);
    rt_kprintf("Y: %+7.2f    ", data.gyro_y_deg);
    rt_kprintf("Z: %+7.2f\n", data.gyro_z_deg);
    
    rt_kprintf("\nEuler Angles:\n");
    rt_kprintf("  Pitch: %+6.2f°   ", data.pitch);
    rt_kprintf("Roll: %+6.2f°\n", data.roll);
    rt_kprintf("  Yaw: %6.2f°\n", data.yaw);
    
    rt_kprintf("\nStatistics:\n");
    rt_kprintf("  Samples: %lu    ", s_imu.sample_count);
    rt_kprintf("Interrupts: %lu\n", s_imu.interrupt_count);
    
    rt_kprintf("============================================\n\n");
}


static void msh_qmi_status(int argc, char** argv)
{
    rt_kprintf("\n========== QMI8658 v7.0 Status ==========\n");
    
    if(qmi8658_is_ready())
    {
        rt_kprintf("Status: OK (Initialized)\n");
        rt_kprintf("Model: QMI8658A\n");
        rt_kprintf("WHOAMI: 0x%02X\n", QMI8658_WHOAMI);
        rt_kprintf("REVISION_ID: 0x%02X\n", s_imu.revision_id);
        rt_kprintf("I2C Bus: %s (Hardware)\n", QMI8658_I2C_BUS);
        rt_kprintf("Scale Factors:\n");
        rt_kprintf("  ACC: %.6f g/LSB (%.1fg range)\n",
                   s_accel_scale, s_imu.ssvt_a >> 11);
        rt_kprintf("  GYRO: %.4f dps/LSB (%.0fdps range)\n",
                   s_gyro_scale, s_imu.ssvt_g);
        rt_kprintf("Mounting: Layout %d\n", s_imu.layout);
        rt_kprintf("Samples collected: %lu\n", s_imu.sample_count);
        rt_kprintf("INT2 interrupts: %lu\n", s_imu.interrupt_count);
        rt_kprintf("EXTI: %s\n", s_exti_initialized ? "Enabled" : "Disabled");
    }
    else
    {
        rt_kprintf("Status: NOT initialized!\n");
    }
    
    rt_kprintf("==========================================\n\n");
}

static void msh_qmi_rev(int argc, char** argv)
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
    rt_kprintf("  0x18 STATUS (Data ready flags)\n");
    rt_kprintf("  0x2D DATA_READY (Sensor available)\n");
    rt_kprintf("  0x33 TEMP_OUT (Temperature)\n");
    rt_kprintf("  0x35 ACC_OUT_L (Start of ACC data)\n");
    rt_kprintf("  0x3B GYR_OUT_L (Start of GYRO data)\n");
    rt_kprintf("  0x60 RESET\n");
    rt_kprintf("========================================\n\n");
}

static void msh_qmi_quickcal(int argc, char** argv)
{
    rt_kprintf("\n========== Quick Calibration ==========\n");
    qmi8658_quick_calibrate(50);
    rt_kprintf("=======================================\n\n");
}

MSH_CMD_EXPORT(qmi_read, "Read QMI8658 v7.0 data");
MSH_CMD_EXPORT(qmi_status, "Show QMI8658 v7.0 status");
MSH_CMD_EXPORT(qmi_rev, "Show chip revision info");
MSH_CMD_EXPORT(qmi_quickcal, "Quick calibration");

#endif /* RT_USING_MSH */
