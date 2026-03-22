/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief QMI8658C 6-axis IMU driver v7.0 - FOCUS ON DATA ACQUISITION
 * 
 * Architecture Changes:
 *   ✓ 下位机专注数据采集、通信和实时控制
 *   ✓ 姿态解算和校准任务移到上位机（树莓派 + ROS2）
 *   ✓ 默认不打印数据，通过 mSH 控制台开关定时打印
 *   ✓ 支持线程停止/重新开启
 *   ✓ 移除下位机姿态解算（Mahony filter）
 *   ✓ 移除自动校准（移到上位机）
 * 
 * Critical Correction Based on User Feedback & Official Datasheet:
 *   ✓ REGISTERS ARE DIFFERENT THAN NORMAL/OIS MODE ASSUMPTIONS!
 *   ✓ WHOAMI = 0x05 only (QMI8658A)
 *   ✓ CTRL1 = 0x02 (NOT 0x01 as previously assumed!)
 *   ✓ CTRL2 = 0x03 (NOT 0x02 as previously assumed!)
 *   ✓ CTRL3 = 0x04 (NOT 0x03 as previously assumed!)
 *   ✓ CTRL5 = 0x06 (Confirmed)
 *   ✓ CTRL7 = 0x08 (Confirmed)
 *   ✓ CTRL9 = 0x0A (Confirmed)
 *   ✓ REVISION_ID after WHOAMI check
 *   ✓ Auto-self-calibration on startup via CTRL[2,3][7] bits
 *   ✓ Default: 250Hz sampling, ±4g range
 *
 * Hardware connections:
 *   - I2C1 (Hardware): PB6(SCL), PB7(SDA) @ 400kHz fast mode
 *   - INT2 (EXTI): PB5 (falling edge trigger)
 *   - Sensor address: 0x6B (ADR_PIN = VCC)
 */

#ifndef PERIPHERALS_QMI8658_H__
#define PERIPHERALS_QMI8658_H__

#include <rtthread.h>


/* ========== Configuration (override in global_conf.h if needed) ========== */

#ifndef QMI8658_I2C_BUS
#define QMI8658_I2C_BUS    "hwi2c1"  /* Hardware I2C bus name = hwi2c1 ;Soft I2C bus name = i2c1*/
#endif


/* ========== I2C Slave Addresses ========== */

#define QMI8658_SLAVE_ADDR_L    0x6B  /* ADR_PIN = GND  If SA0 = 0, I2C address = 0x6B*/
#define QMI8658_SLAVE_ADDR_H    0x6A  /* ADR_PIN = VCC  If PIN1=SA0 = 1, I2C address = 0x6A */


/* ========== CORRECTED REGISTER MAP (Per Official Datasheet) ========== */

/* Device identification */
#define QMI8658_REG_WHOAMI      0x00      /* WHOAMI register */
#define QMI8658_REG_REVISION    0x01      /* REVISION_ID register (after WHOAMI) */

/* Control registers - CORRECTED PER MANUAL! */
#define QMI8658_REG_CTRL1       0x02      /* ACC & GYRO ODR configuration */
#define QMI8658_REG_CTRL2       0x03      /* ACC range, LPF, self-test/cal */
#define QMI8658_REG_CTRL3       0x04      /* GYRO range, LPF, self-test/cal */
#define QMI8658_REG_CTRL5       0x06      /* Sensor data processing (LPF modes) */
#define QMI8658_REG_CTRL7       0x08      /* Interrupt enable */
#define QMI8658_REG_CTRL8       0x09      /* Special settings */
#define QMI8658_REG_CTRL9       0x0A      /* Commands */

/* Status and output registers - CORRECTED PER OFFICIAL DATASHEET! */
#define QMI8658_REG_RESET       0x60      /* Reset register */
/* Output registers - Normal Mode per official manual! */
#define QMI8658_REG_DATA_READY_STATUS0  0x2E      /* Sensor Data Available Register */
#define QMI8658_STATUS0_ACC_NEW  (0x01)  /* bit0: Accelerometer new data */
#define QMI8658_STATUS0_GYRO_NEW (0x02)  /* bit1: Gyroscope new data */
#define QMI8658_REG_TEMP_OUT    0x33      /* Temperature sensor output */
#define QMI8658_REG_ACC_OUT_L   0x35      /* Accelerometer X LSB (burst start) */
#define QMI8658_REG_GYR_OUT_L   0x3B      /* Gyroscope X LSB (burst start) */


/* WHOAMI value - QMI8658A ONLY */
#define QMI8658_WHOAMI          0x05      /* QMI8658A (verified by user/manual) */

/* ========== Accelerometer Configuration (CTRL1 = 0x02) ========== */
/* Sensor enable bits (written to CTRL1) */
#define QMI8658_INT1_ENABLE				(0x08)		/* Bit 3: INT1 pin output is enabled */
#define QMI8658_INT2_ENABLE				(0x10)		/* Bit 4: INT2 pin output is enabled  (PB5)*/
#define QMI8658_BE_BIG_ENDIAN			(0x20)		/* Bit 5: read data Big-Endian */
#define QMI8658_ADDR_AI						(0x40)		/* Bit 6: I2C address auto increment */

/* ========== Accelerometer Configuration (CTRL2 = 0x03) ========== */

enum qmi8658_AccRange
{
    Qmi8658AccRange_2g   = 0x00 << 4,   /* ±2g */
    Qmi8658AccRange_4g   = 0x01 << 4,   /* ±4g (RECOMMENDED default) */
    Qmi8658AccRange_8g   = 0x02 << 4,   /* ±8g */
    Qmi8658AccRange_16g  = 0x03 << 4    /* ±16g */
};

enum qmi8658_AccOdr
{
    Qmi8658AccOdr_8000Hz = 0x00,        /* 8000 Hz */
    Qmi8658AccOdr_4000Hz = 0x01,        /* 4000 Hz */
    Qmi8658AccOdr_2000Hz = 0x02,        /* 2000 Hz */
    Qmi8658AccOdr_1000Hz = 0x03,        /* 1000 Hz */
    Qmi8658AccOdr_500Hz  = 0x04,        /* 500 Hz */
    Qmi8658AccOdr_250Hz  = 0x05,        /* 250Hz (RECOMMENDED default) */
    Qmi8658AccOdr_125Hz  = 0x06,        /* 125 Hz */
    Qmi8658AccOdr_62_5Hz = 0x07,        /* 62.5 Hz */
    Qmi8658AccOdr_31_25Hz= 0x08,        /* 31.25 Hz */
    Qmi8658AccOdr_LP_128Hz = 0x0C,      /* Low power 128 Hz */
    Qmi8658AccOdr_LP_21Hz  = 0x0D,      /* Low power 21 Hz */
    Qmi8658AccOdr_LP_11Hz  = 0x0E,      /* Low power 11 Hz */
    Qmi8658AccOdr_LP_3Hz   = 0x0F       /* Low power 3 Hz */
};

enum qmi8658_LpfConfig
{
    Qmi8658Lpf_Disable = 0,             /* LPF disabled */
    Qmi8658Lpf_Enable  = 1              /* LPF enabled (reduces noise) */
};

enum qmi8658_StConfig
{
    Qmi8658St_Disable = 0,              /* Self-test/cal disabled */
    Qmi8658St_Enable  = 1               /* Self-test/cal enabled */
};


/* Default ACC configuration - UPDATED */
#define QMI8658_ACCEL_RANGE    Qmi8658AccRange_4g    /* ±4g (user requirement) */
#define QMI8658_ACCEL_ODR      Qmi8658AccOdr_250Hz   /* 250Hz (user requirement) */
#define QMI8658_ACCEL_LPF      Qmi8658Lpf_Disable	//Qmi8658Lpf_Enable     /* Enable LPF */
#define QMI8658_ACCEL_ST       Qmi8658St_Disable     /* Disable self-test */

#define QMI8658_aST						(0x80)		/* Bit 7: Enable Accelerometer Self-Test */

/* ========== Gyroscope Configuration (CTRL3 = 0x04) ========== */

enum qmi8658_GyrRange
{	
    Qmi8658GyrRange_16dps   = 0 << 4,      /* ±16 dps */
    Qmi8658GyrRange_32dps   = 1 << 4,      /* ±32 dps */
    Qmi8658GyrRange_64dps   = 2 << 4,      /* ±64 dps */
    Qmi8658GyrRange_128dps  = 3 << 4,      /* ±128 dps */
    Qmi8658GyrRange_256dps  = 4 << 4,      /* ±256 dps */
    Qmi8658GyrRange_512dps  = 5 << 4,      /* ±512 dps */
    Qmi8658GyrRange_1024dps = 6 << 4,      /* ±1024 dps */
    Qmi8658GyrRange_2048dps = 7 << 4       /* ±2048 dps (max) */
};

enum qmi8658_GyrOdr
{
    Qmi8658GyrOdr_8000Hz = 0x00,           /* 8000 Hz */
    Qmi8658GyrOdr_4000Hz = 0x01,           /* 4000 Hz */
    Qmi8658GyrOdr_2000Hz = 0x02,           /* 2000 Hz */
    Qmi8658GyrOdr_1000Hz = 0x03,           /* 1000 Hz */
    Qmi8658GyrOdr_500Hz  = 0x04,           /* 500 Hz */
    Qmi8658GyrOdr_250Hz  = 0x05,           /* 250Hz (RECOMMENDED default) */
    Qmi8658GyrOdr_125Hz  = 0x06,           /* 125 Hz */
    Qmi8658GyrOdr_62_5Hz = 0x07,           /* 62.5 Hz */
    Qmi8658GyrOdr_31_25Hz= 0x08            /* 31.25 Hz */
};


/* Default GYRO configuration - UPDATED */
#define QMI8658_GYRO_RANGE    Qmi8658GyrRange_128dps //Qmi8658GyrRange_2048dps   /* ±2048 dps max */
#define QMI8658_GYRO_ODR      Qmi8658GyrOdr_250Hz       /* 250Hz */
#define QMI8658_GYRO_LPF      Qmi8658Lpf_Disable //Qmi8658Lpf_Enable         /* Enable LPF */
#define QMI8658_GYRO_ST       Qmi8658St_Disable         /* Disable self-test */

#define QMI8658_gST						(0x80)		/* Bit 7: Enable  Gyro Self-Test */

/* ========== LPF Configuration (Ctrl5 = 0x06) ========== */
#define QMI8658_aLPF_EN    (0x01)    /* Bit 0:  Enable Accelerometer Low-Pass Filter with the mode given by aLPF_MODE */
/* Accelerometer LPF modes (bits [2:1]) */
#define A_LSP_MODE_0    (0x00 << 1)
#define A_LSP_MODE_1    (0x01 << 1)
#define A_LSP_MODE_2    (0x02 << 1)
#define A_LSP_MODE_3    (0x03 << 1)  /* Maximum filtering (recommended) 13.37% of ODR*/

#define QMI8658_gLPF_EN    (0x10)    /* Bit 4:  Enable Accelerometer Low-Pass Filter with the mode given by aLPF_MODE */
/* Gyroscope LPF modes (bits [6:5]) */
#define G_LSP_MODE_0    (0x00 << 5)
#define G_LSP_MODE_1    (0x01 << 5)
#define G_LSP_MODE_2    (0x02 << 5)
#define G_LSP_MODE_3    (0x03 << 5)  /* Maximum filtering (recommended) 13.37% of ODR */

/* ========== Interrupt Configuration (CTRL7 = 0x08)========== */								
/* Sensor enable bits (written to CTRL7) */
#define QMI8658_DISABLE_ALL		(0x0)
#define QMI8658_ACC_ENABLE    (0x01)        /*Bit 0: Enable accelerometer */
#define QMI8658_GYR_ENABLE    (0x02)        /*Bit 1: Enable gyroscope */
#define QMI8658_ACCGYR_ENABLE (QMI8658_ACC_ENABLE | QMI8658_GYR_ENABLE)
#define QMI8658_DRDY_DIS       (0)    /*Bit 5: 0: DRDY(Data Ready) is enabled, is driven to the INT2 pin */
#define QMI8658_gSN			       (0)    /*Bit 4: 0: Gyroscope in Full Mode (Drive and Sense are enabled). */
#define QMI8658_USE_FIFO			/* Bit 7:  0: Disable SyncSample mode  1: Enable SyncSample mode */

/* ========== Interrupt Configuration (CTRL8 = 0x09)========== */
/* Additional interrupt sources (optional) */
#define QMI8658_CTRL9_HandShake_Type (0x80) 	/* Bit 7: use STATUSINT.bit7 as CTRL9 handshake */
#define QMI8658_ACTIVITY_INT_SEL (0x40) 			/* Bit 6:  INT1 is used for Activity Detection event interrupt*/
#define QMI8658_Pedo_EN     		(0x10)        /* Bit 4: Pedometer engine */
#define QMI8658_Sig_Motion_EN  	(0x08)        /* Bit 2: Sig-motion detection */
#define QMI8658_NO_MOTION_EN  	(0x04)        /* Bit 2: No-motion detection */
#define QMI8658_ANY_MOTION_EN 	(0x02)        /* Bit 1: Any-motion detection */
#define QMI8658_TAP_MOTION_EN 	(0x01)        /* Bit 0: Tap-motion detection */

/* ========== CTRL9 Command Codes ========== */

enum qmi8658_Ctrl9Command
{
    qmi8658_Ctrl9_Cmd_NOP             = 0x00,     /* No operation */
    qmi8658_Ctrl9_Cmd_GyroBias        = 0x01,     /* Gyro bias calibration */
    qmi8658_Ctrl9_Cmd_Rst_Fifo        = 0x04,     /* Reset FIFO */
    qmi8658_Ctrl9_Cmd_On_Demand_Cali  = 0xA2      /* On-demand self-calibration */
};
#define QMI8658_REG_COD_STATUS          0x46      /* Calibration-On-Demand (COD) Status Register */


/* ========== CTRL2/CTRL3 Self-Calibration Bits ========== */

#define QMI8658_CTRL2_CAL_EN    (0x80)    /* Bit 7: ACC self-test/calibration enable */
#define QMI8658_CTRL3_CAL_EN    (0x80)    /* Bit 7: GYRO self-test/calibration enable */

/* ========== Data Structures ========== */

/**
 * @brief Raw 6-axis sensor data
 */
typedef struct
{
    int16_t acc_x;          /* X-axis accelerometer (raw LSB) */
    int16_t acc_y;          /* Y-axis accelerometer (raw LSB) */
    int16_t acc_z;          /* Z-axis accelerometer (raw LSB) */
    
    int16_t gyro_x;         /* X-axis gyroscope (raw LSB) */
    int16_t gyro_y;         /* Y-axis gyroscope (raw LSB) */
    int16_t gyro_z;         /* Z-axis gyroscope (raw LSB) */
} QmiDataRaw_t;


/**
 * @brief Decoded physical data with Euler angles (pose estimation moved to PC)
 */
typedef struct
{
    float acc_x_g;          /* X-axis acceleration (unit: g) */
    float acc_y_g;          /* Y-axis acceleration (unit: g) */
    float acc_z_g;          /* Z-axis acceleration (unit: g) */
    
    float gyro_x_deg;       /* X-axis angular velocity (deg/s) */
    float gyro_y_deg;       /* Y-axis angular velocity (deg/s) */
    float gyro_z_deg;       /* Z-axis angular velocity (deg/s) */
    
    float pitch;            /* Pitch angle (-90°~+90°) - NOT calculated on MCU */
    float roll;             /* Roll angle (-180°~+180°) - NOT calculated on MCU */
    float yaw;              /* Yaw angle (-180°~+180°) - NOT calculated on MCU */
} QmiDataDecoded_t;


/* ========== Internal Object Structure ========== */

struct QMI8658Object
{
    rt_bool_t initialized;      /* Initialization flag */
    uint8_t enSensors;          /* Enabled sensors mask */
    uint8_t layout;             /* Mounting orientation (0-7) */
    
    int16_t ssvt_a;             /* ACC scale factor divisor */
    int16_t ssvt_g;             /* GYRO scale factor divisor */
    
    float last_acc[3];          /* Cached last valid ACC data */
    float last_gyro[3];         /* Cached last valid GYRO data */
    
    rt_uint32_t sample_count;   /* Total samples collected */
    rt_uint32_t interrupt_count;/* Number of interrupts triggered */
    
    void* task_handle;          /* Reference to imu_task_entry */
    
    uint8_t revision_id;        /* Chip revision ID from REVISION_ID register */
};


/* ========== Public API Functions ========== */

int qmi8658_init(void);
void qmi8658_imu_thread_entry(void* parameter);
rt_uint32_t qmi8658_create_imu_thread(void);
int qmi8658_read_once(QmiDataRaw_t* raw, QmiDataDecoded_t* decoded);
int qmi8658_read_average(rt_uint8_t samples, QmiDataDecoded_t* decoded);
void qmi8658_get_latest(QmiDataDecoded_t* decoded);
void qmi8658_start_continuous(uint16_t freq_hz);
void qmi8658_stop_continuous(void);
rt_bool_t qmi8658_is_ready(void);
rt_uint32_t qmi8658_get_interrupt_count(void);
void qmi8658_self_calibrate(void);
int qmi8658_quick_calibrate(rt_uint16_t timeout_ms);
void qmi8658_set_layout(uint8_t layout);
uint8_t qmi8658_read_revision(void);
rt_err_t qmi8658_validate_connection(int max_attempts);

/* Thread control functions */
int qmi8658_stop_thread(void);
int qmi8658_start_thread(void);
int qmi8658_toggle_print(int argc, char** argv);

/* MSH Debug Commands */
#ifdef RT_USING_MSH
extern void msh_qmi_read(int argc, char** argv);
extern void msh_qmi_status(int argc, char** argv);
extern void msh_qmi_rev(int argc, char** argv);
extern void msh_qmi_quickcal(int argc, char** argv);
#endif


#endif /* PERIPHERALS_QMI8658_H__ */
