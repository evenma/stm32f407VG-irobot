/**
 * @file    main.c
 * @brief   iRobot_baseboard 主程序入口
 * @version 1.0
 * @date    2026-03-26
 * @author  马哥
 *
 * @note
 * - RT-Thread RTOS 主程序入口
 * - 系统初始化
 * - 任务创建
 * - 主循环
 */

#include <rtthread.h>
#include "global_conf.h"
#include "Peripherals/irobot_can.h"
#include "Peripherals/canopen_motor.h"

#ifdef ULOG_ENABLE
#include <ulog.h>
#else
#define LOG_E(...)
#define LOG_W(...)
#define LOG_I(...)
#define LOG_D(...)
#define LOG_RAW(...)
#endif

/* 任务句柄 */
static rt_thread_t tid_monitor = RT_NULL;
static rt_thread_t tid_oled = RT_NULL;
static rt_thread_t tid_led = RT_NULL;
static rt_thread_t tid_packet = RT_NULL;
static rt_thread_t tid_ble = RT_NULL;
static rt_thread_t tid_can = RT_NULL;
static rt_thread_t tid_motor = RT_NULL;
static rt_thread_t tid_ai_voice = RT_NULL;
static rt_thread_t tid_charger = RT_NULL;
static rt_thread_t tid_sensor = RT_NULL;

/* 队列句柄 */
static rt_queue_t queue_packet = RT_NULL;
static rt_queue_t queue_ble = RT_NULL;
static rt_queue_t queue_sensor = RT_NULL;

/* 信号量句柄 */
static rt_sem_t sem_can = RT_NULL;
static rt_sem_t sem_motor = RT_NULL;

/* 系统状态 */
app_state_t g_app_state = APP_STATE_INIT;
component_state_t g_component_state = COMPONENT_OFF;
comm_state_t g_comm_state = COMM_STATE_DISCONNECTED;

/* 任务状态 */
task_state_t g_task_state = TASK_STOPPED;

/* 版本信息 */
const char *g_app_version_string = RT_STRINGIFY(APP_VERSION_MAJOR) "." RT_STRINGIFY(APP_VERSION_MINOR) "." RT_STRINGIFY(APP_VERSION_PATCH);
const char *g_app_name_string = APP_NAME;
const char *g_app_vendor_string = APP_VENDOR;

/*============================================================================*/
/* 任务函数声明 */
/*============================================================================*/

static void monitor_task(void *parameter);
static void oled_task(void *parameter);
static void led_task(void *parameter);
static void packet_task(void *parameter);
static void ble_task(void *parameter);
static void can_task(void *parameter);
static void motor_task(void *parameter);
static void ai_voice_task(void *parameter);
static void charger_task(void *parameter);
static void sensor_task(void *parameter);

/*============================================================================*/
/* 系统初始化函数 */
/*============================================================================*/

/**
 * @brief 系统初始化
 */
static void system_init(void)
{
    LOG_I("========================================");
    LOG_I("iRobot_baseboard System Init");
    LOG_I("========================================");
    LOG_I("Version: %s", g_app_version_string);
    LOG_I("Vendor:  %s", g_app_vendor_string);

    /* 初始化系统状态 */
    g_app_state = APP_STATE_POWER_ON;
    g_component_state = COMPONENT_OFF;
    g_comm_state = COMM_STATE_DISCONNECTED;
    g_task_state = TASK_STOPPED;

    LOG_I("System initialized successfully");
}

/**
 * @brief 创建队列
 */
static void create_queues(void)
{
    /* Packet通讯队列 */
#if PACKET_PROTOCOL_ENABLE
    queue_packet = rt_queue_create("packet_q", PACKET_QUEUE_SIZE * sizeof(uint8_t),
                                    RT_IPC_FLAG_FIFO);
    if (queue_packet == RT_NULL) {
        LOG_E("Failed to create packet queue");
        return;
    }
    LOG_I("Packet queue created: %d items", PACKET_QUEUE_SIZE);
#endif

    /* 蓝牙通讯队列 */
#if BLE_ENABLE
    queue_ble = rt_queue_create("ble_q", BLE_QUEUE_SIZE * sizeof(uint8_t),
                                 RT_IPC_FLAG_FIFO);
    if (queue_ble == RT_NULL) {
        LOG_E("Failed to create BLE queue");
        return;
    }
    LOG_I("BLE queue created: %d items", BLE_QUEUE_SIZE);
#endif

    /* 传感器数据队列 */
#if US_ENABLE
    queue_sensor = rt_queue_create("sensor_q", SENSOR_QUEUE_SIZE * sizeof(uint16_t),
                                    RT_IPC_FLAG_FIFO);
    if (queue_sensor == RT_NULL) {
        LOG_E("Failed to create sensor queue");
        return;
    }
    LOG_I("Sensor queue created: %d items", SENSOR_QUEUE_SIZE);
#endif
}

/**
 * @brief 创建信号量
 */
static void create_semaphores(void)
{
    /* CAN通讯信号量 */
#if CAN_ENABLE
    sem_can = rt_sem_create("can_sem", 1, RT_IPC_FLAG_FIFO);
    if (sem_can == RT_NULL) {
        LOG_E("Failed to create CAN semaphore");
        return;
    }
    LOG_I("CAN semaphore created");
#endif

    /* 电机控制信号量 */
#if TASK_MOTOR_ENABLE
    sem_motor = rt_sem_create("motor_sem", 1, RT_IPC_FLAG_FIFO);
    if (sem_motor == RT_NULL) {
        LOG_E("Failed to create motor semaphore");
        return;
    }
    LOG_I("Motor semaphore created");
#endif

    /* CAN驱动初始化 */
#if CAN_ENABLE
    if (irobot_can_init() != RT_EOK) {
        LOG_E("Failed to initialize CAN driver");
        return;
    }
    LOG_I("CAN driver initialized");
#endif
}

/**
 * @brief 创建任务
 */
static void create_tasks(void)
{
    /* 监视任务 */
#if TASK_MONITOR_ENABLE
    tid_monitor = rt_thread_create("monitor",
                                    monitor_task, RT_NULL,
                                    TASK_MONITOR_STACK_SIZE,
                                    TASK_MONITOR_PRIORITY,
                                    20);
    if (tid_monitor != RT_NULL) {
        rt_thread_startup(tid_monitor);
    } else {
        LOG_E("Failed to create monitor task");
    }
#endif

    /* OLED显示任务 */
#if TASK_OLED_ENABLE
    tid_oled = rt_thread_create("oled",
                                oled_task, RT_NULL,
                                TASK_OLED_STACK_SIZE,
                                TASK_OLED_PRIORITY,
                                20);
    if (tid_oled != RT_NULL) {
        rt_thread_startup(tid_oled);
    } else {
        LOG_E("Failed to create OLED task");
    }
#endif

    /* LED指示灯任务 */
#if TASK_LED_ENABLE
    tid_led = rt_thread_create("led",
                               led_task, RT_NULL,
                               TASK_LED_STACK_SIZE,
                               TASK_LED_PRIORITY,
                               20);
    if (tid_led != RT_NULL) {
        rt_thread_startup(tid_led);
    } else {
        LOG_E("Failed to create LED task");
    }
#endif

    /* Packet通讯任务 */
#if TASK_PACKET_ENABLE
    tid_packet = rt_thread_create("packet",
                                  packet_task, RT_NULL,
                                  TASK_PACKET_STACK_SIZE,
                                  TASK_PACKET_PRIORITY,
                                  20);
    if (tid_packet != RT_NULL) {
        rt_thread_startup(tid_packet);
    } else {
        LOG_E("Failed to create packet task");
    }
#endif

    /* 蓝牙任务 */
#if TASK_BLE_ENABLE
    tid_ble = rt_thread_create("ble",
                               ble_task, RT_NULL,
                               TASK_BLE_STACK_SIZE,
                               TASK_BLE_PRIORITY,
                               20);
    if (tid_ble != RT_NULL) {
        rt_thread_startup(tid_ble);
    } else {
        LOG_E("Failed to create BLE task");
    }
#endif

    /* CAN通讯任务 */
#if TASK_CAN_ENABLE
    tid_can = rt_thread_create("can",
                               can_task, RT_NULL,
                               TASK_CAN_STACK_SIZE,
                               TASK_CAN_PRIORITY,
                               20);
    if (tid_can != RT_NULL) {
        rt_thread_startup(tid_can);
    } else {
        LOG_E("Failed to create CAN task");
    }
#endif

    /* 电机控制任务 */
#if TASK_MOTOR_ENABLE
    tid_motor = rt_thread_create("motor",
                                 motor_task, RT_NULL,
                                 TASK_MOTOR_STACK_SIZE,
                                 TASK_MOTOR_PRIORITY,
                                 20);
    if (tid_motor != RT_NULL) {
        rt_thread_startup(tid_motor);
    } else {
        LOG_E("Failed to create motor task");
    }
#endif

    /* AI语音任务 */
#if TASK_AI_VOICE_ENABLE
    tid_ai_voice = rt_thread_create("ai_voice",
                                    ai_voice_task, RT_NULL,
                                    TASK_AI_VOICE_STACK_SIZE,
                                    TASK_AI_VOICE_PRIORITY,
                                    20);
    if (tid_ai_voice != RT_NULL) {
        rt_thread_startup(tid_ai_voice);
    } else {
        LOG_E("Failed to create AI voice task");
    }
#endif

    /* 充电检测任务 */
#if TASK_CHARGER_ENABLE
    tid_charger = rt_thread_create("charger",
                                   charger_task, RT_NULL,
                                   TASK_CHARGER_STACK_SIZE,
                                   TASK_CHARGER_PRIORITY,
                                   20);
    if (tid_charger != RT_NULL) {
        rt_thread_startup(tid_charger);
    } else {
        LOG_E("Failed to create charger task");
    }
#endif

    /* 传感器读取任务 */
#if TASK_SENSOR_ENABLE
    tid_sensor = rt_thread_create("sensor",
                                  sensor_task, RT_NULL,
                                  TASK_SENSOR_STACK_SIZE,
                                  TASK_SENSOR_PRIORITY,
                                  20);
    if (tid_sensor != RT_NULL) {
        rt_thread_startup(tid_sensor);
    } else {
        LOG_E("Failed to create sensor task");
    }
#endif
}

/*============================================================================*/
/* 任务实现 */
/*============================================================================*/

/**
 * @brief 监视任务
 */
static void monitor_task(void *parameter)
{
    rt_tick_t last_tick = rt_tick_get();
    rt_tick_t period = RT_TICK_PER_SECOND * TASK_MONITOR_PERIOD_MS / 1000;

    LOG_I("Monitor task started");

    while (1) {
        /* 检查系统状态 */
        switch (g_app_state) {
            case APP_STATE_INIT:
                LOG_D("System state: INIT");
                break;

            case APP_STATE_POWER_ON:
                LOG_D("System state: POWER_ON");
                g_app_state = APP_STATE_READY;
                break;

            case APP_STATE_READY:
                LOG_D("System state: READY");
                break;

            case APP_STATE_WORKING:
                LOG_D("System state: WORKING");
                break;

            case APP_STATE_CHARGING:
                LOG_D("System state: CHARGING");
                break;

            case APP_STATE_ERROR:
                LOG_E("System state: ERROR");
                break;

            case APP_STATE_FACTORY:
                LOG_W("System state: FACTORY");
                break;

            default:
                LOG_E("Unknown system state: %d", g_app_state);
                break;
        }

        /* 检查任务状态 */
        LOG_D("Task state: %d", g_task_state);

        /* 等待周期 */
        rt_thread_msdelay(TASK_MONITOR_PERIOD_MS);
    }
}

/**
 * @brief OLED显示任务
 */
static void oled_task(void *parameter)
{
    LOG_I("OLED task started");

    while (1) {
        /* TODO: 实现OLED显示逻辑 */
        rt_thread_msdelay(TASK_OLED_PERIOD_MS);
    }
}

/**
 * @brief LED指示灯任务
 */
static void led_task(void *parameter)
{
    LOG_I("LED task started");

    while (1) {
        /* TODO: 实现LED指示灯逻辑 */
        rt_thread_msdelay(TASK_LED_PERIOD_MS);
    }
}

/**
 * @brief Packet通讯任务
 */
static void packet_task(void *parameter)
{
    LOG_I("Packet task started");

    while (1) {
        /* TODO: 实现Packet协议处理逻辑 */
        rt_thread_msdelay(TASK_PACKET_PERIOD_MS);
    }
}

/**
 * @brief 蓝牙任务
 */
static void ble_task(void *parameter)
{
    LOG_I("BLE task started");

    while (1) {
        /* TODO: 实现蓝牙协议处理逻辑 */
        rt_thread_msdelay(TASK_BLE_PERIOD_MS);
    }
}

/**
 * @brief CAN通讯任务
 */
static void can_task(void *parameter)
{
    LOG_I("CAN task started");

    while (1) {
        /* TODO: 实现CAN通讯协议处理逻辑 */
        rt_thread_msdelay(TASK_CAN_PERIOD_MS);
    }
}

/**
 * @brief 电机控制任务
 */
static void motor_task(void *parameter)
{
    rt_bool_t driver_ready = RT_FALSE;

    RT_UNUSED(parameter);
    LOG_I("Motor task started");

    while (1) {
#if CAN_ENABLE
        if (!driver_ready) {
            if (!irobot_can_get_state()) {
                (void)irobot_can_init();
                rt_thread_msdelay(200);
                continue;
            }

            (void)differential_drive_set_geometry(ZLAC_DEFAULT_WHEEL_TRACK_MM,
                                                  ZLAC_DEFAULT_WHEEL_DIAMETER_MM);
            (void)differential_drive_set_max_rpm(ZLAC_DEFAULT_MAX_RPM);
            (void)differential_drive_stop();
            (void)differential_drive_apply();
            driver_ready = RT_TRUE;
            LOG_I("Motor driver ready");
        }

        if (canopen_motor_service() != RT_EOK) {
            driver_ready = RT_FALSE;
            LOG_W("Motor service failed, try re-init");
            rt_thread_msdelay(100);
            continue;
        }

        (void)differential_drive_apply();
#endif
        rt_thread_msdelay(TASK_MOTOR_PERIOD_MS);
    }
}

/**
 * @brief AI语音任务
 */
static void ai_voice_task(void *parameter)
{
    LOG_I("AI voice task started");

    while (1) {
        /* TODO: 实现AI语音交互逻辑 */
        rt_thread_msdelay(TASK_AI_VOICE_PERIOD_MS);
    }
}

/**
 * @brief 充电检测任务
 */
static void charger_task(void *parameter)
{
    LOG_I("Charger task started");

    while (1) {
        /* TODO: 实现充电检测逻辑 */
        rt_thread_msdelay(TASK_CHARGER_PERIOD_MS);
    }
}

/**
 * @brief 传感器读取任务
 */
static void sensor_task(void *parameter)
{
    LOG_I("Sensor task started");

    while (1) {
        /* TODO: 实现传感器读取逻辑 */
        rt_thread_msdelay(TASK_SENSOR_PERIOD_MS);
    }
}

/*============================================================================*/
/* 主函数 */
/*============================================================================*/

int main(void)
{
    /* 系统初始化 */
    system_init();

    /* 创建队列 */
    create_queues();

    /* 创建信号量 */
    create_semaphores();

    /* 创建任务 */
    create_tasks();

    LOG_I("========================================");
    LOG_I("System Ready!");
    LOG_I("========================================");

    /* 主循环 */
    while (1) {
        /* 系统运行 */
        rt_thread_mdelay(100);
    }
}

/*============================================================================*/
/* MSH命令实现 */
/*============================================================================*/

#ifdef MSH_ENABLE

/* 打印系统状态 */
static int cmd_system_status(int argc, char **argv)
{
    rt_kprintf("========================================\n");
    rt_kprintf("System Status\n");
    rt_kprintf("========================================\n");
    rt_kprintf("State: %d\n", g_app_state);
    rt_kprintf("Component State: %d\n", g_component_state);
    rt_kprintf("Comm State: %d\n", g_comm_state);
    rt_kprintf("Task State: %d\n", g_task_state);
    rt_kprintf("Version: %s\n", g_app_version_string);
    rt_kprintf("========================================\n");

    return 0;
}
MSH_CMD_EXPORT(cmd_system_status, System status);

/* 打印任务状态 */
static int cmd_task_status(int argc, char **argv)
{
    rt_kprintf("========================================\n");
    rt_kprintf("Task Status\n");
    rt_kprintf("========================================\n");

#if TASK_MONITOR_ENABLE
    rt_kprintf("Monitor: %s\n", tid_monitor ? "Running" : "Stopped");
#endif

#if TASK_OLED_ENABLE
    rt_kprintf("OLED: %s\n", tid_oled ? "Running" : "Stopped");
#endif

#if TASK_LED_ENABLE
    rt_kprintf("LED: %s\n", tid_led ? "Running" : "Stopped");
#endif

#if TASK_PACKET_ENABLE
    rt_kprintf("Packet: %s\n", tid_packet ? "Running" : "Stopped");
#endif

#if TASK_BLE_ENABLE
    rt_kprintf("BLE: %s\n", tid_ble ? "Running" : "Stopped");
#endif

#if TASK_CAN_ENABLE
    rt_kprintf("CAN: %s\n", tid_can ? "Running" : "Stopped");
#endif

#if TASK_MOTOR_ENABLE
    rt_kprintf("Motor: %s\n", tid_motor ? "Running" : "Stopped");
#endif

#if TASK_AI_VOICE_ENABLE
    rt_kprintf("AI Voice: %s\n", tid_ai_voice ? "Running" : "Stopped");
#endif

#if TASK_CHARGER_ENABLE
    rt_kprintf("Charger: %s\n", tid_charger ? "Running" : "Stopped");
#endif

#if TASK_SENSOR_ENABLE
    rt_kprintf("Sensor: %s\n", tid_sensor ? "Running" : "Stopped");
#endif

    rt_kprintf("========================================\n");

    return 0;
}
MSH_CMD_EXPORT(cmd_task_status, Task status);

/* 重启系统 */
static int cmd_reboot(int argc, char **argv)
{
    rt_kprintf("Rebooting system...\n");
    rt_system_reboot();

    return 0;
}
MSH_CMD_EXPORT(cmd_reboot, Reboot system);

#endif /* MSH_ENABLE */
