/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 * 2026-03-14     Wuji         Added buzzer support
 * 2026-03-14     Wuji         Integrated LED driver v1.0.7
 * 2026-03-14     Wuji         Integrated OLED display system v1.0.8
 * 2026-03-14     Wuji         Integrated QMI8658 IMU sensor v1.0.9
 * 2026-03-14     Wuji         Integrated RS485 ultrasonic sensor v1.0.10
 * 2026-03-14     Wuji         Integrated WonderEcho AI voice module v1.0.11
 * 2026-03-14     Wuji         Integrated CANopen motor driver v1.0.11
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "fal.h"
#include "buzzer.h"
#include "led.h"
#include "oled_handle.h"
#include "qmi8658.h"
#include "rs485_ultrasonic.h"
#include "wonder_echo.h"        // ⭐ NEW: AI 语音交互模块
#include "canopen_motor.h"      // ⭐ NEW: CANopen 电机驱动
#include "global_conf.h"


#define LOG_TAG "main.tag"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>

#define APP_VERSION "1.0.11"    /* V1.0.11 - Voice + Motor integrated */


int main(void)
{
    static uint32_t state = 0;
    uint32_t count = 1;
    
    /* fal 文件系统初始化 */
    fal_init();
    
    /* 开机顺序：先基础外设，再高级功能 */
    buzz_poweron();
    led_init();
    led_poweron_sequence();
    
    rt_thread_mdelay(100);
    oled_handle_init();
    oled_switch_page(PAGE_HOME);
    
    /* ========== 语音模块初始化 ========== */
#ifdef PERIPHERALS_WONDER_ECHO_H__
    if (wonder_echo_init(MODE_FREE_TRIGGER, 70) == 0)
    {
        LOG_I("WonderEcho AI initialized!");
        
        /* 注册回调函数 */
        wonder_echo_register_on_wakeup([]() {
            rt_kprintf("[Echo] Wakeup detected!\n");
            buzz_keypress();
            oled_switch_page(PAGE_HOME);
        });
        
        wonder_echo_register_on_recognized([](const char* result) {
            rt_kprintf("[Echo] Recognition: %s\n", result);
            buzz_info();
        });
        
        wonder_echo_register_on_tts_done([]() {
            rt_kprintf("[Echo] TTS completed\n");
        });
        
        /* 欢迎词播报 */
        wonder_echo_speak_now("Welcome to iBed-body, system ready.", 80);
    }
    else
    {
        LOG_W("WonderEcho initialization failed!");
    }
#endif
    
    /* ========== RS485 超声波传感器 ========== */
#ifdef PERIPHERALS_RS485_ULTRASONIC_H__
    if (rs485_us_init() == 0)
    {
        LOG_I("RS485 Ultrasonic initialized!");
        
        uint16_t online = rs485_us_get_online_status();
        if (online != 0)
        {
            rt_kprintf("[RS485] Found %d online sensors\n", __builtin_popcount(online));
            oled_switch_page(PAGE_ULTRASONIC);
            oled_set_auto_refresh(PAGE_ULTRASONIC, 200);
        }
        else
        {
            LOG_W("No RS485 sensors detected!");
        }
    }
#endif
    
    /* ========== IMU 姿态传感器 ========== */
#ifdef PERIPHERALS_QMI8658_H__
    if (qmi8658_init() == 0)
    {
        LOG_I("QMI8658C IMU initialized!");
        qmi8658_start_continuous(1000);
        
        oled_switch_page(PAGE_IMU_DATA);
        oled_set_auto_refresh(PAGE_IMU_DATA, 100);
    }
#endif
    
    /* ========== CANopen 电机驱动 ========== */
#ifdef PERIPHERALS_CANOPEN_MOTOR_H__
    rt_uint8_t motor_nodes[] = {MOTOR_LEFT_NODE_ID, MOTOR_RIGHT_NODE_ID};
    
    if (canopen_motor_init(motor_nodes, 2) == 0)
    {
        LOG_I("CANopen Motor system initialized!");
        
        /* 使能双电机 */
        canopen_motor_enable(0);  /* 左轮 */
        canopen_motor_enable(1);  /* 右轮 */
        
        rt_kprintf("[CAN] Both motors enabled (node %d, %d)\n", 
                   MOTOR_LEFT_NODE_ID, MOTOR_RIGHT_NODE_ID);
    }
    else
    {
        LOG_W("CANopen motor init failed! Check wiring.");
    }
#endif
    
    rt_kprintf("/****************************************************/\n");
    LOG_I("The current version of APP firmware is iBed-body-V%s\n", APP_VERSION);
    LOG_I("LED driver initialized with %d LEDs\n", LED_TOTAL_COUNT);
    LOG_I("OLED display system initialized with %d pages\n", PAGE_COUNT);
#ifdef PERIPHERALS_WONDER_ECHO_H__
    LOG_I("WonderEcho AI voice module ready");
#else
    LOG_W("Voice module not compiled in");
#endif
#ifdef PERIPHERALS_RS485_ULTRASONIC_H__
    LOG_I("RS485 Ultrasonic: Modbus RTU ready");
#else
    LOG_W("Ultrasonic module not compiled in");
#endif
#ifdef PERIPHERALS_QMI8658_H__
    LOG_I("QMI8658C IMU: 6-axis (acc+gyro) ready");
#else
    LOG_W("IMU module not compiled in");
#endif
#ifdef PERIPHERALS_CANOPEN_MOTOR_H__
    LOG_I("ZLAC8015D CANopen motors ready");
#else
    LOG_W("Motor driver not compiled in");
#endif
    rt_kprintf("/****************************************************/\n");
 
    while (count++)
    {
        switch(state)
        {
            case 0:
                rt_thread_mdelay(1000);
                
                if (count % 9 == 0)
                {
                    buzz_task_start();
                    for(int i = 0; i < 3; i++) {
                        led_set_color(LED_IDX_WORKING_GREEN, LED_COLOR_FLASH_FAST);
                        rt_thread_mdelay(100);
                        led_set_color(LED_IDX_WORKING_GREEN, LED_COLOR_OFF);
                        rt_thread_mdelay(100);
                    }
                    oled_trigger_refresh();
                    LOG_I("Task started...");
                }
                else if (count % 12 == 0)
                {
                    buzz_task_done();
                    led_set_working(RT_TRUE);
                    LOG_I("Task completed!");
                }
                else if (count % 6 == 0)
                {
                    buzz_info();
                    led_set_color(LED_IDX_CUSTOM_7, LED_COLOR_FLASH_SLOW);
                }
                else if (count % 8 == 0)
                {
                    buzz_keypress();
                    led_scan_effect();
                }
                else if (count % 18 == 0)
                {
                    buzz_low_battery();
                    led_set_battery_low(RT_TRUE);
                    oled_switch_page(PAGE_BATTERY_INFO);
                    LOG_I("Low battery warning!");
                }
                else if (count % 24 == 0)
                {
                    buzz_general_error();
                    led_set_fault(RT_TRUE);
                    oled_switch_page(PAGE_FAULT_LOG);
                    LOG_W("General error occurred!");
                }
                
                /* PID 控制循环示例（如果启用了电机驱动）*/
#ifdef PERIPHERALS_CANOPEN_MOTOR_H__
                canopen_motor_pid_update(0);
                canopen_motor_pid_update(1);
#endif
                
                break;
                
            default:
                rt_thread_mdelay(100);
                break;
        }
        
        state++;
        if (state > 30) state = 0;
    }

    return RT_EOK;
}

static int ota_app_vtor_reconfig(void)
{
    #define NVIC_VTOR_MASK   0x3FFFFF80
    SCB->VTOR = RT_APP_PART_ADDR & NVIC_VTOR_MASK;
    return 0;
}
INIT_BOARD_EXPORT(ota_app_vtor_reconfig);
