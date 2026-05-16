/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "fal.h"
#include "oled_handle.h"
#include "led.h"
#include "button.h"
#include "qmi8658.h"
#include "buzzer.h"
#include "monitor.h"
#include "user_action.h"
#include "global_conf.h"
//#ifdef ULTRASONIC_GPIO
#include "ultrasonic_hc_sr04.h"
//#elif defined(ULTRASONIC_485)
#include "ultrasonic_485.h"
//#endif
#include "zltech_can_motor.h"
#include "uart_packet.h"
#include "packet_handle.h"
#include "car_action.h"
#include "irm_8601m2.h"

#define LOG_TAG "main.tag"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>

#define APP_VERSION "1.0.5"   

/* defined the LED0 pin: PB2 */
//#define LED0_PIN    GET_PIN(B, 2)

int main(void)
{
    int count = 1;
    /* set LED0 pin mode to output */
//    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	
	/* fal文件系统分区 bootloader相关 用于OTA */
	fal_init();
	rt_kprintf("/****************************************************/\n");
	LOG_I("The current version of APP fireware is iBed-body-V%s\n",APP_VERSION);
	rt_kprintf("/****************************************************/\n");

	uart_packet_init();  // 上下位机串口协议初始化
	packet_handle_init();
	
	oled_handle_init();  // oled,led公用spi 顺序不能掉换
	led_init();
	led_off_all();
	
	button_init();
	buzzer_init();
	irm_8601m2_init();

//#ifdef ULTRASONIC_GPIO
    hc_sr04_init();
//#elif defined(ULTRASONIC_485)
    ultrasonic_485_init();
//#endif
	zlac_motor_init();
	qmi8658_init(); 
	user_action_init();
	car_action_init();
	
	monitor_init();   // 优先初始化，因为读取data分区参数设置	
	
	buzzer_beep_once();  // 初始化完毕，蜂鸣一次
    while (1)
    {
//        rt_pin_write(LED0_PIN, PIN_HIGH);
//        rt_thread_mdelay(500);
//        rt_pin_write(LED0_PIN, PIN_LOW);
			
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}

/**
 * Function    ota_app_vtor_reconfig
 * Description Set Vector Table base location to the start addr of app(RT_APP_PART_ADDR).
*/
static int ota_app_vtor_reconfig(void)
{
    #define NVIC_VTOR_MASK   0x3FFFFF80
    /* Set the Vector Table base location by user application firmware definition */
    SCB->VTOR = RT_APP_PART_ADDR & NVIC_VTOR_MASK;

    return 0;
}
INIT_BOARD_EXPORT(ota_app_vtor_reconfig);


