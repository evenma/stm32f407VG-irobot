/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

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
#include "ir_receiver.h"
#include "asr_action.h"
#include "ble_action.h"

#define LOG_TAG "main.tag"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>

rt_uint8_t enableDebug = 1;

/* defined the LED0 pin: PB2 */
//#define LED0_PIN    GET_PIN(B, 2)

#define WDT_DEVICE_NAME    "wdt"    /* 看门狗设备名称 */
rt_uint32_t wdg_timeout = 30;       /* 溢出时间，单位：秒  <32*/
rt_device_t wdg_dev;    /* 看门狗设备句柄 */
static rt_timer_t wdg_timer;

/* 在空闲线程的回调函数里喂狗  看门狗一旦启动不能停止和关闭*/
//static void idle_hook(void)
//{
//    static rt_tick_t last_feed = 0;
//    rt_tick_t now = rt_tick_get();
//    // 每 1 秒喂一次狗，避免过于频繁
//    if (wdg_dev && (now - last_feed) >= RT_TICK_PER_SECOND) {
//        rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
//        last_feed = now;
//    }
//}

static void wdg_feed(void *param)
{
    if (wdg_dev) {
        rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
    }
}

rt_err_t wdg_init(void)
{
		rt_err_t ret = RT_EOK;
/* 根据设备名称查找看门狗设备，获取设备句柄 */
		wdg_dev = rt_device_find(WDT_DEVICE_NAME);
    if (!wdg_dev)
    {
				if(enableDebug){
					LOG_D("find %s failed!\n", WDT_DEVICE_NAME);
				}
        return RT_ERROR;
    }
/* 初始化设备 */
		ret = rt_device_init(wdg_dev);
		if (ret != RT_EOK)
		{
			if(enableDebug){
				LOG_D("init %s failed!\n", WDT_DEVICE_NAME);
			}
				return RT_ERROR;
		}		
		
/* 设置看门狗溢出时间 */
		ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &wdg_timeout);
		if (ret != RT_EOK)
		{
			if(enableDebug){
				LOG_D("set %s timeout failed!\n", WDT_DEVICE_NAME);
			}
				return RT_ERROR;
		}
/* 启动看门狗  注意：看门狗一旦启动，不能关闭 */
    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    if (ret != RT_EOK)
    {
			if(enableDebug){
        LOG_D("start %s failed!\n", WDT_DEVICE_NAME);
			}
        return RT_ERROR;
    }
		
///* 设置空闲线程回调函数 */
//		rt_thread_idle_sethook(idle_hook);			
		
		wdg_timer = rt_timer_create("wdg_feed", wdg_feed, NULL,
                            RT_TICK_PER_SECOND * 5, RT_TIMER_FLAG_PERIODIC);
		if (wdg_timer) rt_timer_start(wdg_timer);
		
		LOG_D("start %s!\n", WDT_DEVICE_NAME);
		return ret;
}


int main(void)
{
    int count = 1;
			rt_err_t ret = RT_EOK;
	
    /* set LED0 pin mode to output */
//    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	
	/* fal文件系统分区 bootloader相关 用于OTA */
	fal_init();
	rt_kprintf("/****************************************************/\n");
	LOG_I("The current version of APP fireware is iBed-body-V%s\n",APP_VERSION_STRING);
	rt_kprintf("/****************************************************/\n");
	
	uart_packet_init();  // 上下位机串口协议初始化
	packet_handle_init();
	
	oled_handle_init();  // oled,led公用spi 顺序不能掉换
	led_init();
	
	button_init();
	buzzer_init();

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
	
	ir_receiver_init();
	asr_action_init();
	ble_action_init();
	
	wdg_init();					/* 开启看门狗 */	
	
	rt_kprintf("=================== ALL INIT END ==================================\r\n");
	rt_thread_mdelay(500);
	buzzer_beep_once();  // 初始化完毕，蜂鸣一次
	led_off_all();



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


