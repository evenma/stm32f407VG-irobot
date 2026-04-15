#ifndef __ULTRASONIC_485_H__
#define __ULTRASONIC_485_H__

#include <rtthread.h>

/* 初始化 RS485 超声波传感器驱动 */
rt_err_t ultrasonic_485_init(void);

/* 读取指定地址传感器的距离（单位：mm）*/
rt_err_t ultrasonic_485_read_distance(uint8_t addr, uint32_t *distance_mm, uint32_t timeout_ms);

/* 启动轮询线程（每轮读取所有传感器）*/
void ultrasonic_485_start_poll(void);

/* 停止轮询线程 */
void ultrasonic_485_stop_poll(void);

uint32_t ultrasonic_485_get_baudrate(void);

/* Oled屏刷新数据用 */
void ultrasonic_485_get_distances(uint32_t *dist_array, uint8_t len);

#endif /* __ULTRASONIC_485_H__ */

