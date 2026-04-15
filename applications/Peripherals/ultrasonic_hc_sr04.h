#ifndef __ULTRASONIC_HC_SR04_H__
#define __ULTRASONIC_HC_SR04_H__

#include <rtthread.h>

/**
 * @brief 初始化 HC-SR04 超声波传感器驱动
 * @return RT_EOK 成功，否则失败
 */
rt_err_t hc_sr04_init(void);

/**
 * @brief 读取指定超声波传感器的距离值
 * @param index 传感器索引 0~7
 * @param distance_mm 输出距离（毫米）
 * @param timeout_ms 等待 Echo 超时时间（毫秒）
 * @return RT_EOK 成功，-RT_ETIMEOUT 超时，-RT_ERROR 参数错误
 */
rt_err_t hc_sr04_read_distance(uint8_t index, uint32_t *distance_mm, uint32_t timeout_ms);

/**
 * @brief 获取所有传感器的距离值（用于 OLED 显示）
 * @param dist_array 输出数组，长度至少为 HC_SR04_NUM
 * @param len 数组长度
 */
void hc_sr04_get_distances(uint32_t *dist_array, uint8_t len);

#endif /* __ULTRASONIC_HC_SR04_H__ */
