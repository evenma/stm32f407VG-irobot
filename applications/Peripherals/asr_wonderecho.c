/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 幻尔语音模块 WonderEcho Pro 驱动实现
 *
 * 使用硬件 I2C2 (PB10,PB11) 与模块通信
 */

#include "asr_wonderecho.h"
#include <rtdbg.h>

#define DBG_TAG "asr.wonderecho"
#define DBG_LVL DBG_INFO

static struct rt_i2c_bus_device *asr_bus = RT_NULL;

/* 初始化：查找 I2C 总线，并发送一个停止信号 */
int asr_wonderecho_init(const char *i2c_bus_name)
{
    rt_device_t dev;

    dev = rt_device_find(i2c_bus_name);
    if (dev == RT_NULL) {
        rt_kprintf("[ASR] I2C bus '%s' not found!\n", i2c_bus_name);
        return -RT_ERROR;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK) {
        rt_kprintf("[ASR] Failed to open I2C device!\n");
        return -RT_ERROR;
    }

    asr_bus = (struct rt_i2c_bus_device *)dev;
    if (asr_bus == RT_NULL)
    {
        LOG_E("I2C bus %s not found!", i2c_bus_name);
        return -RT_ENOSYS;
    }

    /* 发送一个停止条件，清除之前可能遗留的未完成传输 */
//    asr_send_stop_condition();

    LOG_I("WonderEcho Pro initialized on bus %s, addr=0x%02X",
          i2c_bus_name, ASR_DEV_ADDR);
    return RT_EOK;
}

/* 发送 I2C 停止条件（通过尝试读一个不存在的地址）*/
void asr_send_stop_condition(void)
{
    if (asr_bus == RT_NULL) return;

    uint8_t dummy = 0;
    struct rt_i2c_msg msg;
    msg.addr  = 0x00;               /* 不存在的从机地址 */
    msg.flags = RT_I2C_RD;
    msg.buf   = &dummy;
    msg.len   = 1;

    /* 忽略返回值，目的是产生一个起始条件 + NACK + 停止条件 */
    rt_i2c_transfer(asr_bus, &msg, 1);
    /* 此时总线已释放 */
    rt_thread_mdelay(1);
}

/* 读取识别结果：先写寄存器地址(0x64)，再读1字节 ID */
uint8_t asr_get_result_flag(void)
{
    uint8_t reg = ASR_RESULT_REG;
    uint8_t result = 0;
    struct rt_i2c_msg msgs[2];
    rt_ssize_t ret;

    if (asr_bus == RT_NULL)
    {
        LOG_E("ASR bus not ready");
        return 0;
    }

    /* msg1: 写寄存器地址，不产生 STOP */
    msgs[0].addr  = ASR_DEV_ADDR;
    msgs[0].flags = RT_I2C_WR | RT_I2C_NO_STOP;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    /* msg2: 读取数据，不产生 START，最后自动 STOP */
    msgs[1].addr  = ASR_DEV_ADDR;
    msgs[1].flags = RT_I2C_RD | RT_I2C_NO_START;
    msgs[1].buf   = &result;
    msgs[1].len   = 1;

    ret = rt_i2c_transfer(asr_bus, msgs, 2);
    if (ret != 2)
    {
        LOG_W("I2C read result failed, ret=%d", (int)ret);
        return 0;
    }

    if (result != 0)
    {
        LOG_D("ASR recognized ID: 0x%02X", result);
    }
    return result;
}

uint8_t asr_get_result(void)
{
    uint8_t reg = ASR_RESULT_REG;
    uint8_t result = 0;

    if (asr_bus == RT_NULL) return 0;

    // 第一次：写寄存器地址，不产生 STOP
    if (rt_i2c_master_send(asr_bus, ASR_DEV_ADDR, RT_I2C_NO_STOP, &reg, 1) != 1)
        return 0;

    // 第二次：读数据，不产生 START（紧接上次），最后自动 STOP
    if (rt_i2c_master_recv(asr_bus, ASR_DEV_ADDR, RT_I2C_NO_START, &result, 1) != 1)
        return 0;

    return result;
}

/* 主动播报：向寄存器 0x6E 写入 2 字节（类型+ID）*/
void asr_speak(uint8_t type, uint8_t id)
{
    uint8_t buffer[3];   /* buffer[0]=reg, buffer[1]=type, buffer[2]=id */
    struct rt_i2c_msg msg;
    rt_ssize_t ret;

    if (asr_bus == RT_NULL)
    {
        LOG_E("ASR bus not ready");
        return;
    }

    buffer[0] = ASR_SPEAK_REG;
    buffer[1] = type;
    buffer[2] = id;

    msg.addr  = ASR_DEV_ADDR;
    msg.flags = RT_I2C_WR;
    msg.buf   = buffer;
    msg.len   = 3;

    ret = rt_i2c_transfer(asr_bus, &msg, 1);
    if (ret != 1)
    {
        LOG_E("Speak failed: type=0x%02X id=0x%02X ret=%d", type, id, (int)ret);
    }
//    else
//    {
//        LOG_D("Speak: type=0x%02X id=0x%02X", type, id);
//    }
}

/**
 * @brief 带冷却的播报（用于警告类消息，避免短时间内重复播报）
 * @param talk_id 播报 ID
 * @param cooldown_ms 冷却时间（毫秒），同一 ID 在此时间内只播报一次
 */
void asr_speak_cooldown(uint8_t talk_id, uint32_t cooldown_ms)
{
    static struct {
        uint8_t id;
        rt_tick_t last_tick;
    } cooldown_cache[8] = {0};  // 最多缓存 8 个不同 ID 的冷却记录
    static uint8_t cache_count = 0;
    
    rt_tick_t now = rt_tick_get_millisecond();
    
    // 查找是否已存在该 ID 的记录
    for (int i = 0; i < cache_count; i++) {
        if (cooldown_cache[i].id == talk_id) {
            if (now - cooldown_cache[i].last_tick >= cooldown_ms) {
                cooldown_cache[i].last_tick = now;
                asr_speak(ASR_TYPE_ANNOUNCER, talk_id);
            }
            return;
        }
    }
    
    // 新 ID，记录并播报
    if (cache_count < 8) {
        cooldown_cache[cache_count].id = talk_id;
        cooldown_cache[cache_count].last_tick = now;
        cache_count++;
    }
    asr_speak(ASR_TYPE_ANNOUNCER, talk_id);
}

