/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 蜂鸣器音效系统 - 对象化重构版 v2.0
 * 
 * 硬件连接: BUZZER_PIN = PE6 (✅已确认：PE6=蜂鸣器，PE5=加热管)
 */

#include <rtthread.h>
#include "buzzer.h"
#include "global_conf.h"


/**
 * @brief 蜂鸣器对象实例
 */
static BuzzerObject_t s_buzzer;

/**
 * @brief 定时器句柄（用于循环播放）
 */
static struct rt_timer s_periodic_timer;

/**
 * @brief 音效任务线程
 */
static rt_thread_t s_audio_thread = RT_NULL;


/* ========== 内部函数声明 ========== */

static void buzzer_play_tone(ToneType_t type);
static void buzzer_audio_task_entry(void* parameter);
static void buzzer_periodic_callback(rt_timer_t timer);


/* ========== 公共 API 实现 ========== */

int buzzer_init_object(BuzzerObject_t* obj, rt_pin_t pin)
{
    if (obj == RT_NULL)
    {
        return -RT_ERROR;
    }
    
    /* 1. 初始化引脚 */
    rt_pin_mode(pin, PIN_MODE_OUTPUT);
    rt_pin_write(pin, PIN_LOW);  // 初始关闭
    
    /* 2. 填充对象结构体 */
    obj->pin = pin;
    obj->enabled = RT_FALSE;
    obj->current_tone = TONE_NONE;
    obj->repeat_count = 0;
    obj->is_playing = RT_FALSE;
    
    /* 3. 创建周期定时器 */
    rt_timer_init(&s_periodic_timer, 
                  "bztr", 
                  buzzer_periodic_callback, 
                  RT_NULL, 
                  10,  // 10ms 周期
                  RT_TIMER_FLAG_HARD_TIMER);
    
    /* 4. 创建音频任务线程 */
    s_audio_thread = rt_thread_create("buzzer",
                                      buzzer_audio_task_entry,
                                      RT_NULL,
                                      512,                // 小栈空间
                                      RT_THREAD_PRIORITY_MAX / 5,
                                      10);
                                      
    if (s_audio_thread != RT_NULL)
    {
        rt_thread_startup(s_audio_thread);
        rt_kprintf("[Buzzer] Audio task thread created\n");
    }
    
    rt_kprintf("[Buzzer] Initialized on pin %d\n", pin);
    
    return 0;
}


void buzzer_init(void)
{
    /* ✅ 已确认：PE6 = 蜂鸣器，PE5 = 加热管 */
    buzzer_init_object(&s_buzzer, GET_PIN(E, 6));
}


int buzzer_set_tone(BuzzerObject_t* obj, ToneType_t type, uint8_t repeat_count)
{
    if (obj == RT_NULL || obj->pin == 0xFFFFFFFF)
    {
        return -RT_ERROR;
    }
    
    obj->current_tone = type;
    obj->repeat_count = repeat_count;
    obj->is_playing = RT_TRUE;
    
    rt_kprintf("[Buzzer] Set tone: %d (x%d)\n", type, repeat_count);
    
    return 0;
}


void buzzer_stop(BuzzerObject_t* obj)
{
    if (obj == RT_NULL || obj->pin == 0xFFFFFFFF)
    {
        return;
    }
    
    rt_pin_write(obj->pin, PIN_LOW);
    obj->is_playing = RT_FALSE;
    obj->current_tone = TONE_NONE;
    
    /* 停止周期定时器 */
    rt_timer_stop(&s_periodic_timer);
}


uint8_t buzzer_get_repeat_remaining(BuzzerObject_t* obj)
{
    if (obj == RT_NULL || obj->pin == 0xFFFFFFFF)
    {
        return 0;
    }
    
    return obj->repeat_count;
}


void buzzer_enable(BuzzerObject_t* obj, rt_bool_t on)
{
    if (obj == RT_NULL || obj->pin == 0xFFFFFFFF)
    {
        return;
    }
    
    obj->enabled = on;
    
    if (!on)
    {
        rt_pin_write(obj->pin, PIN_LOW);
        obj->is_playing = RT_FALSE;
    }
}


/* ========== 音效实现 ========== */

static void buzzer_play_tone(ToneType_t type)
{
    int duration_ms;
    
    switch (type)
    {
        case TONE_SHORT:
            duration_ms = TONE_SHORT_DURATION;
            break;
        case TONE_MEDIUM:
            duration_ms = TONE_MEDIUM_DURATION;
            break;
        case TONE_LONG:
            duration_ms = TONE_LONG_DURATION;
            break;
        default:
            return;
    }
    
    rt_pin_write(s_buzzer.pin, PIN_HIGH);
    rt_thread_mdelay(duration_ms);
    rt_pin_write(s_buzzer.pin, PIN_LOW);
}


static void buzzer_audio_task_entry(void* parameter)
{
    while (1)
    {
        if (s_buzzer.is_playing && s_buzzer.enabled)
        {
            switch (s_buzzer.current_tone)
            {
                case TONE_POWERON:           // 开机提示：短 - 短 - 中
                    for (int i = 0; i < 3; i++)
                    {
                        if (i == 2) 
                            buzzer_play_tone(TONE_MEDIUM);
                        else
                            buzzer_play_tone(TONE_SHORT);
                        
                        if (i < 2)
                            rt_thread_mdelay(PAUSE_SHORT);
                        
                        if (s_buzzer.repeat_count > 0 && --s_buzzer.repeat_count == 0)
                        {
                            s_buzzer.is_playing = RT_FALSE;
                            return;
                        }
                    }
                    break;
                
                case TONE_TASK_START:        // 三声快速短音
                    for (uint8_t i = 0; i < 3; i++)
                    {
                        buzzer_play_tone(TONE_SHORT);
                        rt_thread_mdelay(PAUSE_SHORT / 2);
                        
                        if (s_buzzer.repeat_count > 0 && --s_buzzer.repeat_count == 0)
                        {
                            s_buzzer.is_playing = RT_FALSE;
                            return;
                        }
                    }
                    break;
                
                case TONE_TASK_DONE:         // 两声长音
                    for (uint8_t i = 0; i < 2; i++)
                    {
                        buzzer_play_tone(TONE_LONG);
                        if (i < 1) rt_thread_mdelay(PAUSE_SHORT);
                        
                        if (s_buzzer.repeat_count > 0 && --s_buzzer.repeat_count == 0)
                        {
                            s_buzzer.is_playing = RT_FALSE;
                            return;
                        }
                    }
                    break;
                
                case TONE_LOW_BATTERY:       // 短 - 短 - 长循环
                    if (s_buzzer.repeat_count == 0)
                    {
                        /* 无限循环模式 */
                        while (s_buzzer.is_playing)
                        {
                            buzzer_play_tone(TONE_SHORT);
                            rt_thread_mdelay(PAUSE_SHORT / 2);
                            
                            buzzer_play_tone(TONE_SHORT);
                            rt_thread_mdelay(PAUSE_SHORT);
                            
                            buzzer_play_tone(TONE_LONG);
                            rt_thread_mdelay(PAUSE_LONG);
                        }
                    }
                    else
                    {
                        /* 有限次循环 */
                        for (uint8_t cycle = 0; cycle < s_buzzer.repeat_count; cycle++)
                        {
                            if (!s_buzzer.is_playing) break;
                            
                            buzzer_play_tone(TONE_SHORT);
                            rt_thread_mdelay(PAUSE_SHORT / 2);
                            
                            buzzer_play_tone(TONE_SHORT);
                            rt_thread_mdelay(PAUSE_SHORT);
                            
                            buzzer_play_tone(TONE_LONG);
                            rt_thread_mdelay(PAUSE_LONG);
                        }
                    }
                    break;
                
                case TONE_CRITICAL_ERROR:    // 持续警报
                    while (s_buzzer.is_playing)
                    {
                        buzzer_play_tone(TONE_LONG);
                        rt_thread_mdelay(PAUSE_SHORT);
                    }
                    break;
                
                case TONE_GENERAL_ERROR:     // 长 - 短 - 短 ×3
                    for (uint8_t cycle = 0; cycle < s_buzzer.repeat_count + 1; cycle++)
                    {
                        if (!s_buzzer.is_playing) break;
                        
                        buzzer_play_tone(TONE_LONG);
                        rt_thread_mdelay(PAUSE_SHORT);
                        
                        buzzer_play_tone(TONE_SHORT);
                        rt_thread_mdelay(PAUSE_SHORT / 2);
                        
                        buzzer_play_tone(TONE_SHORT);
                        rt_thread_mdelay(PAUSE_SHORT);
                    }
                    break;
                
                case TONE_KEYPRESS:          // 短促提示
                    buzzer_play_tone(TONE_SHORT / 2);
                    break;
                
                case TONE_INFO:              // 简短提示
                    buzzer_play_tone(TONE_MEDIUM / 2);
                    break;
                
                default:
                    break;
            }
            
            s_buzzer.is_playing = RT_FALSE;  // 单次播放完成
        }
        
        rt_thread_mdelay(10);
    }
}


static void buzzer_periodic_callback(rt_timer_t timer)
{
    /* 周期性维护回调（可拓展）*/
    /* 例如：心跳检测、状态监控 */
}


/* ========== 便捷 API (保持向后兼容) ========== */

void buzz_poweron(void)
{
    buzzer_set_tone(&s_buzzer, TONE_POWERON, 1);
}


void buzz_shutdown(void)
{
    buzzer_set_tone(&s_buzzer, TONE_SHORT, 1);
}


void buzz_task_start(void)
{
    buzzer_set_tone(&s_buzzer, TONE_TASK_START, 1);
}


void buzz_task_done(void)
{
    buzzer_set_tone(&s_buzzer, TONE_TASK_DONE, 1);
}


void buzz_task_fail(void)
{
    buzzer_set_tone(&s_buzzer, TONE_SHORT, 3);
}


int buzz_low_battery(void)
{
    /* 返回 0 表示可继续循环 */
    buzzer_set_tone(&s_buzzer, TONE_LOW_BATTERY, 0);  /* 0 = 无限循环 */
    return 0;
}


void buzz_charged(void)
{
    buzzer_set_tone(&s_buzzer, TONE_SHORT, 5);
}


void buzz_critical_error(void)
{
    buzzer_set_tone(&s_buzzer, TONE_CRITICAL_ERROR, 0);  /* 0 = 直到人工干预 */
}


void buzz_general_error(void)
{
    buzzer_set_tone(&s_buzzer, TONE_GENERAL_ERROR, 3);
}


void buzz_keypress(void)
{
    buzzer_set_tone(&s_buzzer, TONE_KEYPRESS, 1);
}


void buzz_info(void)
{
    buzzer_set_tone(&s_buzzer, TONE_INFO, 1);
}


void buzz_off_all(void)
{
    buzzer_stop(&s_buzzer);
}


/* ========== 系统初始化导出 ========== */

INIT_BOARD_EXPORT(buzzer_init);
