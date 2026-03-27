/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief Buzzer driver implementation
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>          // 提供 GPIO 端口枚举和 HAL 库引脚定义
#include "buzzer.h"
#include "global_conf.h"
#include <string.h>

/* ========== Thread Configuration ========== */
#define BUZZER_THREAD_STACK_SIZE   512
#define BUZZER_THREAD_PRIORITY     25
#define BUZZER_MQ_SIZE             8       /* Maximum pending commands */
#define BUZZER_MQ_MSG_SIZE         sizeof(BuzzerCommandTypeDef)

/* ========== Buzzer Object ========== */
typedef struct {
    rt_thread_t thread;                     /* Thread handle */
    rt_uint8_t stack[BUZZER_THREAD_STACK_SIZE];
    rt_mq_t mq;                             /* Message queue */
    rt_bool_t initialized;                  /* Initialization flag */

    /* State machine variables */
    BuzzerCommandTypeDef cmd;       				/* Currently executing command */
    rt_tick_t next_tick;                    /* Next transition tick */
    uint16_t remaining_repeat;              /* Remaining cycles (0 = infinite) */
    BuzzerState_t state;                        /* Current output state */
} BuzzerObject;

static BuzzerObject s_buzzer;

/* ========== Internal Functions ========== */
/**
 * @brief Buzzer thread entry
 */
static void buzzer_thread_entry(void *parameter)
{
    rt_tick_t now;
    rt_ssize_t ret;
    s_buzzer.state = BUZZER_IDLE;
    s_buzzer.next_tick = 0;
    s_buzzer.remaining_repeat = 0;
    s_buzzer.cmd.on_time = 0;
		s_buzzer.cmd.off_time = 0;
		s_buzzer.cmd.repeat = 0;

    rt_pin_mode(BUZZER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BUZZER_PIN, PIN_LOW);
    
		while (1)
    {
        now = rt_tick_get();

        /* 接收新命令（超时 10ms） */
        ret = rt_mq_recv(s_buzzer.mq, &s_buzzer.cmd, BUZZER_MQ_MSG_SIZE,
                         rt_tick_from_millisecond(10));
        if (ret > 0)
        {
            /* 立即停止当前输出 */
            rt_pin_write(BUZZER_PIN, PIN_LOW);
            s_buzzer.state = BUZZER_IDLE;
            s_buzzer.next_tick = 0;
            s_buzzer.remaining_repeat = 0;

            /* 检查停止命令（on_time==0 且 off_time==0） */
            if (s_buzzer.cmd.on_time == 0 && s_buzzer.cmd.off_time == 0)
            {
                /* 清空命令，继续等待 */
                s_buzzer.cmd.on_time = 0;
                s_buzzer.cmd.off_time = 0;
                s_buzzer.cmd.repeat = 0;
								rt_kprintf("[BUZZER] clean\n");
                continue;
            }

            /* 有效命令：初始化状态机 */
            s_buzzer.remaining_repeat = s_buzzer.cmd.repeat;
            s_buzzer.next_tick = now + rt_tick_from_millisecond(s_buzzer.cmd.on_time);
            rt_pin_write(BUZZER_PIN, PIN_HIGH);
            s_buzzer.state = BUZZER_ON;
            continue;
        }

        /* 无新命令，执行当前状态机 */
        if (s_buzzer.state == BUZZER_IDLE)
        {
            continue;   // 空闲，无动作
        }

        /* 检查是否到达切换时间 */
        if (now >= s_buzzer.next_tick)
        {
            if (s_buzzer.state == BUZZER_ON)
            {
                // 当前 ON -> 切换到 OFF
                rt_pin_write(BUZZER_PIN, PIN_LOW);
                s_buzzer.state = BUZZER_OFF;

                if (s_buzzer.remaining_repeat > 0)
                {
                    s_buzzer.remaining_repeat--;
                    if (s_buzzer.remaining_repeat == 0)
                    {
                        // 周期结束，回到空闲
                        s_buzzer.state = BUZZER_IDLE;
                        continue;
                    }
                }

                // 安排下一个 ON（如果 off_time > 0）
                if (s_buzzer.cmd.off_time > 0)
                {
                    s_buzzer.next_tick = now + rt_tick_from_millisecond(s_buzzer.cmd.off_time);
                }
                else
                {
                    // off_time=0，直接结束
                    s_buzzer.state = BUZZER_IDLE;
                }
            }
            else if (s_buzzer.state == BUZZER_OFF)
            {
                // 当前 OFF -> 切换到 ON
                rt_pin_write(BUZZER_PIN, PIN_HIGH);
                s_buzzer.state = BUZZER_ON;
                if (s_buzzer.cmd.on_time > 0)
                {
                    s_buzzer.next_tick = now + rt_tick_from_millisecond(s_buzzer.cmd.on_time);
                }
                else
                {
                    // 理论上 on_time>0，但防御
                    s_buzzer.state = BUZZER_IDLE;
                }
            }
        }
    }
}

/* ========== Public API ========== */

void buzzer_init(void)
{
    if (s_buzzer.initialized)
        return;

    /* Create message queue */
    s_buzzer.mq = rt_mq_create("buzzer_mq", BUZZER_MQ_MSG_SIZE, BUZZER_MQ_SIZE, RT_IPC_FLAG_FIFO);
    if (s_buzzer.mq == RT_NULL)
    {
        rt_kprintf("[BUZZER] Failed to create message queue\n");
        return;
    }

    /* Create thread */
    s_buzzer.thread = rt_thread_create("buzzer",
                                        buzzer_thread_entry,
                                        RT_NULL,
                                        BUZZER_THREAD_STACK_SIZE,
                                        BUZZER_THREAD_PRIORITY,
                                        5);
    if (s_buzzer.thread == RT_NULL)
    {
        rt_kprintf("[BUZZER] Failed to create thread\n");
        rt_mq_delete(s_buzzer.mq);
        return;
    }
    rt_thread_startup(s_buzzer.thread);

    s_buzzer.initialized = RT_TRUE;
    rt_kprintf("[BUZZER] Initialized\n");
}

void buzzer_start(const BuzzerCommandTypeDef *cmd)
{
    if (!s_buzzer.initialized || cmd == NULL) {
        rt_kprintf("[BUZZER] start: not initialized or cmd NULL\n");
        return;
    }
    rt_mq_send(s_buzzer.mq, cmd, BUZZER_MQ_MSG_SIZE);
}

void buzzer_beep_once(void)
{
    BuzzerCommandTypeDef cmd = {
        .on_time = 200,
        .off_time = 0,
        .repeat = 1
    };
    buzzer_start(&cmd);
		
}

void buzzer_beep_alarm(void)
{
    BuzzerCommandTypeDef cmd = {
        .on_time = 500,
        .off_time = 500,
        .repeat = 0    /* infinite loop */
    };
    buzzer_start(&cmd);
}

void buzzer_beep_error(void)
{
    BuzzerCommandTypeDef cmd = {
        .on_time = 100,
        .off_time = 100,
        .repeat = 0    /* infinite loop */
    };
    buzzer_start(&cmd);
}

void buzzer_stop(void)
{
    BuzzerCommandTypeDef cmd = {
        .on_time = 0,
        .off_time = 0,
        .repeat = 0
    };
    buzzer_start(&cmd);  /* Sending zero command will stop current pattern */
}

#ifdef RT_USING_MSH
#include <stdlib.h>   // for atoi

static void buzzer_cmd_usage(void)
{
    rt_kprintf("Usage:\n");
    rt_kprintf("  buzzer_cmd beep          - Single short beep (200ms)\n");
    rt_kprintf("  buzzer_cmd alarm         - Alarm mode (500ms on, 500ms off, infinite)\n");
    rt_kprintf("  buzzer_cmd error         - Error alarm (100ms on, 100ms off, infinite)\n");
    rt_kprintf("  buzzer_cmd stop          - Stop any ongoing pattern\n");
    rt_kprintf("  buzzer_cmd custom <on> <off> <repeat> - Custom pattern\n");
    rt_kprintf("      on: ON duration (ms)\n");
    rt_kprintf("      off: OFF duration (ms)\n");
    rt_kprintf("      repeat: number of cycles (0 = infinite)\n");
}

static void buzzer_cmd(int argc, char** argv)
{
    if (argc < 2)
    {
        buzzer_cmd_usage();
        return;
    }

    if (strcmp(argv[1], "beep") == 0)
    {
        buzzer_beep_once();
        rt_kprintf("[BUZZER] Single beep started\n");
    }
    else if (strcmp(argv[1], "alarm") == 0)
    {
        buzzer_beep_alarm();
        rt_kprintf("[BUZZER] Alarm mode started\n");
    }
    else if (strcmp(argv[1], "error") == 0)
    {
        buzzer_beep_error();
        rt_kprintf("[BUZZER] Error alarm started\n");
    }
    else if (strcmp(argv[1], "stop") == 0)
    {
        buzzer_stop();
        rt_kprintf("[BUZZER] Stopped\n");
    }
    else if (strcmp(argv[1], "custom") == 0)
    {
        if (argc < 5)
        {
            rt_kprintf("[BUZZER] Missing parameters. Usage: buzzer custom <on> <off> <repeat>\n");
            return;
        }
        BuzzerCommandTypeDef cmd;
        cmd.on_time = (uint16_t)atoi(argv[2]);
        cmd.off_time = (uint16_t)atoi(argv[3]);
        cmd.repeat = (uint16_t)atoi(argv[4]);
        buzzer_start(&cmd);
        rt_kprintf("[BUZZER] Custom pattern started: on=%dms off=%dms repeat=%d\n",
                   cmd.on_time, cmd.off_time, cmd.repeat);
    }
    else
    {
        buzzer_cmd_usage();
    }
}

MSH_CMD_EXPORT(buzzer_cmd, buzzer control);

static void buzzer_hardware_test(int argc, char** argv)
{
		  rt_pin_write(BUZZER_PIN, PIN_HIGH);
			rt_thread_mdelay(100);
	    rt_pin_write(BUZZER_PIN, PIN_LOW);
}
MSH_CMD_EXPORT(buzzer_hardware_test, buzzer control);
#endif /* RT_USING_MSH */

