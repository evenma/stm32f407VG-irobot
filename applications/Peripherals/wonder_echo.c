/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief WonderEcho AI 语音交互模块驱动 - 实现文件
 */

#include <rtthread.h>
#include "wonder_echo.h"
#include "global_conf.h"


/**
 * @brief WonderEcho 对象实例
 */
static WonderEchoObject_t s_echo;

/**
 * @brief I2C RX 中断线程句柄
 */
static rt_thread_t s_rx_thread = RT_NULL;

/**
 * @brief TTS 播报任务队列
 */
#define TTS_QUEUE_SIZE        10
static TTSTask_t s_tts_queue[TTS_QUEUE_SIZE];
static volatile uint8_t s_queue_head = 0;
static volatile uint8_t s_queue_tail = 0;
static volatile uint8_t s_queue_count = 0;
static struct rt_semaphore s_tts_sem;

/**
 * @brief TTS 处理线程
 */
static rt_thread_t s_tts_thread = RT_NULL;


/* ========== I2C 通信函数 ========== */

/**
 * @brief 读取 I2C 寄存器（单字节）
 */
static rt_err_t echo_i2c_read_reg(uint8_t reg, uint8_t* value)
{
    if (s_echo.i2c_dev == RT_NULL || !rt_device_is_available(&s_echo.i2c_dev->parent))
    {
        return -RT_ERROR;
    }
    
    if (rt_i2c_transfer(s_echo.i2c_dev, &reg, 1, value, 1) != RT_EOK)
    {
        return -RT_ERROR;
    }
    
    return RT_EOK;
}


/**
 * @brief 读取 I2C 寄存器（多字节）
 */
static rt_err_t echo_i2c_read_regs(uint8_t reg, uint8_t* buffer, rt_uint16_t length)
{
    if (s_echo.i2c_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    
    if (rt_i2c_transfer(s_echo.i2c_dev, &reg, 1, buffer, length) != RT_EOK)
    {
        return -RT_ERROR;
    }
    
    return RT_EOK;
}


/**
 * @brief 写入 I2C 寄存器（多字节）
 */
static rt_err_t echo_i2c_write_regs(uint8_t reg, const uint8_t* buffer, rt_uint16_t length)
{
    if (s_echo.i2c_dev == RT_NULL || !rt_device_is_available(&s_echo.i2c_dev->parent))
    {
        return -RT_ERROR;
    }
    
    /* 构建完整的 I2C 帧：寄存器地址 + 数据 */
    uint8_t tx_frame[258];
    tx_frame[0] = reg;
    memcpy(&tx_frame[1], buffer, length);
    
    if (rt_i2c_transfer(s_echo.i2c_dev, tx_frame, length + 1, NULL, 0) != RT_EOK)
    {
        return -RT_ERROR;
    }
    
    return RT_EOK;
}


/**
 * @brief 发送简单指令（无参数）
 */
static rt_err_t echo_send_command(uint8_t cmd)
{
    return echo_i2c_write_regs(cmd, NULL, 0);
}


/* ========== 心跳检测与状态轮询 ========== */

/**
 * @brief 检查 WonderEcho 设备是否就绪
 */
static rt_bool_t echo_check_device_ready(void)
{
    uint8_t status;
    
    /* TODO: 实际应读取状态寄存器判断
     * 这里简化为直接返回成功
     */
    echo_i2c_read_reg(0x00, &status);  /* 示例：尝试读取 WHOAMI 或状态寄存器 */
    
    return RT_TRUE;
}


/* ========== TTS 队列管理 ========== */

static rt_bool_t tts_queue_is_full(void)
{
    return s_queue_count >= TTS_QUEUE_SIZE;
}


static rt_bool_t tts_queue_is_empty(void)
{
    return s_queue_count == 0;
}


static int tts_queue_push(const char* text, uint8_t volume)
{
    if (tts_queue_is_full())
    {
        return -RT_ERROR;
    }
    
    s_tts_queue[s_queue_head].text = text;
    s_tts_queue[s_queue_head].volume = volume;
    s_tts_queue[s_queue_head].wait_finish = RT_TRUE;
    
    s_queue_head = (s_queue_head + 1) % TTS_QUEUE_SIZE;
    s_queue_count++;
    
    /* 信号量通知 */
    rt_sem_release(&s_tts_sem);
    
    return 0;
}


static int tts_queue_pop(TTSTask_t* task_out)
{
    if (tts_queue_is_empty())
    {
        return -RT_ERROR;
    }
    
    *task_out = s_tts_queue[s_queue_tail];
    
    s_queue_tail = (s_queue_tail + 1) % TTS_QUEUE_SIZE;
    s_queue_count--;
    
    return 0;
}


void wonder_echo_clear_tts_queue(void)
{
    s_queue_head = s_queue_tail;
    s_queue_count = 0;
}


/* ========== TTS 处理线程 ========== */

static void tts_task_entry(void* parameter)
{
    while (1)
    {
        /* 等待队列中有数据 */
        rt_sem_take(&s_tts_sem, RT_WAITING_FOREVER);
        
        TTSTask_t task;
        
        while (tts_queue_pop(&task) == 0)
        {
            if (s_echo.on_tts_done != RT_NULL)
            {
                s_echo.is_tts_playing = RT_TRUE;
                
                /* TODO: 实际的 TTS 播放逻辑
                 * 这里需要调用 WonderEcho 的语音合成接口
                 * 
                 * 例如：
                 * echo_send_tts_text(task.text, task.volume);
                 */
                 
                rt_kprintf("[Echo] TTS playing: \"%s\"@vol%d\n", 
                          task.text, task.volume);
                
                /* 模拟播放完成（阻塞式）*/
                if (task.wait_finish)
                {
                    rt_thread_mdelay(2000);  /* 模拟 2 秒播放时间 */
                }
                
                if (s_echo.on_tts_done != RT_NULL)
                {
                    s_echo.is_tts_playing = RT_FALSE;
                    s_echo.on_tts_done();
                }
            }
            
            rt_thread_mdelay(100);  // 避免忙等待
        }
    }
}


/* ========== 公共 API 实现 ========== */

int wonder_echo_init(WonderEchoMode_t mode, uint8_t volume)
{
    rt_err_t err;
    
    /* 1. 查找 I2C 设备 */
    s_echo.i2c_dev = rt_i2c_bus_device_find(WONDERECHO_I2C_BUS);
    if (s_echo.i2c_dev == RT_NULL)
    {
        rt_kprintf("[Echo] I2C device not found: %s\n", WONDERECHO_I2C_BUS);
        return -RT_ERROR;
    }
    
    /* 2. 配置 I2C 参数 */
    struct rt_i2c_bus_config_info config = {
        .freq = RT_I2C_SPEED_STANDARD,  // 100kHz
        .bus_mode = RT_I2C_MODE_MASTER,
    };
    rt_i2c_bus_configure(s_echo.i2c_dev, &config);
    
    /* 3. 初始化数据结构 */
    memset(&s_echo, 0, sizeof(WonderEchoObject_t));
    s_echo.initialized = RT_TRUE;
    s_echo.mode = mode;
    s_echo.volume = volume;
    s_echo.timeout_ms = 500;
    
    /* 4. 注册默认回调 */
    s_echo.on_wakeup = RT_NULL;
    s_echo.on_recognized = RT_NULL;
    s_echo.on_tts_done = RT_NULL;
    s_echo.on_error = RT_NULL;
    
    /* 5. 创建 TTS 信号量和队列 */
    s_tts_sem = rt_sem_create("tts", 0, RT_IPC_FLAG_PRIO);
    
    /* 6. 创建 TTS 处理线程 */
    s_tts_thread = rt_thread_create("echo_tts",
                                    tts_task_entry,
                                    RT_NULL,
                                    1024,
                                    RT_THREAD_PRIORITY_MAX / 4,
                                    20);
    
    if (s_tts_thread != RT_NULL)
    {
        rt_thread_startup(s_tts_thread);
    }
    else
    {
        rt_kprintf("[Echo] Failed to create TTS thread!\n");
    }
    
    /* 7. 验证设备连接 */
    if (!echo_check_device_ready())
    {
        rt_kprintf("[Echo] Device check failed!\n");
    }
    else
    {
        rt_kprintf("[Echo] WonderEcho initialized on %s, mode=%d, vol=%d\n", 
                   WONDERECHO_I2C_BUS, mode, volume);
    }
    
    return 0;
}


void wonder_echo_deinit(void)
{
    if (s_echo.i2c_dev != RT_NULL)
    {
        rt_i2c_disconnect(s_echo.i2c_dev);
        s_echo.i2c_dev = RT_NULL;
    }
    
    s_echo.initialized = RT_FALSE;
    
    if (s_tts_thread != RT_NULL)
    {
        rt_thread_delete(s_tts_thread);
        s_tts_thread = RT_NULL;
    }
}


int wonder_echo_set_mode(WonderEchoMode_t mode)
{
    if (!s_echo.initialized)
    {
        return -RT_ERROR;
    }
    
    s_echo.mode = mode;
    
    /* TODO: 发送到 WonderEcho 设置模式 */
    echo_send_command(0x0A);  /* 示例命令 */
    
    return 0;
}


WonderEchoMode_t wonder_echo_get_mode(void)
{
    return s_echo.mode;
}


int wonder_echo_set_volume(uint8_t vol)
{
    if (!s_echo.initialized)
    {
        return -RT_ERROR;
    }
    
    if (vol > VOLUME_MAX)
    {
        vol = VOLUME_MAX;
    }
    
    s_echo.volume = vol;
    
    /* 发送音量设置命令 */
    uint8_t data[2] = {0x0B, vol};  /* 假设命令地址为 0x0B */
    echo_i2c_write_regs(0x0B, data, 1);
    
    return 0;
}


uint8_t wonder_echo_get_volume(void)
{
    return s_echo.volume;
}


rt_bool_t wonder_echo_is_waking(void)
{
    return s_echo.is_waking;
}


void wonder_echo_play_keypress(uint8_t type)
{
    if (!s_echo.initialized)
    {
        return;
    }
    
    switch (type)
    {
        case 0:  // 短音
            echo_send_command(0x20);  /* 示例按键音命令 */
            break;
        case 1:  // 双音
            echo_send_command(0x21);
            break;
        case 2:  // 长音
            echo_send_command(0x22);
            break;
        default:
            break;
    }
}


void wonder_echo_stop_audio(void)
{
    if (!s_echo.initialized)
    {
        return;
    }
    
    echo_send_command(0x30);  /* 停止音频播放命令 */
}


int wonder_echo_speak_now(const char* text, uint8_t volume)
{
    if (!s_echo.initialized || text == RT_NULL)
    {
        return -RT_ERROR;
    }
    
    /* 同步模式：直接添加到队列并等待完成 */
    int result = tts_queue_push(text, volume);
    
    if (result != 0)
    {
        return result;
    }
    
    /* 等待播放完成 */
    while (s_echo.is_tts_playing)
    {
        rt_thread_mdelay(100);
    }
    
    return 0;
}


int wonder_echo_queue_tts(const char* text, uint8_t volume)
{
    if (!s_echo.initialized || text == RT_NULL)
    {
        return -RT_ERROR;
    }
    
    return tts_queue_push(text, volume);
}


void wonder_echo_force_listen(void)
{
    if (!s_echo.initialized)
    {
        return;
    }
    
    /* 强制进入聆听模式 */
    echo_send_command(0x10);  /* 示例：开始监听命令 */
}


int wonder_echo_read_recognition(SpeechRecognition_t* result_out)
{
    if (!s_echo.initialized || result_out == RT_NULL)
    {
        return -RT_ERROR;
    }
    
    /* TODO: 从 I2C 读取 ASR 结果
     * 这里简化为模拟数据
     */
    result_out->is_wakeup = RT_FALSE;
    snprintf((char*)result_out->recognition_result, 64, "[模拟] 测试识别结果");
    result_out->confidence = 95;
    result_out->duration_ms = 1500;
    
    return 0;
}


void wonder_echo_clear_recognition(void)
{
    /* 清空内部识别结果缓存 */
}


rt_bool_t wonder_echo_has_new_recognition(void)
{
    return RT_FALSE;  /* 简化实现 */
}


/* ========== 回调函数注册 ========== */

void wonder_echo_register_on_wakeup(void (*callback)(void))
{
    s_echo.on_wakeup = callback;
}


void wonder_echo_register_on_recognized(void (*callback)(const char* result))
{
    s_echo.on_recognized = callback;
}


void wonder_echo_register_on_tts_done(void (*callback)(void))
{
    s_echo.on_tts_done = callback;
}


void wonder_echo_register_on_error(void (*callback)(WonderEchoError_t err))
{
    s_echo.on_error = callback;
}


/* ========== MSH 调试命令 ========== */

#ifdef RT_USING_MSH

#include <msh.h>

MSH_CMD_DEFINE(echo_status, "echo status",       "Show WonderEcho status")
MSH_CMD_DEFINE(echo_test,   "echo test [text]",  "Test TTS (default: 'hello world')")
MSH_CMD_DEFINE(echo_mode,   "echo mode [0|1|2]", "Set mode: 0-free, 1-continuous, 2-disable")
MSH_CMD_DEFINE(echo_vol,    "echo vol [0-100]",  "Set volume")

#endif /* RT_USING_MSH */

#ifdef RT_USING_MSH

static void msh_wonder_echo_status(int argc, char** argv)
{
    rt_kprintf("\n========== WonderEcho Status ==========\n");
    rt_kprintf("Status: %s\n", s_echo.initialized ? "Active" : "Not initialized");
    rt_kprintf("Mode: %s\n", s_echo.mode == MODE_FREE_TRIGGER ? "Free Trigger" :
                              s_echo.mode == MODE_CONTINUOUS_TRIGGER ? "Continuous" : "No Wakeup");
    rt_kprintf("Volume: %d\n", s_echo.volume);
    rt_kprintf("Waking: %s\n", s_echo.is_waking ? "Yes" : "No");
    rt_kprintf("TTS Playing: %s\n", s_echo.is_tts_playing ? "Yes" : "No");
    rt_kprintf("Queue: %d/%d tasks\n", s_queue_count, TTS_QUEUE_SIZE);
    rt_kprintf("Total commands: %lu\n", s_echo.total_commands);
    rt_kprintf("Errors: %lu\n", s_echo.error_count);
    rt_kprintf("========================================\n\n");
}


static void msh_wonder_echo_test(int argc, char** argv)
{
    const char* text = "Hello, welcome to iHomeRobot!";
    
    if (argc > 1)
    {
        text = argv[1];
    }
    
    rt_kprintf("[Echo] Testing TTS: \"%s\"\n", text);
    
    int ret = wonder_echo_speak_now(text, 80);
    
    if (ret == 0)
    {
        rt_kprintf("[Echo] TTS completed successfully!\n");
    }
    else
    {
        rt_kprintf("[Echo] TTS failed with error code: %d\n", ret);
    }
}


static void msh_wonder_echo_mode(int argc, char** argv)
{
    WonderEchoMode_t mode = s_echo.mode;
    
    if (argc > 1)
    {
        mode = atoi(argv[1]);
        if (mode > MODE_DISABLE_WAKEUP)
        {
            mode = MODE_FREE_TRIGGER;
        }
    }
    
    rt_kprintf("[Echo] Setting mode to: ");
    if (mode == MODE_FREE_TRIGGER) rt_kprintf("Free Trigger\n");
    else if (mode == MODE_CONTINUOUS_TRIGGER) rt_kprintf("Continuous\n");
    else rt_kprintf("Disable Wakeup\n");
    
    wonder_echo_set_mode(mode);
}


static void msh_wonder_echo_volume(int argc, char** argv)
{
    uint8_t vol = s_echo.volume;
    
    if (argc > 1)
    {
        vol = (uint8_t)atoi(argv[1]);
        if (vol > VOLUME_MAX) vol = VOLUME_MAX;
    }
    
    rt_kprintf("[Echo] Setting volume to %d%%\n", vol);
    wonder_echo_set_volume(vol);
}

#endif /* RT_USING_MSH */
