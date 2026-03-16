/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief WonderEcho AI 语音交互模块驱动
 * 
 * 功能：
 *   - I2C 通信协议
 *   - 语音唤醒词识别（自由触发、连续触发、禁用模式）
 *   - TTS 文本转语音播报
 *   - ASR 自动语音识别
 */

#ifndef PERIPHERALS_WONDER_ECHO_H__
#define PERIPHERALS_WONDER_ECHO_H__

#include <rtthread.h>


/**
 * @brief I2C 接口配置 (根据 global_conf.h)
 */
#ifndef WONDERECHO_I2C_BUS
#define WONDERECHO_I2C_BUS    "i2c3"  // PB8(SCL), PB9(SDA)
#endif

/**
 * @brief WonderEcho 设备地址（可根据 DIO 引脚配置）
 */
#define WONDRECHO_ADDR_DEFAULT   0x1C  // DIO=GND
#define WONDRECHO_ADDR_ALT       0x1D  // DIO=VCC


/**
 * @brief 工作模式
 */
typedef enum
{
    MODE_FREE_TRIGGER = 0,        // 自由触发模式（单次唤醒，说完即停）
    MODE_CONTINUOUS_TRIGGER = 1,  // 连续触发模式（唤醒后可连续对话）
    MODE_DISABLE_WAKEUP = 2,      // 禁用唤醒词，仅接收指令
} WonderEchoMode_t;


/**
 * @brief 音频输出音量（0~100）
 */
#define VOLUME_MIN           0
#define VOLUME_MAX           100
#define VOLUME_DEFAULT       50


/**
 * @brief 音频采样率
 */
#define AUDIO_SAMPLE_RATE    16000    // 16kHz


/**
 * @brief 命令标识符
 */
typedef enum
{
    CMD_NONE = 0,
    CMD_WAKE_WORD_DETECTED,         // 唤醒词已识别
    CMD_START_LISTENING,            // 开始监听
    CMD_SPEECH_RECOGNIZED,          // 语音识别结果
    CMD_TTS_STARTED,                // TTS 开始播放
    CMD_TTS_FINISHED,               // TTS 播放完成
    CMD_ERROR_OCCURRED,             // 发生错误
    CMD_HEARTBEAT_RESPONSE,         // 心跳响应
} EchoCommand_t;


/**
 * @brief 错误类型
 */
typedef enum
{
    ERR_NONE = 0,
    ERR_I2C_COMM_FAIL,              // I2C 通信失败
    ERR_TIMEOUT,                    // 操作超时
    ERR_ILLEGAL_PARAM,              // 非法参数
    ERR_MEMORY_FULL,                // 内存不足
    ERR_AUDIO_FORMAT_NOT_SUPPORT,   // 音频格式不支持
    ERR_HARDWARE_FAULT,             // 硬件故障
} WonderEchoError_t;


/* ========== 数据结构 ========== */

/**
 * @brief 语音识别结果
 */
typedef struct
{
    uint8_t is_wakeup;              // 是否唤醒词触发
    uint8_t recognition_result[64]; // 识别出的文字内容（UTF-8 编码）
    uint16_t confidence;            // 置信度 (0~100)
    uint32_t duration_ms;           // 语音时长（毫秒）
} SpeechRecognition_t;


/**
 * @brief TTS 任务参数
 */
typedef struct
{
    const char* text;               // 要播报的文本
    uint8_t volume;                 // 音量 (0~100)
    rt_bool_t wait_finish;          // 是否等待播放完成
} TTSTask_t;


/**
 * @brief WonderEcho 对象结构体
 */
typedef struct
{
    rt_bool_t initialized;          // 是否已初始化
    struct rt_i2c_device* i2c_dev;  // I2C 设备句柄
    
    /* 配置参数 */
    WonderEchoMode_t mode;          // 当前工作模式
    uint8_t volume;                 // 当前音量
    uint16_t timeout_ms;            // 操作超时时间（默认 500ms）
    
    /* 状态标志 */
    rt_bool_t is_waking;            // 是否正在唤醒中
    rt_bool_t is_tts_playing;       // 是否正在播放 TTS
    
    /* 回调函数 */
    void (*on_wakeup)(void);        // 唤醒回调
    void (*on_recognized)(const char* result);  // 识别结果回调
    void (*on_tts_done)(void);      // TTS 完成回调
    void (*on_error)(WonderEchoError_t err);    // 错误回调
    
    /* 内部缓冲区 */
    uint8_t tx_buffer[256];         // 发送缓冲区
    uint8_t rx_buffer[256];         // 接收缓冲区
    
    /* 统计信息 */
    uint32_t total_commands;        // 总命令数
    uint32_t error_count;           // 错误计数
} WonderEchoObject_t;


/* ========== 公共 API ========== */

/**
 * @brief 初始化 WonderEcho 模块
 * @param mode 工作模式
 * @param volume 初始音量
 * @return 0 成功，负值失败
 */
int wonder_echo_init(WonderEchoMode_t mode, uint8_t volume);

/**
 * @brief 反初始化并释放资源
 */
void wonder_echo_deinit(void);

/**
 * @brief 设置工作模式
 */
int wonder_echo_set_mode(WonderEchoMode_t mode);

/**
 * @brief 获取当前工作模式
 */
WonderEchoMode_t wonder_echo_get_mode(void);

/**
 * @brief 设置音量
 * @param vol 0~100
 * @return 0 成功
 */
int wonder_echo_set_volume(uint8_t vol);

/**
 * @brief 获取当前音量
 */
uint8_t wonder_echo_get_volume(void);

/**
 * @brief 检测是否正在唤醒中
 */
rt_bool_t wonder_echo_is_waking(void);

/**
 * @brief 发送简单的按键音效（如确认音）
 * @param type 0=短音，1=双音，2=长音
 */
void wonder_echo_play_keypress(uint8_t type);

/**
 * @brief 停止所有音频输出
 */
void wonder_echo_stop_audio(void);


/* ========== TTS 文本播报 ========== */

/**
 * @brief 立即播报一段文本（阻塞式）
 * @param text 要播报的内容
 * @param volume 音量
 * @return 0 成功
 */
int wonder_echo_speak_now(const char* text, uint8_t volume);

/**
 * @brief 添加 TTS 播报任务到队列
 * @param text 要播报的内容
 * @param volume 音量
 * @return 0 成功
 */
int wonder_echo_queue_tts(const char* text, uint8_t volume);

/**
 * @brief 清空 TTS 队列
 */
void wonder_echo_clear_tts_queue(void);


/* ========== ASR 语音识别相关 ========== */

/**
 * @brief 强制唤醒（忽略唤醒词直接进入聆听模式）
 */
void wonder_echo_force_listen(void);

/**
 * @brief 读取最新识别结果
 * @param result_out 结果输出指针
 * @return 0 有有效数据
 */
int wonder_echo_read_recognition(SpeechRecognition_t* result_out);

/**
 * @brief 清除识别结果
 */
void wonder_echo_clear_recognition(void);

/**
 * @brief 检查是否有新的识别结果待处理
 */
rt_bool_t wonder_echo_has_new_recognition(void);


/* ========== 回调函数注册 ========== */

/**
 * @brief 注册唤醒回调
 */
void wonder_echo_register_on_wakeup(void (*callback)(void));

/**
 * @brief 注册识别结果回调
 */
void wonder_echo_register_on_recognized(void (*callback)(const char* result));

/**
 * @brief 注册 TTS 完成回调
 */
void wonder_echo_register_on_tts_done(void (*callback)(void));

/**
 * @brief 注册错误回调
 */
void wonder_echo_register_on_error(void (*callback)(WonderEchoError_t err));


/* ========== 调试工具 ========== */

#ifdef RT_USING_MSH

/**
 * @brief MSH 命令：显示当前状态
 */
void msh_wonder_echo_status(int argc, char** argv);

/**
 * @brief MSH 命令：测试播报（hello 为测试文本）
 */
void msh_wonder_echo_test(int argc, char** argv);

/**
 * @brief MSH 命令：设置模式
 */
void msh_wonder_echo_mode(int argc, char** argv);

/**
 * @brief MSH 命令：设置音量
 */
void msh_wonder_echo_volume(int argc, char** argv);

#endif /* RT_USING_MSH */


#endif /* PERIPHERALS_WONDER_ECHO_H__ */
