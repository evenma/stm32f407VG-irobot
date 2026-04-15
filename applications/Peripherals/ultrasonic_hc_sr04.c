#include "ultrasonic_hc_sr04.h"
#include "global_conf.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

typedef enum {
    SEND_MODE_PER_SENSOR,   /* 每个传感器测完立即发送 */
    SEND_MODE_PER_ROUND     /* 一轮全部测完后统一发送 */
} send_mode_t;

static send_mode_t current_send_mode = SEND_MODE_PER_ROUND;  /* 默认一轮读取完成再发送 */

/* 硬件定时器设备句柄 */
static rt_device_t hw_timer = RT_NULL;
static struct rt_hwtimerval timeout_val;
static struct rt_completion echo_completion;
static volatile uint8_t measuring = 0;
static volatile uint32_t pulse_width_us = 0;
static volatile uint8_t timeout_occurred = 0;

/* 轮询线程相关变量 */
static rt_thread_t poll_thread = RT_NULL;
static uint8_t poll_running = 0;
static struct rt_mutex poll_mutex;   /* 互斥锁，避免与手动命令冲突 */

static void ultrasonic_poll(void);
static uint8_t poll_print_enabled = 0;   /* 默认关闭打印 */

/* 距离缓存（用于 OLED 显示）*/
static uint32_t s_hc_distances[HC_SR04_NUM] = {0};
static struct rt_mutex s_hc_dist_mutex;

/* 译码器真值表 */
//static const uint8_t decoder_table[8][3] = {
//    {0,0,0}, {1,0,0}, {0,1,0}, {1,1,0},
//    {0,0,1}, {1,0,1}, {0,1,1}, {1,1,1}
//};
static const uint8_t decoder_table[8][3] = {
    {0,0,0}, {1,0,0}, {0,0,1}, {1,0,1},
    {0,1,0}, {1,1,0}, {0,1,1}, {1,1,1}
};

static void select_sensor(uint8_t index)
{
    if (index >= HC_SR04_NUM) return;
    rt_pin_write(ULTRASONIC_A_PIN, decoder_table[index][0]);
    rt_pin_write(ULTRASONIC_B_PIN, decoder_table[index][1]);
    rt_pin_write(ULTRASONIC_C_PIN, decoder_table[index][2]);
    rt_thread_mdelay(1);
}

static void send_trig_pulse(void)
{
    rt_pin_write(ULTRASONIC_TRIG_PIN, PIN_HIGH);
    rt_hw_us_delay(10);
    rt_pin_write(ULTRASONIC_TRIG_PIN, PIN_LOW);
}

/* 定时器超时回调（在中断上下文中）*/
static rt_err_t timer_timeout_cb(rt_device_t dev, rt_size_t size)
{
    if (measuring) {
        timeout_occurred = 1;
        measuring = 0;
        rt_completion_done(&echo_completion);
    }
		return 0;
}

/* Echo 中断回调 */
static void echo_irq_callback(void *args)
{
    if (rt_pin_read(ULTRASONIC_ECHO_PIN) == PIN_HIGH) {
        /* 上升沿：启动定时器（单次模式，超时 70ms）*/
        if (measuring) {
            rt_device_write(hw_timer, 0, &timeout_val, sizeof(timeout_val));
        }
    } else {
        /* 下降沿：停止定时器，读取经过时间 */
        if (measuring) {
            /* 读取当前计数值（即脉冲宽度，us）*/
            struct rt_hwtimerval t;
            if (rt_device_read(hw_timer, 0, &t, sizeof(t)) == sizeof(t)) {
                pulse_width_us = t.sec * 1000000UL + t.usec;
            } else {
                pulse_width_us = 0;
            }
            measuring = 0;
						/* 停止定时器 */
            rt_device_control(hw_timer, HWTIMER_CTRL_STOP, RT_NULL);
            rt_completion_done(&echo_completion);
        }
    }
}

/* 硬件定时器初始化（单次模式，1MHz）*/
static rt_err_t hwtimer_init(void)
{
    rt_err_t ret;
    rt_hwtimer_mode_t mode;
    uint32_t freq = 1000000;   // 1MHz

    hw_timer = rt_device_find("timer3");
    if (hw_timer == RT_NULL) {
        rt_kprintf("[HC-SR04] Cannot find timer3 device\n");
        return -RT_ERROR;
    }

    ret = rt_device_open(hw_timer, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) {
        rt_kprintf("[HC-SR04] Open timer3 failed\n");
        return ret;
    }

    /* 设置计数频率 */
    ret = rt_device_control(hw_timer, HWTIMER_CTRL_FREQ_SET, &freq);
    if (ret != RT_EOK) {
        rt_kprintf("[HC-SR04] Set timer frequency failed\n");
        return ret;
    }

    /* 设置为单次模式 */
    mode = HWTIMER_MODE_ONESHOT;
    ret = rt_device_control(hw_timer, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK) {
        rt_kprintf("[HC-SR04] Set timer mode failed\n");
        return ret;
    }
		
		timeout_val.sec = 0;
		timeout_val.usec = 50000;   // 50ms，大于最大脉冲宽度（约 32ms）
		//rt_device_write(hw_timer, 0, &timeout_val, sizeof(timeout_val));在上升沿中断开启
    /* 注册超时回调 */
    rt_device_set_rx_indicate(hw_timer, timer_timeout_cb);

    rt_kprintf("[HC-SR04] Timer initialized (1MHz, oneshot mode)\n");
    return RT_EOK;
}

static rt_err_t gpio_init(void)
{
    rt_pin_mode(ULTRASONIC_A_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(ULTRASONIC_B_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(ULTRASONIC_C_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(ULTRASONIC_A_PIN, PIN_LOW);
    rt_pin_write(ULTRASONIC_B_PIN, PIN_LOW);
    rt_pin_write(ULTRASONIC_C_PIN, PIN_LOW);

    rt_pin_mode(ULTRASONIC_TRIG_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(ULTRASONIC_TRIG_PIN, PIN_LOW);

    rt_pin_mode(ULTRASONIC_ECHO_PIN, PIN_MODE_INPUT);
    rt_pin_attach_irq(ULTRASONIC_ECHO_PIN, PIN_IRQ_MODE_RISING_FALLING, echo_irq_callback, RT_NULL);
    rt_pin_irq_enable(ULTRASONIC_ECHO_PIN, PIN_IRQ_ENABLE);

    return RT_EOK;
}

/* 单次测量（内部）*/
static rt_err_t measure_once(uint8_t index, uint32_t *distance_mm, uint32_t timeout_ms)
{
    if (measuring) return -RT_EBUSY;

    select_sensor(index);
    pulse_width_us = 0;
    timeout_occurred = 0;
    measuring = 1;

    /* 清除可能残留的中断状态 */
    rt_pin_irq_enable(ULTRASONIC_ECHO_PIN, PIN_IRQ_DISABLE);
    rt_pin_irq_enable(ULTRASONIC_ECHO_PIN, PIN_IRQ_ENABLE);

    /* 发送 Trig 脉冲 */
    send_trig_pulse();

    /* 等待下降沿或超时（用户指定的超时时间，例如 100ms 必须大于超声波最大值50ms）*/
		if(timeout_ms < 50){
			timeout_ms = 50;
		}
    rt_err_t ret = rt_completion_wait(&echo_completion, rt_tick_from_millisecond(timeout_ms));
    measuring = 0;

    if (ret != RT_EOK) {
        /* 用户超时，需要停止可能还在运行的定时器 */
        rt_device_control(hw_timer, HWTIMER_CTRL_STOP, RT_NULL);
//				rt_kprintf("user set timeout\n");
        return -RT_ETIMEOUT;
    }

    if (timeout_occurred) {
//				rt_kprintf("timer3 timeout\n");
        return -RT_ETIMEOUT;
    }

    if (pulse_width_us == 0) {
//				rt_kprintf("pulse_width_us = 0\n");
        return -RT_ERROR;
    }

    *distance_mm = (uint32_t)((float)pulse_width_us * 0.17f);
//		rt_kprintf("pulse_width_us: %d us\n", pulse_width_us);
    return RT_EOK;
}

/* 外部接口：中值滤波版读取距离 */
rt_err_t hc_sr04_read_distance(uint8_t index, uint32_t *distance_mm, uint32_t timeout_ms)
{
    if (index >= HC_SR04_NUM || distance_mm == NULL) {
        return -RT_ERROR;
    }

    uint32_t samples[3];
    uint8_t valid_cnt = 0;

    for (int i = 0; i < 3; i++) {
        if (measure_once(index, &samples[valid_cnt], timeout_ms) == RT_EOK) {
            valid_cnt++;
        }
        rt_thread_mdelay(20);
        if (valid_cnt == 3) break;
    }

    if (valid_cnt == 0) {
        return -RT_ETIMEOUT;
    }

    /* 排序取中位数 */
    for (int i = 0; i < valid_cnt - 1; i++) {
        for (int j = 0; j < valid_cnt - i - 1; j++) {
            if (samples[j] > samples[j+1]) {
                uint32_t tmp = samples[j];
                samples[j] = samples[j+1];
                samples[j+1] = tmp;
            }
        }
    }
    *distance_mm = samples[valid_cnt / 2];

    /* 异常值过滤 */
    if (*distance_mm > 5000 || *distance_mm < 20) {
        rt_thread_mdelay(30);
        if (measure_once(index, distance_mm, timeout_ms) != RT_EOK) {
            return -RT_ERROR;
        }
    }

    return RT_EOK;
}

void hc_sr04_get_distances(uint32_t *dist_array, uint8_t len)
{
    uint8_t i;
    uint8_t copy_len = len < HC_SR04_NUM ? len : HC_SR04_NUM;
    rt_mutex_take(&s_hc_dist_mutex, RT_WAITING_FOREVER);
    for (i = 0; i < copy_len; i++) {
        dist_array[i] = s_hc_distances[i];
    }
    rt_mutex_release(&s_hc_dist_mutex);
}

/* 初始化互斥锁（在驱动初始化函数中调用） */
static void poll_mutex_init(void)
{
    rt_mutex_init(&poll_mutex, "hc_mutex", RT_IPC_FLAG_PRIO);
}

/* 驱动初始化 */
rt_err_t hc_sr04_init(void)
{
    rt_err_t ret;

    ret = gpio_init();
    if (ret != RT_EOK) return ret;

    ret = hwtimer_init();
    if (ret != RT_EOK) return ret;

    rt_completion_init(&echo_completion);
	
		poll_mutex_init();
		rt_mutex_init(&s_hc_dist_mutex, "hc_dist", RT_IPC_FLAG_PRIO);
	
		ultrasonic_poll();

    rt_kprintf("[HC-SR04] Driver initialized (oneshot timer mode)\n");
    return RT_EOK;
}

static int32_t rt_tick_to_millisecond(rt_tick_t tick_diff)
{
    // 返回 tick_diff * 1000 / RT_TICK_PER_SECOND
    return (int32_t)((tick_diff * 1000) / RT_TICK_PER_SECOND);
}

/* 轮询线程入口：每秒轮询所有 8 个传感器 */
static void poll_entry(void *parameter)
{
    uint32_t dist_mm;
    rt_err_t ret;
    uint8_t i;
    char single_buf[64];
    char round_buf[256];
    int len;

    while (poll_running)
    {
			  rt_tick_t start_tick = rt_tick_get();
        if (current_send_mode == SEND_MODE_PER_ROUND)
        {
            len = rt_snprintf(round_buf, sizeof(round_buf), "[Poll] ");
        }

        for (i = 0; i < HC_SR04_NUM; i++)
        {
            rt_mutex_take(&poll_mutex, RT_WAITING_FOREVER);
            ret = measure_once(i, &dist_mm, 100);
            rt_mutex_release(&poll_mutex);
					    // 更新距离缓存（用于 OLED）
						if (ret == RT_EOK) {
								rt_mutex_take(&s_hc_dist_mutex, RT_WAITING_FOREVER);
								s_hc_distances[i] = dist_mm;
								rt_mutex_release(&s_hc_dist_mutex);
						} else {
								rt_mutex_take(&s_hc_dist_mutex, RT_WAITING_FOREVER);
								s_hc_distances[i] = 0;   // 无效值，OLED 会显示 ---
								rt_mutex_release(&s_hc_dist_mutex);
						}

            if (current_send_mode == SEND_MODE_PER_SENSOR)
            {
                /* 立即发送单个传感器数据 */
                if (ret == RT_EOK)
                {
                    rt_snprintf(single_buf, sizeof(single_buf), "S%d=%dmm\n", i, dist_mm);
                }
                else
                {
                    rt_snprintf(single_buf, sizeof(single_buf), "S%d=ERR\n", i);
                }
								if (poll_print_enabled)
									rt_kprintf("%s", single_buf);  /* 可改为串口直接输出，或使用回调发送至上位机 */
            }
            else  /* SEND_MODE_PER_ROUND */
            {
                if (ret == RT_EOK)
                {
                    len += rt_snprintf(round_buf + len, sizeof(round_buf) - len,
                                       "S%d=%dmm ", i, dist_mm);
                }
                else
                {
                    len += rt_snprintf(round_buf + len, sizeof(round_buf) - len,
                                       "S%d=ERR ", i);
                }
            }
            rt_thread_mdelay(5);
        }

				if (current_send_mode == SEND_MODE_PER_ROUND && poll_print_enabled)
        {
            rt_kprintf("%s\n", round_buf);
        }
				if (current_send_mode == SEND_MODE_PER_SENSOR && poll_print_enabled)
        {
            rt_kprintf("\n-------\n");
        }
				rt_tick_t elapsed = rt_tick_get() - start_tick;
        int32_t delay_ms = POLL_INTERVAL_MS - rt_tick_to_millisecond(elapsed);
        if (delay_ms > 0)
            rt_thread_mdelay(delay_ms);
				else 
					rt_thread_mdelay(10);    // 超时，下一次轮询过渡时间10ms
    }
}

/* 启动 / 停止轮询的 MSH 命令 */
static void ultrasonic_poll(void)
{
		poll_running = 1;
		poll_thread = rt_thread_create("ultrasonic",
																		poll_entry,
																		RT_NULL,
																		2048,
																		17,
																		10);
		if (poll_thread == RT_NULL)
		{
				poll_running = 0;
				rt_kprintf("Failed to create poll thread\n");
				return;
		}
		rt_thread_startup(poll_thread);
		rt_kprintf("Polling started (all 8 sensors, interval 1s)\n");
}

#ifdef RT_USING_MSH
#include <stdlib.h>
static void hc_sr04_cmd(int argc, char **argv)
{
    uint32_t dist;
    uint8_t idx = 0;
    if (argc > 1) {
        idx = (uint8_t)atoi(argv[1]);
        if (idx >= HC_SR04_NUM) {
            rt_kprintf("Invalid index (0~7)\n");
            return;
        }
    }
    /* 加锁保护 */
    rt_mutex_take(&poll_mutex, RT_WAITING_FOREVER);
    rt_err_t ret = measure_once(idx, &dist, 100);
    rt_mutex_release(&poll_mutex);
    
    if (ret == RT_EOK) {
        rt_kprintf("Sensor %d distance: %d mm\n", idx, dist);
    } else {
        rt_kprintf("Sensor %d read failed/timeout\n", idx);
    }
}
MSH_CMD_EXPORT(hc_sr04_cmd, "hc_sr04_cmd [0-7] - read ultrasonic distance");

static void hc_sr04_sendmode(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Current send mode: %s\n",
                   current_send_mode == SEND_MODE_PER_SENSOR ? "per_sensor" : "per_round");
        rt_kprintf("Usage: hc_sr04_sendmode [per_sensor|per_round]\n");
        return;
    }

    if (rt_strcmp(argv[1], "per_sensor") == 0)
    {
        current_send_mode = SEND_MODE_PER_SENSOR;
        rt_kprintf("Send mode changed to: per sensor (immediate)\n");
    }
    else if (rt_strcmp(argv[1], "per_round") == 0)
    {
        current_send_mode = SEND_MODE_PER_ROUND;
        rt_kprintf("Send mode changed to: per round (batch)\n");
    }
    else
    {
        rt_kprintf("Invalid mode. Choose 'per_sensor' or 'per_round'.\n");
    }
}
MSH_CMD_EXPORT(hc_sr04_sendmode, "hc_sr04_sendmode [per_sensor|per_round] - set data send mode");

/*开启/关闭轮询打印 */
static void hc_sr04_print(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Current print status: %s\n", poll_print_enabled ? "on" : "off");
        rt_kprintf("Usage: hc_sr04_print on/off\n");
        return;
    }
    if (rt_strcmp(argv[1], "on") == 0)
    {
        poll_print_enabled = 1;
        rt_kprintf("Poll printing enabled\n");
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        poll_print_enabled = 0;
        rt_kprintf("Poll printing disabled\n");
    }
    else
    {
        rt_kprintf("Invalid argument. Use 'on' or 'off'.\n");
    }
}
MSH_CMD_EXPORT(hc_sr04_print, "hc_sr04_print on/off - enable/disable poll data printing");
#endif
