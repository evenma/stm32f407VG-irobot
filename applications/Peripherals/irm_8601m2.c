#include <rtdevice.h>
#include <string.h>
#include <board.h>
#include "irm_8601m2.h"
#include "global_conf.h"
#include "uart_packet.h"
#include "packet_reports.h"

/* ======================== 硬件引脚定义 ======================== */
/* 从 global_conf.h 中引用实际引脚定义 */
#ifndef IR_SENSOR_LEFT_PIN
#define IR_SENSOR_LEFT_PIN   GET_PIN(E, 13)   // 与 IR_SENSOR1_PIN 对应
#endif
#ifndef IR_SENSOR_RIGHT_PIN
#define IR_SENSOR_RIGHT_PIN  GET_PIN(E, 14)   // 与 IR_SENSOR2_PIN 对应
#endif

/* ======================== 协议参数 ======================== */
#define BIT_PERIOD_MS        1               // 每位周期 1ms
#define SAMPLE_DELAY_US      500             // 下降沿后500us采样（信号中点）
#define SYMBOL_BITS          4               // 每4位组成一个符号
#define SYMBOL_COUNT         10              // 每个特征码固定10个符号
#define MAX_SYMBOLS          30              // 环形缓冲区大小
typedef enum {
    STATE_IDLE,            // 空闲，等待下降沿
    STATE_WAIT_SAMPLE,     // 等待500us延时
    STATE_SAMPLING         // 正在采样
} decode_state_t;

typedef struct {
    decode_state_t state;
    uint8_t bit_buffer;                // 临时位缓存（MSB first）
    uint8_t bit_cnt;                   // 已收集位数
    uint8_t symbol_buf[MAX_SYMBOLS];   // 环形符号缓冲区
    uint8_t head;                      // 缓冲区头索引
    uint8_t len;                       // 缓冲区有效长度
    uint8_t matched_up;                // 是否检测到上管
    uint8_t matched_left;              // 是否检测到左管
    uint8_t matched_right;             // 是否检测到右管
    rt_device_t timer;                 // 硬件定时器句柄
	    /* 对外公开的最新状态（带互斥保护）*/
		uint16_t match_cnt;								 // 更新次数，235ms周期更新1次
    uint8_t last_up;                		// 上管最新状态
    uint8_t last_left;              		// 左管最新状态
    uint8_t last_right;             		// 右管最新状态	
} IrDecoder_t;


/* 特征码（每个10个符号，MSB first）*/
static const uint8_t PATTERN_UP[SYMBOL_COUNT] =      {3,1,1,1,1,7,7,1,7,15};   // "311117717f"
static const uint8_t PATTERN_LEFT[SYMBOL_COUNT] =    {3,1,1,1,1,7,7,1,1,15};   // "311117711f"
static const uint8_t PATTERN_RIGHT[SYMBOL_COUNT] =   {15,3,1,1,1,1,7,7,1,1};   // "f311117711"
static const uint8_t PATTERN_LR_FULL[SYMBOL_COUNT] = {3,1,1,1,1,1,7,1,1,1};    // "3111117111"

#define PATTERN_NUM 4
static const uint8_t* PATTERNS[PATTERN_NUM] = {PATTERN_UP, PATTERN_LEFT, PATTERN_RIGHT, PATTERN_LR_FULL};
static const char* PATTERN_NAME[PATTERN_NUM] = {"UP", "LEFT", "RIGHT", "LR_FULL"};

/* ======================== 解码状态机 ======================== */

static IrDecoder_t s_left_decoder;
static IrDecoder_t s_right_decoder;
static rt_thread_t s_report_thread = RT_NULL;
static struct rt_event s_ir_evt;
static struct rt_mutex s_left_mutex;
static struct rt_mutex s_right_mutex;

#define IR_EVENT_LEFT   (1 << 0)   // 左接收管有新匹配
#define IR_EVENT_RIGHT  (1 << 1)   // 右接收管有新匹配

/* ======================== 内部函数声明 ======================== */
static void reset_decoder(IrDecoder_t *dec);
static void analyze_full_frame(IrDecoder_t *dec);
static void update_public_status(IrDecoder_t *dec);
static void ir_send_event(IrDecoder_t *dec);


/* 重置解码器（停止定时器，清空所有状态）*/
static void reset_decoder(IrDecoder_t *dec)
{
		if (dec->timer)
			rt_device_control(dec->timer, HWTIMER_CTRL_STOP, RT_NULL);
    dec->state = STATE_IDLE;
    dec->bit_cnt = 0;
    dec->bit_buffer = 0;
    dec->head = 0;
    dec->len = 0;
    dec->matched_up = 0;
    dec->matched_left = 0;
    dec->matched_right = 0;
}
/* 将本次匹配结果更新到公开状态（带锁）*/
static void update_public_status(IrDecoder_t *dec)
{
    struct rt_mutex *mutex = (dec == &s_left_decoder) ? &s_left_mutex : &s_right_mutex;
    rt_mutex_take(mutex, RT_WAITING_FOREVER);
    dec->last_up = dec->matched_up;
    dec->last_left = dec->matched_left;
    dec->last_right = dec->matched_right;
    dec->match_cnt++;
    rt_mutex_release(mutex);
}

/* 发送事件通知上报线程 */
static void ir_send_event(IrDecoder_t *dec)
{
    if (dec == &s_left_decoder)
        rt_event_send(&s_ir_evt, IR_EVENT_LEFT);
    else
        rt_event_send(&s_ir_evt, IR_EVENT_RIGHT);
}

/* ======================== 精确匹配函数（10个符号，包括 f） ======================== */
/* 检查从缓冲区指定位置开始的连续 SYMBOL_COUNT 个符号是否与某模式完全匹配 */
//static int match_pattern_exact(const uint8_t *buf, uint8_t start, uint8_t pattern_idx)
//{
//    const uint8_t *pattern = PATTERNS[pattern_idx];
//    for (uint8_t i = 0; i < SYMBOL_COUNT; i++) {
//        if (buf[(start + i) % MAX_SYMBOLS] != pattern[i]) return 0;
//    }
//    return 1;
//}

/* 扫描缓冲区，查找所有完整匹配的模式，并更新状态 */
//static void scan_and_update_matches(IrDecoder_t *dec)
//{
//    /* 只有当缓冲区长度足够时才进行完整匹配 */
//    if(dec->len >= SYMBOL_COUNT) {
//        int matched = -1;
//        for (int p = 0; p < PATTERN_NUM; p++) {
//            if (match_pattern_exact(dec->symbol_buf, dec->head, p)) {
//                matched = p;
//                break;
//            }
//        }
//        if (matched >= 0) {
//            switch (matched) {
//                case 0: 
//									dec->matched_up = 1;
//									dec->last_up = dec->matched_up; 
//									/* 移除已匹配的 SYMBOL_COUNT 个符号 */
//									dec->head = (dec->head + SYMBOL_COUNT) % MAX_SYMBOLS;
//									dec->len -= SYMBOL_COUNT;
//									break; //未完成  
//                case 1: 
//									dec->matched_left = 1; 
//										update_public_status(dec);
//										ir_send_event(dec);	
//									reset_decoder(dec);
//									break;// 完成1帧数据解析 
//                case 2: 
//									dec->matched_right = 1; 
//									update_public_status(dec);
//										ir_send_event(dec);	
//									reset_decoder(dec);
//									break;// 完成1帧数据解析 
//                case 3: 
//									dec->matched_left = 1; 
//									dec->matched_right = 1; 
//									update_public_status(dec);
//										ir_send_event(dec);	
//									reset_decoder(dec);
//									break;// 完成1帧数据解析   周期  235ms一次
//            }
//        } 
//    }
//}

/* 一次性分析20个符号，确定包含哪些模式 */
static void analyze_full_frame(IrDecoder_t *dec)
{
    uint8_t first_frame[SYMBOL_COUNT];
    uint8_t second_frame[SYMBOL_COUNT];
	
    /* 提取第一帧（10个符号）*/
    for (uint8_t i = 0; i < SYMBOL_COUNT; i++) {
        first_frame[i] = dec->symbol_buf[(dec->head + i) % MAX_SYMBOLS];
    }
    
    /* 匹配第一帧 */
    int first_match = -1;
    for (int p = 0; p < PATTERN_NUM; p++) {
        if (memcmp(first_frame, PATTERNS[p], SYMBOL_COUNT) == 0) {
            first_match = p;
            break;
        }
    }
    
    if (first_match == 0) {  // 上管
        dec->matched_up = 1;
        
        /* 提取第二帧（后10个符号）并匹配左右管 */

        for (uint8_t i = 0; i < SYMBOL_COUNT; i++) {
            second_frame[i] = dec->symbol_buf[(dec->head + SYMBOL_COUNT + i) % MAX_SYMBOLS];
        }
        
        /* 检查是否全为0x0F（空闲）*/
        uint8_t all_f = 1;
        for (uint8_t i = 0; i < SYMBOL_COUNT; i++) {
            if (second_frame[i] != 0x0F) { all_f = 0; break; }
        }
        if (all_f) {
            /* 只有上管，无左右管，直接返回 */
            update_public_status(dec);
            ir_send_event(dec);
            return;
        }
        
        int second_match = -1;
        for (int p = 0; p < PATTERN_NUM; p++) {
            if (memcmp(second_frame, PATTERNS[p], SYMBOL_COUNT) == 0) {
                second_match = p;
                break;
            }
        }
        switch (second_match) {
            case 1: dec->matched_left = 1;break;
            case 2: dec->matched_right = 1;break;
            case 3: dec->matched_left = 1; dec->matched_right = 1;	break;
            default:         /* 无效或干扰数据，明确清零左右管状态 */
									dec->matched_left = 0;
									dec->matched_right = 0;
									break;
        }
				update_public_status(dec);
				ir_send_event(dec);						
    } else if (first_match >= 1){
        /* 第一帧不是上管，直接作为左右管处理 */
        switch (first_match) {
            case 1: dec->matched_left = 1; 
										update_public_status(dec);
										ir_send_event(dec);					
										break;
            case 2: dec->matched_right = 1; 
										update_public_status(dec);
										ir_send_event(dec);
										break;
            case 3: dec->matched_left = 1; dec->matched_right = 1;
										update_public_status(dec);
										ir_send_event(dec);									
									break;
            default: break;  // 无匹配，忽略
        }
        /* 注意：这里不需要处理第二帧，因为只有左右管时总帧长就是10符号 */
    }else {
        /* 无匹配，忽略 */
    }
}

/* 逐符号校验缓冲区中的符号是否与期望前缀 "3,1,1,1" 的前缀部分匹配 */
/* 返回 0 表示不匹配（需要重置），1 表示匹配（可以继续接收）*/
static int check_prefix_match(IrDecoder_t *dec)
{
	   /* 如果已经收到完整的第一帧，则第二帧不检查前缀（直接返回匹配）*/
    if (dec->len >= SYMBOL_COUNT)
        return 1;
    /* 期望的前缀数组 */
    static const uint8_t expected_prefix[] = {0x03, 0x01, 0x01, 0x01};
    for (uint8_t i = 0; i < dec->len && i < 4; i++) {
        uint8_t actual = dec->symbol_buf[(dec->head + i) % MAX_SYMBOLS];
        if (actual != expected_prefix[i]) {
            return 0;   // 不匹配，需要重置
        }
    }
    return 1;   // 到目前为止全部匹配
}

/* 向缓冲区添加一个新符号 */
static void push_symbol(IrDecoder_t *dec, uint8_t nibble)
{
	  if (dec->len >= MAX_SYMBOLS) {
        dec->head = (dec->head + 1) % MAX_SYMBOLS;
        dec->len--;
    }
		
    uint8_t write_pos = (dec->head + dec->len) % MAX_SYMBOLS;
    dec->symbol_buf[write_pos] = nibble;
    dec->len++;

    /* 校验前缀是否匹配 */
    if (!check_prefix_match(dec)) {
        /* 不匹配，重置解码器，放弃当前帧 */
        reset_decoder(dec);
        return;
    }
//方式一:
    /* 收集满10个符号，尝试匹配 */
//    if (dec->len >= SYMBOL_COUNT) {
//        scan_and_update_matches(dec);   
//    }
//方式二：		
		/* 收集满20个符号（两个完整帧）后，一次性进行匹配 */
    if (dec->len == SYMBOL_COUNT * 2) {
        analyze_full_frame(dec);
        /* 匹配完成后可以重置或保留缓冲区，根据需求选择 */
        reset_decoder(dec);  // 匹配后清空，等待下一帧
    }
}



/* ======================== 定时器回调（1ms采样，MSB first） ======================== */
static void timer_sampling_callback(rt_device_t dev, rt_size_t size, IrDecoder_t *dec, rt_uint8_t pin)
{
    if (dec->state == STATE_WAIT_SAMPLE) {
        /* 500us延时结束，切换到采样模式，并开始1ms周期采样 */
        dec->state = STATE_SAMPLING;
        dec->bit_cnt = 0;
        dec->bit_buffer = 0;
        int level = rt_pin_read(pin);
        dec->bit_buffer = (dec->bit_buffer << 1) | (level & 1);
        dec->bit_cnt = 1;
        struct rt_hwtimerval period;
        period.sec = 0;
        period.usec = BIT_PERIOD_MS * 1000;
        rt_device_write(dec->timer, 0, &period, sizeof(period));
        return;
    }
    
    if (dec->state != STATE_SAMPLING) return;

    int level = rt_pin_read(pin);
    dec->bit_buffer = (dec->bit_buffer << 1) | (level & 1);
    dec->bit_cnt++;

    if (dec->bit_cnt == SYMBOL_BITS) {
        uint8_t nibble = dec->bit_buffer & 0x0F;   // 取低4位
        push_symbol(dec, nibble);
        dec->bit_cnt = 0;
        dec->bit_buffer = 0;
    }
    
    struct rt_hwtimerval period;
    period.sec = 0;
    period.usec = BIT_PERIOD_MS * 1000;
    rt_device_write(dec->timer, 0, &period, sizeof(period));
}

static rt_err_t timer_left_callback(rt_device_t dev, rt_size_t size)
{
    timer_sampling_callback(dev, size, &s_left_decoder, IR_SENSOR_LEFT_PIN);
    return 0;
}

static rt_err_t timer_right_callback(rt_device_t dev, rt_size_t size)
{
    timer_sampling_callback(dev, size, &s_right_decoder, IR_SENSOR_RIGHT_PIN);
    return 0;
}

/* ======================== 外部中断回调（下降沿触发） ======================== */
static void left_irq_callback(void *args)
{
    IrDecoder_t *dec = &s_left_decoder;
    if (dec->state == STATE_IDLE) {
        reset_decoder(dec);
        dec->state = STATE_WAIT_SAMPLE;

        struct rt_hwtimerval delay;
        delay.sec = 0;
        delay.usec = SAMPLE_DELAY_US;
        rt_device_write(dec->timer, 0, &delay, sizeof(delay));
    }
}

static void right_irq_callback(void *args)
{
    IrDecoder_t *dec = &s_right_decoder;
    if (dec->state == STATE_IDLE) {
        reset_decoder(dec);
        dec->state = STATE_WAIT_SAMPLE;

        struct rt_hwtimerval delay;
        delay.sec = 0;
        delay.usec = SAMPLE_DELAY_US;
        rt_device_write(dec->timer, 0, &delay, sizeof(delay));
    }
}

/* ======================== 定时器初始化 ======================== */
static void init_timer(IrDecoder_t *dec, const char *timer_name, rt_err_t (*callback)(rt_device_t, rt_size_t))
{
    dec->timer = rt_device_find(timer_name);
    if (!dec->timer) {
        rt_kprintf("[IR] Cannot find %s\n", timer_name);
        return;
    }
    rt_device_open(dec->timer, RT_DEVICE_OFLAG_RDWR);
    
    uint32_t freq = 1000000;  // 1MHz
    rt_device_control(dec->timer, HWTIMER_CTRL_FREQ_SET, &freq);
    
    rt_hwtimer_mode_t mode = HWTIMER_MODE_ONESHOT;
    rt_device_control(dec->timer, HWTIMER_CTRL_MODE_SET, &mode);
    
    rt_device_set_rx_indicate(dec->timer, callback);
}

/* ======================== 状态上报线程（500ms周期） ======================== */
static void ir_report_thread(void *param)
{
    rt_uint32_t recv_evt;
    PacketReportIRSwitchTypeDef pkt;
    pkt.sub_cmd = SYS_SUB_IR_SWITCH;

    while (1) {
        if (rt_event_recv(&s_ir_evt, IR_EVENT_LEFT | IR_EVENT_RIGHT,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &recv_evt) == RT_EOK) {
            /* 读取最新状态（直接访问 last_xxx，因为已加锁保护）*/
            if (recv_evt & IR_EVENT_LEFT) {
                uint8_t left_mask = (s_left_decoder.last_up << 2) |
                                    (s_left_decoder.last_left << 1) |
                                    (s_left_decoder.last_right);
                pkt.state_mask = (left_mask << 4) | (pkt.state_mask & 0x0F);
                rt_kprintf("[IR] Left: UP=%d, LEFT=%d, RIGHT=%d\n",
                           s_left_decoder.last_up, s_left_decoder.last_left, s_left_decoder.last_right);
            }
            if (recv_evt & IR_EVENT_RIGHT) {
                uint8_t right_mask = (s_right_decoder.last_up << 2) |
                                     (s_right_decoder.last_left << 1) |
                                     (s_right_decoder.last_right);
                pkt.state_mask = (pkt.state_mask & 0xF0) | right_mask;
                rt_kprintf("[IR] Right: UP=%d, LEFT=%d, RIGHT=%d\n",
                           s_right_decoder.last_up, s_right_decoder.last_left, s_right_decoder.last_right);
            }
            uart_packet_send(PKT_FUNC_SYS, &pkt, sizeof(pkt));
        }
    }
}

/* ======================== 对外初始化接口 ======================== */
void irm_8601m2_init(void)
{
    memset(&s_left_decoder, 0, sizeof(s_left_decoder));
    memset(&s_right_decoder, 0, sizeof(s_right_decoder));
    s_left_decoder.state = STATE_IDLE;
    s_right_decoder.state = STATE_IDLE;
    
    rt_pin_mode(IR_SENSOR_LEFT_PIN, PIN_MODE_INPUT);
    rt_pin_mode(IR_SENSOR_RIGHT_PIN, PIN_MODE_INPUT);
    
    init_timer(&s_left_decoder, "timer6", timer_left_callback);
    init_timer(&s_right_decoder, "timer7", timer_right_callback);
    /* 初始化事件 */
    rt_event_init(&s_ir_evt, "ir_evt", RT_IPC_FLAG_FIFO);
	  rt_mutex_init(&s_left_mutex, "ir_l_mtx", RT_IPC_FLAG_PRIO);
    rt_mutex_init(&s_right_mutex, "ir_r_mtx", RT_IPC_FLAG_PRIO);
	
    rt_pin_attach_irq(IR_SENSOR_LEFT_PIN, PIN_IRQ_MODE_FALLING, left_irq_callback, RT_NULL);
    rt_pin_attach_irq(IR_SENSOR_RIGHT_PIN, PIN_IRQ_MODE_FALLING, right_irq_callback, RT_NULL);
    rt_pin_irq_enable(IR_SENSOR_LEFT_PIN, PIN_IRQ_ENABLE);
    rt_pin_irq_enable(IR_SENSOR_RIGHT_PIN, PIN_IRQ_ENABLE);
    
    s_report_thread = rt_thread_create("ir_report", ir_report_thread, NULL, 1024, 20, 10);
    if (s_report_thread) rt_thread_startup(s_report_thread);
    
    rt_kprintf("[IR] IRM-8601M2 initialized (10-symbol exact match)\n");
}
uint8_t irm_get_left_status(void)
{
    uint8_t ret;
    rt_mutex_take(&s_left_mutex, RT_WAITING_FOREVER);
    ret = (s_left_decoder.last_up << 2) | (s_left_decoder.last_left << 1) | s_left_decoder.last_right;
    rt_mutex_release(&s_left_mutex);
    return ret;
}

uint8_t irm_get_right_status(void)
{
    uint8_t ret;
    rt_mutex_take(&s_right_mutex, RT_WAITING_FOREVER);
    ret = (s_right_decoder.last_up << 2) | (s_right_decoder.last_left << 1) | s_right_decoder.last_right;
    rt_mutex_release(&s_right_mutex);
    return ret;
}

uint16_t irm_get_left_match_cnt(void)
{
    uint16_t cnt;
    rt_mutex_take(&s_left_mutex, RT_WAITING_FOREVER);
    cnt = s_left_decoder.match_cnt;
    rt_mutex_release(&s_left_mutex);
    return cnt;
}

uint16_t irm_get_right_match_cnt(void)
{
    uint16_t cnt;
    rt_mutex_take(&s_right_mutex, RT_WAITING_FOREVER);
    cnt = s_right_decoder.match_cnt;
    rt_mutex_release(&s_right_mutex);
    return cnt;
}

void irm_clear_left_status(void)
{
    rt_mutex_take(&s_left_mutex, RT_WAITING_FOREVER);
    s_left_decoder.last_up = 0;
    s_left_decoder.last_left = 0;
    s_left_decoder.last_right = 0;
    s_left_decoder.match_cnt = 0;
    rt_mutex_release(&s_left_mutex);
}

void irm_clear_right_status(void)
{
    rt_mutex_take(&s_right_mutex, RT_WAITING_FOREVER);
    s_right_decoder.last_up = 0;
    s_right_decoder.last_left = 0;
    s_right_decoder.last_right = 0;
    s_right_decoder.match_cnt = 0;
    rt_mutex_release(&s_right_mutex);
}


#ifdef RT_USING_MSH
#include <stdlib.h>

static void ir_status(int argc, char **argv)
{
    uint8_t left_status = irm_get_left_status();
    uint8_t right_status = irm_get_right_status();
    uint16_t left_cnt = irm_get_left_match_cnt();
    uint16_t right_cnt = irm_get_right_match_cnt();

    rt_kprintf("\n========== IR Status ==========\n");
    rt_kprintf("Left:  UP=%d, LEFT=%d, RIGHT=%d  (mask=0x%02X)  match_cnt=%d\n",
               (left_status >> 2) & 1, (left_status >> 1) & 1, left_status & 1,
               left_status, left_cnt);
    rt_kprintf("Right: UP=%d, LEFT=%d, RIGHT=%d  (mask=0x%02X)  match_cnt=%d\n",
               (right_status >> 2) & 1, (right_status >> 1) & 1, right_status & 1,
               right_status, right_cnt);
    rt_kprintf("================================\n");
}

static void ir_clear(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: ir_clear left|right|all\n");
        return;
    }
    if (strcmp(argv[1], "left") == 0) {
        irm_clear_left_status();
        rt_kprintf("Left status cleared\n");
    } else if (strcmp(argv[1], "right") == 0) {
        irm_clear_right_status();
        rt_kprintf("Right status cleared\n");
    } else if (strcmp(argv[1], "all") == 0) {
        irm_clear_left_status();
        irm_clear_right_status();
        rt_kprintf("Both status cleared\n");
    } else {
        rt_kprintf("Invalid argument. Use left/right/all\n");
    }
}

MSH_CMD_EXPORT(ir_status, "Show infrared charging alignment status");
MSH_CMD_EXPORT(ir_clear, "Clear infrared status (left/right/all)");
#endif /* RT_USING_MSH */

