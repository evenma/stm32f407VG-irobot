#include "ir_receiver.h"
#include "global_conf.h"
#include "uart_packet.h"
#include "packet_reports.h"
#include <rtdevice.h>
#include <string.h>
#include <board.h>

/* ======================== 硬件引脚 ======================== */
#ifndef IR_SENSOR_LEFT_PIN
#define IR_SENSOR_LEFT_PIN   GET_PIN(E, 13)
#endif
#ifndef IR_SENSOR_RIGHT_PIN
#define IR_SENSOR_RIGHT_PIN  GET_PIN(E, 14)
#endif

/* ======================== 协议参数 ======================== */
#define BIT_NOMINAL_US       1000
#define BIT_TOLERANCE_US     400
#define MAX_CONSECUTIVE_BITS 12     // 7 15 15 3  = 11    
#define FRAME_TIMEOUT_MS     50    // 5*3(11)+5=20/pcs ; pcs*2 <200  pcs<100;
#define SYMBOL_BITS          4
#define SYMBOL_COUNT         10
#define MAX_SYMBOLS          30
#define BIT_FIFO_SIZE        1024

/* 特征码 */
static const uint8_t PATTERN_UP[SYMBOL_COUNT] =      {3,1,1,1,1,7,7,1,7,15};
static const uint8_t PATTERN_LEFT[SYMBOL_COUNT] =    {3,1,1,1,1,7,7,1,1,15};
static const uint8_t PATTERN_RIGHT[SYMBOL_COUNT] =   {15,3,1,1,1,1,7,7,1,1};
static const uint8_t PATTERN_LR_FULL[SYMBOL_COUNT] = {3,1,1,1,1,1,7,1,1,1};

#define PATTERN_NUM 4
static const uint8_t* PATTERNS[PATTERN_NUM] = {PATTERN_UP,PATTERN_LR_FULL, PATTERN_LEFT, PATTERN_RIGHT};

/* 位FIFO */
typedef struct {
    uint8_t buffer[BIT_FIFO_SIZE];
    volatile uint16_t write_idx;
    volatile uint16_t read_idx;
    volatile uint8_t overflow;
} bit_fifo_t;

static bit_fifo_t s_left_fifo;
static bit_fifo_t s_right_fifo;

/* 解码器状态 */
typedef enum {
    DECODE_STATE_IDLE,
    DECODE_STATE_RECEIVING
} decode_state_t;

typedef struct {
    decode_state_t state;
    uint64_t last_timestamp; 				// 上一次边沿的时间戳（微秒，从定时器启动开始累计）
    uint8_t last_level;
    uint8_t current_byte;
    uint8_t bit_in_byte;
    uint8_t symbol_buf[MAX_SYMBOLS];
    uint8_t head;
    uint8_t len;
    uint8_t matched_up;
    uint8_t matched_left;
    uint8_t matched_right;
    uint16_t match_cnt;
    uint8_t last_up;
    uint8_t last_left;
    uint8_t last_right;
    uint32_t last_activity_tick;
} IrDecoder_t;

static IrDecoder_t s_left_decoder;
static IrDecoder_t s_right_decoder;

static rt_thread_t s_parse_thread = RT_NULL;
static struct rt_event s_ir_event;
#define IR_EVENT_LEFT   (1 << 0)
#define IR_EVENT_RIGHT  (1 << 1)

//volatile rt_bool_t g_ir_alignment_enable = RT_FALSE;

/* ======================== 硬件定时器 (1us 时基) ======================== */
static rt_device_t s_timer_dev = RT_NULL;

static void timer_us_init(void)
{
    s_timer_dev = rt_device_find("timer6");
    if (s_timer_dev == RT_NULL) {
        rt_kprintf("[IR] Cannot find timer6, using fallback to register operation\n");
        // 后备方案：直接操作寄存器（略，但保留原有代码）
        return;
    }
    rt_device_open(s_timer_dev, RT_DEVICE_OFLAG_RDWR);
    
    /* 设置频率为 1MHz */
    uint32_t freq = 1000000;
    rt_device_control(s_timer_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    
    /* 设置为周期模式 */
    rt_hwtimer_mode_t mode = HWTIMER_MODE_PERIOD;
    rt_device_control(s_timer_dev, HWTIMER_CTRL_MODE_SET, &mode);

    /* 设置一个非常大的周期，使其相当于自由计数器（）*/
    struct rt_hwtimerval period;
    period.sec = 5;   
    period.usec = 0;
		if (rt_device_write(s_timer_dev, 0, &period, sizeof(period)) != sizeof(period)) {
				rt_kprintf("[IR] Failed to set timer period\n");
				return;
		}
    
    /* 启动定时器 */
//    rt_device_control(s_timer_dev, HWTIMER_CTRL_START, RT_NULL);
}

static uint64_t get_timestamp_us(void)
{
    if (s_timer_dev == RT_NULL) {
        // 后备方案：直接读寄存器（假设 timer6 基地址已知）
        return TIM6->CNT;
    }
    struct rt_hwtimerval tv;
    if (rt_device_read(s_timer_dev, 0, &tv, sizeof(tv)) != sizeof(tv)) {
        return 0;
    }
    // tv.sec 和 tv.usec 是驱动层已经处理过溢出的总时间
    return (uint64_t)tv.sec * 1000000 + tv.usec;
}

/* ======================== 内部函数 ======================== */
static void update_public_status(IrDecoder_t *dec)
{
    rt_base_t level = rt_hw_interrupt_disable();
    dec->last_up = dec->matched_up;
    dec->last_left = dec->matched_left;
    dec->last_right = dec->matched_right;
    dec->match_cnt++;
    rt_hw_interrupt_enable(level);
}

static void ir_send_event(IrDecoder_t *dec)
{
    if (dec == &s_left_decoder)
        rt_event_send(&s_ir_event, IR_EVENT_LEFT);
    else
        rt_event_send(&s_ir_event, IR_EVENT_RIGHT);
}

static void reset_decoder(IrDecoder_t *dec)
{
    rt_base_t level = rt_hw_interrupt_disable();
    dec->state = DECODE_STATE_IDLE;
    dec->last_timestamp = 0;
    dec->last_level = 0;
    dec->current_byte = 0;
    dec->bit_in_byte = 0;
    dec->head = 0;
    dec->len = 0;
    dec->matched_up = 0;
    dec->matched_left = 0;
    dec->matched_right = 0;
    memset(dec->symbol_buf, 0, sizeof(dec->symbol_buf));
    rt_hw_interrupt_enable(level);
}

/* 精确匹配10个符号 */
static int match_pattern_exact(const uint8_t *buf, uint8_t start, uint8_t pattern_idx)
{
    const uint8_t *pattern = PATTERNS[pattern_idx];
    for (uint8_t i = 0; i < SYMBOL_COUNT; i++) {
        if (buf[(start + i) % MAX_SYMBOLS] != pattern[i])
            return 0;
    }
    return 1;
}

/* 计算两个符号数组的海明距离（不同符号个数）*/
static int hamming_distance(const uint8_t *a, const uint8_t *b, int len)
{
    int dist = 0;
    for (int i = 0; i < len; i++) {
        if (a[i] != b[i]) dist++;
    }
    return dist;
}

/* 滑动窗口匹配 */
static void scan_and_update_matches(IrDecoder_t *dec)
{
    while (dec->len >= SYMBOL_COUNT) {
        int matched = -1;
        for (int p = 0; p < PATTERN_NUM; p++) {
            if (match_pattern_exact(dec->symbol_buf, dec->head, p)) {
                matched = p;
                break;
            }
        }
				
				 /* 精确匹配失败时，尝试容错匹配 */
        if (matched < 0) {
            int best_match = -1;
            int best_dist = SYMBOL_COUNT + 1;  // 初始大于最大可能距离
            uint8_t cur_sym[SYMBOL_COUNT];
            for (int i = 0; i < SYMBOL_COUNT; i++) {
                cur_sym[i] = dec->symbol_buf[(dec->head + i) % MAX_SYMBOLS];
            }
            for (int p = 0; p < PATTERN_NUM; p++) {
                int dist = hamming_distance(cur_sym, PATTERNS[p], SYMBOL_COUNT);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_match = p;
                }
            }
            /* 若最佳距离 ≤2（即允许最多2个符号错误），则视为匹配成功 */
            if (best_dist < 2) {
                matched = best_match;
                // 打印调试信息，显示纠正后的匹配
               //rt_kprintf("[IR] Fuzzy match: pattern %d, dist=%d\n", matched, best_dist);
//							if (dec == &s_left_decoder) {
//								rt_kprintf("L ");
//								if(dec->head){
//									rt_kprintf("D=");
//								}else{
//									rt_kprintf("U=");
//								}
//									for (int i = 0; i < SYMBOL_COUNT && i < dec->len; i++) {
//											rt_kprintf("%d ", dec->symbol_buf[(dec->head + i) % MAX_SYMBOLS]);
//									}
//									rt_kprintf("\n");
//							}
//							if (dec == &s_right_decoder) {
//								rt_kprintf("R ");
//								if(dec->head){
//									rt_kprintf("D=");
//								}else{
//									rt_kprintf("U=");
//								}
//									for (int i = 0; i < SYMBOL_COUNT && i < dec->len; i++) {
//											rt_kprintf("%d ", dec->symbol_buf[(dec->head + i) % MAX_SYMBOLS]);
//									}
//									rt_kprintf("\n");
//							}
            }
        }

        if (matched >= 0) {
            switch (matched) {
                case 0: dec->matched_up = 1; break;
                case 1: dec->matched_left = 1; dec->matched_right = 1; break;
                case 2: dec->matched_left = 1; break;
                case 3: dec->matched_right = 1; break;
            }
//            update_public_status(dec);
//            ir_send_event(dec);
            dec->head = (dec->head + SYMBOL_COUNT) % MAX_SYMBOLS;
            dec->len -= SYMBOL_COUNT;
            // 匹配成功后，如果 len == 0，则自动等待后续符号（第二帧）
//						rt_kprintf("-%d-",matched);
        } else {
					 // 未匹配时打印当前10个符号

            // 无效数据，重置整个解码器
            reset_decoder(dec);
            return;
        }
    }
}

/* 添加一个符号（线程中调用）*/
static void push_symbol(IrDecoder_t *dec, uint8_t nibble)
{
    rt_base_t level = rt_hw_interrupt_disable();
    if (dec->len >= MAX_SYMBOLS) {
        dec->head = (dec->head + 1) % MAX_SYMBOLS;
        dec->len--;
    }
//		if(dec == &s_left_decoder){
//				rt_kprintf("%d ",nibble);
//				if (dec->len >= SYMBOL_COUNT) {
//						rt_kprintf("\r\n");
//				}	
//		}
    uint8_t pos = (dec->head + dec->len) % MAX_SYMBOLS;
    dec->symbol_buf[pos] = nibble;
    dec->len++;
    rt_hw_interrupt_enable(level);

    // 快速前缀检查：仅当 head==0 且接收初期（len<=4）时进行
    if (dec->head == 0 && dec->len <= 4) {
        static const uint8_t exp[] = {3,1,1,1};
        uint8_t idx = dec->head;
        if (dec->symbol_buf[idx] != exp[0]) { reset_decoder(dec); return; }
        if (dec->len >= 2 && dec->symbol_buf[(idx+1)%MAX_SYMBOLS] != exp[1]) { reset_decoder(dec); return; }
        if (dec->len >= 3 && dec->symbol_buf[(idx+2)%MAX_SYMBOLS] != exp[2]) { reset_decoder(dec); return; }
        if (dec->len >= 4 && dec->symbol_buf[(idx+3)%MAX_SYMBOLS] != exp[3]) { reset_decoder(dec); return; }
    }

    if (dec->len >= SYMBOL_COUNT) {
        scan_and_update_matches(dec);
    }
}

/* ======================== 中断处理 ======================== */
static void ir_edge_handler(IrDecoder_t *dec, bit_fifo_t *fifo, uint8_t current_level)
{
    uint64_t now = get_timestamp_us();
    if (dec->last_timestamp == 0) {
        dec->last_timestamp = now;
				dec->last_level = current_level;
        dec->state = DECODE_STATE_RECEIVING;
        dec->last_activity_tick = rt_tick_get_millisecond();
        return;
    }
//	if(now < dec->last_timestamp) return;  // 如果定时器周期到了，now最小，last_timestamp最大
    uint32_t diff = now - dec->last_timestamp;
    if (diff < 100) return;  // 防抖

    // 计算连续相同电平的位数（四舍五入）
    int bits = (diff + BIT_NOMINAL_US/2) / BIT_NOMINAL_US;
    if (bits < 1) bits = 1;
    if (bits > MAX_CONSECUTIVE_BITS) bits = MAX_CONSECUTIVE_BITS;

    // 可选：容差检查（1位时检查是否超差）
    // if (bits == 1 && (diff < BIT_NOMINAL_US - BIT_TOLERANCE_US || diff > BIT_NOMINAL_US + BIT_TOLERANCE_US)) {
    //     reset_decoder(dec); return;
    // }

    uint8_t level_bit = dec->last_level;
    rt_base_t irq_level = rt_hw_interrupt_disable();
    for (int i = 0; i < bits; i++) {
        uint16_t next = (fifo->write_idx + 1) % BIT_FIFO_SIZE;
        if (next == fifo->read_idx) {
            fifo->overflow = 1;
            break;
        }
        fifo->buffer[fifo->write_idx] = level_bit;
        fifo->write_idx = next;
    }
    rt_hw_interrupt_enable(irq_level);

    dec->last_timestamp = now;
    dec->last_level = current_level;
    dec->last_activity_tick = rt_tick_get_millisecond();
}

static void left_irq_callback(void *args)
{
    uint8_t level = rt_pin_read(IR_SENSOR_LEFT_PIN);
    ir_edge_handler(&s_left_decoder, &s_left_fifo, level);
}

static void right_irq_callback(void *args)
{
    uint8_t level = rt_pin_read(IR_SENSOR_RIGHT_PIN);
    ir_edge_handler(&s_right_decoder, &s_right_fifo, level);
}

/* ======================== 解析线程 ======================== */
static void ir_parse_thread(void *param)
{
	static uint8_t Ltimeout_cnt = 0,Rtimeout_cnt=0;
    while (1) {
        // 超时检测
        uint32_t now = rt_tick_get_millisecond();
        if (s_left_decoder.state == DECODE_STATE_RECEIVING &&
            now - s_left_decoder.last_activity_tick > FRAME_TIMEOUT_MS) {
					if (Ltimeout_cnt == 0) {
					// 如果当前 last_level 为高电平（1），且距离上次边沿的时间差 > 0，则将这最后的持续时间转换为位
						if (s_left_decoder.last_level == 1) {
							// 计算1帧还差多少bits
							IrDecoder_t *dec = &s_left_decoder;
							uint8_t cnt = SYMBOL_BITS - dec->bit_in_byte + SYMBOL_BITS * ( SYMBOL_COUNT - 1 - dec->len);
							if(cnt < 8){    // 合理的补1 数量太大说明数据就不对。正常是7个1,1个1或者5个1。其他其实多不正常
								// 将 bits 个 1 写入 FIFO（因为 last_level=1）
									rt_base_t irq_level = rt_hw_interrupt_disable();								
									for(int i=0;i<cnt;i++){
										uint16_t next = (s_left_fifo.write_idx + 1) % BIT_FIFO_SIZE;
										if (next != s_left_fifo.read_idx) {
												s_left_fifo.buffer[s_left_fifo.write_idx] = 1;
												s_left_fifo.write_idx = next;
										} else {
												s_left_fifo.overflow = 1;
										}											
									}
									rt_hw_interrupt_enable(irq_level);
//									rt_kprintf("L+%d ", cnt);	
							}
							Ltimeout_cnt = 1;	
							s_left_decoder.last_activity_tick = now;							
						}	
					}else {
							update_public_status(&s_left_decoder);
							ir_send_event(&s_left_decoder);
							// 第二次超时：重置解码器
							reset_decoder(&s_left_decoder);
							Ltimeout_cnt = 0;
					}							
        }
				if (s_right_decoder.state == DECODE_STATE_RECEIVING &&
						now - s_right_decoder.last_activity_tick > FRAME_TIMEOUT_MS) {
					if (Rtimeout_cnt == 0) {
						if (s_right_decoder.last_level == 1) {
							 // 计算1帧还差多少bits
							IrDecoder_t *dec = &s_right_decoder;
							uint8_t cnt = SYMBOL_BITS - dec->bit_in_byte + SYMBOL_BITS * ( SYMBOL_COUNT - 1 - dec->len);
							if(cnt < 8){    // 合理的补1 数量太大说明数据就不对。正常是1个1或者5个1。其他其实多不正常
								// 将 bits 个 1 写入 FIFO（因为 last_level=1）				
									rt_base_t irq_level = rt_hw_interrupt_disable();								
									for(int i=0;i<cnt;i++){
											uint16_t next = (s_right_fifo.write_idx + 1) % BIT_FIFO_SIZE;
											if (next != s_right_fifo.read_idx) {
													s_right_fifo.buffer[s_right_fifo.write_idx] = 1;
													s_right_fifo.write_idx = next;
											} else {
													s_right_fifo.overflow = 1;
											}										
									}
									rt_hw_interrupt_enable(irq_level);
//								rt_kprintf("R+%d ", cnt);
							}
							Rtimeout_cnt = 1;
							s_right_decoder.last_activity_tick = now;
						}	
					}else {
						update_public_status(&s_right_decoder);
							ir_send_event(&s_right_decoder);
							// 第二次超时：重置解码器
							reset_decoder(&s_right_decoder);
							Rtimeout_cnt = 0;
//						rt_hwtimerval_t timeout_s;
//						 rt_device_read(s_timer_dev, 0, &timeout_s, sizeof(timeout_s));
//							rt_kprintf("Sec = %d\n", timeout_s.sec);
					}							
				}

        // 处理左通道
        if (s_left_fifo.read_idx != s_left_fifo.write_idx) {
            uint8_t bit;
            rt_base_t level = rt_hw_interrupt_disable();
            bit = s_left_fifo.buffer[s_left_fifo.read_idx];
            s_left_fifo.read_idx = (s_left_fifo.read_idx + 1) % BIT_FIFO_SIZE;
            rt_hw_interrupt_enable(level);

            IrDecoder_t *dec = &s_left_decoder;
            dec->current_byte = (dec->current_byte << 1) | (bit & 1);
            dec->bit_in_byte++;
            if (dec->bit_in_byte == SYMBOL_BITS) {
							// 对位加一层判断，数据位可能不对齐，如果偏移了，如果不是 3,1,7,15,丢弃最前面那一位
								push_symbol(dec, dec->current_byte & 0x0F);
								dec->bit_in_byte = 0;
								dec->current_byte = 0;								
//								rt_kprintf(".");
            }
        }

        // 处理右通道
        if (s_right_fifo.read_idx != s_right_fifo.write_idx) {
            uint8_t bit;
            rt_base_t level = rt_hw_interrupt_disable();
            bit = s_right_fifo.buffer[s_right_fifo.read_idx];
            s_right_fifo.read_idx = (s_right_fifo.read_idx + 1) % BIT_FIFO_SIZE;
            rt_hw_interrupt_enable(level);

            IrDecoder_t *dec = &s_right_decoder;
            dec->current_byte = (dec->current_byte << 1) | (bit & 1);
            dec->bit_in_byte++;
            if (dec->bit_in_byte == SYMBOL_BITS) {
								push_symbol(dec, dec->current_byte & 0x0F);
								dec->bit_in_byte = 0;
								dec->current_byte = 0;
            }
        }

        if (s_left_fifo.read_idx == s_left_fifo.write_idx &&
            s_right_fifo.read_idx == s_right_fifo.write_idx) {
            rt_thread_mdelay(5);
        }
    }
}

/* ======================== 上报线程 ======================== */
static void ir_report_thread(void *param)
{
    rt_uint32_t recv_evt;
    PacketReportIRSwitchTypeDef pkt;
    pkt.sub_cmd = SYS_SUB_IR_SWITCH;

    while (1) {
        if (rt_event_recv(&s_ir_event, IR_EVENT_LEFT | IR_EVENT_RIGHT,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &recv_evt) == RT_EOK) {
            if (recv_evt & IR_EVENT_LEFT) {
                pkt.lr = 1;   // 左接收管
                pkt.mask = (s_left_decoder.last_up << 2) |
                           (s_left_decoder.last_left << 1) |
                           s_left_decoder.last_right;
								if(pkt.mask && pkt.mask != 0x02){
									uart_packet_send(PKT_FUNC_SYS, &pkt, sizeof(pkt));
//									rt_kprintf("L:%d\n", pkt.mask);
								}
            }
            if (recv_evt & IR_EVENT_RIGHT) {
                pkt.lr = 2;   // 右接收管
                pkt.mask = (s_right_decoder.last_up << 2) |
                           (s_right_decoder.last_left << 1) |
                           s_right_decoder.last_right;
								if(pkt.mask && pkt.mask != 0x02){
									uart_packet_send(PKT_FUNC_SYS, &pkt, sizeof(pkt));
//									rt_kprintf("R:%d\n", pkt.mask);
								}
            }					
        }
    }
}

/* ======================== 对外接口 ======================== */
void ir_receiver_init(void)
{
		rt_kprintf("[IR] initialized \n");
    memset(&s_left_decoder, 0, sizeof(s_left_decoder));
    memset(&s_right_decoder, 0, sizeof(s_right_decoder));
    memset(&s_left_fifo, 0, sizeof(s_left_fifo));
    memset(&s_right_fifo, 0, sizeof(s_right_fifo));
    s_left_decoder.state = DECODE_STATE_IDLE;
    s_right_decoder.state = DECODE_STATE_IDLE;

    rt_pin_mode(IR_SENSOR_LEFT_PIN, PIN_MODE_INPUT);
    rt_pin_mode(IR_SENSOR_RIGHT_PIN, PIN_MODE_INPUT);

    timer_us_init();
		rt_kprintf("[IR] ----\n");
    rt_event_init(&s_ir_event, "ir_evt", RT_IPC_FLAG_FIFO);

    rt_pin_attach_irq(IR_SENSOR_LEFT_PIN, PIN_IRQ_MODE_RISING_FALLING, left_irq_callback, RT_NULL);
    rt_pin_attach_irq(IR_SENSOR_RIGHT_PIN, PIN_IRQ_MODE_RISING_FALLING, right_irq_callback, RT_NULL);
    rt_pin_irq_enable(IR_SENSOR_LEFT_PIN, PIN_IRQ_ENABLE);
    rt_pin_irq_enable(IR_SENSOR_RIGHT_PIN, PIN_IRQ_ENABLE);

    s_parse_thread = rt_thread_create("ir_parse", ir_parse_thread, NULL, 2048, 15, 10);
    if (s_parse_thread) rt_thread_startup(s_parse_thread);

    rt_thread_t report_thread = rt_thread_create("ir_report", ir_report_thread, NULL, 1024, 20, 10);
    if (report_thread) rt_thread_startup(report_thread);

    rt_kprintf("[IR] New receiver initialized (edge-triggered, pulse-width measurement)\n");
}

uint8_t ir_get_left_status(void)
{
    uint8_t ret;
    rt_base_t level = rt_hw_interrupt_disable();
    ret = (s_left_decoder.last_up << 2) | (s_left_decoder.last_left << 1) | s_left_decoder.last_right;
    rt_hw_interrupt_enable(level);
    return ret;
}

uint8_t ir_get_right_status(void)
{
    uint8_t ret;
    rt_base_t level = rt_hw_interrupt_disable();
    ret = (s_right_decoder.last_up << 2) | (s_right_decoder.last_left << 1) | s_right_decoder.last_right;
    rt_hw_interrupt_enable(level);
    return ret;
}

uint16_t ir_get_left_match_cnt(void)
{
    uint16_t cnt;
    rt_base_t level = rt_hw_interrupt_disable();
    cnt = s_left_decoder.match_cnt;
    rt_hw_interrupt_enable(level);
    return cnt;
}

uint16_t ir_get_right_match_cnt(void)
{
    uint16_t cnt;
    rt_base_t level = rt_hw_interrupt_disable();
    cnt = s_right_decoder.match_cnt;
    rt_hw_interrupt_enable(level);
    return cnt;
}

void ir_clear_left_status(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    s_left_decoder.last_up = 0;
    s_left_decoder.last_left = 0;
    s_left_decoder.last_right = 0;
    s_left_decoder.match_cnt = 0;
    rt_hw_interrupt_enable(level);
}

void ir_clear_right_status(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    s_right_decoder.last_up = 0;
    s_right_decoder.last_left = 0;
    s_right_decoder.last_right = 0;
    s_right_decoder.match_cnt = 0;
    rt_hw_interrupt_enable(level);
}

