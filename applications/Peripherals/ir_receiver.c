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

/* ======================== 线程配置 ======================== */

#define IR_REPORT_THREAD_STACK_SIZE  1024
#define IR_REPORT_THREAD_PRIORITY    18

#define IR_EVENT_LEFT   (1 << 0)
#define IR_EVENT_RIGHT  (1 << 1)

/* ======================== 协议参数 ======================== */
#define BIT_NOMINAL_US       1000		// 1ms/bit
#define BIT_TOLERANCE_US     400		// +-0.4us容差
#define MAX_CONSECUTIVE_BITS 12     // 7 15 15 3  = 11   最多补1的位数 
#define SYMBOL_BITS          4			// 4bits一数据
#define SYMBOL_COUNT         10			// 10数据一特征码
#define MAX_SYMBOLS          30			// 1帧数组最大个数
#define FRAME_TIMEOUT_MS     50    // 5*3(11)+5=20/pcs ; pcs*2 <200  pcs<100;
#define FRAME_IDLE_TIMEOUT_MS 250   // 250ms 周期
#define IR_RECIVE_TIMEOUT_MS 500     // 500ms超时未收到事件，清除上传数据

/* 特征码 */
static const uint8_t PATTERN_UP[SYMBOL_COUNT] =      {3,1,1,1,1,7,7,1,7,15};
static const uint8_t PATTERN_LEFT[SYMBOL_COUNT] =    {3,1,1,1,1,7,7,1,1,15};
static const uint8_t PATTERN_RIGHT[SYMBOL_COUNT] =   {15,3,1,1,1,1,7,7,1,1};
static const uint8_t PATTERN_LR_FULL[SYMBOL_COUNT] = {3,1,1,1,1,1,7,1,1,1};

#define PATTERN_NUM 4
static const uint8_t* PATTERNS[PATTERN_NUM] = {PATTERN_UP,PATTERN_LR_FULL, PATTERN_LEFT, PATTERN_RIGHT};

/* 解码器状态 */
typedef enum {
    DECODE_STATE_IDLE,
    DECODE_STATE_RECEIVING
} decode_state_t;

typedef struct {
    decode_state_t state;
	uint8_t frame_ready;    				//  帧结束标志
	uint8_t symbol_buf[MAX_SYMBOLS];		// 帧数据
	rt_device_t timer;                 // 硬件定时器句柄
/*临时数据*/
    uint8_t last_level;						// 最新电平值 1bit
    uint8_t current_byte;					// 装配当前一个数据
    uint8_t bit_in_byte;					// 装配4bits一个数据的计数
	uint8_t buffer[MAX_SYMBOLS];			// 缓存	
	uint8_t buf_index;						// 记录缓存当前地址
    uint8_t head;							// 第0/1个特征码的起始位
    uint8_t len;							// 装配了几个数据   1特征码10个数据
    uint8_t matched_up;						// 匹配到品字型的上管特征码
    uint8_t matched_left;					// 匹配到品字型的左管特征码
    uint8_t matched_right;					// 匹配到品字型的右管特征码
/*最新一次帧数据*/	
    uint16_t frame_cnt;						// 帧累计
    uint8_t last_up;						// 帧的上管值
    uint8_t last_left;						// 帧的左管值
    uint8_t last_right;						// 帧的右管值
		uint32_t t_irq_cnt;   // 中断触发计数  调试用
		uint32_t io_irq_cnt;   // 中断触发计数  调试用		
} IrDecoder_t;
static struct rt_event s_ir_event;
static IrDecoder_t s_left_decoder;
static IrDecoder_t s_right_decoder;
//设置超时时间为 FRAME_TIMEOUT_MS ms 并启动定时器
static struct rt_hwtimerval timeout;

/* ======================== 内部函数 ======================== */
static void update_public_status(IrDecoder_t *dec)
{
    rt_base_t level = rt_hw_interrupt_disable();
    dec->last_up = dec->matched_up;
    dec->last_left = dec->matched_left;
    dec->last_right = dec->matched_right;
    dec->frame_cnt++;
    rt_hw_interrupt_enable(level);
}

static void reset_decoder(IrDecoder_t *dec)
{
    rt_base_t level = rt_hw_interrupt_disable();
    dec->state = DECODE_STATE_IDLE;
    dec->last_level = 0;
    dec->current_byte = 0;
    dec->bit_in_byte = 0;
    dec->head = 0;
    dec->len = 0;
    dec->matched_up = 0;
    dec->matched_left = 0;
    dec->matched_right = 0;
    memset(dec->buffer, 0, sizeof(dec->buffer));
	dec->buf_index =0;
	memset(dec->symbol_buf, 0, sizeof(dec->symbol_buf));
	dec->frame_ready = 0;
    rt_hw_interrupt_enable(level);
}

static void ir_send_event(IrDecoder_t *dec)
{
    if (dec == &s_left_decoder)
        rt_event_send(&s_ir_event, IR_EVENT_LEFT);
    else
        rt_event_send(&s_ir_event, IR_EVENT_RIGHT);
}

/* ======================== 定时器中断处理 ======================== */
static void timer_isr(IrDecoder_t *dec)
{
	dec->t_irq_cnt++;   // 记录中断次数
	rt_device_control(dec->timer, HWTIMER_CTRL_STOP, RT_NULL);
	if (dec->state == DECODE_STATE_RECEIVING ) {
		if (dec->last_level == 1) {
			 // 计算1帧还差多少bits
//			uint8_t cnt = SYMBOL_BITS - dec->bit_in_byte + SYMBOL_BITS * ( SYMBOL_COUNT - 1 - dec->buf_index);
//			if(cnt < 8){    // 合理的补1 数量太大说明数据就不对。正常是1个1或者5个1。其他其实多不正常
//				// 将 bits 个 1 写入buffer（因为 last_level=1）				
//					rt_base_t irq_level = rt_hw_interrupt_disable();								
//					for(int i=0;i<cnt;i++){
//						dec->current_byte = (dec->current_byte << 1) | 0x01;
//						dec->bit_in_byte++;
//						if (dec->bit_in_byte == SYMBOL_BITS) {
//							if(dec->buf_index>= MAX_SYMBOLS){
//								dec->buf_index--;
//							}
//							dec->buffer[dec->buf_index] = dec->current_byte & 0x0F;
//							dec->buf_index++;
//							dec->bit_in_byte = 0;
//							dec->current_byte = 0;
//						}
//					}
//					rt_hw_interrupt_enable(irq_level);
//			}
			
					// 期望总符号数（最多2帧）
					uint8_t target_total_symbols = SYMBOL_COUNT * 2;  // 20
					if (dec->buf_index < target_total_symbols) {
							uint8_t missing_symbols = target_total_symbols - dec->buf_index;
							uint8_t missing_bits = missing_symbols * SYMBOL_BITS - dec->bit_in_byte;
							if (missing_bits > 0 && missing_bits <= MAX_CONSECUTIVE_BITS * SYMBOL_BITS) {
									rt_base_t irq_level = rt_hw_interrupt_disable();
									for (int i = 0; i < missing_bits; i++) {
											dec->current_byte = (dec->current_byte << 1) | 0x01;
											dec->bit_in_byte++;
											if (dec->bit_in_byte == SYMBOL_BITS) {
													if (dec->buf_index >= MAX_SYMBOLS) dec->buf_index--;
													dec->buffer[dec->buf_index++] = dec->current_byte & 0x0F;
													dec->bit_in_byte = 0;
													dec->current_byte = 0;
											}
									}
									rt_hw_interrupt_enable(irq_level);
							}
					}					
			}				
			dec->frame_ready = 1;    // 事件处理完清标志
			ir_send_event(dec);		// 发起事件处理		
	}
}

// 右接收管定时器超时中断
static rt_err_t timer6_isr(rt_device_t dev, rt_size_t size)
{	
	timer_isr(&s_left_decoder);
	return 0;
}
// 右接收管定时器超时中断
static rt_err_t timer7_isr(rt_device_t dev, rt_size_t size)
{	
	timer_isr(&s_right_decoder);
	return 0;
}

/* ======================== IO中断处理 ======================== */
static void ir_edge_handler(IrDecoder_t *dec, uint8_t current_level)
{
//    if (dec->frame_ready && current_level == 0) {
//        // 上一帧已结束但未处理，新下降沿到来，强制重置并重新开始
//        reset_decoder(dec);
//    }
//	dec->io_irq_cnt++;
	// 第一个下降沿 开启定时器计时
    if (dec->state == DECODE_STATE_IDLE && current_level == 0) {
			reset_decoder(dec);          // 清空所有状态
		dec->last_level = current_level;
        dec->state = DECODE_STATE_RECEIVING;
		dec->buf_index = 0;
		rt_device_write(dec->timer, 0, &timeout, sizeof(timeout));
        return;
    }

	/* 读取当前计数值（即脉冲宽度，us）*/
	struct rt_hwtimerval t;
	uint32_t pulse_width_us;
	if (rt_device_read(dec->timer, 0, &t, sizeof(t)) == sizeof(t)) {
		pulse_width_us = t.sec * 1000000UL + t.usec;
	} else {
		pulse_width_us = 0;
	}	
    uint32_t diff = pulse_width_us;
    if (diff < 300){
			dec->last_level = current_level;
			return;  // 防抖，去毛刺
		} 

    // 计算连续相同电平的位数（四舍五入）
    int bits = (diff + 500) / BIT_NOMINAL_US;   // 500 = BIT_NOMINAL_US/2 减少运算时间
    if (bits < 1) bits = 1;    // [300,500)=1 或 [500,1500)=1
//    if (bits > MAX_CONSECUTIVE_BITS) bits = MAX_CONSECUTIVE_BITS;  // 连续同一个电平,最大11个1

    uint8_t level_bit = dec->last_level;
    rt_base_t irq_level = rt_hw_interrupt_disable();
    for (int i = 0; i < bits; i++) {
		dec->current_byte = (dec->current_byte << 1) | level_bit;
		dec->bit_in_byte++;
		if (dec->bit_in_byte == SYMBOL_BITS) {
			if(dec->buf_index>= MAX_SYMBOLS){
				dec->buf_index--;
			}
			dec->buffer[dec->buf_index] = dec->current_byte & 0x0F;
			dec->buf_index++;
			dec->bit_in_byte = 0;
			dec->current_byte = 0;
		}
    }
    rt_hw_interrupt_enable(irq_level);
	
    dec->last_level = current_level;
		rt_device_write(dec->timer, 0, &timeout, sizeof(timeout));
	return ;
}

static void left_irq_callback(void *args)
{
    uint8_t level = rt_pin_read(IR_SENSOR_LEFT_PIN);
    ir_edge_handler(&s_left_decoder, level);
}

static void right_irq_callback(void *args)
{
    uint8_t level = rt_pin_read(IR_SENSOR_RIGHT_PIN);
    ir_edge_handler(&s_right_decoder,level);
}


static void timer6_init(void)
{
    s_left_decoder.timer = rt_device_find("timer6");
    if (s_left_decoder.timer == RT_NULL) {
        rt_kprintf("[IR] Cannot find timer6, using fallback to register operation\n");
        return;
    }
    rt_device_open(s_left_decoder.timer, RT_DEVICE_OFLAG_RDWR);
    
    /* 设置频率为 1MHz */
    uint32_t freq = 1000000;
    rt_device_control(s_left_decoder.timer, HWTIMER_CTRL_FREQ_SET, &freq);
    
    /* 设置为周期模式 */
    rt_hwtimer_mode_t mode = HWTIMER_MODE_ONESHOT;
    rt_device_control(s_left_decoder.timer, HWTIMER_CTRL_MODE_SET, &mode);

    rt_device_set_rx_indicate(s_left_decoder.timer, timer6_isr);

    // if (rt_device_write(s_left_decoder.timer, 0, &timeout, sizeof(timeout)) != sizeof(timeout)) {
        // rt_kprintf("[IR] Failed to set timer6 timeout\n");
        // return;
    // }
}

static void timer7_init(void)
{
    s_right_decoder.timer = rt_device_find("timer7");
    if (s_right_decoder.timer == RT_NULL) {
        rt_kprintf("[IR] Cannot find timer7\n");
        return;
    }
    rt_device_open(s_right_decoder.timer, RT_DEVICE_OFLAG_RDWR);

    /* 设置频率为 1MHz */
    uint32_t freq = 1000000;
    rt_device_control(s_right_decoder.timer, HWTIMER_CTRL_FREQ_SET, &freq);

    rt_hwtimer_mode_t mode = HWTIMER_MODE_ONESHOT;
    rt_device_control(s_right_decoder.timer, HWTIMER_CTRL_MODE_SET, &mode);

    rt_device_set_rx_indicate(s_right_decoder.timer, timer7_isr);
    // 设置超时时间为 FRAME_TIMEOUT_MS ms 并启动定时器
    // if (rt_device_write(s_right_decoder.timer, 0, &timeout, sizeof(timeout)) != sizeof(timeout)) {
        // rt_kprintf("[IR] Failed to set timer7 timeout\n");
        // return;
    // }
}

/* ======================== 数据处理 ======================== */
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
            dec->head = (dec->head + SYMBOL_COUNT) % MAX_SYMBOLS;
            dec->len -= SYMBOL_COUNT;
            // 匹配成功后，如果 len == 0，则自动等待后续符号（第二帧）
//						rt_kprintf("-%d-",matched);
        } else {
					 // 无法匹配，丢弃一个符号，滑动窗口
						dec->head = (dec->head + 1) % MAX_SYMBOLS;
						dec->len--;
            // 无效数据，重置整个解码器
            // reset_decoder(dec);
            // return;
        }
    }
}

// buffer to symbol and matches
static int push_symbol(IrDecoder_t *dec)
{
	memcpy(dec->symbol_buf,dec->buffer,dec->buf_index);
	dec->len = dec->buf_index;
    if (dec->len >= SYMBOL_COUNT) {
        scan_and_update_matches(dec);
    }
    // 返回是否有任何匹配
    return (dec->matched_up || dec->matched_left || dec->matched_right);	
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
				// 打印左通道原始符号数据（调试用）
//				rt_kprintf("[IR] Left frame symbols: ");
//				for (int i = 0; i < s_left_decoder.buf_index; i++) {
//						rt_kprintf("%d ", s_left_decoder.buffer[i]);
//				}
//				rt_kprintf("\n");	
//				rt_kprintf("bit=%d\n ",s_left_decoder.bit_in_byte);				

//    static uint32_t left_event_cnt = 0;
//    left_event_cnt++;
//    rt_kprintf("Left event count: %d\n", left_event_cnt);							
				if(push_symbol(&s_left_decoder)){
					update_public_status(&s_left_decoder);			
					pkt.lr = 1;   // 左接收管
					pkt.mask = (s_left_decoder.last_up << 2) |
							   (s_left_decoder.last_left << 1) |
							   s_left_decoder.last_right;
					if(pkt.mask > 2){     // 没有上管时区分不了左右管
							uart_packet_send(PKT_FUNC_SYS, &pkt, sizeof(pkt));
//									rt_kprintf("L:%d\n", pkt.mask);
						}					
				}						
				reset_decoder(&s_left_decoder);
            }
            if (recv_evt & IR_EVENT_RIGHT) {
											// 打印左通道原始符号数据（调试用）
//				rt_kprintf("[IR] Right frame symbols: ");
//				for (int i = 0; i < s_right_decoder.buf_index; i++) {
//						rt_kprintf("%d ", s_right_decoder.buffer[i]);
//				}
//				rt_kprintf("\n");			

//    static uint32_t right_event_cnt = 0;
//    right_event_cnt++;
//    rt_kprintf("Right event count: %d\n", right_event_cnt);							
				if(push_symbol(&s_right_decoder)){
					update_public_status(&s_right_decoder);			
					pkt.lr = 2;   // 右接收管
					pkt.mask = (s_right_decoder.last_up << 2) |
							   (s_right_decoder.last_left << 1) |
							   s_right_decoder.last_right;
					if(pkt.mask > 2){
						uart_packet_send(PKT_FUNC_SYS, &pkt, sizeof(pkt));
//									rt_kprintf("R:%d\n", pkt.mask);
					}				
				}						
				reset_decoder(&s_right_decoder);				
            }			

//				rt_kprintf("Timer Left IRQ: %d, Right IRQ: %d\n", 
//               s_left_decoder.t_irq_cnt, s_right_decoder.t_irq_cnt);
//				rt_kprintf("IO Left IRQ: %d, Right IRQ: %d\n", 
//               s_left_decoder.io_irq_cnt, s_right_decoder.io_irq_cnt);						
        }
    }
}

/* ======================== 对外接口 ======================== */
void ir_receiver_init(void)
{
		rt_kprintf("[IR] initialized \n");
    memset(&s_left_decoder, 0, sizeof(s_left_decoder));
    memset(&s_right_decoder, 0, sizeof(s_right_decoder));
    s_left_decoder.state = DECODE_STATE_IDLE;
    s_right_decoder.state = DECODE_STATE_IDLE;

    rt_pin_mode(IR_SENSOR_LEFT_PIN, PIN_MODE_INPUT);
    rt_pin_mode(IR_SENSOR_RIGHT_PIN, PIN_MODE_INPUT);

    timer6_init();
	timer7_init();
	timeout.sec = 0;
	timeout.usec = FRAME_TIMEOUT_MS * 1000;
    rt_event_init(&s_ir_event, "ir_evt", RT_IPC_FLAG_FIFO);

    rt_pin_attach_irq(IR_SENSOR_LEFT_PIN, PIN_IRQ_MODE_RISING_FALLING, left_irq_callback, RT_NULL);
    rt_pin_attach_irq(IR_SENSOR_RIGHT_PIN, PIN_IRQ_MODE_RISING_FALLING, right_irq_callback, RT_NULL);
    rt_pin_irq_enable(IR_SENSOR_LEFT_PIN, PIN_IRQ_ENABLE);
    rt_pin_irq_enable(IR_SENSOR_RIGHT_PIN, PIN_IRQ_ENABLE);

	rt_thread_t report_thread = rt_thread_create("ir_report", ir_report_thread, NULL,
														IR_REPORT_THREAD_STACK_SIZE,
														IR_REPORT_THREAD_PRIORITY,
														10);
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
    cnt = s_left_decoder.frame_cnt;
    rt_hw_interrupt_enable(level);
    return cnt;
}

uint16_t ir_get_right_match_cnt(void)
{
    uint16_t cnt;
    rt_base_t level = rt_hw_interrupt_disable();
    cnt = s_right_decoder.frame_cnt;
    rt_hw_interrupt_enable(level);
    return cnt;
}

void ir_clear_left_status(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    s_left_decoder.last_up = 0;
    s_left_decoder.last_left = 0;
    s_left_decoder.last_right = 0;
    rt_hw_interrupt_enable(level);
}

void ir_clear_right_status(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    s_right_decoder.last_up = 0;
    s_right_decoder.last_left = 0;
    s_right_decoder.last_right = 0;
    rt_hw_interrupt_enable(level);
}

/****************msh 控制台调试******************************/
#ifdef RT_USING_MSH
#include <stdlib.h>

static void ir_show_status(void)
{
    rt_kprintf("\n========== IR Receiver Status ==========\n");
    
    // 左通道信息
    rt_kprintf("Left:\n");
    rt_kprintf("  state=%d, frame_ready=%d, buf_index=%d, bit_in_byte=%d\n",
               s_left_decoder.state, s_left_decoder.frame_ready,
               s_left_decoder.buf_index, s_left_decoder.bit_in_byte);
    rt_kprintf("  matched: up=%d, left=%d, right=%d\n",
               s_left_decoder.matched_up, s_left_decoder.matched_left, s_left_decoder.matched_right);
    rt_kprintf("  last: up=%d, left=%d, right=%d, frame_cnt=%d\n",
               s_left_decoder.last_up, s_left_decoder.last_left, s_left_decoder.last_right,
               s_left_decoder.frame_cnt);
    rt_kprintf("  buffer (first 20): ");
    for (int i = 0; i < (s_left_decoder.buf_index < 20 ? s_left_decoder.buf_index : 20); i++) {
        rt_kprintf("%d ", s_left_decoder.buffer[i]);
    }
    rt_kprintf("\n");
    
    // 右通道信息
    rt_kprintf("Right:\n");
    rt_kprintf("  state=%d, frame_ready=%d, buf_index=%d, bit_in_byte=%d\n",
               s_right_decoder.state, s_right_decoder.frame_ready,
               s_right_decoder.buf_index, s_right_decoder.bit_in_byte);
    rt_kprintf("  matched: up=%d, left=%d, right=%d\n",
               s_right_decoder.matched_up, s_right_decoder.matched_left, s_right_decoder.matched_right);
    rt_kprintf("  last: up=%d, left=%d, right=%d, frame_cnt=%d\n",
               s_right_decoder.last_up, s_right_decoder.last_left, s_right_decoder.last_right,
               s_right_decoder.frame_cnt);
    rt_kprintf("  buffer (first 20): ");
    for (int i = 0; i < (s_right_decoder.buf_index < 20 ? s_right_decoder.buf_index : 20); i++) {
        rt_kprintf("%d ", s_right_decoder.buffer[i]);
    }
    rt_kprintf("\n");
    
		rt_kprintf("Timer Left IRQ: %d, Right IRQ: %d\n", 
               s_left_decoder.t_irq_cnt, s_right_decoder.t_irq_cnt);
//		rt_kprintf("IO Left IRQ: %d, Right IRQ: %d\n", 
//               s_left_decoder.io_irq_cnt, s_right_decoder.io_irq_cnt);			
    rt_kprintf("=========================================\n");
}

static void ir_cmd(int argc, char **argv)
{
    if (argc >= 2 && rt_strcmp(argv[1], "clear") == 0) {
        ir_clear_left_status();
        ir_clear_right_status();
        rt_kprintf("[IR] Status cleared\n");
        return;
    }
    ir_show_status();
}

MSH_CMD_EXPORT(ir_cmd, "ir_cmd - show IR receiver status, ir_cmd clear - clear status");
#endif /* RT_USING_MSH */
