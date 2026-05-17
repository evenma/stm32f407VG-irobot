#include "packet_handle.h"
#include "packet_reports.h"
#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "monitor.h"
#include "global_conf.h"
#include "uart_packet.h"    
#include "oled_handle.h"
#include "led.h"
#include "buzzer.h"
#include "car_action.h"
#include "user_action.h"
#include "zltech_can_motor.h"
#include "wc_drv.h"

#pragma pack(1)
/* 定义各种命令结构体（与原代码相同）*/
// 注意：收到的speed是计算转速 单位rps, 转每秒 和rpm的转换：rpm = rps *60  此处只支持多电机同步操作，不支持单电机操作，防止不同步
typedef struct {
    uint8_t cmd;
    uint8_t motor_num;
    struct {
        uint8_t motor_id;
        float speed;   //4 字节（float，小端模式）
    } element[];
} MotorMutilCtrlCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint8_t motor_mask;
} MotorMultiStopCommandTypeDef;

/* 多电机位置控制命令 */
typedef struct {
    uint8_t cmd;           // 子命令（0x0a）
    uint8_t motor_num;
    struct {
        uint8_t motor_id;
        int32_t pos;       // 单位：脉冲数
    } element[];
} MotorMultiPosCommandTypeDef;

/* LED */
typedef struct {
    uint8_t led_id;
    uint16_t on_time;
    uint16_t off_time;
    uint16_t repeat;
} LedCommandTypeDef;

/* 无源蜂鸣器 */
typedef struct {
    uint16_t freq;	
    uint16_t on_time;
    uint16_t off_time;
    uint16_t repeat;
} BuzzerNoSourceCommandTypeDef;

/* OLED */
typedef struct {
    uint8_t sub_cmd;
    uint8_t length;
    uint8_t data[];
} OLEDCommandTypeDef;

/* 电机类型切换 */
typedef struct {
    uint8_t func;
    uint8_t type;
} MotorTypeCtlTypeDef;

/* 电压报警值设置 */
typedef struct {
    uint8_t cmd;
    uint16_t limit;
} BatteryWarnTypeDef;

/* 灯光控制命令 */
typedef struct {
    uint8_t cmd;        // 0x01 = 开, 0x02 = 关
} LightCommandTypeDef;

/* 充电控制命令 */
typedef struct {
    uint8_t cmd;        // 0x01 = 开, 0x02 = 关
} ChargerCommandTypeDef;

/* 马桶控制命令（通用）*/
typedef struct {
    uint8_t cmd;        // 对应 ActionCmd_t 枚举值
    uint8_t param[4];   // 可选参数（小端，如 uint16_t 只使用低2字节）
} ToiletCommandTypeDef;

#pragma pack()

/* OLED 互斥量（RT-Thread）*/
#if ENABLE_OLED
static rt_mutex_t oled_mutex = RT_NULL;
extern char oled_l1[];
extern char oled_l2[];
extern char oled_l3[];
extern char oled_l4[];
#endif

#ifndef CAR_PACK_PARAM
#define CAR_PACK_PARAM(low16, high16)  (((uint32_t)(high16) << 16) | ((uint32_t)(low16) & 0xFFFF))
#endif

/* ========== 命令回调函数（与 uart_packet 注册）========== */
static void packet_led_handle(pkt_frame_t *frame)
{
    LedCommandTypeDef *cmd = (LedCommandTypeDef*)frame->data_and_checksum;
    uint8_t led_id = cmd->led_id - 1;
    if(led_id < 2) {
//				led_set_color(led_id + 6, LED_COLOR_ON); 测试直接点亮
        led_flash(led_id, cmd->on_time, cmd->off_time, cmd->repeat);
    }
}

static void packet_buzzer_handle(pkt_frame_t *frame)
{
    BuzzerNoSourceCommandTypeDef *cmd = (BuzzerNoSourceCommandTypeDef*)frame->data_and_checksum;
	BuzzerCommandTypeDef local_cmd = {
        .on_time = cmd->on_time,
        .off_time = cmd->off_time,
        .repeat = cmd->repeat
    };
	buzzer_start(&local_cmd);
}

static void packet_motor_handle(pkt_frame_t *frame)
{
    uint8_t *data = frame->data_and_checksum;
	if (frame->data_len < 1) return;  // 至少要有子命令字节

    uint8_t sub_cmd = data[0];
    uint32_t param = 0;	
    switch(sub_cmd) {
        case 0: {
			// 不支持单电机操作，防止不同步
            break;
        }
        case 1: {  /* 设置速度：需要左速度(2字节) + 右速度(2字节) */
		    // 检查数据长度至少为 2 (cmd + motor_num)
            if (frame->data_len < 2) {
                rt_kprintf("[PACKET] motor speed cmd too short\n");
                break;
            }
            MotorMutilCtrlCommandTypeDef *cmd = (MotorMutilCtrlCommandTypeDef*)data;
            uint8_t motor_num = cmd->motor_num;
            // 检查数据长度是否匹配：2 + motor_num * (1+4)
            if (frame->data_len < (2 + motor_num * 5)) {
                rt_kprintf("[PACKET] motor speed cmd length mismatch\n");
                break;
            }			
		//	上位机发送时，float 通过 struct.pack("<f", speed) 转换为 4 字节小端二进制数据，与嵌入式中的 float 内存布局一致。因此，下位机直接 memcpy 即可正确解析
            float left_rps = 0.0f;   // 左后轮 ID2
            float right_rps = 0.0f;  // 右后轮 ID4
            int left_found = 0, right_found = 0;
			for (int i = 0; i < motor_num; i++) {
                uint8_t id = cmd->element[i].motor_id;
                float speed = cmd->element[i].speed;  // 单位 rps
                if (id == 2) {
                    left_rps = speed;
                    left_found = 1;
                } else if (id == 4) {
                    right_rps = speed;
                    right_found = 1;
                }
                // 其他 ID 忽略（ID1, ID3 速度无效）
            }
			if (!left_found || !right_found) {
				rt_kprintf("[PACKET] Missing left (ID2) or right (ID4) motor speed\n");
				// 可以选择不更新速度，或保持上一帧速度
				break;
			}
			
            // 转换为 rpm，并限幅
            int16_t left_rpm = (int16_t)(left_rps * 60.0f + 0.5f);
            int16_t right_rpm = (int16_t)(right_rps * 60.0f + 0.5f);
			if (left_rpm > ZLAC_MOTOR_MAX_RPM) left_rpm = ZLAC_MOTOR_MAX_RPM;
			if (left_rpm < -ZLAC_MOTOR_MAX_RPM) left_rpm = -ZLAC_MOTOR_MAX_RPM;
			if (right_rpm > ZLAC_MOTOR_MAX_RPM) right_rpm = ZLAC_MOTOR_MAX_RPM;
			if (right_rpm < -ZLAC_MOTOR_MAX_RPM) right_rpm = -ZLAC_MOTOR_MAX_RPM;

			param = CAR_PACK_VEL(left_rpm, right_rpm);  // 左轮在高16位，右轮在低16位
            car_action_send_cmd(CAR_CMD_SET_VEL, param);			
            break;
        }
        case 2: {
            car_action_send_cmd(CAR_CMD_STOP, 0);
            break;
        }
        case 3: {
            MotorMultiStopCommandTypeDef *cmd = (MotorMultiStopCommandTypeDef*)data;
			car_action_send_cmd(CAR_CMD_STOP, 0);
            break;
        }
        case 5: { // 电机类型切换
			// 目前只支持ZLAC8015D驱动器和ZLLG65ASM250-4096-B V2.0单出轴 不做类型切换
            break;
        }
		
		case 0x0a: {
			/* 多电机位置控制：格式同速度，但 pos 为 int32_t */
			if (frame->data_len < 2) {
				rt_kprintf("[PACKET] Position cmd too short\n");
				break;
			}
			MotorMultiPosCommandTypeDef *cmd = (MotorMultiPosCommandTypeDef*)data;
			uint8_t motor_num = cmd->motor_num;
			// 总长度 = 2 (cmd + motor_num) + motor_num * (1 + 4)
			if (frame->data_len < (2 + motor_num * 5)) {
				rt_kprintf("[PACKET] Position cmd length mismatch\n");
				break;
			}

			int32_t left_pos = 0, right_pos = 0;
			int left_found = 0, right_found = 0;

			for (int i = 0; i < motor_num; i++) {
				uint8_t id = cmd->element[i].motor_id;
				int32_t pos = cmd->element[i].pos;
				if (id == 2) {       // 左后轮
					left_pos = pos;
					left_found = 1;
				} else if (id == 4) { // 右后轮
					right_pos = pos;
					right_found = 1;
				}
				// 其他 ID 忽略
			}

			if (!left_found || !right_found) {
				rt_kprintf("[PACKET] Missing required motor ID (2 and 4) for position\n");
				break;
			}

			// 设置全局变量
			car_action_set_position_global(left_pos, right_pos);
			// 发送邮箱命令（参数 param 可以随意，不使用）
			car_action_send_cmd(CAR_CMD_SET_POS, 0);

			break;
		}
		
        case 0x11:
            car_action_send_cmd(CAR_CMD_VEL_MODE, 0);
            break;
        case 0x12:
            car_action_send_cmd(CAR_CMD_POS_MODE_ABS, 0);
            break;
        case 0x13:
            car_action_send_cmd(CAR_CMD_POS_MODE_REL, 0);
            break;
        case 0x14:
            car_action_send_cmd(CAR_CMD_ZERO_POS, 0);
            break;
        case 0x15:
            car_action_send_cmd(CAR_CMD_CLEAR_FAULT, 0);
            break;
        case 0x20:
            car_action_send_cmd(CAR_CMD_ENABLE, 0);
            break;
        case 0x21:
            car_action_send_cmd(CAR_CMD_DISABLE, 0);
            break;
        case 0x22:
            car_action_send_cmd(CAR_CMD_FREE, 0);
            break;
        case 0x30:
            car_action_send_cmd(CAR_CMD_BRAKE_ON, 0);
            break;
        case 0x31:
            car_action_send_cmd(CAR_CMD_BRAKE_OFF, 0);
            break;
        case 0x40:
            car_action_send_cmd(CAR_CMD_QUICK_STOP, 0);
            break;
        case 0x41:
            car_action_send_cmd(CAR_CMD_STOP, 0);
            break;
        case 0x42:
            car_action_send_cmd(CAR_CMD_MOVE_START, 0);
            break;		
				
        default:
            break;
    }
}

static void packet_battery_limit_handle(pkt_frame_t *frame)
{
    BatteryWarnTypeDef *cmd = (BatteryWarnTypeDef*)frame->data_and_checksum;
    if(cmd->cmd == 1) {
        change_battery_limit(cmd->limit);
    }
}

#if ENABLE_OLED
static void packet_oled_handle(pkt_frame_t *frame)
{
    OLEDCommandTypeDef *cmd = (OLEDCommandTypeDef*)frame->data_and_checksum;
    if (oled_mutex == RT_NULL) {
        oled_mutex = rt_mutex_create("oled_mtx", RT_IPC_FLAG_PRIO);
        if (oled_mutex == RT_NULL) {
            rt_kprintf("[ERROR] packet_oled: create mutex failed\n");
            return;
        }
    }
    rt_mutex_take(oled_mutex, RT_WAITING_FOREVER);
    switch(cmd->sub_cmd) {
        case 0x01:
            memcpy(oled_l1, cmd->data, cmd->length);
            oled_l1[cmd->length] = '\0';
            break;
        case 0x02:
            memcpy(oled_l2, cmd->data, cmd->length);
            oled_l2[cmd->length] = '\0';
            break;
		case 0x03:  
            memcpy(oled_l3, cmd->data, cmd->length);
            oled_l3[cmd->length] = '\0';
            break;
        case 0x04:  
            memcpy(oled_l4, cmd->data, cmd->length);
            oled_l4[cmd->length] = '\0';
            break;
        default:
            break;
    }
    rt_mutex_release(oled_mutex);
	oled_trigger_refresh();  // 确保更新显示
}
#endif

static void packet_light_handle(pkt_frame_t *frame)
{
    if (frame->data_len < sizeof(LightCommandTypeDef)) {
        rt_kprintf("[PACKET] Light cmd too short\n");
        return;
    }
    LightCommandTypeDef *cmd = (LightCommandTypeDef*)frame->data_and_checksum;
    switch (cmd->cmd) {
        case 0x01:
            // 开灯
            car_action_send_cmd(CAR_CMD_LIGHT_ON, 0);
            rt_kprintf("[LIGHT] ON\n");
            break;
        case 0x02:
            // 关灯
            car_action_send_cmd(CAR_CMD_LIGHT_OFF, 0);
            rt_kprintf("[LIGHT] OFF\n");
            break;
        default:
            rt_kprintf("[PACKET] Unknown light cmd: 0x%02X\n", cmd->cmd);
            break;
    }
}

static void packet_charger_handle(pkt_frame_t *frame)
{
    if (frame->data_len < sizeof(ChargerCommandTypeDef)) {
        rt_kprintf("[PACKET] Charger cmd too short\n");
        return;
    }
    ChargerCommandTypeDef *cmd = (ChargerCommandTypeDef*)frame->data_and_checksum;
    switch (cmd->cmd) {
        case 0x01:
            car_action_send_cmd(CAR_CMD_CHARGER_ON, 0);
            rt_kprintf("[CHARGER] ON\n");
            break;
        case 0x02:
            car_action_send_cmd(CAR_CMD_CHARGER_OFF, 0);
            rt_kprintf("[CHARGER] OFF\n");
            break;
        default:
            rt_kprintf("[PACKET] Unknown charger cmd: 0x%02X\n", cmd->cmd);
            break;
    }
}

static void packet_toilet_handle(pkt_frame_t *frame)
{
    uint8_t *data = frame->data_and_checksum;
    if (frame->data_len < 1) {
        rt_kprintf("[PACKET] Toilet cmd too short\n");
        return;
    }
    uint8_t sub_cmd = data[0];
    uint32_t param = 0;

    // 提取参数（根据不同命令决定是否有效）
    if (frame->data_len >= 5) {
        // 小端模式读取 4 字节参数
        param = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
    } else if (frame->data_len >= 3) {
        // 兼容 2 字节参数（例如位置值）
        param = data[1] | (data[2] << 8);
    } else if(frame->data_len >= 2) {
        // 兼容 1 字节参数（）
        param = data[1];
    }

    switch (sub_cmd) {
        case ACTION_FLUSH_TOILET:
            user_action_send_cmd(ACTION_FLUSH_TOILET, 0);
            rt_kprintf("[TOILET] Flush\n");
            break;
        case ACTION_CLEAN_REAR:
            user_action_send_cmd(ACTION_CLEAN_REAR, 0);
            rt_kprintf("[TOILET] Rear clean\n");
            break;
        case ACTION_CLEAN_FEMALE:
            user_action_send_cmd(ACTION_CLEAN_FEMALE, 0);
            rt_kprintf("[TOILET] Female clean\n");
            break;
        case ACTION_DRY:
            user_action_send_cmd(ACTION_DRY, 0);
            rt_kprintf("[TOILET] Dry\n");
            break;
        case ACTION_STOP:
//            user_action_send_cmd(ACTION_STOP, 0);
						user_action_stop();
            rt_kprintf("[TOILET] Stop\n");
            break;
        case ACTION_SELF_CLEAN:
            user_action_send_cmd(ACTION_SELF_CLEAN, 0);
            rt_kprintf("[TOILET] Self clean\n");
            break;
        case ACTION_SEAT_OPEN:
            user_action_send_cmd(ACTION_SEAT_OPEN, 0);
            rt_kprintf("[TOILET] Seat open\n");
            break;
        case ACTION_SEAT_CLOSE:
            user_action_send_cmd(ACTION_SEAT_CLOSE, 0);
            rt_kprintf("[TOILET] Seat close\n");
            break;
        case ACTION_LID_OPEN:
            user_action_send_cmd(ACTION_LID_OPEN, 0);
            rt_kprintf("[TOILET] Lid open\n");
            break;
        case ACTION_LID_CLOSE:
            user_action_send_cmd(ACTION_LID_CLOSE, 0);
            rt_kprintf("[TOILET] Lid close\n");
            break;
        case ACTION_SEWAGE_PUMP:
            user_action_send_cmd(ACTION_SEWAGE_PUMP, 0);
            rt_kprintf("[TOILET] Sewage pump\n");
            break;
        case ACTION_UV_LIGHT_ON:
            user_action_send_cmd(ACTION_UV_LIGHT_ON, 0);
            rt_kprintf("[TOILET] UV on\n");
            break;
        case ACTION_UV_LIGHT_OFF:
            user_action_send_cmd(ACTION_UV_LIGHT_OFF, 0);
            rt_kprintf("[TOILET] UV off\n");
            break;
        case ACTION_IR_LIGHT_ON:
            user_action_send_cmd(ACTION_IR_LIGHT_ON, 0);
            rt_kprintf("[TOILET] IR on\n");
            break;
        case ACTION_IR_LIGHT_OFF:
            user_action_send_cmd(ACTION_IR_LIGHT_OFF, 0);
            rt_kprintf("[TOILET] IR off\n");
            break;
        case ACTION_SET_CLEAN_ROD_POS:
            if (param <= CLEAN_ROD_MAX_STEPS) {
                user_action_send_cmd(ACTION_SET_CLEAN_ROD_POS, param);
                rt_kprintf("[TOILET] Set rod pos=%ld\n", param);
            } else {
                rt_kprintf("[TOILET] Invalid rod pos %ld\n", param);
            }
            break;
        case ACTION_SET_SMALL_PUMP_DUTY:
            if (param <= 100) {
//                user_action_send_cmd(ACTION_SET_SMALL_PUMP_DUTY, param);
							cmd_set_pump_duty(param);
                rt_kprintf("[TOILET] Set pump duty=%ld\n", param);
            } else {
                rt_kprintf("[TOILET] Invalid duty %ld\n", param);
            }
            break;
			  case ACTION_SET_WATER_HEATER:
            user_action_send_cmd(ACTION_SET_WATER_HEATER, param ? 1 : 0);
            rt_kprintf("[TOILET] Water heater %s\n", param ? "ON" : "OFF");
            break;		
        case ACTION_SET_WARM_HEATER:
            if (param <= 100) {
                user_action_send_cmd(ACTION_SET_WARM_HEATER, param);
                rt_kprintf("[TOILET] Set heater power=%ld\n", param);
            } else {
                rt_kprintf("[TOILET] Invalid power %ld\n", param);
            }
            break;
				case ACTION_SET_WATER_TEMP: {  /* 设置水温目标值 */
						if (frame->data_len < 3) {
								rt_kprintf("[TOILET] Set target temp cmd too short\n");
								break;
						}
						uint16_t target_temp = data[1] | (data[2] << 8);
						// 范围限制：30°C ~ 45°C（300 ~ 450）
						if (target_temp < 300) target_temp = 300;
						if (target_temp > 450) target_temp = 450;
						user_action_set_water_target_temp(target_temp);
						rt_kprintf("[TOILET] Water target temperature set to %d.%d°C\n",
											 target_temp / 10, target_temp % 10);
						break;
				}
        case ACTION_SET_WARM_FAN:
            user_action_send_cmd(ACTION_SET_WARM_FAN, param ? 1 : 0);
            rt_kprintf("[TOILET] Warm fan %s\n", param ? "ON" : "OFF");
            break;
        case ACTION_TOGGLE_CLEAN_MODE:
//            user_action_send_cmd(ACTION_TOGGLE_CLEAN_MODE, 0);
						user_action_toggle_clean_mode();
            rt_kprintf("[TOILET] Toggle clean mode\n");
            break;
        case ACTION_CLEAN_ROD_INC:
//            user_action_send_cmd(ACTION_CLEAN_ROD_INC, 0);
						user_action_clean_rod_inc();
            rt_kprintf("[TOILET] Rod inc\n");
            break;
        case ACTION_CLEAN_ROD_DEC:
						user_action_clean_rod_dec();			
//            user_action_send_cmd(ACTION_CLEAN_ROD_DEC, 0);
            rt_kprintf("[TOILET] Rod dec\n");
            break;
				case ACTION_SET_WATER_HEATER_AUTO:
						user_action_send_cmd(ACTION_SET_WATER_HEATER_AUTO, param);
						rt_kprintf("[TOILET] Auto water heater %s\n", param ? "ON" : "OFF");
						break;
        default:
            rt_kprintf("[PACKET] Unknown toilet cmd: 0x%02X\n", sub_cmd);
            break;
    }
}

/* ========== 初始化：向 uart_packet 注册所有回调 ========== */
int packet_handle_init(void)
{
    uart_packet_register_callback(PKT_FUNC_LED, packet_led_handle);
    uart_packet_register_callback(PKT_FUNC_BUZZER, packet_buzzer_handle);
    uart_packet_register_callback(PKT_FUNC_MOTOR, packet_motor_handle);
    uart_packet_register_callback(PKT_FUNC_SYS, packet_battery_limit_handle);
#if ENABLE_OLED
    uart_packet_register_callback(PKT_FUNC_OLED, packet_oled_handle);
#endif
    uart_packet_register_callback(PKT_FUNC_LIGHT, packet_light_handle);
    uart_packet_register_callback(PKT_FUNC_CHARGER, packet_charger_handle);
	uart_packet_register_callback(PKT_FUNC_TOILET, packet_toilet_handle);
    rt_kprintf("[PACKET] Handle registered to uart_packet\n");
	return RT_EOK;
}
//INIT_BOARD_EXPORT(packet_handle_init);
