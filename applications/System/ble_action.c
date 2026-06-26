#include <rtthread.h>
#include <rtdevice.h>
#include "ble_action.h"
#include "dx2002_at.h"
#include "ATCmdParser.h"
#include "car_action.h"
#include "user_action.h"
#include "uart_packet.h"
#include "packet_reports.h"
#include "global_conf.h"
#include "asr_action.h"
#include "ble_serial.h"
#include "zltech_can_motor.h"
// 日志
#define LOG_TAG "BLE"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>

extern struct rt_event g_stop_evt;   // asr_action

// 蓝牙串口设备名称（与 at_serial 中一致，假设使用 uart5）
#define BLE_UART_NAME "uart5"

// 接收线程栈大小与优先级
#define BLE_RX_STACK_SIZE 2048
#define BLE_RX_PRIORITY   10

rt_bool_t ble_connected = RT_FALSE;
static rt_bool_t free_mode = RT_FALSE;
static rt_bool_t clean_mode = RT_FALSE;		
static rt_uint8_t clean_strength = 2;	
static uint8_t pump_param = 80;
			
// 本地执行函数声明


// 映射手控器按键码到本地动作或上报事件
static void handle_key_event(uint8_t key_code);

// 连接/断开回调
static void ble_conn_callback(void)
{
    LOG_I("Bluetooth connected (slave)");
	ble_connected = RT_TRUE;
}

static void ble_disc_callback(void)
{
    LOG_I("Bluetooth disconnected");
	ble_connected = RT_FALSE;
}

static void oob_qy_callback(void)
{
    int key;
    if (!ATCmdParser_recv("%d\r\n", &key)) {
        LOG_E("QY parse error");
        return;
    }
//    LOG_D("OOB QY: key=%d", key);
    handle_key_event((uint8_t)key);
}

static rt_err_t ble_set_slave_mode(void)
{
    uint8_t mode;
    int retry = 3;
    
    /* Check current mode 
			Bit0:BLE 从机使能 Bit2:BLE 主机使能 默认值：01
	*/
    mode = at_module_get_MASTER();
	LOG_D("mode=%x",mode);
	if(mode == 0x01){
			LOG_D("Already in slave mode");
			return RT_EOK;
	}

    /* Set master mode */
    while (retry--) {
        if (at_module_set_MASTER(0x01) == kNoErr) {
            LOG_D("Slave mode set, module will reboot...");
            /* Module reboots automatically, wait for it */
            rt_thread_mdelay(1500);
            /* Re-enter AT mode */
            if (waitReady() == 0) {
                return RT_EOK;
            }
        }
        rt_thread_mdelay(500);
    }
    return RT_ERROR;
}

// 初始化蓝牙从机模式
static int ble_slave_init(void)
{
    LOG_I("Initializing BLE slave...");

    // 硬复位模块
    ble_module_Hard_reboot();
   
	
	LOG_D("wait connect if not first");
    ATCmdParser_set_mode(0);
    int wait_cnt = 500;  // 等待最多 5S (20ms * 50)
    while (wait_cnt-- && !ble_connected) {
		if (wait_cnt % 25 == 0) rt_kprintf("."); 
		 ATCmdParser_process_oob();   // 处理 OOB，使 IM_CONN 能触发回调
        rt_thread_mdelay(20);
    }
    if (ble_connected) {
        LOG_D("Already connected (auto-reconnect), skip AT init.");
        return RT_EOK;   // 直接成功，不发送任何 AT 指令
    }	
		LOG_D("no connect,AT mode");

    // 进入 AT 模式
    ATCmdParser_set_mode(1);
    if (waitReady() != 0) {
        LOG_E("Module not ready");
    }

    // 设置从机模式（默认可能是从机，但显式设置）
    if (ble_set_slave_mode() != RT_EOK) {
        LOG_E("Failed to set slave mode");
    }		

	 if (ble_connected) {
			ATCmdParser_set_mode(0);
        LOG_D("Already connected (auto-reconnect), skip AT init.");
        return RT_EOK;   // 直接成功，不发送任何 AT 指令
    }	
    // 设置蓝牙名称
	if (ble_module_init() != kNoErr) {
        LOG_E("Failed to read module info");
    }

    // 进入透传模式
    ATCmdParser_set_mode(0);

    LOG_I("BLE slave ready");
    return RT_EOK;
}

static void report_key_command(uint8_t key_code)
{
	    // 单机模式下不上报
    // if (asr_action_is_single_mode()) {
        // return;
    // }
	PacketReportKeyEventTypeDef pkt;
	pkt.key_id = key_code;
	pkt.event = (key_code == KEY_RELEASE) ? 0 : 1;   // 1表示按下（释放事件 key_code=0，但手控器释放时发送0，可以作为 event=0）
	uart_packet_send(PKT_FUNC_KEY, &pkt, sizeof(pkt));
//	LOG_D("Report key %d to host", key_code);
}

static void send_velocity_safely(int16_t left_rpm, int16_t right_rpm)
{
    // 检查电机是否已处于速度模式、使能且抱闸释放
    if (zlac_is_velocity_mode_ready() && zlac_is_left_enabled() && 
        zlac_is_right_enabled() && zlac_is_brake_released()) {
        // 已就绪，直接发送速度指令
        car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(left_rpm, right_rpm));
				LOG_D("RUN(%d,%d)rpm",left_rpm,right_rpm);
    } else {
				LOG_D("START MOVE");
        // 未就绪，先执行启动准备
        car_action_send_cmd(CAR_CMD_MOVE_START, 0);
        // 短暂延时等待电机进入就绪状态（MOVE_START 内部有延时，但加上更可靠）
        rt_thread_mdelay(100);
				LOG_D("START RUN(%d,%d)rpm",left_rpm,right_rpm);
        car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(left_rpm, right_rpm));
    }
}

// 按键指令处理
static void handle_key_event(uint8_t key_code)
{
	LOG_D("BLE cmd: %d", key_code);
	// 上报有按键事件
    report_key_command(key_code);
	switch (key_code) {
/* ----- 小车运动（需判断模式）----- */		
		case KEY_SINGLE_MODE:
			asr_action_set_single_mode(RT_TRUE);
			LOG_I("Single mode from BLE");
			break;
		case KEY_NORMAL_MODE:
			asr_action_set_single_mode(RT_FALSE);
			LOG_I("Normal mode from BLE");
			break;
		case KEY_FORWORD:
            if (asr_action_is_single_mode()) {							
								send_velocity_safely(30, 30);
            } 		
			break;
        case KEY_BACKWORD: 
            if (asr_action_is_single_mode()) {
								send_velocity_safely(-15, -15);
            } 			           
            break;
        case KEY_LEFT: 
            if (asr_action_is_single_mode()) {
								send_velocity_safely(-15, 15);
            } 			           
            break;
        case KEY_RIGHT:  
						if (asr_action_is_single_mode()) {
								send_velocity_safely(15, -15);
            } 	           
            break;
		case KEY_HOME:
			if (asr_action_is_single_mode()) {
				robot_auto_charge();					 
			}
			break;
		case KEY_FREE:
					if (free_mode) {
							car_action_send_cmd(CAR_CMD_BRAKE_ON, 0);
							free_mode = RT_FALSE;
					} else {
							car_action_send_cmd(CAR_CMD_BRAKE_OFF, 0);
							car_action_send_cmd(CAR_CMD_FREE, 0);	
							free_mode = RT_TRUE;
					}		
			break;	
    case KEY_STOPWORK:
				rt_event_send(&g_stop_evt, EVENT_STOP); // 如果是单机模式并且是go HOME动作线程
		    car_action_send_cmd(CAR_CMD_STOP, 0);
				LOG_D("STOP RUN!");
			break;
		case KEY_RELEASE:
            if (asr_action_is_single_mode()) {
								if(zlac_is_left_enabled() && zlac_is_right_enabled()){
										car_action_send_cmd(CAR_CMD_SET_VEL, CAR_PACK_VEL(0, 0));
										LOG_D("RUN(0,0)rpm");
								}
            } 		
			break;
			/* ----- 建图/导航模式（只上报，下位机不处理）----- */
			case KEY_MAPPING_MODE:
			case KEY_NAV_MODE:
			case KEY_ROME_A:
			case KEY_ROME_B:
					// 只上报，不做本地处理
					break;
				
/* ----- 智能马桶功能（直接执行）----- */		
        case KEY_FLUSH:  
            user_action_send_cmd(ACTION_FLUSH_TOILET, 0);
            break;
        case KEY_CLEAN_REAR:  
            user_action_send_cmd(ACTION_CLEAN_REAR, 0);
            break;
        case KEY_CLEAN_FEMALE:  
            user_action_send_cmd(ACTION_CLEAN_FEMALE, 0);
            break;
        case KEY_GANZAO:  
            user_action_send_cmd(ACTION_DRY, 0);
            break;
        case KEY_STOP:  
            user_action_stop();
            break;
        case KEY_BIANMEN_KAI:  
            user_action_send_cmd(ACTION_LID_OPEN, 0);
			rt_kprintf("[TOILET] Seat open\n");
            break;
        case KEY_BIANMEN_GUAN:  
            user_action_send_cmd(ACTION_LID_CLOSE, 0);
            rt_kprintf("[TOILET] Seat close\n");
            break;
         case KEY_SEWAGE:  
            user_action_send_cmd(ACTION_SEWAGE_PUMP, 0);
            break;
		case KEY_CLEAN_MODE:
			if(clean_mode){
				user_action_set_clean_mode(CLEAN_MODE_MASSAGE);
				clean_mode = RT_FALSE;
			}else{
				user_action_set_clean_mode(CLEAN_MODE_FIXED);
				clean_mode = RT_TRUE;
			}
			break;
		case KEY_CLEAN_STRENGTH:// 清洁力度调节	1-3档位 按一下切一次
			clean_strength += 1;
			if(clean_strength > 3)
				clean_strength = 1;
			
			if(clean_strength == 3)
				pump_param = 100;
			else if(clean_strength == 2)
				pump_param = 80;
			else if(clean_strength == 1)
				pump_param = 60;
			
       if (pump_param <= 100) {
				cmd_set_pump_duty(pump_param);
                rt_kprintf("[TOILET] Set pump duty=%ld\n", pump_param);
            } 
			break;
			
        default:
            LOG_W("Unhandled BLE cmd: %d", key_code);
            break;
    }			
			
}

// 接收线程：循环读取蓝牙串口数据
static void ble_rx_thread_entry(void *param)
{
    uint8_t byte;
    int ret;

    while (1) {
//        ret = ble_serial_getc_timeout(1000);   // 阻塞100ms，等待数据
//        if (ret >= 0) {
//            handle_key_event((uint8_t)ret);
//        }
        // 处理 OOB 事件（连接/断开）
        ATCmdParser_process_oob();
				rt_thread_mdelay(10);
    }
}

// 初始化线程
int ble_action_init(void)
{
    rt_thread_t tid;
    int ret;
	
    ATCmdParser_init("\r\n", "\r\n", 1000);
	
   // 注册连接/断开 OOB 回调
    ATCmdParser_add_oob("IM_CONN", ble_conn_callback);
    ATCmdParser_add_oob("IM_DISC", ble_disc_callback);
		ATCmdParser_add_oob("QY=", oob_qy_callback);
	
	ble_slave_init();
		
    tid = rt_thread_create("ble_rx", ble_rx_thread_entry, NULL,
                           BLE_RX_STACK_SIZE, BLE_RX_PRIORITY, 10);
    if (tid == RT_NULL) {
        LOG_E("Create BLE rx thread failed");
        return -RT_ERROR;
    }
    rt_thread_startup(tid);
	
    LOG_I("BLE action initialized");
    return RT_EOK;
}

