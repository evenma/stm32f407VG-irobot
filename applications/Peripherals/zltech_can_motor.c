/**
 * @file zltech_can_motor.c
 * @brief ZLAC8015D 双电机驱动器 CANopen 底层驱动 (速度模式 + 位置模式)
 * @version 1.0
 * @date 2026-04-17
 * 
 * 支持功能:
 * - CAN 通信初始化，硬件滤波，接收回调
 * - SDO 读写 (任意长度，自动分段)
 * - NMT 服务 (启动、停止、复位)
 * - 状态机控制 (使能、停用、急停、故障复位)
 * - PDO 动态配置 (速度模式 / 位置模式)
 * - 实时速度控制 (PDO 发送目标速度)
 * - 实时位置控制 (PDO 发送目标位置)
 * - 读取实际速度/位置 (PDO 接收)
 * - 心跳监测 (离线检测)
 * - 驱动器参数配置 (编码器线数、电流限幅、加减速时间、PID 等)
 * - 抱闸控制
 * - 故障码读取与清除
 * - MSH 测试命令
 */

#include "zltech_can_motor.h"
#include <string.h>
#include <stdlib.h>

volatile rt_bool_t s_velocity_mode_ready = RT_FALSE;
volatile rt_bool_t s_brake_released = RT_FALSE;      // 刹车是否已释放
volatile rt_bool_t s_left_enabled = RT_FALSE;        // 左电机使能状态
volatile rt_bool_t s_right_enabled = RT_FALSE;       // 右电机使能状态

/* ======================== 内部数据结构 ======================== */
static rt_device_t s_can_dev = RT_NULL;
static struct rt_semaphore s_sdo_sem;
static struct rt_mutex s_tx_mutex;
static rt_thread_t s_heartbeat_thread = RT_NULL;
static rt_bool_t s_heartbeat_running = RT_FALSE;
static volatile uint32_t s_last_hb_tick = 0;
static volatile uint8_t s_node_state = 0;           /* 心跳状态 (0x00 boot, 0x04 stop, 0x05 op, 0x7F preop) */
static volatile rt_bool_t s_online = RT_FALSE;
static uint8_t s_fail_count = 0;
static uint32_t s_last_recovery_tick = 0;

/* 接收信号量与线程 */
static struct rt_semaphore s_rx_sem;
static rt_thread_t s_rx_thread = RT_NULL;
static rt_bool_t s_rx_thread_running = RT_FALSE;
/* SDO 响应缓存 */
static struct {
    uint8_t data[8];
    uint8_t len;
    rt_bool_t complete;
} s_sdo_resp;

static rt_bool_t s_right_motor_reverse = RT_TRUE;  // 默认反转右电机 
/* 当前工作模式 (用于 PDO 解析) */
static ZlacOpMode_t s_current_mode = ZLAC_MODE_UNKNOWN;
/* 当前期望的位置运动模式（默认绝对）*/
static ZlacPositionMode_t s_position_mode = ZLAC_POS_MODE_ABSOLUTE;

static rt_bool_t s_position_mode_ready = RT_FALSE;

/* 接收到的实际速度 (通过 TPDO 更新) */
static volatile int16_t s_actual_vel_left = 0;
static volatile int16_t s_actual_vel_right = 0;

/* 接收到的实际位置 (通过 TPDO 更新) */
static volatile int32_t s_actual_pos_left = 0;
static volatile int32_t s_actual_pos_right = 0;

/* 故障码解析表 */
static const struct {
    uint32_t mask;
    const char *description;
} s_fault_table[] = {
    // 共用故障（高低16位相同）
    {0x00000001, "Overvoltage"},
    {0x00000002, "Undervoltage"},
    {0x00000100, "EEPROM error"},
    // 左电机故障（高16位）
    {0x00040000, "Overcurrent"},
    {0x00080000, "Overload"},
    {0x00100000, "Current error (reserved)"},
    {0x00200000, "Encoder deviation"},
    {0x00400000, "Speed error (reserved)"},
    {0x00800000, "Reference voltage error"},
    {0x02000000, "Hall error"},
    {0x04000000, "Motor overtemperature"},
    {0x08000000, "Encoder error"},
    {0x20000000, "Speed command error"},
    // 右电机故障（低16位）
    {0x00000004, "Overcurrent"},
    {0x00000008, "Overload"},
    {0x00000010, "Current error (reserved)"},
    {0x00000020, "Encoder deviation"},
    {0x00000040, "Speed error (reserved)"},
    {0x00000080, "Reference voltage error"},
    {0x00000200, "Hall error"},
    {0x00000400, "Motor overtemperature"},
    {0x00000800, "Encoder error"},
    {0x00002000, "Speed command error"},
};


static rt_thread_t s_monitor_print_thread = RT_NULL;
static rt_bool_t s_monitor_print_run = RT_FALSE;
/* ======================== 辅助函数 ======================== */


/* 发送 CAN 帧 */
static rt_err_t can_send(uint32_t id, uint8_t *data, uint8_t len, rt_bool_t nonblocking)
{
    struct rt_can_msg msg = {0};
    msg.id = id;
    msg.ide = RT_CAN_STDID;
    msg.rtr = RT_CAN_DTR;
    msg.len = len;
    msg.nonblocking = nonblocking ? 1 : 0;
    memcpy(msg.data, data, len);

    rt_mutex_take(&s_tx_mutex, RT_WAITING_FOREVER);
    rt_size_t ret = rt_device_write(s_can_dev, 0, &msg, sizeof(msg));
    rt_mutex_release(&s_tx_mutex);
    return (ret == sizeof(msg)) ? RT_EOK : -ZLAC_ERR_CAN_TX;
}

/* 发送 NMT 命令 */
static rt_err_t nmt_send(uint8_t cs, uint8_t node_id)
{
    uint8_t data[2] = {cs, node_id};
	// 阻塞模式
    return can_send(ZLAC_COBID_NMT, data, 2,RT_FALSE);
}

/* ======================== SDO 读写 (支持任意长度，自动分段) ======================== */
/* 注意：这里简化实现，只支持 4 字节以内的读写，因为常用参数都在此范围内。实际可扩展分段传输 */
static rt_err_t sdo_request(uint16_t index, uint8_t subindex, uint8_t *data, uint8_t data_len, rt_bool_t is_write)
{
    uint8_t cmd;
    uint8_t req[8] = {0};
    if (is_write) {
        if (data_len == 1) cmd = 0x2F;
        else if (data_len == 2) cmd = 0x2B;
        else if (data_len == 3) cmd = 0x27;
        else if (data_len == 4) cmd = 0x23;
        else return -ZLAC_ERR_INVALID_PARAM;
        req[0] = cmd;
        req[1] = index & 0xFF;
        req[2] = (index >> 8) & 0xFF;
        req[3] = subindex;
        memcpy(&req[4], data, data_len);
    } else {
        cmd = 0x40;
        req[0] = cmd;
        req[1] = index & 0xFF;
        req[2] = (index >> 8) & 0xFF;
        req[3] = subindex;
        // 其余为 0
    }
    s_sdo_resp.complete = RT_FALSE;
	 // 发送请求时使用阻塞模式（等待响应）
	rt_err_t ret = can_send(ZLAC_COBID_RSDO(ZLAC_NODE_ID), req, 8, RT_FALSE);
    if (ret != RT_EOK) return ret;

    if (rt_sem_take(&s_sdo_sem, rt_tick_from_millisecond(ZLAC_SDO_TIMEOUT_MS)) != RT_EOK)
        return -ZLAC_ERR_TIMEOUT;
    if (!s_sdo_resp.complete) return -ZLAC_ERR_TIMEOUT;

    uint8_t *resp = s_sdo_resp.data;
    uint8_t resp_cmd = resp[0];
    if (resp_cmd == 0x80) {
        uint32_t abort = (resp[4] | (resp[5]<<8) | (resp[6]<<16) | (resp[7]<<24));
        rt_kprintf("[ZLAC] SDO abort: idx=0x%04X sub=%d code=0x%08X\n", index, subindex, abort);
        return -ZLAC_ERR_SDO_ABORT;
    }
    if (is_write) {
        if (resp_cmd != 0x60) return -ZLAC_ERR_SDO_ABORT;
    } else {
        uint8_t exp_cmd = (data_len == 1) ? 0x4F : ((data_len == 2) ? 0x4B : ((data_len == 3) ? 0x47 : 0x43));
        if (resp_cmd != exp_cmd) return -ZLAC_ERR_SDO_ABORT;
        memcpy(data, &resp[4], data_len);
    }
    return RT_EOK;
}
/* 内部 SDO 请求，带重试机制 */
static rt_err_t sdo_request_with_retry(uint16_t index, uint8_t subindex,
                                       uint8_t *data, uint8_t data_len,
                                       rt_bool_t is_write, uint8_t retry)
{
    rt_err_t ret;
    for (uint8_t i = 0; i < retry; i++) {
        ret = sdo_request(index, subindex, data, data_len, is_write);
        if (ret == RT_EOK) return RT_EOK;
        rt_thread_mdelay(10);
    }
    return ret;
}

/* SDO 读写接口 (供外部使用) */
static rt_err_t zlac_sdo_read(uint16_t index, uint8_t subindex, uint8_t *data, uint8_t len)
{
//    return sdo_request(index, subindex, data, len, RT_FALSE);
	return sdo_request_with_retry(index, subindex, data, len, RT_FALSE, 2);
}
static rt_err_t zlac_sdo_write(uint16_t index, uint8_t subindex, uint8_t *data, uint8_t len)
{
//    return sdo_request(index, subindex, data, len, RT_TRUE);
	return sdo_request_with_retry(index, subindex, data, len, RT_TRUE, 2);
}

/* ======================== 状态机控制 (基于控制字 6040h U16) ======================== */
static rt_err_t write_controlword(uint16_t value)
{
    uint8_t data[2] = {value & 0xFF, (value >> 8) & 0xFF};
    return zlac_sdo_write(ZLAC_OD_CONTROLWORD, 0, data, 2);
}

static rt_err_t set_controlword_with_delay(uint16_t value, uint32_t delay_ms)
{
    uint8_t data[2] = {value & 0xFF, (value >> 8) & 0xFF};
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_CONTROLWORD, 0, data, 2);
    if (ret == RT_EOK && delay_ms > 0) {
        rt_thread_mdelay(delay_ms);
    }
    return ret;
}

// bit10 为目标位置到达标志（0:未到达, 1:到达）
// bit12 为目标位置生效标志（0:未生效, 1:生效）
// 通常等待 bit10 = 1 且 bit12 = 1
/* 读取完整的状态字（32位）并返回左右部分 */
uint32_t read_full_statusword(void)
{
    uint8_t data[4];
    if (zlac_sdo_read(ZLAC_OD_STATUSWORD, 0, data, 4) == RT_EOK) {
        return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    }
    return 0;
}

static uint16_t zlac_get_left_statusword(void)
{
    uint32_t sw = read_full_statusword();
    return (uint16_t)(sw & 0xFFFF);
}

static uint16_t zlac_get_right_statusword(void)
{
    uint32_t sw = read_full_statusword();
    return (uint16_t)((sw >> 16) & 0xFFFF);
}

/* 获取左右电机的状态字位定义 (符合 CiA402) */
ZlacState_t zlac_get_state(uint16_t sw)
{
    uint8_t low = sw & 0x0F;      /* bit0-3 */
    uint8_t bit5 = (sw >> 5) & 1;
    uint8_t bit6 = (sw >> 6) & 1;

	    // 处理低4位全0的状态
    if (low == 0x00) {
        if (bit6 == 0) return ZLAC_STATE_NOT_READY;
        else return ZLAC_STATE_SWITCH_ON_DISABLED;
    }	
		
		// 使用掩码 0x6F (bit6,5,3,2,1,0) 匹配状态
		switch (sw & 0x6F) {    
			case 0x27: return ZLAC_STATE_OPERATION_ENABLED;
			case 0x23: return ZLAC_STATE_SWITCHED_ON;
			case 0x21: return ZLAC_STATE_READY_TO_SWITCH_ON;
			case 0x07: return ZLAC_STATE_QUICK_STOP_ACTIVE;		
			case 0x08:case 0x28: return ZLAC_STATE_FAULT;
			case 0x0F:case 0x2F: return ZLAC_STATE_FAULT_REACTION_ACTIVE;
			default: return ZLAC_STATE_NOT_READY;
		}
    return ZLAC_STATE_NOT_READY;
}
// 判定左电机是否到位
rt_bool_t zlac_is_left_target_reached(void)
{
    uint16_t left = zlac_get_left_statusword();
    return (left & 0x1400) == 0x1400;   // bit10 和 bit12 同时为1才认为到达
}
// 判定右电机是否到位
rt_bool_t zlac_is_right_target_reached(void)
{
    uint16_t right = zlac_get_right_statusword();
    return (right & 0x1400) == 0x1400;
}

rt_err_t zlac_control_enable(void)
{
    // 标准使能序列: Shutdown (0x06) -> Switch on (0x07) -> Enable operation (0x0F)
    rt_err_t ret;
    ret = write_controlword(0x06);      // Shutdown
    if (ret != RT_EOK) return ret;
    rt_thread_mdelay(20);               // 等待状态稳定
    ret = write_controlword(0x07);      // Switch on
    if (ret != RT_EOK) return ret;
    rt_thread_mdelay(20);
    ret = write_controlword(0x0F); 
    if (ret == RT_EOK) {
        // 启用后，可直接认为使能成功，但为了准确，可读取状态字验证
        s_left_enabled = RT_TRUE;
        s_right_enabled = RT_TRUE;
    }	// Enable operation
    return ret;
}

rt_err_t zlac_control_disable(void)
{
	    rt_err_t ret;
		ret = write_controlword(0x07);   // Disable operation
		if (ret == RT_EOK) {
			s_left_enabled = RT_FALSE;
			s_right_enabled = RT_FALSE;	
		}
    return ret;
}

rt_err_t zlac_control_quickstop(void)
{
    return write_controlword(0x02);
}

rt_err_t zlac_control_shutdown(void)
{
    return write_controlword(0x06);
}

rt_err_t zlac_control_fault_reset(void)
{
    return write_controlword(0x80);
}
/* 完全解轴：关闭功率级，电机自由（无保持力矩）*/
rt_err_t zlac_control_free(void)
{
    // 写入控制字 0x00，使驱动器进入 Switch on disabled 状态
	rt_err_t ret;
		ret = write_controlword(0x00);    
		if (ret == RT_EOK) {
			s_left_enabled = RT_FALSE;
			s_right_enabled = RT_FALSE;	
		}
    return ret;
}

/* ======================== NMT 服务 ======================== */
static rt_err_t zlac_nmt_start(void)
{
    return nmt_send(0x01, ZLAC_NODE_ID);   // 启动命令 让节点进入操作状态  节点的 PDO 通信被激活
}
static rt_err_t zlac_nmt_stop(void)
{
    return nmt_send(0x02, ZLAC_NODE_ID);	// 停止命令 让节点进入停止状态  节点的 PDO 通信被禁止
}
static rt_err_t zlac_nmt_reset_communication(void)
{
    return nmt_send(0x82, ZLAC_NODE_ID);	// 复位节点通讯（让节点进入通讯复位状态）
}
static rt_err_t zlac_nmt_reset_node(void)
{
    return nmt_send(0x81, ZLAC_NODE_ID);	// 复位节点应用层（让节点进入应用层复位状态）
}
static rt_err_t zlac_nmt_pre_operation(void)
{
    return nmt_send(0x80, ZLAC_NODE_ID);	// 预操作命令（让节点进入预操作状态）可以进行 SDO 和 NMT 通信
}

/* ======================== 工作模式设置 6060h I8 ======================== */
static rt_err_t zlac_set_op_mode(ZlacOpMode_t mode)
{
    uint8_t data = (uint8_t)mode;
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_MODE_OF_OPERATION, 0, &data, 1);
    if (ret == RT_EOK) s_current_mode = mode;
    return ret;
}
ZlacOpMode_t zlac_get_op_mode(void)
{
    uint8_t data;
    if (zlac_sdo_read(ZLAC_OD_MODE_DISPLAY, 0, &data, 1) == RT_EOK)
        return (ZlacOpMode_t)data;
    return ZLAC_MODE_UNKNOWN;
}

/* ======================== PDO 配置 (速度模式) ======================== */
/* 配置 RPDO1 映射: 目标速度 60FF:01 (左) I32 和 60FF:02 (右) I32 */
/* static rt_err_t config_rpdo1_for_velocity(void)
{
    uint8_t data[4];
    // 1. 清除原有映射 (子索引0设为0)
    data[0] = 0;
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_RPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;
    // 2. 设置映射条目1: 60FF:01 长度 32 位 (0x20)
    uint32_t map1 = 0x60FF0120;   // 索引60FF, 子索引01, 长度32位 (4字节)
    data[0] = map1 & 0xFF;
    data[1] = (map1 >> 8) & 0xFF;
    data[2] = (map1 >> 16) & 0xFF;
    data[3] = (map1 >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_MAPPING + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;
    // 3. 设置映射条目2: 60FF:02 长度 32 位
    uint32_t map2 = 0x60FF0220;
    data[0] = map2 & 0xFF;
    data[1] = (map2 >> 8) & 0xFF;
    data[2] = (map2 >> 16) & 0xFF;
    data[3] = (map2 >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_MAPPING + 1, 2, data, 4);
    if (ret != RT_EOK) return ret;
    // 4. 设置映射数量为2  启动 RPDO1 映射
    data[0] = 2;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;
    // 5. 设置 RPDO1 通信参数: COB-ID 有效 (清除最高位), 传输类型 255 (异步)
    uint32_t cob_id = ZLAC_COBID_RPDO1(ZLAC_NODE_ID) & 0x7FFFFFFF; // 清除最高位使能
    data[0] = cob_id & 0xFF;
    data[1] = (cob_id >> 8) & 0xFF;
    data[2] = (cob_id >> 16) & 0xFF;
    data[3] = (cob_id >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_COMM_PARAM + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;
    data[0] = 255; // 传输类型
    ret = zlac_sdo_write(ZLAC_OD_RPDO_COMM_PARAM + 1, 2, data, 1);
    return ret;
} */

/* 配置 TPDO1 映射: 实际速度 606C:01 (左) 和 606C:02 (右) */
/* static rt_err_t config_tpdo1_for_velocity(void)
{
    uint8_t data[4];
    // 清除原有映射
    data[0] = 0;
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;
    // 映射条目1: 606C:01 长度 32 位
    uint32_t map1 = 0x606C0120;
    data[0] = map1 & 0xFF;
    data[1] = (map1 >> 8) & 0xFF;
    data[2] = (map1 >> 16) & 0xFF;
    data[3] = (map1 >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;
    // 映射条目2: 606C:02 长度 32 位
    uint32_t map2 = 0x606C0220;
    data[0] = map2 & 0xFF;
    data[1] = (map2 >> 8) & 0xFF;
    data[2] = (map2 >> 16) & 0xFF;
    data[3] = (map2 >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 1, 2, data, 4);
    if (ret != RT_EOK) return ret;
    // 映射数量为2
    data[0] = 2;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;
    // 设置 TPDO1 通信参数: COB-ID 有效, 传输类型 255 (异步), 禁止时间 50 (5ms)
    uint32_t cob_id = ZLAC_COBID_TPDO1(ZLAC_NODE_ID) & 0x7FFFFFFF;
    data[0] = cob_id & 0xFF;
    data[1] = (cob_id >> 8) & 0xFF;
    data[2] = (cob_id >> 16) & 0xFF;
    data[3] = (cob_id >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;
    data[0] = 255; // 异步
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 1, 2, data, 1);
    if (ret != RT_EOK) return ret;
    // 禁止时间 50 (单位100us => 5ms)
    data[0] = 50 & 0xFF;
    data[1] = (50 >> 8) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 1, 3, data, 2);
    return ret;
} */
/* 配置 TPDO1 映射 1600h U32 1400h U32/U8/U16: 目标速度 60FF:03 U32=I16+I16 同步模式 同时设置左右电机速度 */
static rt_err_t config_rpdo1_for_velocity(void)
{
    uint8_t data[4];
    // 1. 清除原有映射 (子索引0设为0)
    data[0] = 0;
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_RPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;
    // 2. 设置映射条目1: 60FF:03 (组合速度) 32 位
    uint32_t map1 = 0x60FF0320;   // 索引60FF, 子索引03, 长度32位 (4字节)
    data[0] = map1 & 0xFF;
    data[1] = (map1 >> 8) & 0xFF;
    data[2] = (map1 >> 16) & 0xFF;
    data[3] = (map1 >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_MAPPING + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;

    // 3. 设置 RPDO1 通信参数: COB-ID 有效 (清除最高位), 传输类型 255 (异步)
    uint32_t cob_id = ZLAC_COBID_RPDO1(ZLAC_NODE_ID) & 0x7FFFFFFF; // 清除最高位使能
    data[0] = cob_id & 0xFF;
    data[1] = (cob_id >> 8) & 0xFF;
    data[2] = (cob_id >> 16) & 0xFF;
    data[3] = (cob_id >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_COMM_PARAM + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;
    data[0] = 254; // 传输类型 254事件 255事件+定时
    ret = zlac_sdo_write(ZLAC_OD_RPDO_COMM_PARAM + 1, 2, data, 1);
    if (ret != RT_EOK) return ret;	
	    // 禁止时间 50 (5ms)
    // uint16_t inhibit = 50;
    // data[0] = inhibit & 0xFF;
    // data[1] = (inhibit >> 8) & 0xFF;
    // ret = zlac_sdo_write(ZLAC_OD_RPDO_COMM_PARAM + 1, 3, data, 2);
    // if (ret != RT_EOK) return ret;
    // 事件计时器 500ms (单位 0.5ms，500ms = 1000)
    // uint16_t timer = 1000;
    // data[0] = timer & 0xFF;
    // data[1] = (timer >> 8) & 0xFF;
    // ret = zlac_sdo_write(ZLAC_OD_RPDO_COMM_PARAM + 1, 5, data, 2);
	
	// 4. 设置映射数量为1  启动 RPDO1 映射
    data[0] = 1;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;
    return ret;
}
/* 配置 TPDO1 映射1A00 U32 1800 U32/U8/U16: 实际速度 606C:03 U32=I16+I16 同步模式 同时获取左右电机速度 */
static rt_err_t config_tpdo1_for_velocity(void)
{
    uint8_t data[4];
    // 清除原有映射
    data[0] = 0;
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;
    // 映射 606C:03 (组合速度) 32 位
    uint32_t map = 0x606C0320;
    data[0] = map & 0xFF;
    data[1] = (map >> 8) & 0xFF;
    data[2] = (map >> 16) & 0xFF;
    data[3] = (map >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;

    // 设置 TPDO1 通信参数
    uint32_t cob_id = ZLAC_COBID_TPDO1(ZLAC_NODE_ID) & 0x7FFFFFFF;
    data[0] = cob_id & 0xFF;
    data[1] = (cob_id >> 8) & 0xFF;
    data[2] = (cob_id >> 16) & 0xFF;
    data[3] = (cob_id >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;
    // 传输类型 255 (事件触发+定时器)
    data[0] = 255;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 1, 2, data, 1);
    if (ret != RT_EOK) return ret;
    // 禁止时间 50 (5ms)
    uint16_t inhibit = 50;
    data[0] = inhibit & 0xFF;
    data[1] = (inhibit >> 8) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 1, 3, data, 2);
    if (ret != RT_EOK) return ret;
    // 事件计时器 500ms (单位 0.5ms，500ms = 1000)
    uint16_t timer = 1000;
    data[0] = timer & 0xFF;
    data[1] = (timer >> 8) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 1, 5, data, 2);
	
	// 映射数量为 1 开启映射
    data[0] = 1;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;
	
    return ret;
}

static rt_err_t zlac_config_pdo_for_velocity_mode(void)
{
    rt_err_t ret;
    // 1. 发送 NMT 命令，进入预操作状态 (0x80)
    ret = nmt_send(0x80, ZLAC_NODE_ID);
    if (ret != RT_EOK) return ret;
    rt_thread_mdelay(50);  // 等待状态切换

    // 2. 配置 PDO 映射
//	  uint8_t data[2] = {1, 0};
//    ret = zlac_sdo_write(ZLAC_OD_SYNC_ASYNC, 0, data, 2);
//		if (ret != RT_EOK) return ret;	
    ret = config_rpdo1_for_velocity();
    if (ret != RT_EOK) return ret;
    ret = config_tpdo1_for_velocity();
    if (ret != RT_EOK) return ret;

    // 3. 重新启动节点，进入操作状态
    ret = nmt_send(0x01, ZLAC_NODE_ID);
    if (ret != RT_EOK) return ret;
    rt_thread_mdelay(50);
    return RT_EOK;
}

/* 速度模式参数配置（加减速时间，单位 ms）*/
static rt_err_t zlac_config_velocity_mode_params(uint32_t accel_ms, uint32_t decel_ms)
{
    uint8_t data[4];
    rt_err_t ret;

    // 1. 设置同步控制标志 (200Fh = 1)  可选，默认同步模式
	  uint8_t sync[2] = {1, 0};
    ret = zlac_sdo_write(ZLAC_OD_SYNC_ASYNC, 0, sync, 2);
		if (ret != RT_EOK) return ret;	

    // 2. 设置工作模式为速度模式 (0x6060 = 3)
    uint8_t mode = 3;
//    ret = zlac_sdo_write(ZLAC_OD_MODE_OF_OPERATION, 0, &mode, 1);
		ret = zlac_set_op_mode((ZlacOpMode_t)mode);
    if (ret != RT_EOK) return ret;
		ZlacOpMode_t op = zlac_get_op_mode();
		rt_kprintf("op mode = %d\n",op);
		
    // 3. 设置 S 形加速时间 (0x6083:01/02)
//	if(accel_ms != ZLAC_MOTOR_V_ACCEL_TIME_MS){
		data[0] = accel_ms & 0xFF;
		data[1] = (accel_ms >> 8) & 0xFF;
		data[2] = (accel_ms >> 16) & 0xFF;
		data[3] = (accel_ms >> 24) & 0xFF;
		ret = zlac_sdo_write(ZLAC_OD_PROFILE_ACCEL, 1, data, 4);
		if (ret != RT_EOK) return ret;
		ret = zlac_sdo_write(ZLAC_OD_PROFILE_ACCEL, 2, data, 4);
		if (ret != RT_EOK) return ret;	
//	}
    // 4. 设置 S 形减速时间 (0x6084:01/02)
//	if(decel_ms != ZLAC_MOTOR_V_DECEL_TIME_MS){		
		data[0] = decel_ms & 0xFF;
		data[1] = (decel_ms >> 8) & 0xFF;
		data[2] = (decel_ms >> 16) & 0xFF;
		data[3] = (decel_ms >> 24) & 0xFF;
		ret = zlac_sdo_write(ZLAC_OD_PROFILE_DECEL, 1, data, 4);
		if (ret != RT_EOK) return ret;
		ret = zlac_sdo_write(ZLAC_OD_PROFILE_DECEL, 2, data, 4);
		if (ret != RT_EOK) return ret;
//	}
    return RT_EOK;
}

/* 速度模式完整初始化（包括 PDO 配置、参数设置、使能）*/
rt_err_t zlac_init_velocity_mode(void)
{
    rt_err_t ret;
    // 1. 配置速度模式 PDO（内部会进入预操作状态、配置映射、然后启动节点）
    ret = zlac_config_pdo_for_velocity_mode();
    if (ret != RT_EOK) return ret;
		rt_kprintf("zlac_config_pdo_for_velocity_mode ok!\n");

    // 2. 设置速度模式参数（同步标志、工作模式、加减速时间）
    ret = zlac_config_velocity_mode_params(ZLAC_MOTOR_V_ACCEL_TIME_MS, ZLAC_MOTOR_V_DECEL_TIME_MS);
    if (ret != RT_EOK) return ret;
		rt_kprintf("zlac_config_velocity_mode_params ok!\n");

    // 3. 使能电机（标准序列）
    ret = zlac_control_enable();
    if (ret != RT_EOK) return ret;
		rt_kprintf("zlac_control_enable ok!\n");
	
		if (ret == RT_EOK) {
        s_velocity_mode_ready = RT_TRUE;
    }
    return RT_EOK;
}


/* ======================== PDO 配置 (位置模式) ======================== */
/* 配置 RPDO1 映射: 目标位置 607A:01 (左) I32和 607A:02 (右)I32 均为 32 位 */
static rt_err_t config_rpdo1_for_position(void)
{
    uint8_t data[4];
    data[0] = 0;
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_RPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;
    uint32_t map1 = 0x607A0120;   // 32 位
    data[0] = map1 & 0xFF;
    data[1] = (map1 >> 8) & 0xFF;
    data[2] = (map1 >> 16) & 0xFF;
    data[3] = (map1 >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_MAPPING + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;
    uint32_t map2 = 0x607A0220;
    data[0] = map2 & 0xFF;
    data[1] = (map2 >> 8) & 0xFF;
    data[2] = (map2 >> 16) & 0xFF;
    data[3] = (map2 >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_MAPPING + 1, 2, data, 4);
    if (ret != RT_EOK) return ret;

    // 设置 COB-ID 和传输类型
    uint32_t cob_id = ZLAC_COBID_RPDO1(ZLAC_NODE_ID) & 0x7FFFFFFF;
    data[0] = cob_id & 0xFF;
    data[1] = (cob_id >> 8) & 0xFF;
    data[2] = (cob_id >> 16) & 0xFF;
    data[3] = (cob_id >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_COMM_PARAM + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;
    data[0] = 254;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_COMM_PARAM + 1, 2, data, 1);
	if (ret != RT_EOK) return ret;
		
    data[0] = 2;
    ret = zlac_sdo_write(ZLAC_OD_RPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;		
    return ret;
}

/* 配置 TPDO1 映射: 实际位置 6064:01 (左) I32和 6064:02 (右) I32 位 */
// 废弃 位置映射到TPDO1 ,需要同时显示实时速度和实际位置 需要配置2个TPDO
static rt_err_t config_tpdo1_for_position(void)
{
    uint8_t data[4];
    data[0] = 0;
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;
    uint32_t map1 = 0x60640120;
    data[0] = map1 & 0xFF;
    data[1] = (map1 >> 8) & 0xFF;
    data[2] = (map1 >> 16) & 0xFF;
    data[3] = (map1 >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;
    uint32_t map2 = 0x60640220;
    data[0] = map2 & 0xFF;
    data[1] = (map2 >> 8) & 0xFF;
    data[2] = (map2 >> 16) & 0xFF;
    data[3] = (map2 >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 1, 2, data, 4);
    if (ret != RT_EOK) return ret;

    uint32_t cob_id = ZLAC_COBID_TPDO1(ZLAC_NODE_ID) & 0x7FFFFFFF;
    data[0] = cob_id & 0xFF;
    data[1] = (cob_id >> 8) & 0xFF;
    data[2] = (cob_id >> 16) & 0xFF;
    data[3] = (cob_id >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 1, 1, data, 4);
    if (ret != RT_EOK) return ret;
    data[0] = 255;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 1, 2, data, 1);
    if (ret != RT_EOK) return ret;
    data[0] = 50; data[1]=0;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 1, 3, data, 2);
    if (ret != RT_EOK) return ret;
    // 事件计时器 500ms (单位 0.5ms，500ms = 1000)
    uint16_t timer = 1000;
    data[0] = timer & 0xFF;
    data[1] = (timer >> 8) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 1, 5, data, 2);
	
	data[0] = 2;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 1, 0, data, 1);
    if (ret != RT_EOK) return ret;
    return ret;
}
/* 配置 TPDO2 映射: 实际位置 6064:01 (左) 和 6064:02 (右) 32 位 */
static rt_err_t config_tpdo2_for_position(void)
{
    uint8_t data[4];
    // 清除原有映射
    data[0] = 0;
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 2, 0, data, 1); // 索引 1A02h
    if (ret != RT_EOK) return ret;
    uint32_t map1 = 0x60640120;
    data[0] = map1 & 0xFF;
    data[1] = (map1 >> 8) & 0xFF;
    data[2] = (map1 >> 16) & 0xFF;
    data[3] = (map1 >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 2, 1, data, 4);
    if (ret != RT_EOK) return ret;
    uint32_t map2 = 0x60640220;
    data[0] = map2 & 0xFF;
    data[1] = (map2 >> 8) & 0xFF;
    data[2] = (map2 >> 16) & 0xFF;
    data[3] = (map2 >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 2, 2, data, 4);
    if (ret != RT_EOK) return ret;

    // 设置 TPDO2 通信参数 (1802h)
    uint32_t cob_id = ZLAC_COBID_TPDO2(ZLAC_NODE_ID) & 0x7FFFFFFF; // 0x381
    data[0] = cob_id & 0xFF;
    data[1] = (cob_id >> 8) & 0xFF;
    data[2] = (cob_id >> 16) & 0xFF;
    data[3] = (cob_id >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 2, 1, data, 4); // 1802:01
    if (ret != RT_EOK) return ret;
    data[0] = 255; // 传输类型 255
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 2, 2, data, 1);
    if (ret != RT_EOK) return ret;
    data[0] = 50; data[1] = 0; // 禁止时间 5ms
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 2, 3, data, 2);
    if (ret != RT_EOK) return ret;
    uint16_t timer = 1000; // 事件计时器 500ms
    data[0] = timer & 0xFF;
    data[1] = (timer >> 8) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_COMM_PARAM + 2, 5, data, 2);
		
		// 映射数量为2 并使能
    data[0] = 2;
    ret = zlac_sdo_write(ZLAC_OD_TPDO_MAPPING + 2, 0, data, 1);
    if (ret != RT_EOK) return ret;
    return ret;
}

static rt_err_t zlac_config_pdo_for_position_mode(void)
{
	// 绝对运动，适合已知全局坐标系的应用，用于轨迹规划
	// 相对运动适合增量式移动 
	    rt_err_t ret;
    // 1. 发送 NMT 命令，进入预操作状态 (0x80)
    ret = nmt_send(0x80, ZLAC_NODE_ID);
    if (ret != RT_EOK) return ret;
    rt_thread_mdelay(50);  // 等待状态切换
	
    ret = config_rpdo1_for_position();
    if (ret != RT_EOK) return ret;
//    ret = config_tpdo1_for_position();
	 ret = config_tpdo2_for_position();
    if (ret != RT_EOK) return ret;

    // 3. 重新启动节点，进入操作状态
    ret = nmt_send(0x01, ZLAC_NODE_ID);
    if (ret != RT_EOK) return ret;
    rt_thread_mdelay(50);
	
    return ret;
}

/* 位置模式参数配置 */
static rt_err_t zlac_config_position_mode_params(uint32_t accel_ms, uint32_t decel_ms, uint16_t max_speed_rpm)
{
    uint8_t data[4];
    rt_err_t ret;

    // 1. 设置工作模式为位置模式 (0x6060 = 1)
    uint8_t mode = 1;
//    ret = zlac_sdo_write(ZLAC_OD_MODE_OF_OPERATION, 0, &mode, 1);
			ret = zlac_set_op_mode((ZlacOpMode_t)mode);
    if (ret != RT_EOK) return ret;
		ZlacOpMode_t op = zlac_get_op_mode();
		rt_kprintf("op mode = %d\n",op);

    // 2. 设置 S 形加速时间 (0x6083:01/02)
//	if(accel_ms != ZLAC_MOTOR_ACCEL_TIME_MS){
    data[0] = accel_ms & 0xFF;
    data[1] = (accel_ms >> 8) & 0xFF;
    data[2] = (accel_ms >> 16) & 0xFF;
    data[3] = (accel_ms >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_PROFILE_ACCEL, 1, data, 4);  	// 左电机
    if (ret != RT_EOK) return ret;
    ret = zlac_sdo_write(ZLAC_OD_PROFILE_ACCEL, 2, data, 4);	// 右电机
    if (ret != RT_EOK) return ret;
//	}
    // 3. 设置 S 形减速时间 (0x6084:01/02)
//	if(decel_ms != ZLAC_MOTOR_DECEL_TIME_MS){
    data[0] = decel_ms & 0xFF;
    data[1] = (decel_ms >> 8) & 0xFF;
    data[2] = (decel_ms >> 16) & 0xFF;
    data[3] = (decel_ms >> 24) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_PROFILE_DECEL, 1, data, 4);
    if (ret != RT_EOK) return ret;
    ret = zlac_sdo_write(ZLAC_OD_PROFILE_DECEL, 2, data, 4);
    if (ret != RT_EOK) return ret;
//	}
	// 4. 设置最大速度 (0x6081:01/02) 单位 rpm  最大速度设置为室内速度
    data[0] = max_speed_rpm & 0xFF;
    data[1] = (max_speed_rpm >> 8) & 0xFF;
    data[2] = 0;
    data[3] = 0;
    ret = zlac_sdo_write(ZLAC_OD_PROFILE_VELOCITY, 1, data, 4);
    if (ret != RT_EOK) return ret;
    ret = zlac_sdo_write(ZLAC_OD_PROFILE_VELOCITY, 2, data, 4);
    if (ret != RT_EOK) return ret;
	
    return RT_EOK;
}

/* 位置模式初始化（包括 PDO 配置、参数设置、使能）*/
rt_err_t zlac_init_position_mode(ZlacPositionMode_t mode)
{
    rt_err_t ret;
	
		// 选择相对位置模式或者绝对位置模式
    s_position_mode = mode;
	
    // 1. 配置位置模式 PDO（已在 zlac_config_pdo_for_position_mode 中实现）
    ret = zlac_config_pdo_for_position_mode();
    if (ret != RT_EOK) return ret;

    // 2. 设置位置模式参数
    ret = zlac_config_position_mode_params(ZLAC_MOTOR_ACCEL_TIME_MS, ZLAC_MOTOR_DECEL_TIME_MS, ZLAC_MOTOR_NORMAL_RPM);
    if (ret != RT_EOK) return ret;

    // 3. 使能电机（标准序列）
    ret = zlac_control_enable();
    if (ret != RT_EOK) return ret;

		if (ret == RT_EOK) {
        s_position_mode_ready = RT_TRUE;
    }
    return RT_EOK;
}

/* 刷新状态缓存（由 monitor 线程定期调用）*/
void zlac_refresh_status_cache(void)
{
    // 仅在速度模式或位置模式已初始化时才刷新，避免不必要的 SDO
    if (!s_velocity_mode_ready && !s_position_mode_ready) {
        return;
    }
    // 读取抱闸状态
    uint16_t left_brake, right_brake;
    if (zlac_get_brake(&left_brake, &right_brake) == RT_EOK) {
        s_brake_released = (left_brake == 0 && right_brake == 0);  // 0 表示释放
    }
    // 读取左右电机状态字并判断使能状态
    uint16_t left_sw = zlac_get_left_statusword();
    uint16_t right_sw = zlac_get_right_statusword();
    ZlacState_t left_state = zlac_get_state(left_sw);
    ZlacState_t right_state = zlac_get_state(right_sw);
    s_left_enabled = (left_state == ZLAC_STATE_OPERATION_ENABLED);
    s_right_enabled = (right_state == ZLAC_STATE_OPERATION_ENABLED);
}

/* ======================== 速度控制 (PDO 发送) U32=I16+I16 带方向======================== */
void zlac_set_right_motor_reverse(rt_bool_t reverse)
{
    s_right_motor_reverse = reverse;
}
// 支持速度模式需要高频指令（如 20ms 周期）实现闭环调速
rt_err_t zlac_set_velocity(int16_t left_rpm, int16_t right_rpm)
{
		if (!s_online) {
				return -ZLAC_ERR_OFFLINE;
		}
			// 模式未就绪
	  if (!s_velocity_mode_ready) {
        return -ZLAC_ERR_MODE_UNREADY;  // 或自定义错误码
    }
	// 检测刹车是否释放    耗时操作，用标志位替代
//	    rt_err_t ret;
//				uint16_t left,right;
//			ret = zlac_get_brake(&left,&right);
//   if (ret != RT_EOK) return ret;
//		if(left || right) return ZLAC_ERR_BRAKE_UNRELEASE;

		
	 // 检查驱动器是否处于使能状态（状态字低4位 = 0x07 且 bit5=1？实际标准为 Operation enabled = 0x27）耗时操作
//    if (zlac_get_state(zlac_get_left_statusword()) != ZLAC_STATE_OPERATION_ENABLED) {
//        return ZLAC_ERR_INVALID_STATE;
//    }
//		if (zlac_get_state(zlac_get_right_statusword()) != ZLAC_STATE_OPERATION_ENABLED) {
//        return ZLAC_ERR_INVALID_STATE;
//    }
		
    // 仅检查缓存标志，不调用任何 SDO 函数
    if (!s_brake_released) return ZLAC_ERR_BRAKE_UNRELEASE;
    if (!s_left_enabled || !s_right_enabled) return ZLAC_ERR_INVALID_STATE;
		
		if(left_rpm > ZLAC_MOTOR_MAX_RPM)
			left_rpm = ZLAC_MOTOR_MAX_RPM;
		else if(left_rpm < -ZLAC_MOTOR_MAX_RPM)
			left_rpm = -ZLAC_MOTOR_MAX_RPM;
		if(right_rpm > ZLAC_MOTOR_MAX_RPM)
			right_rpm = ZLAC_MOTOR_MAX_RPM;		
		else if(right_rpm < -ZLAC_MOTOR_MAX_RPM)
			right_rpm = -ZLAC_MOTOR_MAX_RPM;
		
	// 小车前进，需要左轮逆时针，右轮顺时针运动
		if (s_right_motor_reverse) {
        right_rpm = -right_rpm;
    }
	// PDO的方式是为了速度控制及时性

    uint8_t data[4];
    data[0] = left_rpm & 0xFF;
    data[1] = (left_rpm >> 8) & 0xFF;
    data[2] = right_rpm & 0xFF;
    data[3] = (right_rpm >> 8) & 0xFF;
	//使用非阻塞发送
    return can_send(ZLAC_COBID_RPDO1(ZLAC_NODE_ID), data, 4, RT_TRUE);
}

rt_err_t zlac_get_velocity(int16_t *left_rpm, int16_t *right_rpm)
{
    if (left_rpm) *left_rpm = s_actual_vel_left;
    if (right_rpm) *right_rpm = s_actual_vel_right;
    return RT_EOK;
}

rt_err_t zlac_get_velocity_by_sdo(int16_t *left_rpm, int16_t *right_rpm)
{
	rt_err_t ret;
	uint8_t data[4];
	ret = zlac_sdo_read(ZLAC_OD_ACTUAL_VELOCITY, 3, data, 4) ;
	if(ret != RT_EOK) return ret;	
	int32_t combined = data[0] | (data[1]<<8) | (data[2]<<16) | (data[3]<<24);
	*left_rpm = (int16_t)(combined & 0xFFFF);
	*right_rpm = (int16_t)((combined >> 16) & 0xFFFF);
	return RT_EOK;
}

/* ======================== 位置控制 (PDO 发送) I32 带正反 ======================== */

/* 绝对位置运动：目标位置为相对于原点的绝对脉冲数 I32 */
static rt_err_t zlac_set_position_abs(int32_t left_pulses, int32_t right_pulses)
{
		    rt_err_t ret;
				uint16_t left,right;
			ret = zlac_get_brake(&left,&right);
   if (ret != RT_EOK) return ret;
		if(left || right) return ZLAC_ERR_BRAKE_UNRELEASE;
	
		if (!s_online) {
				return -ZLAC_ERR_OFFLINE;
		}

	// 应用右电机方向反转（与速度模式保持一致）
    if (s_right_motor_reverse) {
        right_pulses = -right_pulses;
    }

			// 模式未就绪
	  if (!s_position_mode_ready) {
        return -ZLAC_ERR_MODE_UNREADY;  // 或自定义错误码
    }

	 // 检查驱动器是否处于使能状态（状态字低4位 = 0x07 且 bit5=1？实际标准为 Operation enabled = 0x27）
    if (zlac_get_state(zlac_get_left_statusword()) != ZLAC_STATE_OPERATION_ENABLED) {
        return ZLAC_ERR_INVALID_STATE;
    }
		if (zlac_get_state(zlac_get_right_statusword()) != ZLAC_STATE_OPERATION_ENABLED) {
        return ZLAC_ERR_INVALID_STATE;
    }
		
    uint8_t data[8];

    // 1. 通过 PDO 发送目标位置（RPDO1 已映射 607A:01 和 607A:02）
    data[0] = left_pulses & 0xFF;
    data[1] = (left_pulses >> 8) & 0xFF;
    data[2] = (left_pulses >> 16) & 0xFF;
    data[3] = (left_pulses >> 24) & 0xFF;
    data[4] = right_pulses & 0xFF;
    data[5] = (right_pulses >> 8) & 0xFF;
    data[6] = (right_pulses >> 16) & 0xFF;
    data[7] = (right_pulses >> 24) & 0xFF;
    ret = can_send(ZLAC_COBID_RPDO1(ZLAC_NODE_ID), data, 8, RT_TRUE);
    if (ret != RT_EOK) return ret;

    // 2. 确保控制字处于使能且绝对运动模式（bit4=0）
    // 如果当前控制字不是 0x0F，则设置；否则无需重复设置（但某些驱动器需要重新触发）
    // 2. 先写 0x0F（绝对模式使能）
    ret = set_controlword_with_delay(0x0F, 10);
    if (ret != RT_EOK) return ret;
    // 3. 再写 0x1F 启动绝对运动
    ret = set_controlword_with_delay(0x1F, 0);
    return ret;
}

/* 相对位置运动：目标位置为相对于当前位置的增量脉冲数 I32*/
static rt_err_t zlac_set_position_rel(int32_t left_delta, int32_t right_delta)
{
		    rt_err_t ret;
				uint16_t left,right;
			ret = zlac_get_brake(&left,&right);
   if (ret != RT_EOK) return ret;
		if(left || right) return ZLAC_ERR_BRAKE_UNRELEASE;

		if (!s_online) {
				return -ZLAC_ERR_OFFLINE;
		}
	
    // 应用右电机方向反转
    if (s_right_motor_reverse) {
        right_delta = -right_delta;
    }

					// 模式未就绪
	  if (!s_position_mode_ready) {
        return -ZLAC_ERR_MODE_UNREADY;  // 或自定义错误码
    }

	 // 检查驱动器是否处于使能状态（状态字低4位 = 0x07 且 bit5=1？实际标准为 Operation enabled = 0x27）
    if (zlac_get_state(zlac_get_left_statusword()) != ZLAC_STATE_OPERATION_ENABLED) {
        return ZLAC_ERR_INVALID_STATE;
    }
		if (zlac_get_state(zlac_get_right_statusword()) != ZLAC_STATE_OPERATION_ENABLED) {
        return ZLAC_ERR_INVALID_STATE;
    }		
		
		uint8_t data[8];

    // 1. 发送增量位置（RPDO1 同样使用 607A:01/02，但控制字 bit4=1 表示相对）
    data[0] = left_delta & 0xFF;
    data[1] = (left_delta >> 8) & 0xFF;
    data[2] = (left_delta >> 16) & 0xFF;
    data[3] = (left_delta >> 24) & 0xFF;
    data[4] = right_delta & 0xFF;
    data[5] = (right_delta >> 8) & 0xFF;
    data[6] = (right_delta >> 16) & 0xFF;
    data[7] = (right_delta >> 24) & 0xFF;
    ret = can_send(ZLAC_COBID_RPDO1(ZLAC_NODE_ID), data, 8, RT_TRUE);
    if (ret != RT_EOK) return ret;

    // 2. 先写 0x4F（相对模式使能）
    ret = set_controlword_with_delay(0x4F, 10);
    if (ret != RT_EOK) return ret;
    // 3. 再写 0x5F 启动相对运动
    ret = set_controlword_with_delay(0x5F, 0);
    return ret;

//    return RT_EOK;
}

rt_err_t zlac_set_position(int32_t left_pulses, int32_t right_pulses, rt_bool_t relative)
{
	if(relative){
		return zlac_set_position_rel(left_pulses,right_pulses);
	}else{
		return zlac_set_position_abs(left_pulses,right_pulses);
	}
}
rt_err_t zlac_set_position_by_mode(int32_t left_pulses, int32_t right_pulses)
{		
    if (s_position_mode == ZLAC_POS_MODE_ABSOLUTE) {
        return zlac_set_position_abs(left_pulses, right_pulses);
    } else {
        return zlac_set_position_rel(left_pulses, right_pulses);
    }
}

rt_err_t zlac_get_position(int32_t *left_pulses, int32_t *right_pulses)
{
    if (left_pulses) *left_pulses = s_actual_pos_left;
    if (right_pulses) *right_pulses = s_actual_pos_right;
    return RT_EOK;
}

static void zlac_set_position_mode(ZlacPositionMode_t mode)
{
    s_position_mode = mode;
}

ZlacPositionMode_t zlac_get_position_mode(void)
{
    return s_position_mode;
}

/* 等待左右电机运动到位，超时 ms，返回 0 表示都到位，否则超时 注意：此方法为阻塞式*/
rt_err_t zlac_wait_position_done(uint32_t timeout_ms)
{
    uint32_t start = rt_tick_get_millisecond();
    while (1) {
        if (zlac_is_left_target_reached() && zlac_is_right_target_reached())
            return RT_EOK;
        if (rt_tick_get_millisecond() - start > timeout_ms)
            return -ZLAC_ERR_TIMEOUT;
        rt_thread_mdelay(10);
    }
}

/* ======================== 抱闸控制 ======================== */
// 断电默认抱闸锁紧状态。通电后默认抱闸开启1 释放 电机可自由运动
rt_err_t zlac_brake_release(void)
{
    uint8_t data[2] = {0, 0};  // 0: 开启抱闸 (释放)
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_BRAKE_CONFIG, 7, data, 2);
    if (ret == RT_EOK) ret = zlac_sdo_write(ZLAC_OD_BRAKE_CONFIG, 8, data, 2);
    if (ret == RT_EOK) {
        s_brake_released = RT_TRUE;
    }		
    return ret;
}
rt_err_t zlac_brake_engage(void)   
{
    uint8_t data[2] = {1, 0};  // 1: 关闭抱闸 (锁紧)
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_BRAKE_CONFIG, 7, data, 2);
    if (ret == RT_EOK) ret = zlac_sdo_write(ZLAC_OD_BRAKE_CONFIG, 8, data, 2);
	// 注意： 电机开启刹车，表示已经工作结束						
		if(ret == RT_EOK){
			s_brake_released = RT_FALSE;
			s_velocity_mode_ready = RT_FALSE;
			s_position_mode_ready = RT_FALSE;
		}
    return ret;
}

rt_err_t zlac_get_brake(uint16_t *left_val, uint16_t *right_val)   
{
    uint8_t data[2];
    rt_err_t ret = zlac_sdo_read(ZLAC_OD_BRAKE_CONFIG, 7, data, 2);
    if (ret == RT_EOK && left_val) *left_val = data[0] | (data[1] << 8);
    if (ret != RT_EOK) return ret;
    ret = zlac_sdo_read(ZLAC_OD_BRAKE_CONFIG, 8, data, 2);
    if (ret == RT_EOK && right_val) *right_val = data[0] | (data[1] << 8);
    return ret;	
}


/* ======================== 参数配置 ======================== */
//左右参数 U16
static rt_err_t set_left_right_param(uint16_t index, uint16_t left_val, uint16_t right_val)
{
    uint8_t data[2];
    data[0] = left_val & 0xFF;
    data[1] = (left_val >> 8) & 0xFF;
    rt_err_t ret = zlac_sdo_write(index, 1, data, 2);
    if (ret != RT_EOK) return ret;
    data[0] = right_val & 0xFF;
    data[1] = (right_val >> 8) & 0xFF;
    return zlac_sdo_write(index, 2, data, 2);
}

static rt_err_t get_left_right_param(uint16_t index, uint16_t *left_val, uint16_t *right_val)
{
    uint8_t data[2];
    rt_err_t ret = zlac_sdo_read(index, 1, data, 2);
    if (ret == RT_EOK && left_val) *left_val = data[0] | (data[1] << 8);
    if (ret != RT_EOK) return ret;
    ret = zlac_sdo_read(index, 2, data, 2);
    if (ret == RT_EOK && right_val) *right_val = data[0] | (data[1] << 8);
    return ret;
}
// 左右参数 U32
static rt_err_t set_left_right_param32(uint16_t index, uint32_t left_val, uint32_t right_val)
{
    uint8_t data[4];
    data[0] = left_val & 0xFF;
    data[1] = (left_val >> 8) & 0xFF;
    data[2] = (left_val >> 16) & 0xFF;
    data[3] = (left_val >> 24) & 0xFF;
    rt_err_t ret = zlac_sdo_write(index, 1, data, 4);
    if (ret != RT_EOK) return ret;
    data[0] = right_val & 0xFF;
    data[1] = (right_val >> 8) & 0xFF;
    data[2] = (right_val >> 16) & 0xFF;
    data[3] = (right_val >> 24) & 0xFF;
    return zlac_sdo_write(index, 2, data, 4);
}

static rt_err_t get_left_right_param32(uint16_t index, uint32_t *left_val, uint32_t *right_val)
{
    uint8_t data[4];
    rt_err_t ret = zlac_sdo_read(index, 1, data, 4);
    if (ret == RT_EOK && left_val) *left_val = data[0] | (data[1]<<8) | (data[2]<<16) | (data[3]<<24);
    if (ret != RT_EOK) return ret;
    ret = zlac_sdo_read(index, 2, data, 4);
    if (ret == RT_EOK && right_val) *right_val = data[0] | (data[1]<<8) | (data[2]<<16) | (data[3]<<24);
    return ret;
}
/*--------------- CiA 402参数组  ---------------*/
// 设置S形加速时间   6083h:01/02 U32 0-32767ms
static rt_err_t zlac_set_accel_time(uint32_t left_ms, uint32_t right_ms)
{
    return set_left_right_param32(ZLAC_OD_PROFILE_ACCEL, left_ms, right_ms);
}
rt_err_t zlac_get_accel_time(uint32_t *left_ms, uint32_t *right_ms)
{
    return get_left_right_param32(ZLAC_OD_PROFILE_ACCEL, left_ms, right_ms);
}

// 设置S形减速时间 6084h:01/02 U32 0-32767ms
static rt_err_t zlac_set_decel_time(uint32_t left_ms, uint32_t right_ms)
{
    return set_left_right_param32(ZLAC_OD_PROFILE_DECEL, left_ms, right_ms);	
}
rt_err_t zlac_get_decel_time(uint32_t *left_ms, uint32_t *right_ms)
{
    return get_left_right_param32(ZLAC_OD_PROFILE_DECEL, left_ms, right_ms);
}
// 设置急速减速时间 6085h:01/02 U32 0-32767ms
static rt_err_t zlac_set_quick_stop_time(uint32_t left_ms, uint32_t right_ms)
{
    return set_left_right_param32(ZLAC_OD_QUICKSTOP_DECEL, left_ms, right_ms);	
}
static rt_err_t zlac_get_quick_stop_time(uint32_t *left_ms, uint32_t *right_ms)
{
    return get_left_right_param32(ZLAC_OD_QUICKSTOP_DECEL, left_ms, right_ms);
}

/*----------------- 厂家自定义参数 -----------------------*/

//绝对位置模式，2006h:01/02/03 U16需要设置原点--起始位置 3：设置同步原点 绝对位置模式时用于清除当前位置
rt_err_t zlac_set_position_abs_home(void)
{
    uint8_t data[2] = {3, 0};
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_SET_HOME, 0, data, 2);
    return ret;
}
// 设置编码器线数 4096  200Eh:01/02 U16
static rt_err_t zlac_set_encoder_lines(uint16_t left_lines, uint16_t right_lines)
{
    return set_left_right_param(ZLAC_OD_ENCODER_LINES, left_lines, right_lines);	
}
rt_err_t zlac_get_encoder_lines(uint16_t *left, uint16_t *right)
{
    return get_left_right_param(ZLAC_OD_ENCODER_LINES, left, right);
}
// 设置额定电流以及最大电流 2014h:01/02 U16 0.1A  2015h:01/02 U16 0.1A
static rt_err_t zlac_set_current_limits(uint16_t rated_ma, uint16_t peak_ma)
{
    uint8_t data[2];
    data[0] = rated_ma & 0xFF; data[1] = (rated_ma >> 8) & 0xFF;
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_RATED_CURRENT, 1, data, 2);
    if (ret == RT_EOK) ret = zlac_sdo_write(ZLAC_OD_RATED_CURRENT, 2, data, 2);
    if (ret != RT_EOK) return ret;
    data[0] = peak_ma & 0xFF; data[1] = (peak_ma >> 8) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_MAX_CURRENT, 1, data, 2);
    if (ret == RT_EOK) ret = zlac_sdo_write(ZLAC_OD_MAX_CURRENT, 2, data, 2);
    return ret;
}
static rt_err_t zlac_set_rated_current(uint16_t left_rated_ma, uint16_t right_rated_ma)
{
    return set_left_right_param(ZLAC_OD_RATED_CURRENT, left_rated_ma, right_rated_ma);	
}
static rt_err_t zlac_set_peak_current(uint16_t left_peak_ma, uint16_t right_peak_ma)
{
    return set_left_right_param(ZLAC_OD_MAX_CURRENT, left_peak_ma, right_peak_ma);	
}
static rt_err_t zlac_get_current_limits(uint16_t *left, uint16_t *right)
{
    return get_left_right_param(ZLAC_OD_RATED_CURRENT, left, right);
}
rt_err_t zlac_get_current_peak(uint16_t *left, uint16_t *right)
{
    return get_left_right_param(ZLAC_OD_MAX_CURRENT, left, right);
}
// 设置最大速度 (无额定速度设置) 2008h:U16 rpm 1-1000
static rt_err_t zlac_set_max_speed(uint16_t rpm)
{
    uint8_t data[2] = {rpm & 0xFF, (rpm >> 8) & 0xFF};
    return zlac_sdo_write(ZLAC_OD_MAX_SPEED, 0, data, 2);
}
uint16_t zlac_get_max_speed(void)
{
    uint8_t data[2];
    if (zlac_sdo_read(ZLAC_OD_MAX_SPEED, 0, data, 2) == RT_EOK)
        return data[0] | (data[1] << 8);
    return 0;
}

// 设置速度PID参数 KP 201Dh KI 201Eh Kf 201Fh  U16
static rt_err_t zlac_set_velocity_pid(uint16_t kp, uint16_t ki, uint16_t kf)
{
    uint8_t data[2];
    data[0] = kp & 0xFF; data[1] = (kp >> 8) & 0xFF;
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_VEL_KP, 1, data, 2);
    if (ret == RT_EOK) ret = zlac_sdo_write(ZLAC_OD_VEL_KP, 2, data, 2);
    if (ret != RT_EOK) return ret;
    data[0] = ki & 0xFF; data[1] = (ki >> 8) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_VEL_KI, 1, data, 2);
    if (ret == RT_EOK) ret = zlac_sdo_write(ZLAC_OD_VEL_KI, 2, data, 2);
    if (ret != RT_EOK) return ret;
    data[0] = kf & 0xFF; data[1] = (kf >> 8) & 0xFF;
    ret = zlac_sdo_write(ZLAC_OD_VEL_KF, 1, data, 2);
    if (ret == RT_EOK) ret = zlac_sdo_write(ZLAC_OD_VEL_KF, 2, data, 2);
    return ret;
}
// 独立设置速度Pid参数  KP 201Dh:01/02 0-30000 U16
static rt_err_t zlac_set_velocity_kp(uint16_t left_kp, uint16_t right_kp)
{
    return set_left_right_param(ZLAC_OD_VEL_KP, left_kp, right_kp);	
}
// Ki 201Eh:01/02 0-30000 U16
static rt_err_t zlac_set_velocity_ki(uint16_t left_ki, uint16_t right_ki)
{
    return set_left_right_param(ZLAC_OD_VEL_KI, left_ki, right_ki);	
}
// Kf 201Fh:01/02 0-30000 U16
static rt_err_t zlac_set_velocity_kf(uint16_t left_kf, uint16_t right_kf)
{
    return set_left_right_param(ZLAC_OD_VEL_KF, left_kf, right_kf);	
}
rt_err_t zlac_get_velocity_pid_kp(uint16_t *left_kp, uint16_t *right_kp)
{
    return get_left_right_param(ZLAC_OD_VEL_KP, left_kp, right_kp);
}
rt_err_t zlac_get_velocity_pid_ki(uint16_t *left_ki, uint16_t *right_ki)
{
    return get_left_right_param(ZLAC_OD_VEL_KI, left_ki, right_ki);
}
rt_err_t zlac_get_velocity_pid_kf(uint16_t *left_kf, uint16_t *right_kf)
{
    return get_left_right_param(ZLAC_OD_VEL_KF, left_kf, right_kf);
}

/* 位置环比例增益 Kp */
// Kp 2020h:01/02 0-30000 U16
static rt_err_t zlac_set_position_kp(uint16_t left_kp, uint16_t right_kp)
{
    return set_left_right_param(ZLAC_OD_POS_KP, left_kp, right_kp);
}
rt_err_t zlac_get_position_kp(uint16_t *left_kp, uint16_t *right_kp)
{
    return get_left_right_param(ZLAC_OD_POS_KP, left_kp, right_kp);
}
/* 位置环前馈 Kf */
// Kf 2021h:01/02 0-30000 U16
static rt_err_t zlac_set_position_kf(uint16_t left_kf, uint16_t right_kf)
{
    return set_left_right_param(ZLAC_OD_POS_KF, left_kf, right_kf);
}
rt_err_t zlac_get_position_kf(uint16_t *left_kf, uint16_t *right_kf)
{
    return get_left_right_param(ZLAC_OD_POS_KF, left_kf, right_kf);
}
/* 速度平滑系数 */
// 2018h:01/02 0-30000 U16
static rt_err_t zlac_set_vel_smooth(uint16_t left_smooth, uint16_t right_smooth)
{
    return set_left_right_param(ZLAC_OD_VEL_SMOOTH, left_smooth, right_smooth);
}
rt_err_t zlac_get_vel_smooth(uint16_t *left_smooth, uint16_t *right_smooth)
{
    return get_left_right_param(ZLAC_OD_VEL_SMOOTH, left_smooth, right_smooth);
}
/* 前馈平滑系数 */
// 201Bh:01/02 0-30000 U16
static rt_err_t zlac_set_ff_smooth(uint16_t left_smooth, uint16_t right_smooth)
{
    return set_left_right_param(ZLAC_OD_FEEDFORWARD_SMOOTH, left_smooth, right_smooth);
}
rt_err_t zlac_get_ff_smooth(uint16_t *left_smooth, uint16_t *right_smooth)
{
    return get_left_right_param(ZLAC_OD_FEEDFORWARD_SMOOTH, left_smooth, right_smooth);
}

/* 通讯掉线保护时间 */
// 2000h 0-32767 U16
static rt_err_t zlac_set_offline_time(uint16_t ms)
{
    uint8_t data[2] = {ms & 0xFF, (ms >> 8) & 0xFF};
    return zlac_sdo_write(ZLAC_OD_OFFLINE_TIME, 0, data, 2);
}
static uint16_t zlac_get_offline_time(void)
{
    uint8_t data[2];
    if (zlac_sdo_read(ZLAC_OD_OFFLINE_TIME, 0, data, 2) == RT_EOK)
        return data[0] | (data[1] << 8);
    return 0;
}

/* 输出信号状态 (2004h) U16 查看B0-B1*/
static uint16_t zlac_get_output_status(void)
{
    uint8_t data[2];
    if (zlac_sdo_read(ZLAC_OD_OUTPUT_STATUS, 0, data, 2) == RT_EOK)
        return data[0] | (data[1] << 8);
    return 0;
}

/* 恢复出厂设置 */
// 2009h 0-1 U16
static rt_err_t zlac_restore_default(void)
{
    uint8_t data[2] = {1, 0};
    return zlac_sdo_write(ZLAC_OD_RESTORE_DEFAULT, 0, data, 2);
}

/* 电机极对数 */
// 200Ch 4-64 U16
static rt_err_t zlac_set_motor_poles(uint16_t left_poles, uint16_t right_poles)
{
    return set_left_right_param(ZLAC_OD_MOTOR_POLES, left_poles, right_poles);
}
rt_err_t zlac_get_motor_poles(uint16_t *left_poles, uint16_t *right_poles)
{
    return get_left_right_param(ZLAC_OD_MOTOR_POLES, left_poles, right_poles);
}

/* 起始速度 (200Dh:01/02) U16 1-256 rpm */
static rt_err_t zlac_set_start_speed(uint16_t left_rpm, uint16_t right_rpm)
{
	return set_left_right_param(ZLAC_OD_START_SPEED, left_rpm, right_rpm);
}
static rt_err_t zlac_get_start_speed(uint16_t *left_rpm, uint16_t *right_rpm)
{
    return get_left_right_param(ZLAC_OD_START_SPEED, left_rpm, right_rpm);
}

// 2026h 01:报警pwm开启(1) 02:过载开启(0) 03:I/O急停0锁轴1解轴(0) 04:驻车开启(0) 05:速度分辨率设置(1) 06:速度超差开启(1) 07:初始方向(0CW)
/* 初始方向 (2026h:07) U16 */
static rt_err_t zlac_set_init_direction(uint16_t direction)
{
    uint8_t data[2] = {direction & 0xFF, (direction >> 8) & 0xFF};
    return zlac_sdo_write(ZLAC_OD_EMERGENCY_STOP, 7, data, 2);
}
uint16_t zlac_get_init_direction(void)
{
    uint8_t data[2];
    if (zlac_sdo_read(ZLAC_OD_EMERGENCY_STOP, 7, data, 2) == RT_EOK)
        return data[0] | (data[1] << 8);
    return 0;
}

/* 电机状态读取 静止或运行 (2033h:01 02) U16*/
uint16_t zlac_get_motor_status_left(void)
{
    uint8_t data[2];
    if (zlac_sdo_read(ZLAC_OD_MOTOR_STATUS, 1, data, 2) == RT_EOK)
        return data[0] | (data[1] << 8);
    return 0;
}
uint16_t zlac_get_motor_status_right(void)
{
    uint8_t data[2];
    if (zlac_sdo_read(ZLAC_OD_MOTOR_STATUS, 2, data, 2) == RT_EOK)
        return data[0] | (data[1] << 8);
    return 0;
}

/* 过载系数 (2012h:01/02) U16 */
static rt_err_t zlac_set_overload_factor(uint16_t left_factor, uint16_t right_factor)
{
	return set_left_right_param(ZLAC_OD_OVERLOAD_FACTOR, left_factor, right_factor);
}
static rt_err_t zlac_get_overload_factor(uint16_t *left_factor, uint16_t *right_factor)
{
    return get_left_right_param(ZLAC_OD_OVERLOAD_FACTOR, left_factor, right_factor);
}

/* 温度保护阈值 (2013h:01/02/03) U16 0.1℃ 0-1200*/
static rt_err_t zlac_set_temp_threshold(uint16_t left_motor_c, uint16_t right_motor_c, uint16_t driver_c)
{
    uint8_t data[2];
    rt_err_t ret;
    if (left_motor_c) {
				data[0] = left_motor_c & 0xFF;
				data[1] = (left_motor_c >> 8) & 0xFF;
        ret = zlac_sdo_write(ZLAC_OD_TEMP_PROTECT, 1, data, 2);
        if (ret != RT_EOK) return ret;
    }
    if (right_motor_c) {
				data[0] = right_motor_c & 0xFF;
				data[1] = (right_motor_c >> 8) & 0xFF;
        ret = zlac_sdo_write(ZLAC_OD_TEMP_PROTECT, 2, data, 2);
        if (ret != RT_EOK) return ret;			
    }
    if (driver_c) {
				data[0] = driver_c & 0xFF;
				data[1] = (driver_c >> 8) & 0xFF;
        ret = zlac_sdo_write(ZLAC_OD_TEMP_PROTECT, 3, data, 2);
        if (ret != RT_EOK) return ret;	
    }
    return RT_EOK;
}
static rt_err_t zlac_get_temp_threshold(uint16_t *left_motor_c, uint16_t *right_motor_c, uint16_t *driver_c)
{
    uint8_t data[2];
    rt_err_t ret;
    if (left_motor_c) {
        ret = zlac_sdo_read(ZLAC_OD_TEMP_PROTECT, 1, data, 2);
        if (ret == RT_EOK) *left_motor_c = data[0] | (data[1] << 8);
        else return ret;
    }
    if (right_motor_c) {
        ret = zlac_sdo_read(ZLAC_OD_TEMP_PROTECT, 2, data, 2);
        if (ret == RT_EOK) *right_motor_c = data[0] | (data[1] << 8);
        else return ret;
    }
    if (driver_c) {
        ret = zlac_sdo_read(ZLAC_OD_TEMP_PROTECT, 3, data, 2);
        if (ret == RT_EOK) *driver_c = data[0] | (data[1] << 8);
        else return ret;
    }
    return RT_EOK;
}

// 保存所有 RW 属性的参数到 EEPROM 2010h U16
static rt_err_t zlac_save_parameters(void)
{
    uint8_t data[2] = {1, 0};
    rt_err_t ret = zlac_sdo_write(ZLAC_OD_SAVE_PARAMS, 0, data, 2);
    if (ret == RT_EOK) {
        rt_thread_mdelay(150);  // 确保写入完成
    }
    return ret;
}

/* ======================== 默认配置（根据电机参数） msh一次性配置即可 ======================== */
static rt_err_t zlac_motor_config_default(void)
{
    rt_err_t ret;
    ret = zlac_set_encoder_lines(ZLAC_MOTOR_ENCODER_LINES,ZLAC_MOTOR_ENCODER_LINES);
    if (ret != RT_EOK) return ret;
	  ret = zlac_set_motor_poles(ZLAC_MOTOR_POLE_PAIRS, ZLAC_MOTOR_POLE_PAIRS);
    if (ret != RT_EOK) return ret;	
    ret = zlac_set_current_limits(ZLAC_MOTOR_RATED_CURRENT_MA, ZLAC_MOTOR_PEAK_CURRENT_MA);
    if (ret != RT_EOK) return ret;
    ret = zlac_set_max_speed(ZLAC_MOTOR_MAX_RPM);
    if (ret != RT_EOK) return ret;
    ret = zlac_set_accel_time(ZLAC_MOTOR_V_ACCEL_TIME_MS,ZLAC_MOTOR_V_ACCEL_TIME_MS);
    if (ret != RT_EOK) return ret;
    ret = zlac_set_decel_time(ZLAC_MOTOR_V_DECEL_TIME_MS,ZLAC_MOTOR_V_DECEL_TIME_MS);
    if (ret != RT_EOK) return ret;
    ret = zlac_set_quick_stop_time(ZLAC_MOTOR_QUICKSTOP_TIME_MS,ZLAC_MOTOR_QUICKSTOP_TIME_MS);
    if (ret != RT_EOK) return ret;	
	
    // 保存参数
     rt_thread_mdelay(50);
     ret = zlac_save_parameters();
    return ret;
}

/* ======================== 参数一致性检查 ======================== */
static void zlac_check_default_config(void)
{
    uint16_t left, right;
    uint16_t rated_left, rated_right, peak_left, peak_right;
    uint16_t encoder_left, encoder_right;
    uint16_t poles_left, poles_right;
    uint32_t accel_left, accel_right, decel_left, decel_right;  // 注意类型为 uint32_t
    uint16_t max_speed;
    int mismatch = 0;

    rt_kprintf("\n========== Parameter Consistency Check ==========\n");

    // 编码器线数
    if (zlac_get_encoder_lines(&encoder_left, &encoder_right) == RT_EOK) {
        if (encoder_left != ZLAC_MOTOR_ENCODER_LINES || encoder_right != ZLAC_MOTOR_ENCODER_LINES) {
            rt_kprintf("[WARN] Encoder lines mismatch: current L=%d R=%d, expected %d\n",
                       encoder_left, encoder_right, ZLAC_MOTOR_ENCODER_LINES);
            mismatch++;
        }
    }

    // 极对数
    if (zlac_get_motor_poles(&poles_left, &poles_right) == RT_EOK) {
        if (poles_left != ZLAC_MOTOR_POLE_PAIRS || poles_right != ZLAC_MOTOR_POLE_PAIRS) {
            rt_kprintf("[WARN] Motor poles mismatch: current L=%d R=%d, expected %d\n",
                       poles_left, poles_right, ZLAC_MOTOR_POLE_PAIRS);
            mismatch++;
        }
    }

    // 额定电流
    if (zlac_get_current_limits(&rated_left, &rated_right) == RT_EOK) {
        if (rated_left != ZLAC_MOTOR_RATED_CURRENT_MA || rated_right != ZLAC_MOTOR_RATED_CURRENT_MA) {
            rt_kprintf("[WARN] Rated current mismatch: current L=%d *0.1A R=%d *0.1A, expected %d *0.1A\n",
                       rated_left, rated_right, ZLAC_MOTOR_RATED_CURRENT_MA);
            mismatch++;
        }
    }

    // 峰值电流
    if (zlac_get_current_peak(&peak_left, &peak_right) == RT_EOK) {
        if (peak_left != ZLAC_MOTOR_PEAK_CURRENT_MA || peak_right != ZLAC_MOTOR_PEAK_CURRENT_MA) {
            rt_kprintf("[WARN] Peak current mismatch: current L=%d *0.1A R=%d *0.1A, expected %d *0.1A\n",
                       peak_left, peak_right, ZLAC_MOTOR_PEAK_CURRENT_MA);
            mismatch++;
        }
    }

    // 最大转速
    max_speed = zlac_get_max_speed();
    if (max_speed != ZLAC_MOTOR_MAX_RPM) {
        rt_kprintf("[WARN] Max speed mismatch: current %d rpm, expected %d rpm\n",
                   max_speed, ZLAC_MOTOR_MAX_RPM);
        mismatch++;
    }

    // 加速时间 (注意：zlac_get_accel_time 返回 uint32_t 类型)
    if (zlac_get_accel_time(&accel_left, &accel_right) == RT_EOK) {
        if (accel_left != ZLAC_MOTOR_V_ACCEL_TIME_MS || accel_right != ZLAC_MOTOR_V_ACCEL_TIME_MS) {
            rt_kprintf("[WARN] Accel time mismatch: current L=%d ms R=%d ms, expected %d ms\n",
                       accel_left, accel_right, ZLAC_MOTOR_ACCEL_TIME_MS);
            mismatch++;
        }
    }

    // 减速时间
    if (zlac_get_decel_time(&decel_left, &decel_right) == RT_EOK) {
        if (decel_left != ZLAC_MOTOR_V_DECEL_TIME_MS || decel_right != ZLAC_MOTOR_V_DECEL_TIME_MS) {
            rt_kprintf("[WARN] Decel time mismatch: current L=%d ms R=%d ms, expected %d ms\n",
                       decel_left, decel_right, ZLAC_MOTOR_DECEL_TIME_MS);
            mismatch++;
        }
    }

    if (mismatch == 0) {
        rt_kprintf("[INFO] All default parameters match the expected values.\n");
    } else {
        rt_kprintf("[INFO] %d parameter(s) mismatch. Use 'zlac_set_config' to correct, then 'zlac_set_config save'.\n", mismatch);
    }
    rt_kprintf("================================================\n");
}

/* ======================== 故障码读取与清除 ======================== */
uint32_t zlac_get_fault_code(void)
{
    uint8_t data[4];
    if (zlac_sdo_read(ZLAC_OD_FAULT_CODE, 0, data, 4) == RT_EOK)
        return data[0] | (data[1]<<8) | (data[2]<<16) | (data[3]<<24);
    return 0;
}
rt_err_t zlac_clear_fault(void)
{
    return zlac_control_fault_reset();
}

/* ======================== 温度、电压读取 ======================== */
int16_t zlac_get_motor_temp_left(void)
{
    uint8_t data[2];
    if (zlac_sdo_read(ZLAC_OD_TEMPERATURE, 1, data, 2) == RT_EOK)
        return (int16_t)(data[0] | (data[1]<<8));
    return 0;
}
int16_t zlac_get_motor_temp_right(void)
{
    uint8_t data[2];
    if (zlac_sdo_read(ZLAC_OD_TEMPERATURE, 2, data, 2) == RT_EOK)
        return (int16_t)(data[0] | (data[1]<<8));
    return 0;
}
int16_t zlac_get_driver_temp(void)
{
    uint8_t data[2];
    if (zlac_sdo_read(ZLAC_OD_TEMPERATURE, 3, data, 2) == RT_EOK)
        return (int16_t)(data[0] | (data[1]<<8));
    return 0;
}
uint16_t zlac_get_bus_voltage(void)
{
    uint8_t data[2];
    if (zlac_sdo_read(ZLAC_OD_BUS_VOLTAGE, 0, data, 2) == RT_EOK)
        return data[0] | (data[1]<<8);
    return 0;
}

/* ======================== 心跳监测 ======================== */
// 自动恢复通讯
static void zlac_recover_communication(void)
{
    rt_kprintf("[ZLAC] Attempting to recover communication...\n");
    // 1. NMT 复位通讯
    zlac_nmt_reset_communication();
    rt_thread_mdelay(200);
    // 2. 重新启动节点
    zlac_nmt_start();
    rt_thread_mdelay(50);
    // 3. 根据当前模式重新初始化
		if (s_velocity_mode_ready) {
        // 仅配置 PDO 和参数，不使能
        zlac_config_pdo_for_velocity_mode();
        zlac_config_velocity_mode_params(ZLAC_MOTOR_V_ACCEL_TIME_MS, ZLAC_MOTOR_V_DECEL_TIME_MS);
    } else if (s_position_mode_ready) {
        zlac_config_pdo_for_position_mode();
        zlac_config_position_mode_params(ZLAC_MOTOR_ACCEL_TIME_MS, ZLAC_MOTOR_DECEL_TIME_MS, ZLAC_MOTOR_NORMAL_RPM);
    } else {
        // 默认配置速度模式 PDO
        zlac_config_pdo_for_velocity_mode();
        zlac_config_velocity_mode_params(ZLAC_MOTOR_V_ACCEL_TIME_MS, ZLAC_MOTOR_V_DECEL_TIME_MS);
    }
    // 4. 重新使能
//    zlac_control_enable();
    rt_kprintf("[ZLAC] Recovery completed.\n");
}

void zlac_check_heartbeat(void)
{
	static uint32_t last_probe = 0;
	    uint32_t now = rt_tick_get_millisecond();
//    if (s_last_hb_tick == 0) return;
//    if (now - s_last_hb_tick > ZLAC_HEARTBEAT_TIMEOUT_MS) {
//        if (s_online) {
//            rt_kprintf("[ZLAC] Heartbeat timeout!\n");
//            s_online = RT_FALSE;
//					// 上传离线信号
//        }
//    } else {
//        if (!s_online) {
//            rt_kprintf("[ZLAC] Online again.\n");
//            s_online = RT_TRUE;
//					// 上传恢复上线信号
//        }
//    }
		   // 主动探测：每 10 秒读取一次错误寄存器 判断通讯是否正常

    if (now - last_probe >= 10000) {
        last_probe = now;
        uint8_t err_reg;
        if (zlac_sdo_read(0x1001, 0, &err_reg, 1) == RT_EOK) {
            s_fail_count = 0;
            if (!s_online) {
                rt_kprintf("[ZLAC] Communication restored.\n");
                s_online = RT_TRUE;
            }
        } else {
            s_fail_count++;
            if (s_fail_count >= 3 && s_online) {
                rt_kprintf("[ZLAC] Communication lost (probe failed).\n");
                s_online = RT_FALSE;
                // 尝试恢复，限制频率
                if (now - s_last_recovery_tick > 10000) {
                    s_last_recovery_tick = now;
                    zlac_recover_communication();
                }
            }
        }
    }
    
    // 可选：保留原有心跳检测（如果驱动器能正常发送心跳）
    if (s_last_hb_tick != 0) {
        if (now - s_last_hb_tick > ZLAC_HEARTBEAT_TIMEOUT_MS && s_online) {
            rt_kprintf("[ZLAC] Heartbeat timeout.\n");
            s_online = RT_FALSE;
            if (now - s_last_recovery_tick > 10000) {
                s_last_recovery_tick = now;
                zlac_recover_communication();
            }
        }
    }
}

rt_bool_t zlac_is_online(void)
{
    return s_online;
}

/* ======================== CAN 接收回调 ======================== */
/* CAN 接收中断回调：只释放信号量 */
static rt_err_t can_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&s_rx_sem);
    return RT_EOK;
}
/* 接收线程：等待信号量，然后读取所有待处理消息 */
static void can_rx_thread_entry(void *param)
{
    struct rt_can_msg rx_msg = {0};
		   rt_err_t ret;
//		struct rt_can_filter_item items[6] =
//		{
//				RT_CAN_FILTER_ITEM_INIT(0x0, 0, 0, 0, 0x7ff, RT_NULL, RT_NULL),
//				RT_CAN_FILTER_ITEM_INIT(0x181, 0, 0, 0, 0x700, RT_NULL, RT_NULL),
//				RT_CAN_FILTER_ITEM_INIT(0x201, 0, 0, 0, 0x700, RT_NULL, RT_NULL), 
//				RT_CAN_FILTER_ITEM_INIT(0x581, 0, 0, 0, 0x700, RT_NULL, RT_NULL), 			
//				RT_CAN_FILTER_ITEM_INIT(0x601, 0, 0, 0, 0x700, RT_NULL, RT_NULL), 			
//				RT_CAN_FILTER_ITEM_INIT(0x701, 0, 0, 0, 0x700, RT_NULL, RT_NULL), 			
//		};
//		struct rt_can_filter_config cfg = {6, 1, items}; /* 一共有 1 个过滤表 */
//		/* 设置硬件过滤表 */
//		ret = rt_device_control(s_can_dev, RT_CAN_CMD_SET_FILTER, &cfg);		
    
		rt_uint32_t cmd_arg = 1; // Argument to enable the controller
     ret = rt_device_control(s_can_dev, RT_CAN_CMD_START, &cmd_arg);
				
    while (s_rx_thread_running)
    {
        rt_sem_take(&s_rx_sem, RT_WAITING_FOREVER);
			  rx_msg.hdr_index = -1;
        while (rt_device_read(s_can_dev, 0, &rx_msg, sizeof(rx_msg)) == sizeof(rx_msg))
        {
            uint32_t id = rx_msg.id;
            uint8_t *d = rx_msg.data;
            uint8_t len = rx_msg.len;

            // 心跳报文
            if ((id & 0x7F0) == 0x700) {
                s_last_hb_tick = rt_tick_get_millisecond();
                s_node_state = d[0];
                s_online = RT_TRUE;
                continue;
            }
            // SDO 响应
            if (id == ZLAC_COBID_TSDO(ZLAC_NODE_ID)) {
                if (len >= 4) {
                    memcpy(s_sdo_resp.data, d, len);
                    s_sdo_resp.len = len;
                    s_sdo_resp.complete = RT_TRUE;
                    rt_sem_release(&s_sdo_sem);
                }
                continue;
            }
            // TPDO1 处理 位置模式和速度模式
//            if (id == ZLAC_COBID_TPDO1(ZLAC_NODE_ID)) {						
//                if (s_current_mode == ZLAC_MODE_PROFILE_VELOCITY && len == 4) {																	
//                    s_actual_vel_left = (int16_t)(d[0] | (d[1]<<8));
//                    s_actual_vel_right = (int16_t)(d[2] | (d[3]<<8));
//                } else if (s_current_mode == ZLAC_MODE_PROFILE_POSITION && len == 8) {
//                    s_actual_pos_left = (int32_t)(d[0] | (d[1]<<8) | (d[2]<<16) | (d[3]<<24));
//                    s_actual_pos_right = (int32_t)(d[4] | (d[5]<<8) | (d[6]<<16) | (d[7]<<24));
//                }
//            }
							// 分开处理 TPDO1 速度模式  TPDO2 位置模式   len的判断可增加错误数据的简单过滤
							if (id == ZLAC_COBID_TPDO1(ZLAC_NODE_ID) && len == 4) {
									// 速度反馈（4字节）
									s_actual_vel_left = (int16_t)(d[0] | (d[1]<<8));
									s_actual_vel_right = (int16_t)(d[2] | (d[3]<<8));
							} else if (id == ZLAC_COBID_TPDO2(ZLAC_NODE_ID) && len == 8) {
									// 位置反馈（8字节）
									s_actual_pos_left = (int32_t)(d[0] | (d[1]<<8) | (d[2]<<16) | (d[3]<<24));
									s_actual_pos_right = (int32_t)(d[4] | (d[5]<<8) | (d[6]<<16) | (d[7]<<24));
							}					
        }
    }
}

/* ======================== 初始化 ======================== */
static void zlac_show_config(void);
static void zlac_show_pid(void);
rt_err_t zlac_motor_init(void)
{
    // 查找 CAN 设备
    s_can_dev = rt_device_find(ZLAC_CAN_DEV_NAME);
    if (s_can_dev == RT_NULL) {
        rt_kprintf("[ZLAC] CAN device %s not found\n", ZLAC_CAN_DEV_NAME);
        return -RT_ERROR;
    }
    rt_err_t ret = rt_device_open(s_can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    if (ret != RT_EOK) {
        rt_kprintf("[ZLAC] Open CAN device failed\n");
        return ret;
    }
		/* 设置 CAN 通信的波特率为 500kbit/s*/
		ret = rt_device_control(s_can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN500kBaud);
		/* 设置 CAN 的工作模式为正常工作模式 */
		ret = rt_device_control(s_can_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);		
    // 不设置滤波器 (接收所有)
		
    // 设置接收回调
    rt_device_set_rx_indicate(s_can_dev, can_rx_ind);
    // 初始化同步对象
    rt_sem_init(&s_sdo_sem, "zlac_sdo", 0, RT_IPC_FLAG_PRIO);
    rt_mutex_init(&s_tx_mutex, "zlac_tx", RT_IPC_FLAG_PRIO);
		rt_sem_init(&s_rx_sem, "zlac_rx", 0, RT_IPC_FLAG_FIFO);
	    // 创建接收线程
    s_rx_thread_running = RT_TRUE;
    s_rx_thread = rt_thread_create("zlac_rx", can_rx_thread_entry, NULL, 2048, 11, 10);
    if (s_rx_thread) rt_thread_startup(s_rx_thread);
	
		rt_thread_mdelay(100);   // 等待系统和通讯建立
		//配置驱动器心跳周期
		uint16_t heartbeat_ms = 1000;  // 1秒
		uint8_t data[2] = {heartbeat_ms & 0xFF, (heartbeat_ms >> 8) & 0xFF};
		zlac_sdo_write(ZLAC_OD_HEARTBEAT_PROD, 0, data, 2);
    // 启动心跳监测线程
    // 读取一次心跳时间 (驱动器默认会发送心跳)
    s_online = RT_TRUE;  // 初始假设在线
	
		rt_thread_mdelay(100);   // 等待系统和通讯建立
		// 打印驱动器默认配置信息
		zlac_show_config();
		zlac_show_pid();
		zlac_check_default_config();
		zlac_set_right_motor_reverse(RT_TRUE);  // 默认左右电机反向
		ret = zlac_control_free();
		rt_kprintf("Motor free (no torque), ret = %d\n", ret);
		zlac_brake_engage();   // 抱闸
		uint16_t left,right;
		zlac_get_brake(&left,&right);
		rt_kprintf("Brake: B0=%s, B1=%s\n", left ? "engage":"release" , right ? "engage":"release");		
    ret = nmt_send(0x80, ZLAC_NODE_ID);   // 进入预操作状态
    if (ret == RT_EOK) {
        rt_thread_mdelay(50);
        ret = config_tpdo2_for_position();
        if (ret == RT_EOK) {
            ret = nmt_send(0x01, ZLAC_NODE_ID);   // 重启节点
            rt_thread_mdelay(50);
            rt_kprintf("[ZLAC] TPDO2 configured for position feedback\n");
        } else {
            rt_kprintf("[ZLAC] TPDO2 configuration failed\n");
        }
    } else {
        rt_kprintf("[ZLAC] Failed to enter pre-operational state for TPDO2 config\n");
    }
		rt_kprintf("[ZLAC] Driver init OK\n");
    return RT_EOK;
}

/**
 * @brief 打印故障码详细信息
 * @param fault 从 0x603F 读取的故障码（32位）
 */
void zlac_print_fault_code(uint32_t fault)
{
    if (fault == 0) {
        rt_kprintf("No fault.\n");
        return;
    }
    rt_kprintf("Fault code: 0x%08X\n", fault);
    
    // 左电机故障（高16位）
    rt_kprintf("  Left motor:\n");
    int printed = 0;
    for (uint32_t i = 0; i < sizeof(s_fault_table)/sizeof(s_fault_table[0]); i++) {
        uint32_t mask = s_fault_table[i].mask;
        // 仅处理高16位掩码（左电机）
        if ((mask & 0xFFFF0000) == 0) continue;
        if (fault & mask) {
            rt_kprintf("    - %s\n", s_fault_table[i].description);
            printed = 1;
        }
    }
    if (!printed) rt_kprintf("    None\n");
    
    // 右电机故障（低16位）
    rt_kprintf("  Right motor:\n");
    printed = 0;
    for (uint32_t i = 0; i < sizeof(s_fault_table)/sizeof(s_fault_table[0]); i++) {
        uint32_t mask = s_fault_table[i].mask;
        // 仅处理低16位掩码（右电机）
        if ((mask & 0x0000FFFF) == 0) continue;
        if (fault & mask) {
            rt_kprintf("    - %s\n", s_fault_table[i].description);
            printed = 1;
        }
    }
    if (!printed) rt_kprintf("    None\n");    
}

/* ======================== MSH 测试命令 ======================== */
#ifdef RT_USING_MSH
static void zlac_msh_test(int argc, char **argv)
{
	rt_err_t ret;
    if (argc < 2) {
        rt_kprintf("Usage:\n");
        rt_kprintf("  zlac_test enable          - enable motor\n");
        rt_kprintf("  zlac_test disable         - stop motor but lock\n");
        rt_kprintf("  zlac_test free            - Motor free (no torque)\n");
				rt_kprintf("  zlac_test quickstop        - emergency quick stop\n");
				rt_kprintf("  zlac_test brake on/off    - engage/release brake\n");
				rt_kprintf("  zlac_test sdo_vel         - read actual velocity via SDO (for debugging)\n");
        rt_kprintf("  zlac_test speed L R       - set speed (rpm)\n");
        rt_kprintf("  zlac_test pos L R         - absolute/relative position (pulses)\n");
        rt_kprintf("  zlac_test mode vel/abs_pos/rel_pos - set operation mode and config PDO\n");
				rt_kprintf("  zlac_test home            - set current position as absolute zero\n");
				rt_kprintf("  zlac_test fault           - show detailed fault code and clear\n");			
				rt_kprintf("  zlac_test rev on/off      - enable/disable right motor direction reverse\n");
        return;
    }
/* 操作流程*/
/*速度模式： brake off / mode vel / speed 10 10 /disable/enable/speed -10 -10/free / brake on*/
/*绝对位置模式： brake off / mode abs_pos / home/ pos 1000 1000 /disable/enable/pos -1000 -1000 /free / brake on*/	
/*相对位置模式： brake off / mode rel_pos / pos 1000 1000 /disable/enable/pos -1000 -1000 /free / brake on*/			
/*电机一圈脉冲数：4096*4 = 16384 脉冲/圈    电机直径 173 mm → 周长 ≈ 0.5435 m*/
/*每个脉冲对应的位移 = 543.5 mm / 16384 ≈ 0.03317 mm 或者 每毫米需要的脉冲数 = 16384 / 543.5 ≈ 30.15 脉冲/mm*/
//一圈值		zlac_test pos 16384 16384
		if (rt_strcmp(argv[1], "rev") == 0) {
				if (argc < 3) {
						rt_kprintf("Usage: zlac_test rev on/off\n");
						return;
				}
				if (rt_strcmp(argv[2], "on") == 0) {
						zlac_set_right_motor_reverse(RT_TRUE);
						rt_kprintf("Right motor reverse enabled.\n");
				} else if (rt_strcmp(argv[2], "off") == 0) {
						zlac_set_right_motor_reverse(RT_FALSE);
						rt_kprintf("Right motor reverse disabled.\n");
				} else {
						rt_kprintf("Invalid argument. Use 'on' or 'off'.\n");
				}
		} else if (rt_strcmp(argv[1], "home") == 0) {
			ret = zlac_set_position_abs_home();
			if (ret == RT_EOK)
					rt_kprintf("Absolute position home set (current position cleared).\n");
			else
					rt_kprintf("Failed to set home, err=%d\n", ret);
		} else if (rt_strcmp(argv[1], "fault") == 0) {
				uint32_t fault = zlac_get_fault_code();
				zlac_print_fault_code(fault);
				if(fault){
					zlac_clear_fault();
					rt_kprintf("Now clear fault!\n");
				}				
		}else if (rt_strcmp(argv[1], "enable") == 0) {
				ret = zlac_control_enable();
			  if (ret != RT_EOK) rt_kprintf("zlac_control_enable --> Failse\n");
        ret = zlac_nmt_start();
				if (ret != RT_EOK) rt_kprintf("zlac_nmt_start --> Failse\n");
        rt_kprintf("Motor enabled\n");
    } else if (rt_strcmp(argv[1], "disable") == 0) {
        ret = zlac_control_disable();
				if (ret != RT_EOK) rt_kprintf("zlac_control_disable --> Failse\n");		
        rt_kprintf("Motor disabled\n");
    } else if (rt_strcmp(argv[1], "free") == 0) {
				ret = zlac_control_free();
				rt_kprintf("Motor free (no torque), ret = %d\n", ret);
		}else if (rt_strcmp(argv[1], "quickstop") == 0) {
			ret = zlac_control_quickstop();
			rt_kprintf("Quick stop executed, ret=%d\n", ret);
		} else if (rt_strcmp(argv[1], "brake") == 0) {
        if (argc < 3) return;
        if (rt_strcmp(argv[2], "on") == 0){   //刹车
            ret = zlac_brake_engage();
						if (ret != RT_EOK) rt_kprintf("zlac_brake_engage --> Failse\n");	
        }else if (rt_strcmp(argv[2], "off") == 0){ //不刹车
            ret = zlac_brake_release();
						if (ret != RT_EOK) rt_kprintf("zlac_brake_release --> Failse\n");	
        }else
            rt_kprintf("Invalid argument. Use 'on' or 'off'.\n");
				uint16_t left,right;
				zlac_get_brake(&left,&right);
				rt_kprintf("Brake: B0=%s, B1=%s\n", left ? "engage":"release" , right ? "engage":"release");			
					// 输出信号状态
				uint16_t out = zlac_get_output_status();
				rt_kprintf("Output status: Y0=%d, Y1=%d, B0=%d, B1=%d\n",
									 (out>>0)&1, (out>>1)&1, (out>>2)&1, (out>>3)&1);

    } else if (rt_strcmp(argv[1], "sdo_vel") == 0) {
				int16_t left,right;
				ret = zlac_get_velocity_by_sdo(&left,&right);
				if(ret == RT_EOK){
					 rt_kprintf("SDO actual velocity: L=%d.%d rpm, R=%d.%d rpm (0.1rpm unit)\n",
                   left/10, abs(left%10), right/10, abs(right%10));
				}else{
					rt_kprintf("Failed to read SDO velocity (606C:03)\n");
				}
		} else if (rt_strcmp(argv[1], "speed") == 0) {
        if (argc < 4) {
            rt_kprintf("Need left and right rpm\n");
            return;
        }
        int16_t left = atoi(argv[2]);
        int16_t right = atoi(argv[3]);
				if(left > ZLAC_MOTOR_MAX_RPM || left < -ZLAC_MOTOR_MAX_RPM || right > ZLAC_MOTOR_MAX_RPM|| right < -ZLAC_MOTOR_MAX_RPM){
					rt_kprintf("speed set error : speed>%d(max)\n",ZLAC_MOTOR_MAX_RPM);
					return;
				}
				// 先检查速度模式是否已初始化
				if (!s_velocity_mode_ready) {
						rt_kprintf("Velocity mode not initialized. Please run 'zlac_test mode vel' first.\n");
						return;
				}
        ret = zlac_set_velocity(left, right);
				if(ret == RT_EOK)
					rt_kprintf("Set speed L=%d R=%d rpm\n", left, right);
				else if(ret == ZLAC_ERR_BRAKE_UNRELEASE){
					rt_kprintf("Error brake unrelease\n");
				}else if(ret == ZLAC_ERR_INVALID_STATE){
				  rt_kprintf("Motor not enabled, cannot set speed.\n");
				}
				else
					rt_kprintf("Error: %d\n",ret);
    } else if (rt_strcmp(argv[1], "pos") == 0) {
//        if (argc < 5) {
//            rt_kprintf("Usage: zlac_test pos abs/rel left right\n");
//            return;
//        }
//        const char *mode_str = argv[2];
//        int32_t left = atoi(argv[3]);
//        int32_t right = atoi(argv[4]);
//        if (rt_strcmp(mode_str, "abs") == 0) {
//            zlac_set_position_abs(left, right);
//            rt_kprintf("Absolute position set: L=%ld R=%ld pulses\n", left, right);
//        } else if (rt_strcmp(mode_str, "rel") == 0) {
//            zlac_set_position_rel(left, right);
//            rt_kprintf("Relative position set: L=%ld R=%ld pulses\n", left, right);
//        } else {
//            rt_kprintf("Invalid mode. Use 'abs' or 'rel'.\n");
//        }
        if (argc < 4) {
            rt_kprintf("Usage: zlac_test pos left right\n");
            return;
        }
			  // 先检查位置模式是否已初始化
				if (!s_position_mode_ready) {
						rt_kprintf("Position mode not initialized. Please run 'zlac_test mode abs_pos/rel_pos' first.\n");
						return;
				}
		    int32_t left = atoi(argv[2]);
        int32_t right = atoi(argv[3]);
				ret = zlac_set_position_by_mode(left,right);
				if(ret != RT_EOK){
					rt_kprintf("Error:%d\n",ret);
				}
				if(s_position_mode == ZLAC_POS_MODE_ABSOLUTE){
					rt_kprintf("Absolute position set: L=%ld R=%ld pulses\n", left, right);
				}else{
					rt_kprintf("Relative position set: L=%ld R=%ld pulses\n", left, right);
				}
    } else if (rt_strcmp(argv[1], "mode") == 0) {
        if (argc < 3) return;
        if (rt_strcmp(argv[2], "vel") == 0) {
//            zlac_config_pdo_for_velocity_mode();
//					  zlac_config_velocity_mode_params(ZLAC_MOTOR_V_ACCEL_TIME_MS, ZLAC_MOTOR_V_DECEL_TIME_MS);
          rt_kprintf("Set velocity mode and configured PDO\n");
					ret = zlac_init_velocity_mode();
					if(ret != RT_EOK){
							rt_kprintf("Error: %d\n",ret);
					}else{
							rt_kprintf("Set velocity mode Success\n");
					}
        } else if (rt_strcmp(argv[2], "abs_pos") == 0) {
//            zlac_config_pdo_for_position_mode();
//					  zlac_config_position_mode_params(ZLAC_MOTOR_ACCEL_TIME_MS, ZLAC_MOTOR_DECEL_TIME_MS, ZLAC_MOTOR_MAX_RPM);
//							zlac_set_position_mode(ZLAC_POS_MODE_ABSOLUTE);
          rt_kprintf("Set Absolute position mode and configured PDO\n");
					ret = zlac_init_position_mode(ZLAC_POS_MODE_ABSOLUTE);
					if(ret != RT_EOK){
							rt_kprintf("Error: %d\n",ret);
					}else{
							rt_kprintf("Set Absolute position  mode Success\n");
					}
        } else if (rt_strcmp(argv[2], "rel_pos") == 0) {
//            zlac_config_pdo_for_position_mode();
//					  zlac_config_position_mode_params(ZLAC_MOTOR_ACCEL_TIME_MS, ZLAC_MOTOR_DECEL_TIME_MS, ZLAC_MOTOR_MAX_RPM);
//							zlac_set_position_mode(ZLAC_POS_MODE_RELATIVE);
					rt_kprintf("Set Relative position mode and configured PDO\n");	
					ret = zlac_init_position_mode(ZLAC_POS_MODE_RELATIVE);		
					if(ret != RT_EOK){
							rt_kprintf("Error: %d\n",ret);
					}else{
							rt_kprintf("Set Relative position  mode Success\n");
					}				
				} else {
            rt_kprintf("Invalid mode. Use 'vel' or 'abs_pos' or 'rel_pos'.\n");
        }
    } else if (rt_strcmp(argv[1], "save") == 0) {
        zlac_save_parameters();
        rt_kprintf("Parameters saved\n");
    } else {
        rt_kprintf("Unknown command\n");
    }
}
MSH_CMD_EXPORT_ALIAS(zlac_msh_test, zlac_test, ZLAC8015D motor test);

/* ======================== 参数打印命令 ======================== */

/* 1. 打印电机基本参数和控制配置 */
static void zlac_show_config(void)
{
    uint16_t left, right;
    uint32_t accel_left, accel_right, decel_left, decel_right;  // 注意类型为 uint32_t

    rt_kprintf("\n========== ZLAC8015D Configuration ==========\n");

    // 编码器线数
    if (zlac_get_encoder_lines(&left, &right) == RT_EOK)
        rt_kprintf("Encoder lines: L=%d, R=%d\n", left, right);
	// 电机极对数
    if (zlac_get_motor_poles(&left, &right) == RT_EOK)
        rt_kprintf("Motor poles: L=%d, R=%d\n", left, right);
    // 额定电流
    if (zlac_get_current_limits(&left, &right) == RT_EOK)
        rt_kprintf("Rated current: L=%d *0.1A, R=%d *0.1A\n", left, right);
    // 峰值电流
    if (zlac_get_current_peak(&left, &right) == RT_EOK)
        rt_kprintf("Peak current: L=%d *0.1A, R=%d *0.1A\n", left, right);
    // 最大转速
    rt_kprintf("Max speed: %d rpm\n", zlac_get_max_speed());
    // 过载系数
    if (zlac_get_overload_factor(&left, &right) == RT_EOK)
        rt_kprintf("Overload factor: L=%d%%, R=%d%%\n", left, right);
    // 温度保护阈值
    uint16_t temp_l, temp_r, temp_d;
    if (zlac_get_temp_threshold(&temp_l, &temp_r, &temp_d) == RT_EOK)
        rt_kprintf("Temp threshold: L=%d *0.1°C, R=%d *0.1°C, Driver=%d *0.1°C\n",
                   temp_l, temp_r, temp_d); 
    // 初始方向
    rt_kprintf("Init direction: %s\n", zlac_get_init_direction() ? "CCW" : "CW");
    // 工作模式
    ZlacOpMode_t mode = zlac_get_op_mode();
    const char* mode_str = (mode == 1) ? "Position" : ((mode == 3) ? "Velocity" : ((mode == 4) ? "Torque" : "Unknown"));
    rt_kprintf("Operation mode: %s (%d)\n", mode_str, mode);
    // 同步/异步标志
    uint8_t sync[2];
    if (zlac_sdo_read(ZLAC_OD_SYNC_ASYNC, 0, sync, 2) == RT_EOK)
        rt_kprintf("Sync/Async: %s\n", sync[0] ? "Sync" : "Async");
		else
			   rt_kprintf("read ZLAC_OD_SYNC_ASYNC --> Failse\n");	
	// 加速时间		
    if (zlac_get_accel_time(&accel_left, &accel_right) == RT_EOK)
        rt_kprintf("Accel time: L=%d ms, R=%d ms\n", accel_left, accel_right);
    // 减速时间
    if (zlac_get_decel_time(&decel_left, &decel_right) == RT_EOK)
        rt_kprintf("Decel time: L=%d ms, R=%d ms\n", decel_left, decel_right);
    // 起始速度
    if (zlac_get_start_speed(&left, &right) == RT_EOK)
        rt_kprintf("Start speed: L=%d rpm, R=%d rpm\n", left, right);
    // 通讯掉线保护时间
    rt_kprintf("Offline time: %d ms\n", zlac_get_offline_time());	
	// 输出信号状态
    uint16_t out = zlac_get_output_status();
    rt_kprintf("Output status: Y0=%d, Y1=%d, B0=%d, B1=%d\n",
               (out>>0)&1, (out>>1)&1, (out>>2)&1, (out>>3)&1);
	// 抱闸状态 (读取 B0/B1 输出状态)
    rt_kprintf("Brake: B0=%s, B1=%s\n", (out>>2)&1 ? "release" : "engage", (out>>3)&1 ? "release" : "engage");

    rt_kprintf("==============================================\n");
}
MSH_CMD_EXPORT(zlac_show_config, "Show ZLAC8015D configuration parameters");

/* ======================== 统一参数设置命令 ======================== */
static void zlac_set_config(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: zlac_set_config <param> [values...]\n");
        rt_kprintf("Available parameters:\n");
        rt_kprintf("  encoder_lines <left> <right>         - set encoder lines\n");
        rt_kprintf("  motor_poles <left> <right>           - set motor poles\n");
        rt_kprintf("  rated_current <left_*0.1A> <right_*0.1A>   - set rated current (*0.1A)\n");
        rt_kprintf("  peak_current <left_*0.1A> <right_*0.1A>    - set peak current (*0.1A)\n");
        rt_kprintf("  max_speed <rpm>                      - set max speed (rpm)\n");
        rt_kprintf("  overload_factor <left%> <right%>     - set overload factor (%%)\n");
        rt_kprintf("  temp_threshold <left_0p1C> <right_0p1C> <driver_0p1C> - set temp thresholds (0.1°C)\n");
        rt_kprintf("  init_direction <0:CW,1:CCW>          - set initial direction\n");
        rt_kprintf("  op_mode <1:Pos,3:Vel,4:Torque>       - set operation mode\n");
        rt_kprintf("  sync_async <0:Async,1:Sync>          - set sync/async flag\n");
        rt_kprintf("  accel_time <left_ms> <right_ms>      - set accel time (ms)\n");
        rt_kprintf("  decel_time <left_ms> <right_ms>      - set decel time (ms)\n");
        rt_kprintf("  start_speed <left_rpm> <right_rpm>   - set start speed (rpm)\n");
        rt_kprintf("  offline_time <ms>                    - set offline protection time (ms)\n");
				rt_kprintf("  brake_default <1:Engaged,0:Released> - set brake default state on power-up\n");
        rt_kprintf("  save                                 - save all parameters to EEPROM\n");
        return;
    }

    const char *param = argv[1];
    rt_err_t ret = RT_EOK;

    if (rt_strcmp(param, "encoder_lines") == 0) {
        if (argc < 4) { rt_kprintf("Need left and right values\n"); return; }
        uint16_t left = atoi(argv[2]), right = atoi(argv[3]);
        ret = zlac_set_encoder_lines(left, right);
        if (ret == RT_EOK) rt_kprintf("Encoder lines set: L=%d, R=%d\n", left, right);
    }
    else if (rt_strcmp(param, "motor_poles") == 0) {
        if (argc < 4) { rt_kprintf("Need left and right values\n"); return; }
        uint16_t left = atoi(argv[2]), right = atoi(argv[3]);
        ret = zlac_set_motor_poles(left, right);
        if (ret == RT_EOK) rt_kprintf("Motor poles set: L=%d, R=%d\n", left, right);
    }
    else if (rt_strcmp(param, "rated_current") == 0) {
        if (argc < 4) { rt_kprintf("Need left and right values (*0.1A)\n"); return; }
        uint16_t left = atoi(argv[2]), right = atoi(argv[3]);
        ret = zlac_set_rated_current(left, right);
        if (ret == RT_EOK) rt_kprintf("Rated current set: L=%d *0.1A, R=%d *0.1A n", left, right);
    }
    else if (rt_strcmp(param, "peak_current") == 0) {
        if (argc < 4) { rt_kprintf("Need left and right values (*0.1A)\n"); return; }
        uint16_t left = atoi(argv[2]), right = atoi(argv[3]);
        ret = zlac_set_peak_current(left, right);
        if (ret == RT_EOK) rt_kprintf("Peak current set: L=%d *0.1A, R=%d *0.1A\n", left, right);
    }
    else if (rt_strcmp(param, "max_speed") == 0) {
        if (argc < 3) { rt_kprintf("Need rpm value\n"); return; }
        uint16_t rpm = atoi(argv[2]);
        ret = zlac_set_max_speed(rpm);
        if (ret == RT_EOK) rt_kprintf("Max speed set: %d rpm\n", rpm);
    }
    else if (rt_strcmp(param, "overload_factor") == 0) {
        if (argc < 4) { rt_kprintf("Need left and right values (%%)\n"); return; }
        uint16_t left = atoi(argv[2]), right = atoi(argv[3]);
        ret = zlac_set_overload_factor(left, right);
        if (ret == RT_EOK) rt_kprintf("Overload factor set: L=%d%%, R=%d%%\n", left, right);
    }
    else if (rt_strcmp(param, "temp_threshold") == 0) {
        if (argc < 5) { rt_kprintf("Need left, right, driver values (0.1°C)\n"); return; }
        uint16_t left = atoi(argv[2]), right = atoi(argv[3]), driver = atoi(argv[4]);
        ret = zlac_set_temp_threshold(left, right, driver);
        if (ret == RT_EOK) rt_kprintf("Temp thresholds set: L=%.1f°C, R=%.1f°C, Driver=%.1f°C\n",
                                      left/10.0, right/10.0, driver/10.0);
    }
    else if (rt_strcmp(param, "init_direction") == 0) {
        if (argc < 3) { rt_kprintf("Need direction (0=CW,1=CCW)\n"); return; }
        uint16_t dir = atoi(argv[2]);
        ret = zlac_set_init_direction(dir);
        if (ret == RT_EOK) rt_kprintf("Init direction set: %s\n", dir ? "CCW" : "CW");
    }
    else if (rt_strcmp(param, "op_mode") == 0) {
        if (argc < 3) { rt_kprintf("Need mode (1=Position,3=Velocity,4=Torque)\n"); return; }
        uint8_t mode = atoi(argv[2]);
        if (mode != 1 && mode != 3 && mode != 4) { rt_kprintf("Invalid mode\n"); return; }
        ret = zlac_set_op_mode((ZlacOpMode_t)mode);
        if (ret == RT_EOK) rt_kprintf("Operation mode set to %d\n", mode);
    }
    else if (rt_strcmp(param, "sync_async") == 0) {
        if (argc < 3) { rt_kprintf("Need value (0=Async,1=Sync)\n"); return; }
        uint8_t sync = atoi(argv[2]);
				uint8_t data[2]={0,0};
				data[0] = sync ;
				ret = zlac_sdo_write(ZLAC_OD_SYNC_ASYNC, 0, data, 2);
        if (ret == RT_EOK) rt_kprintf("Sync/Async set to %s\n", sync ? "Sync" : "Async");
				else rt_kprintf("write ZLAC_OD_SYNC_ASYNC --> Failse\n");	
    }
    else if (rt_strcmp(param, "accel_time") == 0) {
        if (argc < 4) { rt_kprintf("Need left and right values (ms)\n"); return; }
        uint32_t left = atoi(argv[2]), right = atoi(argv[3]);
        ret = zlac_set_accel_time(left, right);
        if (ret == RT_EOK) rt_kprintf("Accel time set: L=%d ms, R=%d ms\n", left, right);
    }
    else if (rt_strcmp(param, "decel_time") == 0) {
        if (argc < 4) { rt_kprintf("Need left and right values (ms)\n"); return; }
        uint32_t left = atoi(argv[2]), right = atoi(argv[3]);
        ret = zlac_set_decel_time(left, right);
        if (ret == RT_EOK) rt_kprintf("Decel time set: L=%d ms, R=%d ms\n", left, right);
    }
    else if (rt_strcmp(param, "start_speed") == 0) {
        if (argc < 4) { rt_kprintf("Need left and right values (rpm)\n"); return; }
        uint16_t left = atoi(argv[2]), right = atoi(argv[3]);
        ret = zlac_set_start_speed(left, right);
        if (ret == RT_EOK) rt_kprintf("Start speed set: L=%d rpm, R=%d rpm\n", left, right);
    }
    else if (rt_strcmp(param, "offline_time") == 0) {
        if (argc < 3) { rt_kprintf("Need time (ms)\n"); return; }
        uint16_t ms = atoi(argv[2]);
        ret = zlac_set_offline_time(ms);
        if (ret == RT_EOK) rt_kprintf("Offline time set: %d ms\n", ms);
    }
		else if (rt_strcmp(param, "save") == 0) {
						ret = zlac_save_parameters();
						if (ret == RT_EOK) rt_kprintf("Parameters saved to EEPROM.\n");
				}
		else if (rt_strcmp(param, "brake_default") == 0) {
				if (argc < 3) { rt_kprintf("Need value (1=Engaged,0=Released)\n"); return; }
				uint16_t val = atoi(argv[2]);
				if (val != 0 && val != 1) { rt_kprintf("Invalid value. Use 0 or 1.\n"); return; }
				if(val){
					zlac_brake_engage();
				}else{
					zlac_brake_release();
				}
				uint16_t left,right;
				zlac_get_brake(&left,&right);
				rt_kprintf("Brake: B0=%s, B1=%s\n", left ? "engage":"release" , right ? "engage":"release");			
					// 输出信号状态
				uint16_t out = zlac_get_output_status();
				rt_kprintf("Output status: Y0=%d, Y1=%d, B0=%d, B1=%d\n",
									 (out>>0)&1, (out>>1)&1, (out>>2)&1, (out>>3)&1);
		}
    else {
        rt_kprintf("Unknown parameter: %s\n", param);
        rt_kprintf("Use 'zlac_set_config' without arguments to see help.\n");
        return;
    }

    if (ret != RT_EOK) {
        rt_kprintf("Failed to set parameter (err=%d)\n", ret);
    }
}
MSH_CMD_EXPORT(zlac_set_config, "Unified parameter setting command");


/* 2. 打印 PID 参数与平滑系数 */
static void zlac_show_pid(void)
{
    uint16_t left, right;

    rt_kprintf("\n========== ZLAC8015D PID & Smoothing ==========\n");

    // 速度环 Kp
    if (zlac_get_velocity_pid_kp(&left, &right) == RT_EOK)
        rt_kprintf("Velocity Kp: L=%d, R=%d\n", left, right);
    // 速度环 Ki
    if (zlac_get_velocity_pid_ki(&left, &right) == RT_EOK)
        rt_kprintf("Velocity Ki: L=%d, R=%d\n", left, right);
    // 速度环 Kf
    if (zlac_get_velocity_pid_kf(&left, &right) == RT_EOK)
        rt_kprintf("Velocity Kf: L=%d, R=%d\n", left, right);
    // 位置环 Kp
    if (zlac_get_position_kp(&left, &right) == RT_EOK)
        rt_kprintf("Position Kp: L=%d, R=%d\n", left, right);
    // 位置环 Kf
    if (zlac_get_position_kf(&left, &right) == RT_EOK)
        rt_kprintf("Position Kf: L=%d, R=%d\n", left, right);
    // 电流环 Kp (厂家参数)
    if (get_left_right_param(ZLAC_OD_CUR_KP, &left, &right) == RT_EOK)
        rt_kprintf("Current Kp: L=%d, R=%d\n", left, right);
    // 电流环 Ki
    if (get_left_right_param(ZLAC_OD_CUR_KI, &left, &right) == RT_EOK)
        rt_kprintf("Current Ki: L=%d, R=%d\n", left, right);
    // 速度平滑系数 
    if (zlac_get_vel_smooth(&left, &right) == RT_EOK)
        rt_kprintf("Vel smooth: L=%d, R=%d\n", left, right);
    // 前馈平滑系数
    if (zlac_get_ff_smooth(&left, &right) == RT_EOK)
        rt_kprintf("FF smooth: L=%d, R=%d\n", left, right);
    // 转矩平滑系数 (201Ch)
    if (get_left_right_param(ZLAC_OD_TORQUE_SMOOTH, &left, &right) == RT_EOK)
        rt_kprintf("Torque smooth: L=%d, R=%d\n", left, right);
 
    rt_kprintf("==============================================\n");
}
MSH_CMD_EXPORT(zlac_show_pid, "Show ZLAC8015D PID and smoothing parameters");

/* ======================== 统一 PID 与平滑系数设置命令 ======================== */
static void zlac_set_pid(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: zlac_set_pid <param> [values...]\n");
        rt_kprintf("Available parameters:\n");
        rt_kprintf("  velocity_kp <left> <right>       - set velocity Kp\n");
        rt_kprintf("  velocity_ki <left> <right>       - set velocity Ki\n");
        rt_kprintf("  velocity_kf <left> <right>       - set velocity Kf\n");
        rt_kprintf("  position_kp <left> <right>       - set position Kp\n");
        rt_kprintf("  position_kf <left> <right>       - set position Kf\n");
        rt_kprintf("  current_kp <left> <right>        - set current Kp\n");
        rt_kprintf("  current_ki <left> <right>        - set current Ki\n");
        rt_kprintf("  vel_smooth <left> <right>        - set velocity smooth coefficient\n");
        rt_kprintf("  ff_smooth <left> <right>         - set feedforward smooth coefficient\n");
        rt_kprintf("  torque_smooth <left> <right>     - set torque smooth coefficient\n");
        return;
    }

    const char *param = argv[1];
    rt_err_t ret = RT_EOK;
    uint16_t left, right;

    if (argc < 4) {
        rt_kprintf("Need left and right values\n");
        return;
    }
    left = atoi(argv[2]);
    right = atoi(argv[3]);

    if (rt_strcmp(param, "velocity_kp") == 0) {
		ret = zlac_set_velocity_kp(left, right);
        if (ret == RT_EOK) rt_kprintf("Velocity Kp set: L=%d, R=%d\n", left, right);
    }
    else if (rt_strcmp(param, "velocity_ki") == 0) {
		ret = zlac_set_velocity_ki(left, right);
        if (ret == RT_EOK) rt_kprintf("Velocity Ki set: L=%d, R=%d\n", left, right);
    }
    else if (rt_strcmp(param, "velocity_kf") == 0) {
		ret = zlac_set_velocity_kf(left, right);
        if (ret == RT_EOK) rt_kprintf("Velocity Kf set: L=%d, R=%d\n", left, right);
    }
    else if (rt_strcmp(param, "position_kp") == 0) {
        ret = zlac_set_position_kp(left, right);
        if (ret == RT_EOK) rt_kprintf("Position Kp set: L=%d, R=%d\n", left, right);
    }
    else if (rt_strcmp(param, "position_kf") == 0) {
        ret = zlac_set_position_kf(left, right);
        if (ret == RT_EOK) rt_kprintf("Position Kf set: L=%d, R=%d\n", left, right);
    }
    else if (rt_strcmp(param, "current_kp") == 0) {
        ret = set_left_right_param(ZLAC_OD_CUR_KP, left, right);
        if (ret == RT_EOK) rt_kprintf("Current Kp set: L=%d, R=%d\n", left, right);
    }
    else if (rt_strcmp(param, "current_ki") == 0) {
        ret = set_left_right_param(ZLAC_OD_CUR_KI, left, right);
        if (ret == RT_EOK) rt_kprintf("Current Ki set: L=%d, R=%d\n", left, right);
    }
    else if (rt_strcmp(param, "vel_smooth") == 0) {
        ret = zlac_set_vel_smooth(left, right);
        if (ret == RT_EOK) rt_kprintf("Velocity smooth set: L=%d, R=%d\n", left, right);
    }
    else if (rt_strcmp(param, "ff_smooth") == 0) {
        ret = zlac_set_ff_smooth(left, right);
        if (ret == RT_EOK) rt_kprintf("Feedforward smooth set: L=%d, R=%d\n", left, right);
    }
    else if (rt_strcmp(param, "torque_smooth") == 0) {
        ret = set_left_right_param(ZLAC_OD_TORQUE_SMOOTH, left, right);
        if (ret == RT_EOK) rt_kprintf("Torque smooth set: L=%d, R=%d\n", left, right);
    }
    else {
        rt_kprintf("Unknown parameter: %s\n", param);
        rt_kprintf("Use 'zlac_set_pid' without arguments to see help.\n");
        return;
    }

    if (ret != RT_EOK) {
        rt_kprintf("Failed to set parameter (err=%d)\n", ret);
    }
}
MSH_CMD_EXPORT(zlac_set_pid, "Unified PID and smoothing parameters setting command");

/* 3. 打印实时监控数据 (电压、电流、转速、位置、温度、状态) */
static void show_monitor_once(void)
{
    int16_t vel_l, vel_r;
    int32_t pos_l, pos_r;
    int16_t cur_l, cur_r;   // 实际转矩反馈 6077h:01/02 单位 0.1A
    uint16_t temp_l, temp_r, temp_d;
    uint16_t motor_status_l, motor_status_r;
    uint16_t fault_code_low, fault_code_high;

    rt_kprintf("\n========== ZLAC8015D Real-time Monitor ==========\n");
		if(zlac_is_online()){
			rt_kprintf("ZLAC8015D device online\n");
		}else{
			rt_kprintf("ZLAC8015D device offline\n");
		}
    // 母线电压
	uint16_t voltage = zlac_get_bus_voltage();
	rt_kprintf("Bus voltage: %d.%02dV\n", voltage/100, voltage%100);

    // 实际速度 (单位 0.1 rpm，需转换)
    zlac_get_velocity(&vel_l, &vel_r);
    rt_kprintf("Actual velocity: L=%d.%d rpm, R=%d.%d rpm\n", vel_l/10, abs(vel_l%10), vel_r/10, abs(vel_r%10));

    // 实际位置
    zlac_get_position(&pos_l, &pos_r);
    rt_kprintf("Actual position: L=%ld pulses, R=%ld pulses\n", pos_l, pos_r);

    // 实际转矩 (6077h:01/02) 单位 0.1A，需读取
    if (get_left_right_param(ZLAC_OD_ACTUAL_TORQUE, (uint16_t*)&cur_l, (uint16_t*)&cur_r) == RT_EOK) {
        rt_kprintf("Actual torque: L=%d.%d A, R=%d.%d A\n", cur_l/10, abs(cur_l%10), cur_r/10, abs(cur_r%10));
    }

    // 温度
    temp_l = zlac_get_motor_temp_left();
    temp_r = zlac_get_motor_temp_right();
    temp_d = zlac_get_driver_temp();
    rt_kprintf("Temperature: L=%d *0.1°C, R=%d *0.1°C, Driver=%d *0.1°C\n", temp_l, temp_r, temp_d);

    // 电机运行状态 (2033h:01/02, 0=静止, 1=运行)
    motor_status_l = zlac_get_motor_status_left();
    motor_status_r = zlac_get_motor_status_right();
    rt_kprintf("Motor running: L=%s, R=%s\n", motor_status_l ? "RUN" : "STOP", motor_status_r ? "RUN" : "STOP");

    // 故障码
    uint32_t fault = zlac_get_fault_code();
		zlac_print_fault_code(fault);

    // 输入信号状态 (2003h) 可选
    uint16_t input = 0;
    if (zlac_sdo_read(ZLAC_OD_INPUT_STATUS, 0, (uint8_t*)&input, 2) == RT_EOK)
        rt_kprintf("Input status: X0=%d, X1=%d\n", (input>>0)&1, (input>>1)&1);

    // 输出信号状态 (含抱闸)
		uint16_t left,right;
		zlac_get_brake(&left,&right);
		rt_kprintf("Brake: B0=%s, B1=%s\n", left ? "engage":"release" , right ? "engage":"release");			
			// 输出信号状态
		uint16_t out = zlac_get_output_status();
		rt_kprintf("Output status: Y0=%d, Y1=%d, B0=%d, B1=%d\n",
							 (out>>0)&1, (out>>1)&1, (out>>2)&1, (out>>3)&1);

    // 状态字 (6041h) 低16位
    rt_kprintf("Left Motor State: %d\n", zlac_get_state(zlac_get_left_statusword()));
    rt_kprintf("Right Motor State: %d\n", zlac_get_state(zlac_get_right_statusword()));
		uint32_t status = read_full_statusword();
    rt_kprintf("Full status: 0x%08X \n", status);		

    rt_kprintf("================================================\n");
}

static void zlac_monitor_print_thread_entry(void *param)
{
    while (s_monitor_print_run) {
        show_monitor_once();
        rt_thread_mdelay(500);
    }
}

static void zlac_show_monitor(int argc, char **argv)
{
    if (argc == 1) {
        show_monitor_once();
    } else if (argc >= 2) {
        if (rt_strcmp(argv[1], "on") == 0) {
            if (s_monitor_print_run) {
                rt_kprintf("Monitor periodic printing already running.\n");
                return;
            }
            s_monitor_print_run = RT_TRUE;
            s_monitor_print_thread = rt_thread_create("zlac_monitor_print",
                                                      zlac_monitor_print_thread_entry,
                                                      RT_NULL,
                                                      2048,
                                                      25,
                                                      10);
            if (s_monitor_print_thread != RT_NULL) {
                rt_thread_startup(s_monitor_print_thread);
                rt_kprintf("Monitor periodic printing started (interval 500ms).\n");
            } else {
                s_monitor_print_run = RT_FALSE;
                rt_kprintf("Failed to create monitor print thread.\n");
            }
        } else if (rt_strcmp(argv[1], "off") == 0) {
            if (s_monitor_print_run) {
                s_monitor_print_run = RT_FALSE;
                if (s_monitor_print_thread) {
                    rt_thread_delete(s_monitor_print_thread);
                    s_monitor_print_thread = RT_NULL;
                }
                rt_kprintf("Monitor periodic printing stopped.\n");
            } else {
                rt_kprintf("Monitor periodic printing not active.\n");
            }
        } else {
            rt_kprintf("Usage: zlac_show_monitor [on|off]\n");
        }
    }
}
MSH_CMD_EXPORT(zlac_show_monitor, "Show ZLAC8015D real-time monitoring data");

static void zlac_test_recover(int argc, char **argv)
{
    zlac_recover_communication();
    rt_kprintf("Communication recovered. Please re-enable or adjust state manually.\n");
}
MSH_CMD_EXPORT(zlac_test_recover, "recover CAN communication without changing motor state");

#endif /* RT_USING_MSH */

