/**
 ******************************************************************************
 * @file    dx2002_at.h
 * @brief   DX2002 Bluetooth module AT command interface
 ******************************************************************************
 * Copyright (c) 2023 
 * Licensed under Apache-2.0
 ******************************************************************************
 */

/**************** 注意事项 ****************************
1. 啥也不输入，重启进入透明传输模式。
2. 模块上电未连接时即为AT指令模式，
3. 上电 模块会自动发送 IM_READY ，
 如果蓝牙模块被设备(app)连接上,蓝牙模块会发送一条 IM_CONN:<ID> 字符串，
 如果设备(app)断开蓝牙模块连接，蓝牙模块会打印 IM_DISC:<ID> 表示设备断开连接。
4. 蓝牙被设备(app)连接上会进入透传模式,所有数据都会被透传，此时发送 AT 指令无效。
**************** AT指令格式 ****************************
1、AT 指令，属于字符行指令，按行解析 (即发 AT 指令时必须以回车换行或者 \r\n 、16进制为 0D0A 结尾)
2、AT 指令为大写，指令前缀为 AT+ ，可分为参数设置指令和读取指令。
3、设置指令格式：AT+<CMD>=<PARAM>，操作成功返回：\r\nOK\r\n。
4、读取指令格式：AT+<CMD>，操作成功返回 \r\n+<CMD>=<PARAM>\r\n 。
5、发送错误指令会返回 ERR:<ID> ；
	ID ：1，结尾没加回车换行（\r\n,hex:0d0a）；2，参数错误；3，命令错误；4，权限错误；
**************** AT指令集 ****************************
AT指令序号 	指令 		说明
1 			AT+CMD 		测试指令
2 			AT+VERSION 	查询—软件版本号
3 			AT+NAME 	查询\设置—设备名称
4 			AT+NAMB 	查询\设置— BLE 名称(双模情况下)
5 			AT+LADDR 	查询— BLE 模块蓝牙地址   BLE=Bluetooth low energy 低功耗蓝牙协议
6 			AT+ADDR 	查询— SPP 模块蓝牙地址   SPP=Serial Port Profile 蓝牙串口协议 
7 			AT+NAMEC 	查询\设置—蓝牙名称后缀 MAC
8 			AT+BAUD 	查询\设置—串口波特率
9 			AT+STOP 	查询\设置—串口停止位
10 			AT+PARI 	查询\设置—串口校验位
11 			AT+NOTI 	查询\设置—事件通知
12 			AT+UUID 	查询\设置—服务 SERVICE UUID
13 			AT+CHAR 	查询\设置—通知 NOTIFY UUID
14 			AT+WRITE 	查询\设置—写入 WRITE UUID
15 			AT+PWRM 	查询\设置—低功耗模式
16 			AT+AST 		查询\设置空闲进入低功耗时间
17 			AT+AUST 	查询\设置空闲进入深睡眠时间
18 			AT+ADVI 	查询\设置—广播时间间隔
19 			AT+POWE 	查询\设置—模块发射功率
20 			AT+TYPE 	查询\设置—蓝牙设备类型
21 			AT+RESET 	软件重启
22 			AT+DEFAULT 	恢复出厂设置
23 			AT+MASTER 	查询\设置主从模式
24 			AT+CONN 	查询\设置蓝牙连接
25 			AT+DSCET=1 	断开连接
26 			AT+SETSCANP 查询\设置—搜索参数
27 			AT+PINCODE 	查询\设置 PINCODE
28 			AT+LED 		查询\设置 LED 状态
29 			AT+IO 		查询\设置—可配置 IO
******************************************************************************/

#ifndef _DX2002_AT_H_
#define _DX2002_AT_H_

#include <stdint.h>
#include <stdbool.h>
#include "mx_common.h"   // for mx_status, kNoErr, etc.

#ifdef __cplusplus
extern "C" {
#endif

/** \addtogroup module */
/** @{*/

#define ENTERS_AT_MODE 0
#define ENTERS_TRANSPARENT_MODE 1 
/**
 *  ble service product information
 */

#define BLE_DEVICE_NAME_MAXLEN (20 + 12 + 1)
#define BLE_DEVICE_SECRET_MAXLEN (64 + 1)
#define BLE_DEVICE_VERSION_MAXLEN (20 + 1)
#define BLE_DEVICE_MAC_MAXLEN (12 + 1)
#define BLE_UUID_MAXLEN (4+1)

typedef struct {
	const char* type; /**< Reference to product TRD document */
	const char* category; /**< Reference to product TRD document */
	const char* manufacture; /**< Reference to product TRD document */   
} ble_dev_info_t;

/*============================================================================
 *                           Module control
 *============================================================================*/

/**
 * @brief Hardware reboot DX2002 module (pull reset pin low then high)
 */
void ble_module_Hard_reboot(void);

/**
 * @brief Check if module is ready (AT+CMD returns OK)
 * @return kNoErr if ready, otherwise error
 */
mx_status at_module_check_ready(void);

/**
 * @brief Wait for module to enter AT mode (with retries and reboot)
 * @return 0 on success, 1 on failure
 */
uint8_t waitReady(void);

/*============================================================================
 *                           Basic AT commands
 *============================================================================*/

/**
 * @brief Get firmware version string
 * @return pointer to version string (e.g., "2.0.8-1") or NULL
 */
const char* at_module_get_fw_version(void);

/**
 * @brief Get current BLE name (NAMB)
 * @return pointer to name string
 */
const char* at_module_get_name(void);

/**
 * @brief Set BLE name (NAMB) – module reboots
 * @param name new name (max 20 bytes)
 * @return kNoErr on success
 */
mx_status at_module_set_NAMB(const char *name);

/**
 * @brief Get BLE MAC address
 * @return pointer to MAC string (12 hex digits)
 */
const char* at_module_get_addr(void);

/**
 * @brief Get name suffix MAC mode (0=none, 1=12bit, 2=6bit)
 */
uint8_t at_module_get_name_MAC(void);

/**
 * @brief Set name suffix MAC mode – module reboots
 * @param param 0,1,2
 */
mx_status at_module_set_name_MAC(uint8_t param);

/**
 * @brief Get current UART baud rate (as integer value, e.g., 115200)
 */
uint32_t at_module_get_UART_BAUD(void);

/**
 * @brief Set UART baud rate by index (1-9) – module reboots
 * @param num 1:9600, 2:19200, ..., 5:115200, ..., 9:921600
 */
mx_status at_module_set_UART_BAUD(uint8_t num);

/**
 * @brief Get notification enable (0=disable, 1=enable)
 */
uint8_t at_module_get_notify(void);

/**
 * @brief Set notification enable – module reboots
 */
mx_status at_module_set_notify(uint8_t param);

/**
 * @brief Get SERVICE UUID (e.g., "FFE0")
 */
const char* at_module_get_SERVICE_UUID(void);

/**
 * @brief Get NOTIFY UUID (e.g., "FFE2")
 */
const char* at_module_get_NOTIFY_UUID(void);

/**
 * @brief Get WRITE UUID (e.g., "FFE1")
 */
const char* at_module_get_WRITE_UUID(void);

/**
 * @brief Get low power mode (0=auto, 1=normal)
 */
uint8_t at_module_get_PWRM(void);

/**
 * @brief Set low power mode – module reboots
 */
mx_status at_module_set_PWRM(uint8_t param);

/**
 * @brief Get idle time to enter low power (seconds)
 */
uint8_t at_module_get_AST(void);

/**
 * @brief Set idle low power time (1-200 seconds)
 */
mx_status at_module_set_AST(uint8_t time);

/**
 * @brief Get deep sleep idle time (0=off, 5-200 seconds)
 */
const char* at_module_get_AUST(void);   // returns string, can be parsed

/**
 * @brief Set deep sleep idle time
 */
mx_status at_module_set_AUST(uint8_t time);

/**
 * @brief Get advertising interval (ms, 30-7000)
 */
uint32_t at_module_get_ADVI(void);

/**
 * @brief Set advertising interval – module reboots
 */
mx_status at_module_set_ADVI(uint32_t time);

/**
 * @brief Get transmit power level (0-9, 9=+6.4dB)
 */
uint8_t at_module_get_POWE(void);

/**
 * @brief Set transmit power – module reboots
 */
mx_status at_module_set_POWE(uint8_t power);

/**
 * @brief Get device type (0=health, 1=mouse, 2=keyboard)
 */
uint8_t at_module_get_TYPE(void);

/**
 * @brief Set device type – module reboots
 */
mx_status at_module_set_TYPE(uint8_t param);

/**
 * @brief Software reset
 */
mx_status at_module_RESET(void);

/**
 * @brief Factory default – module reboots
 */
mx_status at_module_DEFAULT(void);

/*============================================================================
 *                     Master/Slave and connection commands
 *============================================================================*/

/**
 * @brief Get master/slave mode (0=slave, 1=master)
 * @return pointer to string "0" or "1"
 */
uint8_t at_module_get_MASTER(void);

/**
 * @brief Set master/slave mode – module reboots
 */
mx_status at_module_set_MASTER(uint8_t param);

/**
 * @brief Get stored connection MAC address
 * @return pointer to 12-digit hex string
 */
const char* at_module_get_CONN(void);

/**
 * @brief Set connection MAC address (auto-connect)
 * @param mac 12-digit hex string (e.g., "112233445566")
 */
mx_status at_module_set_CONN(char *mac);

/**
 * @brief Disconnect current Bluetooth link and clear stored MAC
 */
mx_status at_module_DSCET(void);

/**
 * @brief Get scan parameters (interval, window)
 * @return string like "160,80"
 */
const char* at_module_get_SETSCANP(void);

/**
 * @brief Set scan parameters (interval, window in 0.625ms units)
 */
mx_status at_module_set_SETSCANP(uint32_t interval, uint32_t window);

/**
 * @brief Get PIN code for classic Bluetooth pairing
 */
const char* at_module_get_PINCODE(void);

/**
 * @brief Set PIN code – module reboots
 */
mx_status at_module_set_PINCODE(char pin[5]);
mx_status at_module_set_SETSCANP(uint32_t interval, uint32_t window);

/*============================================================================
 *                           Data transmission (raw)
 *============================================================================*/
mx_status ble_module_init(void);

/**
 * @brief Send raw data (already in transparent mode)
 * @param data pointer to data buffer
 * @param size number of bytes to send
 * @return number of bytes sent, or -1 on error
 */
// int at_module_send_raw(const char *data, int size);

/**
 * @brief Receive raw data (non-blocking check)
 * @param data buffer
 * @param size max bytes to read
 * @return number of bytes read, or -1 on timeout/error
 */
// int at_module_recv_raw(char *data, int size);

/*============================================================================
 *                        AT mode / transparent mode switch
 *============================================================================*/

/**
 * @brief Set parser mode (1=AT command mode, 0=transparent mode)
 * This is used internally by ATCmdParser; you usually don't call it directly.
 */
// void at_set_parser_mode(char mode);

#ifdef __cplusplus
}
#endif

#endif /* _DX2002_AT_H_ */

