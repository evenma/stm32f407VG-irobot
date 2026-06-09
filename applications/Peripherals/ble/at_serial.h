/**
 ******************************************************************************
 * @file    at_serial.h
 * @author  ele
 * @version V1.0.0
 * @date    3-Sept-2023
 * @brief   MCU peripheral driver PI header file
 ******************************************************************************
 *
 * Copyright (c) 2019-2028 QiangYing Co.,Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

#ifndef _AT_SERIAL_H_
#define _AT_SERIAL_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 *                                   DX2002 BLE蓝牙模块串口通讯
蓝牙协议 	Bluetooth Specification V5.1 BLE
工作频率 	2.4GHz ISM band
通信接口 	UART
供电电源 	3.3V
天线 		可以选择PCB板载天线、或外接DB天线(默认为PCB天线)
通信距离 	80M (空旷环境)
外观尺寸 	19.6(L)mm x 13.0 (W)mm x 2.1(H) mm
蓝牙认证 	FCC CE ROHS REACH
蓝牙名称 	DX2002
串口参数 	115200、8数据位、1停止位、无校验、无流控
空中升级 	支持
工作温度 	MIN:-20℃ - MAX:+70℃
广播间隔 	500ms，
发射功率 	-15.7~+6.4 dB
灵敏度 		-94 dB

透传数据吞吐量：
	上行：UART ->DX2002 -> Android/ios
	下行：Android/ios ->DX2002 -> UART
速率 9600  19200 38400 57600 115200 230400 460800 921600
android
 上行 960   1920  3840  5760  11520  17000  17000  17000
 下行 961   1920  3773  5714  10526  19607  21276  23255 
ios
 上行 960   1920  3840  5760   8259   8259   8259   8259
 下行 960   1920  3773  5714   8778   8778   8778   8778
Android MTU=512dB || IOS MTU=185dB 蓝牙官网的默认值是672  MTU为单个数据包最大传输单元 

引脚功能
 PB2 串口数据输入 RX/可定制IO
 PB3 串口数据输出 TX/可定制IO
 PB4 可定制IO口 默认为复位IO 低电平有效 
 
 4 脚(PB_4): 连接状态指示脚（注：默认为复位脚，需使用 AT+LED=1 来切换该功能）
 
 ******************************************************************************/
#define AT_SERIAL_RB_BUFSZ 512

/******************************************************************************
 *                                   Macros
 ******************************************************************************/



/******************************************************************************
 *                                Serial Function
 *                      Serial port driver used for AT parser
 ******************************************************************************/

/**
 * 
 * @brief 	    Initialises AT instruction port
 * 
 * param[in]    timeout: timeout in milisecond
 * 
 * @return	    status
 *
 */
void at_serial_init(int timeout);

/**
 * 
 * @brief 	    at instruction port receiving timeout time
 * 
 * param[in]    timeout : timeout in milisecond
 * 
 * @return	    status
 *
 */
void at_serial_set_timeout(int timeout);

/**
 * 
 * @brief 	    at instruction port output
 * 
 * param[in]    c : Output one byte
 * 
 * @return	    status
 *
 */
int at_serial_putc(char c);

/**
 * 
 * @brief 	    at instruction port input
 * 
 * @return	    byte
 *
 */
int at_serial_getc(void);

/**
 * 
 * @brief 	    at instruction port status
 * 
 * @return	    status
 *
 */
bool at_serial_readable(void);

/**
 * 
 * @brief  Enable the UART Parity Error interrupt and Data Register Not Empty interrupt
 * 
 * @return	    none
 *
 */
//void application_start( void* parameter );
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
