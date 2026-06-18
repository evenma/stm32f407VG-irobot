/**
 ******************************************************************************
 * @file    emh_module.c
 * @author  ele.ma
 * @version V1.0.0
 * @date    2023-2-10
 * @brief   DX2002 module operation AT commands
 ******************************************************************************
 *
 * Copyright (c) 2021-2030 Zhejiang Qiangying Technology Co., LTD
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
 
#include "ATCmdParser.h"
#include "dx2002_at.h"
#include "rtthread.h"
#include <rtdevice.h>
#include <board.h>
#include "mx_debug.h"

#define LOG_TAG "at_module"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>

/******************************************************************************
 *                              Variable Definitions
 ******************************************************************************/
#ifndef BLE_RESET_PIN
	#define BLE_nRST    GET_PIN(D,1)
#endif

#define BLE_BAUD	115200    // 5:115200
static const char _BLE_NAME[10]= "QYRobotS";   // 自定义从机名称
static char _ble_set_name[10];     
static char _ble_name[BLE_DEVICE_NAME_MAXLEN];
static char _fw_version[BLE_DEVICE_VERSION_MAXLEN];
static char _ble_mac[BLE_DEVICE_MAC_MAXLEN];
static char _ble_service_uuid[BLE_UUID_MAXLEN];
static char _ble_notify_uuid[BLE_UUID_MAXLEN];
static char _ble_write_uuid[BLE_UUID_MAXLEN];



/******************************************************************************
 *                             Function Declarations
 ******************************************************************************/

extern void ble_event_handler(void);
rt_uint8_t waitReady(void);
mx_status ble_module_init(void);

/******************************************************************************
 *                              Function Definitions
 ******************************************************************************/
 
static int BLEChangeName(int argc, char **argv)
{
    int result = 0;
    struct rt_device_pwm *device = RT_NULL;
	

		LOG_I("ble change name");
    if (argc > 2)
    {
			rt_kprintf("BLEChangeName [<name>], name size < 10 or null");
        result = -RT_ERROR;
        goto _exit;
    }
		if(strlen(argv[1]) > 10){
				rt_kprintf("name size need < 10 or null");
        result = -RT_ERROR;
        goto _exit;
		}
		rt_memset(_ble_set_name,0x00,10);
		if(strlen(argv[1]) > 3)
			strcpy(_ble_set_name, argv[1]);
		rt_kprintf("ble set name :",_ble_set_name);
		
	  result =	ble_module_init();

_exit:
    return result;
}
MSH_CMD_EXPORT(BLEChangeName, BLEChangeName [<name>]);
 
 
 
void ble_module_Hard_reboot(void)
{
	rt_pin_mode(BLE_nRST, PIN_MODE_OUTPUT);
	rt_pin_write(BLE_nRST, PIN_LOW);
	rt_thread_delay(500);
	rt_pin_write(BLE_nRST, PIN_HIGH);
	rt_thread_delay(200);  // 等待模块稳定
  rt_pin_mode(BLE_nRST, PIN_MODE_INPUT_PULLUP);
}

mx_status ble_module_init(void)
{
	mx_status err = kNoErr;
	uint8_t cmd = 0;

restart:	
	LOG_D("BLE init");
//	ATCmdParser_set_mode(1);
// at mode
	if(waitReady()){
			LOG_E(" AT MODE FAIL!");	
			err = kGeneralErr;
//		ATCmdParser_set_mode(0);
		return err;
	}
	LOG_D("ENTER AT MODE!");			
	if(at_module_get_fw_version() != RT_NULL){
		LOG_D("version:%s",_fw_version);
	}
	
	if(at_module_get_name() != RT_NULL){
		LOG_D("ble name:%s",_ble_name);	
		LOG_D("set name:%s",_ble_set_name);	
		LOG_D("factory name:%s",_BLE_NAME);
		if(_ble_set_name[0] != 0){  // priority
			if(at_module_set_NAMB(_ble_set_name) == kNoErr){
					LOG_D("Success change name by set name");	
					rt_thread_delay(500);		
					if(at_module_get_name() != RT_NULL){
						LOG_D("name:%s",_ble_name);
					}
			}
		}else{
			if (strncmp(_ble_name, _BLE_NAME, strlen(_BLE_NAME)) != 0){
				if(at_module_set_NAMB(_BLE_NAME) == kNoErr){
						LOG_D("Success change name by factory name");	
						rt_thread_delay(500);		
						if(at_module_get_name() != RT_NULL){
							LOG_D("name:%s",_ble_name);
						}
				}else{
						LOG_E(" SET BLE NAME FAIL!");	
						err = kGeneralErr;
//					ATCmdParser_set_mode(0);
						return err;					
				}
			}else{
				LOG_D("ble name already ,no change");
			}
		}
	}
	
	if(at_module_get_addr() != RT_NULL){
		LOG_D("mac:%s",_ble_mac);
	}
	rt_thread_delay(100);
	cmd = at_module_get_name_MAC();	
	switch(cmd)
	{
		case 0: LOG_D("The name does not have MAC");break;
		case 1: LOG_D("The name have 12 bits MAC");break;
		case 2: LOG_D("The name have 6 bits MAC");break;
		default:break;
	}
	rt_thread_delay(100);
	if(cmd!=2){
		at_module_set_name_MAC(2);
			// module auto reboot ,wait IM_READY
		rt_thread_delay(500);
			if(at_module_get_name() != RT_NULL){
				LOG_D("name:%s",_ble_name);
		}	
	}
	
	if(at_module_get_UART_BAUD() != BLE_BAUD){
		LOG_D("SET BAUD 5:115200");
		at_module_set_UART_BAUD(5);
		// module auto reboot ,wait IM_READY
		rt_thread_delay(500);
	}
	
//	cmd = at_module_get_notify(); //oxff不是数字
//	if(cmd){
//		LOG_D("Notifies the connection disconnection event");
//	}else{
//		LOG_D("Disconnection events are not notified");	
//	}
//	rt_thread_delay(100);
	
	LOG_D("service uuid: %s",at_module_get_SERVICE_UUID());
	rt_thread_delay(100);
	LOG_D("notify uuid: %s",at_module_get_NOTIFY_UUID());
	rt_thread_delay(100);
	LOG_D("write uuid: %s",at_module_get_WRITE_UUID());
	rt_thread_delay(100);
	if(at_module_get_PWRM()){
		LOG_D("Normal operating mode");
	}else{
		LOG_D("Automatic low power mode");	
	}
	rt_thread_delay(100);
	LOG_D("Idle Indicates the low power consumption time: %d S",at_module_get_AST());
	rt_thread_delay(100);
	LOG_D("Module broadcast interval time: %d mS",at_module_get_ADVI());
	rt_thread_delay(100);
//	LOG_D("Module transmitting power: %d ,MAX 9:+6.4 dB",at_module_get_POWE()); // 9.9小数不是整数
//	rt_thread_delay(100);
//	LOG_D("Bluetooth device type: %d 0:Health class",at_module_get_TYPE());  // 0.0小数不是整数
	
	LOG_D("Module info retrieved");
//	ATCmdParser_set_mode(0);
	//	Automatically enter transparent mode when receiving signal IM_CONN:<ID>，then ,Sending the AT command is invalid	
	
	return err;
}



rt_uint8_t waitReady(void)
{
	uint8_t i=3;
	while(i--){
		LOG_D("try times:%d",3 - i);
		rt_thread_delay(200);
		if(at_module_check_ready() == kNoErr){
			return 0;
		}
		LOG_D("False then reboot!");
		ble_module_Hard_reboot();
	}
	return 1;
}

//mx_status at_module_check_ready(void)
//{
//	uint8_t i;
//	char _buf[10]={0,};
//	for(i=0;i<3;i++){
////	if (  ATCmdParser_send("AT+CMD")
////       && ATCmdParser_recv("OK\r\n")) {
////        return kNoErr;
////    }
//		ATCmdParser_send("AT+CMD");
//		ATCmdParser_read(_buf,10);
////		LOG_D("rec data:%s",_buf);	
////		LOG_D("rec len:%d",strlen(_buf));	
//		if(strstr((const char *)_buf, "OK") != NULL)		
//		{	
//			return kNoErr;
//		}
//	}
//	return kGeneralErr;	
//}
mx_status at_module_check_ready(void)
{
    uint8_t i;
    for (i = 0; i < 3; i++) {
        if (ATCmdParser_send("AT+CMD") && ATCmdParser_recv("OK\r\n")) {
            return kNoErr;
        }
        rt_thread_delay(100);
    }
    return kGeneralErr;
}

const char* at_module_get_fw_version(void)
{
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+VERSION")
       && ATCmdParser_recv("%[^\r]\r\n", _fw_version))) {
        return _fw_version;
    }
		
	}
	return NULL;
}

const char* at_module_get_name(void)
{
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+NAMB")
       && ATCmdParser_recv("+NAMB=%[^\r]\r\n", _ble_name))) {
        return _ble_name;
    }
	}
	return NULL;	
}

mx_status at_module_set_NAMB(const char * name)
{
	uint8_t i;
	for(i=0;i<3;i++){
		if (  ATCmdParser_send("AT+NAMB=%s",name)
       && ATCmdParser_recv("OK\r\n")) {
        return kNoErr;
    }
	}
	return kGeneralErr;	
}

const char* at_module_get_addr(void)
{
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+LADDR")
       && ATCmdParser_recv("+LADDR=%[^\r]\r\n", _ble_mac))) {
        return _ble_mac;
    }
	}
	return NULL;	
}

uint8_t at_module_get_name_MAC(void)
{
	uint8_t param;
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+NAMEC")
       && ATCmdParser_recv("+NAMEC=%d\r\n", &param))) {
        return param;
    }
	}			 
	return 0;
}

mx_status at_module_set_name_MAC(uint8_t param)
{
	uint8_t i;
	if(param>2){
		return kGeneralErr;
	}
	for(i=0;i<3;i++){	
		if (  ATCmdParser_send("AT+NAMEC=%d\r\n",param)
       && ATCmdParser_recv("OK\r\n")) {
        return kNoErr;
    }
	}
	return kGeneralErr;	
}

uint32_t at_module_get_UART_BAUD(void)
{
	uint8_t i;
	uint32_t baud;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+BAUD")
       && ATCmdParser_recv("+BAUD=%d\r\n", &baud))) {				 
        return baud;
    }
	}
	return 0;	
}

mx_status at_module_set_UART_BAUD(uint8_t num)
{
	uint8_t i;
	if(num>9){
		return kGeneralErr;
	}
	for(i=0;i<3;i++){	
		if (  ATCmdParser_send("AT+BAUD=%d\r\n",num)
       && ATCmdParser_recv("OK\r\n")) {
        return kNoErr;
    }
	}
	return kGeneralErr;	
}

uint8_t at_module_get_notify(void)
{
	uint8_t i,param;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+NOTI")
       && ATCmdParser_recv("+NOTI=%d\r\n", &param))) {				 
        return param;
    }
	}
	return 0;	
}

const char* at_module_get_SERVICE_UUID(void)
{
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+UUID")
       && ATCmdParser_recv("+UUID=%[^\r]\r\n", _ble_service_uuid))) {
        return _ble_service_uuid;
    }
	}
	return NULL;	
}

const char* at_module_get_NOTIFY_UUID(void)
{
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+CHAR")
       && ATCmdParser_recv("+CHAR=%[^\r]\r\n", _ble_notify_uuid))) {
        return _ble_notify_uuid;
    }
	}
	return NULL;	
}

const char* at_module_get_WRITE_UUID(void)
{
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+WRITE")
       && ATCmdParser_recv("+WRITE=%[^\r]\r\n", _ble_write_uuid))) {
        return _ble_write_uuid;
    }
	}
	return NULL;	
}

uint8_t at_module_get_PWRM(void)
{
	uint8_t param;
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+PWRM")
       && ATCmdParser_recv("+PWRM=%d\r\n", &param))) {
        return param;
    }
	}			 
	return 0;
}

uint8_t at_module_get_AST(void)
{
	uint8_t time;
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+AST")
       && ATCmdParser_recv("+AST=%d\r\n", &time))) {
        return time;
    }
	}			 
	return 0;
}

uint32_t at_module_get_ADVI(void)
{
	uint32_t time;
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+ADVI")
       && ATCmdParser_recv("+ADVI=%d\r\n", &time))) {
        return time;
    }
	}			 
	return 0;
}

uint8_t at_module_get_POWE(void)
{
	uint8_t power;
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+POWE")
       && ATCmdParser_recv("+POWE=%d\r\n", &power))) {
        return power;
    }
	}			 
	return 0;
}

/** 20
 * @brief	Read DX2002 module TYPE
 * cmd： 	AT+TYPE\r\n  responce: \r\n+TYPE=<param>\r\n  <param>:0-2	0: 默认类型(健康类) 1: 鼠标类型 2: 键盘类型  默认值：0
 * @return	Point to string
 */
uint8_t at_module_get_TYPE(void)
{
	uint8_t param;
	uint8_t i;
	for(i=0;i<3;i++){
		if ((ATCmdParser_send("AT+TYPE")
       && ATCmdParser_recv("+TYPE=%d\r\n", &param))) {
        return param;
    }
	}			 
	return 0;
}

mx_status at_module_RESET(void)
{
	if (  ATCmdParser_send("AT+RESET")
       && ATCmdParser_recv("OK\r\n")) {
        return kNoErr;
    }
	return kGeneralErr;	
}

mx_status at_module_DEFAULT(void)
{
	if (  ATCmdParser_send("AT+DEFAULT")
       && ATCmdParser_recv("+DEFAULT\r\nOK\r\n")) {
        return kNoErr;
    }
	return kGeneralErr;	
}

mx_status at_module_set_MASTER(uint8_t param)
{
    if (param > 1) return kParamErr;
    uint8_t i;
    for (i = 0; i < 3; i++) {
        if (ATCmdParser_send("AT+MASTER=%d", param) && ATCmdParser_recv("OK\r\n"))
            return kNoErr;
        rt_thread_delay(100);
    }
    return kGeneralErr;
}

mx_status at_module_set_CONN(char *mac)
{
    if (!mac) return kParamErr;
    uint8_t i;
    for (i = 0; i < 3; i++) {
        if (ATCmdParser_send("AT+CONN=%s", mac) && ATCmdParser_recv("OK\r\n"))
            return kNoErr;
        rt_thread_delay(100);
    }
    return kGeneralErr;
}

const char* at_module_get_CONN(void)
{
    static char buf[13]={0,};
    uint8_t i;
    for (i = 0; i < 3; i++) {
        if (ATCmdParser_send("AT+CONN") && ATCmdParser_recv("+CONN=%[^\r]\r\n", buf)) {
            return buf;
        }
    }
    return NULL;
}

uint8_t at_module_get_MASTER(void)
{
    uint8_t param;
    uint8_t i;
    for (i = 0; i < 3; i++) {
        if (ATCmdParser_send("AT+MASTER") && ATCmdParser_recv("+MASTER=%d\r\n", &param)) {
            return param;
        }
    }
    return 0;
}

mx_status at_module_set_SETSCANP(uint32_t interval, uint32_t window)
{
    uint8_t i;
    for (i = 0; i < 3; i++) {
        if (ATCmdParser_send("AT+SETSCANP=%d,%d", interval, window) && ATCmdParser_recv("OK\r\n")) {
            return kNoErr;
        }
        rt_thread_delay(100);
    }
    return kGeneralErr;
}

mx_status at_module_set_notify(uint8_t param)
{
    if (param > 1) return kParamErr;
    uint8_t i;
    for (i = 0; i < 3; i++) {
        if (ATCmdParser_send("AT+NOTI=%d", param) && ATCmdParser_recv("OK\r\n"))
            return kNoErr;
        rt_thread_delay(100);
    }
    return kGeneralErr;
}

mx_status at_module_set_AST(uint8_t param)
{
    if (param < 1 || param > 200) return kParamErr;
    uint8_t i;
    for (i = 0; i < 3; i++) {
        if (ATCmdParser_send("AT+AST=%d", param) && ATCmdParser_recv("OK\r\n"))
            return kNoErr;
        rt_thread_delay(100);
    }
    return kGeneralErr;
}

mx_status at_module_set_PWRM(uint8_t param)
{
    if (param > 1) return kParamErr;
    uint8_t i;
    for (i = 0; i < 3; i++) {
        if (ATCmdParser_send("AT+PWRM=%d", param) && ATCmdParser_recv("OK\r\n"))
            return kNoErr;
        rt_thread_delay(100);
    }
    return kGeneralErr;
}

/** 25
 * @brief	DX2002 module DSCET 断开当前蓝牙连接
 * cmd： 	AT+DSCET=<param>\r\n  responce: \r\nOK\r\n   <param>:1：断开蓝牙连接
 * @return	status
 */
mx_status at_module_DSCET(void)
{
	if (  ATCmdParser_send("AT+DSCET=%d",1)
       && ATCmdParser_recv("OK\r\n")) {
        return kNoErr;
    }
	return kGeneralErr;	
}

mx_status at_module_set_AUST(uint8_t param)
{
    if (param < 5 || param > 200) return kParamErr;
    uint8_t i;
    for (i = 0; i < 3; i++) {
        if (ATCmdParser_send("AT+AUST=%d", param) && ATCmdParser_recv("OK\r\n"))
            return kNoErr;
        rt_thread_delay(100);
    }
    return kGeneralErr;
}

mx_status at_module_set_PINCODE(char pin[5])
{
    uint8_t i;
    for (i = 0; i < 3; i++) {
        if (ATCmdParser_send("AT+PINCODE=%s", pin) && ATCmdParser_recv("OK\r\n"))
            return kNoErr;
        rt_thread_delay(100);
    }
    return kGeneralErr;
}

MX_WEAK void emh_ev_unknown(void)
{
	return;
}

