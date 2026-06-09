/**
 ******************************************************************************
 * @file    at_seiral.c
 * @author  ele
 * @version V1.0.0
 * @date    3-Step-2023
 * @brief   UART driver used for BT parser
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
#include "mx_common.h"
#include "at_serial.h"
#include "rtthread.h"
#include "rtdef.h"
#include "rtdevice.h"
/******************************************************************************
 *                              Variable Definitions
 ******************************************************************************/

static int at_timeout = 100;
static rt_device_t at_serial = RT_NULL;

/******************************************************************************
 *                               Type Definitions
 ******************************************************************************/
//#define RT_SERIAL_CONFIG_DEFAULT           \
//{                                          \
//    BAUD_RATE_115200, /* 115200 bits/s */  \
//    DATA_BITS_8,      /* 8 databits */     \
//    STOP_BITS_1,      /* 1 stopbit */      \
//    PARITY_NONE,      /* No parity  */     \
//    BIT_ORDER_LSB,    /* LSB first sent */ \
//    NRZ_NORMAL,       /* Normal mode */    \
//    RT_SERIAL_RB_BUFSZ, /* Buffer size */  \
//    0                                      \
//}


/******************************************************************************
 *                              Function Definitions
 ******************************************************************************/

void at_serial_init(int timeout)
{
    mx_status err;
    at_timeout = timeout;	
		struct serial_configure configSerial = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */
	at_serial = rt_device_find("uart2");
		/* step2：修改串口配置参数 */
	configSerial.bufsz	 = AT_SERIAL_RB_BUFSZ; 		//缓冲区 buff size 为 64 1次收发4字节，N次缓存
	/* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
	rt_device_control(at_serial, RT_DEVICE_CTRL_CONFIG, &configSerial);
	
	err = rt_device_open(at_serial, RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_INT_RX);
  if (err != RT_EOK)
      goto __exit;

__exit:
    return;
}

void at_serial_set_timeout(int timeout)
{
   at_timeout = timeout;
}

int at_serial_putc(char c)
{
	rt_size_t  count=0 ;
	
	count =	rt_device_write(at_serial,0,&c, 1);
	return (count>0)?0:1;
}

int at_serial_getc(void)
{
    uint32_t current = rt_tick_get();
    uint8_t ch;
//    rt_kprintf("at_timeout:%d\r\n", at_timeout);

    do {
		if(rt_device_read(at_serial, 0, &ch, 1) == 1)
        {
//          rt_kprintf("-ch[%c]\r\n",ch);
            return ch;
        }
		rt_thread_delay(1);
//        rt_kprintf("time:%d\r\n", (rt_tick_get() - current));
    } while ((rt_tick_get() - current) < at_timeout);
    return -1;
}

bool at_serial_readable(void)
{
//	uint16_t numb;
//	numb = rt_serial_read_number(at_serial);  此函数已不支持改为一下代码
	struct rt_serial_device *serial;
	struct rt_serial_rx_fifo* rx_fifo;
	serial = (struct rt_serial_device *)at_serial;
	 rx_fifo = (struct rt_serial_rx_fifo*) serial->serial_rx;
	 RT_ASSERT(rx_fifo != RT_NULL);
	/* there's no data: */	
	if ((rx_fifo->get_index == rx_fifo->put_index) && (rx_fifo->is_full == RT_FALSE))
	{
			return false;
	}else{
			return true;
	}
}

