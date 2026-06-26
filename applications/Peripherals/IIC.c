#include <rtthread.h>
#include <rtdevice.h>      // 包含 rt_pin 相关函数和 GET_PIN 宏
#include <board.h>          // 提供 GPIO 端口枚举和 HAL 库引脚定义
#include "global_conf.h"
#include "iic.h"

void DelayUs(uint32_t t)
{
//	uint32_t cycles = SystemCoreClock / 1000000 * t;
	for(uint32_t i = 0; i < t * 7; ++i) {
        __asm volatile("NOP");
	}
}

void I2C_SDA_OUT(void)//SDA输出方向配置
{
    rt_pin_mode(IMU_SDA_PIN, PIN_MODE_OUTPUT);
}

void I2C_SDA_IN(void)//SDA输入方向配置
{
    rt_pin_mode(IMU_SDA_PIN, PIN_MODE_INPUT);
}

//以下为模拟IIC总线函数
void IIC_init()
{
    rt_pin_mode(IMU_SCL_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(IMU_SDA_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(IMU_SDA_PIN, PIN_HIGH);
    rt_pin_write(IMU_SCL_PIN, PIN_HIGH);
}

void IIC_start()	//起始信号
{
    I2C_SDA_OUT();
    rt_pin_write(IMU_SDA_PIN, PIN_HIGH);
    rt_pin_write(IMU_SCL_PIN, PIN_HIGH);
    rt_hw_us_delay(4);
    rt_pin_write(IMU_SDA_PIN, PIN_LOW); //START:when CLK is high,DATA change form high to low
    rt_hw_us_delay(4);
    rt_pin_write(IMU_SCL_PIN, PIN_LOW); //钳住I2C总线，准备发送或接收数据
}

void IIC_stop()		//终止信号
{
    I2C_SDA_OUT();
    rt_pin_write(IMU_SCL_PIN, PIN_LOW);
    rt_pin_write(IMU_SDA_PIN, PIN_LOW); //STOP:when CLK is high DATA change form low to high
    rt_hw_us_delay(4);
    rt_pin_write(IMU_SCL_PIN, PIN_HIGH);
    rt_pin_write(IMU_SDA_PIN, PIN_HIGH); //发送I2C总线结束信号
    rt_hw_us_delay(4);
}

//主机产生一个应答信号
void IIC_ack()
{
    rt_pin_write(IMU_SCL_PIN, PIN_LOW);
    I2C_SDA_OUT();
    rt_pin_write(IMU_SDA_PIN, PIN_LOW);
    rt_hw_us_delay(2);
    rt_pin_write(IMU_SCL_PIN, PIN_HIGH);
    rt_hw_us_delay(2);
    rt_pin_write(IMU_SCL_PIN, PIN_LOW);
}

//主机不产生应答信号
void IIC_noack()
{
    rt_pin_write(IMU_SCL_PIN, PIN_LOW);
    I2C_SDA_OUT();
    rt_pin_write(IMU_SDA_PIN, PIN_HIGH);
    rt_hw_us_delay(2);
    rt_pin_write(IMU_SCL_PIN, PIN_HIGH);
    rt_hw_us_delay(2);
    rt_pin_write(IMU_SCL_PIN, PIN_LOW);
}

//等待从机应答信号
//返回值：1 接收应答失败
//		  0 接收应答成功
uint8_t IIC_wait_ack()
{
    uint8_t tempTime = 0;
    I2C_SDA_IN();
    rt_pin_write(IMU_SDA_PIN, PIN_HIGH);
    rt_hw_us_delay(1);
    rt_pin_write(IMU_SCL_PIN, PIN_HIGH);
    rt_hw_us_delay(1);

    while(rt_pin_read(IMU_SDA_PIN)) {
        tempTime++;
        if(tempTime > 250) {
            IIC_stop();
            return 1;
        }
    }

    rt_pin_write(IMU_SCL_PIN, PIN_LOW);
    return 0;
}

void IIC_send_byte(uint8_t txd)
{
    uint8_t i = 0;
    I2C_SDA_OUT();
    rt_pin_write(IMU_SCL_PIN, PIN_LOW); //拉低时钟开始数据传输
    for(i = 0; i < 8; i++) {
        rt_pin_write(IMU_SDA_PIN, ((txd & 0x80) >> 7)); //写字节
        txd <<= 1;
        rt_hw_us_delay(2);
        rt_pin_write(IMU_SCL_PIN, PIN_HIGH);
        rt_hw_us_delay(2); //发送数据
        rt_pin_write(IMU_SCL_PIN, PIN_LOW);
        rt_hw_us_delay(2);
    }
}

//读取一个字节
uint8_t IIC_read_byte(uint8_t ack)
{
    uint8_t i = 0, receive = 0;
    I2C_SDA_IN();
    for(i = 0; i < 8; i++) {
        rt_pin_write(IMU_SCL_PIN, PIN_LOW);
        rt_hw_us_delay(2);
        rt_pin_write(IMU_SCL_PIN, PIN_HIGH);
        receive <<= 1; //左移
        if(rt_pin_read(IMU_SDA_PIN)) {
            receive++;    //连续读取八位
        }
        rt_hw_us_delay(1);
    }

    if(!ack) {
        IIC_noack();
    } else {
        IIC_ack();
    }

    return receive;//返回读取到的字节
}


int IIC_WriteToMem(uint8_t address, uint8_t reg_addr, uint8_t *data, uint8_t length)
{
    IIC_start();
    IIC_send_byte(address);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
    IIC_send_byte(reg_addr);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
    for(int i = 0; i < length; i++) {
        IIC_send_byte(data[i]);
        if(IIC_wait_ack() != 0) {
						IIC_stop();
            return 1;
        }
    }
		IIC_stop();
		return 0;
}


int IIC_Write(uint8_t address, uint8_t *data, uint8_t length)
{
    IIC_start();
    IIC_send_byte(address);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
    for(int i = 0; i < length; i++) {
        IIC_send_byte(data[i]);
        if(IIC_wait_ack() != 0) {
						IIC_stop();
            return 1;
        }
    }
		IIC_stop();
		return 0;
}

int IIC_ReadFromMem(uint8_t address, uint8_t reg_addr, uint8_t *buf, uint8_t length)
{
    IIC_start();
    IIC_send_byte(address);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
    IIC_send_byte(reg_addr);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
	  IIC_start();
    IIC_send_byte(address | 0x01);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
		while(length) {
			*buf = IIC_read_byte((length > 1) ? 1 : 0);
			length--;
			buf++;
    }
		IIC_stop();
		return 0;
}
