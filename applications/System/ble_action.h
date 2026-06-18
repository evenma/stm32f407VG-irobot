#ifndef BLE_ACTION_H
#define BLE_ACTION_H

#include <rtthread.h>

//按键定义
typedef enum
{
  KEY_RELEASE=0,			 // 按键释放
/*小车按键*/	
	KEY_SLAM = 50,				//1 建图和导航切换， 默认导航， 建图时LED灯常亮
	KEY_STOPWORK ,				//2 停止动作，不区分小车运动和智能马桶动作，全部停		
	
	KEY_FORWORD,					//3 前进，点动，按着动作，放开停
	KEY_BACKWORD,         //4 后退，点动，按着动作，放开停
	KEY_LEFT,             //5 左转，点动，按着动作，放开停
	KEY_RIGHT,						//6 右转，点动，按着动作，放开停
	
	KEY_ROME_A,						//7 正常模式下，去房间A--卧室   建图时设锚点坐标
	KEY_ROME_B,						//8 正常模式下，离开房间A--去厕所，避免房间A有臭味    建图时设锚点坐标
	KEY_HOME,							//9 回充电座，正常模式的发送指令和 单机模式 复用同一个按键	 建图时设锚点坐标
	KEY_FREE,							//10 手动推车，打开和关闭

/*智能马桶按键*/
	KEY_FLUSH = 100	,			//1 冲洗马桶	
	KEY_STOP,							//2 停止动作
	
	KEY_BIANMEN_KAI,			//3 马桶盖打开，由于按键不够，复用单机模式下的左转
	KEY_BIANMEN_GUAN,			//4 马桶盖关闭，由于按键不够，复用单机模式下的左转
	KEY_CLEAN_REAR,				//5 清洁臀部 
	KEY_CLEAN_FEMALE,			//6 清洁女性
	
	KEY_CLEAN_MODE,				//7 清洁时固定模式和按摩模式切换
	KEY_CLEAN_STRENGTH,		//8 清洁力度调节	1-3档位 按一下切一次
	KEY_GANZAO,						//9 暖风烘干		
	KEY_SEWAGE,						//10 长按5-6秒才能启动污物泵，防止误操作导致污物冲出到房间里。

/*其他*/
	KEY_SWITCH = 200,			// KEY_SWITCH短按触发，切换按键功能，前十个按键控制小车(前10个LED闪一下)或者控制智能马桶(前10个LED闪三下)，
	KEY_LOCK,							// 长按1-2秒解锁	
	KEY_SINGLE_MODE = 0xF0,  // KEY_SWITCH长按触发 小车操作单机模式，下位机控制小车,长按5-6秒切换单机模式 默认正常模式 ，单机模式点亮LED灯(前进/后退/左转/右转/回充电座)
	KEY_NORMAL_MODE ,
	KEY_MAPPING_MODE= 0xF2,	 // 建图模式
	KEY_NAV_MODE,						 // 导航模式
	
}E_KeyMessage;


int ble_action_init(void);

#endif

