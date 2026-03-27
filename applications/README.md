# iRobot_baseboard - RT-Thread嵌入式程序

## 项目简介

iHomeRobot下位机项目，基于RT-Thread实时操作系统和STM32F407VGT6芯片开发。

## 系统架构

### 上位机（树莓派5）
- ROS2机器人系统
- MS200激光雷达
- Nuwa-HP60C 3D结构光深度相机
- WonderEcho Pro AI语音交互模块

### 下位机（STM32F407VGT6）
- RT-Thread 5.2.2实时操作系统
- 27个功能模块

## 工程目录结构

```
applications/
├── global_conf.h      # 全局配置文件
├── main.c             # 主程序入口
├── Peripherals/       # 外设驱动
│   ├── drv_can.c      # CAN通讯驱动
│   ├── drv_can.h
│   └── ...
├── Portings/          # 移植层
├── System/            # 系统层
└── Misc/              # 杂项算法
```

## 开发流程

### 1. 配置工程

使用env工具配置功能：
```bash
cd rt-thread-5.2.2/bsp/stm32/stm32f407VG-irobot
env
menuconfig
```

### 2. 生成Keil工程

```bash
scons --target=mdk5
```

### 3. 编译工程

使用Keil5打开 `project.uvprojx` 进行编译。

### 4. 调试

- 使用MSH命令行调试
- 使用ULOG日志系统输出调试信息

## 主要功能模块

### 已实现
- ✅ 系统初始化框架
- ✅ 任务调度框架
- ✅ CAN通讯驱动
- ✅ 全局配置系统

### 待实现
- ⏳ 超声波传感器驱动
- ⏳ LED指示灯驱动
- ⏳ 电机控制驱动
- ⏳ 传感器读取任务
- ⏳ Packet通讯协议
- ⏳ 蓝牙通讯
- ⏳ AI语音交互
- ⏳ 充电检测
- ⏳ 水泵控制
- ⏳ 清洁杆控制
- ⏳ 管路分配器控制
- ⏳ 暖风烘干
- ⏳ 加热管控制
- ⏳ 马桶盖/圈电机控制
- ⏳ 大灯控制

## 配置说明

所有硬件配置通过 `global_conf.h` 中的宏定义控制：

```c
#define CAN_ENABLE                       1
#define US_ENABLE                        1
#define LED_ENABLE                       1
#define TASK_MONITOR_ENABLE              1
...
```

修改配置后需要重新编译。

## 调试命令

使用MSH命令行调试：

```bash
system_status    # 查看系统状态
task_status      # 查看任务状态
reboot           # 重启系统
```

## 通讯协议

### CAN通讯
- 使用RT-Thread的CAN设备驱动
- 支持CANopen协议
- 支持ZLAC8015D驱动器

### Packet协议
- 上下位机通讯协议（待实现）

### 蓝牙通讯
- DX2002蓝牙模块
- BLE 5.0协议（待实现）

## 版本信息

- **版本**: 1.0.0
- **日期**: 2026-03-26
- **作者**: 马哥

## 参考资料

- RT-Thread官网: https://www.rt-thread.org/
- RT-Thread设备维护云: https://iot.rt-thread.com/#/homePage
- STM32F407VGT6芯片手册
- RT-Thread源码: https://github.com/RT-Thread/rt-thread

## 开发规范

1. 所有代码只能在 `applications/` 目录下创建和修改
2. 使用ULOG日志系统输出调试信息（不在中断中使用）
3. 使用对象化设计，每个零部件用struct封装
4. 使用MSH命令行进行调试
5. 修改配置后重新编译

## 许可证

内部项目，禁止外传。
