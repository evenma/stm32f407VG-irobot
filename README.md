# iHomeRobot 下位机固件开发日志

## 版本历史

### v1.0.11 (2026-03-14) ⭐ CURRENT
**功能：集成 WonderEcho AI 语音模块 + CANopen 电机驱动**

#### 新增硬件支持
- **WonderEcho AI 语音模块** (`Peripherals/wonder_echo.c/h`)
  - I2C 通信 (PB10-SCL, PB11-SDA)
  - 唤醒词识别、TTS 播报
  - 3 个回调：wake-up / recognition / TTS done
  
- **CANopen ZLAC8015D 电机驱动** (`Peripherals/canopen_motor.c/h`)
  - CAN 总线 (PA11-RX, PA12-TX)
  - 双差速电机控制 (左/右节点 ID 可配置)
  - PDO 实时速度控制 + SDO 参数配置
  - PID 自动调节

#### 主程序入口更新
```c
// applications/main.c
#define APP_VERSION "1.0.11"

int main(void)
{
    // 初始化顺序：
    buzz_poweron();       // 蜂鸣器电源
    led_init();           // LED (SPI+74HC595)
    oled_handle_init();   // OLED 显示屏
    
    wonder_echo_init();   // AI 语音
    rs485_us_init();      // RS485 超声波
    qmi8658_init();       // IMU 姿态
    canopen_motor_init(); // CANopen 电机
}
```

---

### v1.0.10 (2026-03-14)
**功能：RS485 级联超声波传感器阵列**

#### 新增硬件支持
- **RS485 防水超声波** (`Peripherals/rs485_ultrasonic.c/h`)
  - Modbus RTU 协议 (波特率 9600)
  - 最多 7 个传感器级联 (地址 1-7)
  - 检测范围：0.03m~5m
  - 在线状态检测 + 故障标记

#### MSH 调试命令
```bash
us485 status     # 显示在线传感器数量
us485 read <addr># 读取单个传感器距离
us485 list       # 显示所有传感器数据
```

---

### v1.0.9 (2026-03-14)
**功能：QMI8658C 6 轴 IMU 姿态传感器**

#### 新增硬件支持
- **QMI8658C IMU** (`Peripherals/qmi8658.c/h`)
  - I2C 通信 (PB6-SCL, PB7-SDA, PB5-INT)
  - ±16g 加速度计 + ±2000°/s 陀螺仪
  - 1000Hz 连续采样
  - Euler 角计算 (Pitch/Roll/Yaw)

#### OLED 页面扩展
```
PAGE_IMU_DATA = 8  // 新页面显示 Pitch/Roll 角度
```

#### MSH 调试命令
```bash
qmi read         # 单次读取
qmi stream       # 实时流式显示
qmi start [hz]   # 启动连续采样 (默认 1000Hz)
qmi stop         # 停止采样
qmi status       # 显示传感器状态
```

---

### v1.0.8 (2026-03-14)
**功能：OLED 多页面显示系统**

#### 核心改动
- **SSD1315/SSD1306 OLED** (128×64) 多页面管理
- 动态刷新策略（不同页面不同刷新频率）
- 按键翻页 (SW3=上一页，SW4=下一页)

#### 页面布局 (共 11 页)
| 页码 | 名称 | 说明 | 刷新率 |
|------|------|------|--------|
| 0 | PAGE_BOOT | 开机动画 | 一次性 |
| 1 | PAGE_HOME | 主页仪表盘 | 1000ms |
| 2 | PAGE_PID_TUNING | PID 参数调整 | 500ms |
| 3 | PAGE_ULTRASONIC | 超声波距离 | 200ms |
| 4 | PAGE_IR_SENSOR | 红外悬崖检测 | 1000ms |
| 5 | PAGE_BATTERY_INFO | 电池电压/电量 | 2000ms |
| 6 | PAGE_WATER_LEVEL | 水箱水位 | 1000ms |
| 7 | PAGE_MOTOR_STATUS | 电机 RPM | 100ms |
| 8 | PAGE_IMU_DATA | 姿态角度 | 100ms |
| 9 | PAGE_FAULT_LOG | 故障日志 | 手动切换 |
| 10 | PAGE_SETTINGS | 设置项 | 1000ms |
| 11 | PAGE_COUNT | 总页数 | - |

---

### v1.0.7 (2026-03-14)
**功能：LED 指示灯驱动 (SPI+74HC595)**

#### 硬件架构
- **8 个单色 LED** 通过 1 片 74HC595 移位寄存器驱动
- SPI2 总线共享 (PB13-SCK, PB15-MOSI → 74HC595)
- GPIO PB2 → 74HC595 RCLK(时钟锁存)

#### LED 分配
| ID | 颜色 | 用途 | 来源 |
|----|------|------|------|
| 0 | Green | 工作指示灯 | 本地 |
| 1 | Red | 故障指示 | 本地 |
| 2 | Green | 满电指示 | 本地 |
| 3 | Red | 低电警告 | 本地 |
| 4 | Green | 满水指示 | 本地 |
| 5 | Red | 低水警告 | 本地 |
| 6 | Green | 导航灯 | ←上位机 |
| 7 | Red | 异常灯 | ←上位机 |

#### SPI 互斥方案
- OLED 线程拥有 SPI 互斥锁 (`s_spi_mutex`)
- LED 代码通过 `oled_spi_lock()/unlock()` 请求访问
- 避免循环依赖：LED → OLED API → SPI 资源保护

---

### v1.0.6 (2026-03-14)
**功能：蜂鸣器驱动**

#### 硬件
- GPIO PE6 → 有源蜂鸣器驱动 MOSFET
- 队列机制实现异步音效播放

#### 预设音效
| 函数 | 描述 |
|------|------|
| `buzz_keypress()` | 按键音 |
| `buzz_info()` | 信息提示音 |
| `buzz_task_start()` | 任务开始音 |
| `buzz_task_done()` | 任务完成音 |
| `buzz_low_battery()` | 低电量警报 |
| `buzz_general_error()` | 一般错误警报 |

---

## 项目结构

```
applications/
├── main.c                     # 主程序入口 (v1.0.11)
├── global_conf.h              # 全局配置宏定义
├── Peripherals/               # 硬件驱动层
│   ├── buzzer.c/h             # 蜂鸣器 ✅ v1.0.6
│   ├── led.c/h                # LED 指示灯 ✅ v1.0.7
│   ├── wonder_echo.c/h        # AI 语音模块 ⭐ NEW v1.0.11
│   ├── canopen_motor.c/h      # CANopen 电机驱动 ⭐ NEW v1.0.11
│   ├── rs485_ultrasonic.c/h   # RS485 超声波 ⭐ v1.0.10
│   ├── qmi8658.c/h            # IMU 姿态传感器 ⭐ v1.0.9
│   └── ...                    # 更多驱动待添加
├── Portings/                  # IO 初始化 + 队列接口 (待创建)
├── System/                    # 系统服务层 (待创建)
└── Misc/                      # 工具库 (待创建)
```

## 编译与烧录

### 环境准备
```bash
cd D:\iHomeRobotProject\rt-thread\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VG-irobot
.\env-windows-v2.0.0\env-windows\env.exe pkgs --update
```

### 编译
```bash
.\env-windows-v2.0.0\env-windows\env.exe scons --target=mdk5
```

### 烧录
使用 Keil5 MDK 打开生成的 `.uvprojx` 工程文件，点击 Load 按钮。

## 下一步计划

- [ ] CANopen 电机调试
- [ ] WonderEcho 语音唤醒测试
- [ ] OTA 升级验证
- [ ] 故障诊断系统
- [ ] 电源管理系统

---
*最后更新：2026-03-14*  
*作者：Wuji (AI Assistant)*
