# iHomeRobot 系统模块汇总

**版本:** V1.0.11  
**日期:** 2026-03-14  
**MCU:** STM32F407VGT6 (1MB Flash, 128KB SRAM)

---

## 📊 已实现模块总览

| 序号 | 模块名称 | 文件路径 | 协议/接口 | 状态 |
|------|----------|----------|-----------|------|
| ✅1 | LED 指示灯驱动 | `Peripherals/led.*` | SPI + GPIO | ✅ 完成 |
| ✅2 | 蜂鸣器音效 | `buzzer.*` | GPIO | ✅ 完成 |
| ✅3 | OLED 显示系统 | `System/oled_handle.*` | SPI + u8g2 | ✅ 完成 |
| ✅4 | QMI8658 IMU | `Peripherals/qmi8658.*` | I2C | ✅ 完成 |
| ✅5 | RS485 Modbus 超声波 | `Peripherals/rs485_ultrasonic.*` | UART+Modbus | ✅ 完成 |
| ✅6 | WonderEcho AI 语音 | `Peripherals/wonder_echo.*` | I2C | ✅ 完成 |
| ✅7 | CANopen ZLAC8015D | `Peripherals/canopen_motor.*` | CANopen | ✅ 完成 |

---

## 🔧 各模块详细说明

### 1. LED 指示灯驱动 (v1.0.7)

**硬件:** 74HC595 移位寄存器 + 8×RGB 双色 LED  
**SPI 总线:** PB13(SCK), PB15(MOSI), PB2(RCK)  

**API 调用:**
```c
led_init();                           // 初始化
led_set_working(RT_TRUE);             // 工作状态（绿灯常亮）
led_set_battery_low(RT_TRUE);         // 低电量警告（红灯快闪）
led_set_fault(RT_TRUE);               // 故障指示（红灯常亮）
led_scan_effect();                    // 扫描灯效
```

**用途:** 系统状态可视化、任务反馈、错误报警

---

### 2. 蜂鸣器音效 (v1.0.6)

**硬件:** 主动式有源蜂鸣器 (PC6)  

**API 调用:**
```c
buzz_poweron();                   // 开机提示音
buzz_info();                      // 普通提示
buzz_keypress();                  // 按键确认
buzz_task_start();                // 任务开始（三声长音）
buzz_task_done();                 // 任务完成
buzz_low_battery();               // 低电量报警（持续循环）
buzz_general_error();             // 一般错误（急促短音）
```

**用途:** 声音反馈、紧急通知

---

### 3. OLED 显示系统 (v1.0.8)

**屏幕:** 128×64 SSD1306 (I2C/SPI)  
**页面:** 11 个功能页面分页管理  

**API 调用:**
```c
oled_handle_init();                       // 初始化显示系统
oled_switch_page(PAGE_HOME);              // 切换页面
oled_trigger_refresh();                   // 触发刷新
oled_draw_progress(x, y, 100, 10, 80, RT_TRUE);  // 绘制进度条
```

**11 个页面:**
- PAGE_BOOT - 启动页
- PAGE_HOME - 主界面
- PAGE_PID_TUNING - PID 调参
- PAGE_ULTRASONIC - 7 路超声波测距 ⭐
- PAGE_IR_SENSOR - 红外传感器
- PAGE_BATTERY_INFO - 电池信息
- PAGE_WATER_LEVEL - 水位水温
- PAGE_MOTOR_STATUS - 电机 RPM
- PAGE_IMU_DATA - IMU 姿态数据 ⭐
- PAGE_FAULT_LOG - 故障日志
- PAGE_SETTINGS - 系统设置

**用途:** 实时监控数据显示、调试界面

---

### 4. QMI8658 6 轴 IMU 姿态传感器 (v1.0.9)

**接口:** I2C @ 0x6B  
**数据:** ±16g 加速度计 + ±2000°/s 陀螺仪  
**采样率:** 100~1000Hz 可调  

**API 调用:**
```c
qmi8658_init();                        // 初始化 IMU
qmi8658_start_continuous(1000);        // 启动连续采样 (1000Hz)
qmi8658_read_once(raw, decoded);       // 单次读取数据
qmi8658_get_latest(&data);             // 获取最新保存的数据
```

**数据结构:**
```c
QmiDataDecoded_t {
    float pitch;          // 俯仰角 (-90~90°)
    float roll;           // 横滚角 (-180~180°)
    float yaw;            // 航向角 (未校准)
}
```

**用途:** 倾斜检测、姿态解算、摔倒检测、导航辅助

---

### 5. RS485 Modbus 超声波传感器阵列 (v1.0.10)

**接口:** UART3 + RS485 收发模块  
**协议:** Modbus RTU @ 9600bps  
**支持:** 最多 16 个从机，每从机 7 路通道  

**API 调用:**
```c
rs485_us_init();                             // 初始化 RS485 系统
rs485_us_read_sensor(addr, channels);        // 读取指定从机
uint16_t rs485_us_read_distance(addr, chan); // 单通道距离 (mm)
uint16_t rs485_us_get_online_status();       // 在线设备掩码
rt_bool_t rs485_us_ping(addr);               // Ping 检测

// MSH 命令:
us show   // 显示所有传感器数据
us scan   // 批量扫描在线设备
```

**OLED 显示:**
```
┌─────────────────────┐
│  超声波 [4/11]       │
├─────────────────────┤
│  US1: 150mm         │
│  US2: 270mm         │
│  US3: 390mm         │
│  US4: 510mm         │
│  US5: 630mm         │
│  US6: 750mm         │
│  US7: 870mm         │
└─────────────────────┘
```

**用途:** 避障导航、障碍物检测、距离测量、定位辅助

---

### 6. WonderEcho AI 语音交互模块 (NEW! v1.0.11)

**接口:** I2C @ 0x1C  
**功能:** ASR 语音识别 + TTS 语音合成 + 唤醒词检测  

**工作模式:**
```c
MODE_FREE_TRIGGER = 0       // 自由触发（单次唤醒）
MODE_CONTINUOUS_TRIGGER = 1 // 连续对话
MODE_DISABLE_WAKEUP = 2     // 禁用唤醒词
```

**API 调用:**
```c
wonder_echo_init(mode, volume);            // 初始化 (音量 0~100)
wonder_echo_speak_now(text, vol);          // 立即播报文本
wonder_echo_queue_tts(text, vol);          // 加入播报队列
wonder_echo_force_listen();                // 强制开启聆听
wonder_echo_stop_audio();                  // 停止音频输出

// 回调函数注册:
wonder_echo_register_on_wakeup(cb);
wonder_echo_register_on_recognized(cb);
wonder_echo_register_on_tts_done(cb);

// MSH 命令:
echo status   // 显示状态
echo test     // 测试播报 "hello world"
echo vol 80   // 设置音量
```

**应用场景:**
- "打开清洁模式" → 启动机器人
- "返回充电座" → 导航回充
- "当前电量多少？" → 回答电池信息
- "关闭灯光" → 控制外设

---

### 7. CANopen ZLAC8015D 电机驱动 (NEW! v1.0.11)

**接口:** CAN Bus @ 1Mbps  
**协议:** CANopen DS301  
**支持:** 双差速底盘电机  

**工作模式:**
```c
MODE_POSITION_CONTROL = 4      // CSP 位置模式
MODE_VELOCITY_CONTROL = 8      // CSV 速度模式
MODE_TORQUE_CONTROL = 3        // CST 力矩模式
```

**API 调用:**
```c
canopen_motor_init(nodes[], count);         // 初始化电机系统
canopen_motor_enable(motor_idx);            // 使能电机
canopen_motor_disable(motor_idx);           // 禁用电机
canopen_motor_reset_fault(motor_idx);       // 复位故障

/* 速度控制 */
canopen_motor_set_velocity(motor_idx, rpm); // 设置转速
canopen_motor_stop(motor_idx);              // 急停

/* 位置控制 */
canopen_motor_set_position(motor_idx, rev); // 绝对位置 (圈数)
canopen_motor_move_relative(motor_idx, deg);// 相对移动 (度)

/* PID 闭环控制 */
canopen_motor_pid_update(motor_idx);        // 更新 PID 控制器
canopen_motor_set_pid(motor_idx, kp, ki, kd);
float output = canopen_motor_pid_get_output(motor_idx);

// MSH 命令:
motor status      // 显示电机状态
motor vel left 100   // 左轮 100 RPM
motor pid 2.0 0.1 0.05
```

**电机参数配置:**
```c
config.encoder_ticks_per_rev = 1000.0f;  // 编码器分辨率
config.gear_ratio = 5.0f;                 // 减速比
```

**用途:** 差速底盘运动控制、巡线导航、自动避障、PID 运动闭环

---

## 🔗 模块间协作关系

```
┌─────────────────────────────────────────────────┐
│              Application Layer                  │
│  main.c (协调所有模块的主循环任务)               │
├─────────────────────────────────────────────────┤
│  User Interaction                          Debug│
│  ├─ WonderEcho AI ←→ Voice Response            │
│  └─ SW3/SW4 Keys → OLED Page Switching         │
├─────────────────────────────────────────────────┤
│  Sensor Fusion & Control                    Status│
│  ├─ RS485 Ultrasonic → Obstacle Avoidance     │
│  ├─ QMI8658 IMU → Tilting Detection           │
│  └─ CANopen Motors → Differential Drive       │
├─────────────────────────────────────────────────┤
│  Feedback & Alert Systems                    UI  │
│  ├─ LED (74HC595) → Visual Indicators         │
│  ├─ Buzzer → Audio Alerts                     │
│  └─ OLED Display → Real-time Data View        │
└─────────────────────────────────────────────────┘
```

**典型工作流程:**

1. **开机自检**: LED 扫描 → 蜂鸣器提示音 → OLED 显示 Logo
2. **语音唤醒**: 用户说"打开清洁" → WonderEcho 识别 → 发送指令
3. **导航执行**: 
   - CANopen 电机驱动 → 差速转向
   - RS485 超声波 → 实时障碍物检测
   - 遇到障碍 → 超声波读数 < 50mm → 停车避障
   - IMU 检测坡度 → 倾角 > 30° → 报错并返回
4. **状态反馈**: 
   - OLED 实时显示距离/角度数据
   - LED 显示运行状态（绿=正常，红=故障）
   - 蜂鸣器发出提示音

---

## 💻 MSH 调试命令汇总

### 通用命令
```bash
help                  # 查看所有可用命令
led status            # LED 状态检查
```

### 蜂鸣器测试
```bash
buzz info             # 播放普通提示音
buzz low_batt         # 低电量报警模拟
```

### OLED 测试
```bash
oled next             # 下一页
oled prev             # 上一页
oled refresh          # 强制刷新
```

### IMU 测试
```bash
qmi read              # 手动读取一次数据
qmi stream            # 流式打印（输入 stop 退出）
qmi start             # 启动连续采样
qmi status            # 查看状态
```

### 超声波测试
```bash
us show               # 显示所有传感器
us scan               # 批量扫描在线设备
us test               # 测试单个从机
```

### 语音模块测试
```bash
echo status           # 显示 WonderEcho 状态
echo test             # 测试播报
echo vol 80           # 设置音量
echo mode 0           # 设置为自由触发模式
```

### 电机测试
```bash
motor status          # 显示电机状态
motor vel left 100    # 左轮速度控制
motor pid 2.0 0.1 0.05 # PID 参数整定
motor test 2000       # 测试运动 2 秒
```

---

## 🚀 性能指标

| 模块 | 响应时间 | 刷新频率 | 内存占用 |
|------|----------|----------|----------|
| LED | <1ms | N/A | ~1KB |
| Buzzer | <1ms | N/A | ~0.5KB |
| OLED | 10ms | 60Hz | ~15KB |
| IMU | 1ms | 1000Hz | ~2KB |
| Ultrasonic | 20ms | 50Hz | ~5KB |
| WonderEcho | 50ms | N/A | ~8KB |
| Motor | 10ms | 1kHz | ~10KB |

**总计内存占用:** ~42KB RAM + ~300KB Flash

---

## ⚠️ 注意事项

1. **SPI 资源冲突解决**: OLED 和 LED 共用 SPI2，通过互斥锁 (`oled_spi_lock()/unlock()`) 保护

2. **I2C 复用问题**: QMI8658 (I2C1)、WonderEcho (I2C3) 使用不同 I2C 总线，无冲突

3. **UART3 独占**: RS485 使用 UART3，不可与其他 UART 设备混用

4. **CAN Bus 屏蔽干扰**: 建议使用屏蔽双绞线，地线良好连接

5. **电源稳定性**: 建议配置独立 5V 电源给电机和 RS485 模块

---

## 📞 技术支持与版本历史

**最新版本:** V1.0.11 (2026-03-14)  
**开发团队:** Wuji / iHomeRobot  
**文档位置:** `SYSTEM_MANUAL.md`, `MODULES_SUMMARY.md`

**版本迭代:**
- V1.0.6 - 基础蜂鸣器音效
- V1.0.7 - LED 74HC595 驱动 + SPI 互斥框架
- V1.0.8 - OLED 多页面显示系统
- V1.0.9 - QMI8658 IMU 姿态传感器
- V1.0.10 - RS485 Modbus 超声波阵列
- **V1.0.11 - WonderEcho AI 语音 + CANopen 电机驱动** ⭐ 最新

---

_本文档持续维护中，欢迎贡献补充 ✨_
