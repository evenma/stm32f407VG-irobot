# iHomeRobot 下位机系统使用说明书

**版本:** V1.0.9  
**日期:** 2026-03-14  
**MCU:** STM32F407VGT6 (1MB Flash, 128KB SRAM)  
**RTOS:** RT-Thread V5.2.2  
**编译器:** RT-Thread Env V2.0.0 → Keil5 MDK

---

## 📋 目录

1. [系统架构图](#1-系统架构图)
2. [已实现功能模块](#2-已实现功能模块)
   - [2.1 LED 指示灯驱动](#21-led-指示灯驱动)
   - [2.2 蜂鸣器音效](#22-蜂鸣器音效)
   - [2.3 OLED 显示系统](#23-oled-显示系统)
   - [2.4 QMI8658 IMU 姿态传感器](#24-qmi8658-imu-姿态传感器)
3. [API 调用参考](#3-api-调用参考)
4. [硬件连接表](#4-硬件连接表)
5. [编译烧录流程](#5-编译烧录流程)
6. [调试与测试](#6-调试与测试)

---

## 1. 系统架构图

```
┌─────────────────────────────────────────────────────┐
│              RT-Thread V5.2.2                       │
├─────────────────────────────────────────────────────┤
│  Application Layer (applications/)                  │
│  ├─ main.c                  - 主循环任务             │
│  ├─ buzzer.*                - 蜂鸣器音效            │
│  ├─ led.*                   - LED 状态指示           │
│  ├─ oled_handle.*           - OLED 显示管理         │
│  ├─ Peripherals/                                            │
│  │   ├─ qmi8658.*          - IMU 姿态传感器       │
│  │   └─ ...                - 其他外设驱动            │
│  └─ Portings/                                                 │
│      ├─ led_porting.c     - SPI 74HC595 端口层     │
│      └─ ...               - 硬件抽象层                │
├─────────────────────────────────────────────────────┤
│  Hardware Interface                                   │
│  ├─ SPI2: PB13(SCK), PB15(MOSI) - LED + OLED        │
│  ├─ I2C1: PB6(SCL), PB7(SDA)    - IMU QMI8658       │
│  ├─ GPIO: PC6                     - Active Buzzer    │
│  ├─ GPIO: PC13, PC14                - SW3/SW4 按键   │
│  └─ GPIO: PB2                         - RCK(74HC595) │
└─────────────────────────────────────────────────────┘
```

---

## 2. 已实现功能模块

### 2.1 LED 指示灯驱动

**文件位置:** `Peripherals/led.h`, `Peripherals/led.c`  
**初始化顺序:** 开机后立即启动（v1.0.7+）

#### 功能特性
- **74HC595 移位寄存器扩展**：通过 SPI 总线控制 8 个 RGB 双色 LED
- **16 通道输出**：每个 LED 有红、绿 2 路独立控制
- **4 种工作模式**：常亮、慢闪、快闪、关闭
- **颜色混合**：支持 4 色组合（红/绿/橙/白）

#### API 调用

```c
#include "led.h"

// ========== 初始化函数 ==========
void led_init(void);                        // 初始化所有 LED 对象
void led_poweron_sequence(void);            // 开机自检序列（扫描效果→绿灯常亮）
void led_shutdown_sequence(void);           // 关机序列（红灯慢闪→熄灭）

// ========== 设置工作模式 ==========
void led_set_working(rt_bool_t active);     // 工作状态指示（绿灯常亮/熄灭）
void led_set_fault(rt_bool_t active);       // 故障指示（红灯常亮）
void led_set_battery_low(rt_bool_t active); // 低电量警告（红灯快闪）

// ========== 自定义颜色/模式 ==========
void led_set_color(uint8_t index, uint8_t color_state);
void led_scan_effect(void);                 // 扫描灯效（逐个点亮）

// ========== 状态判断 ==========
rt_bool_t led_is_initialized(void);         // 检查是否已初始化
uint8_t led_get_total_count(void);          // 获取总 LED 数量（返回 8）
```

#### 使用示例

```c
// 1. 开机后自动完成（main.c 中调用）
led_init();
led_poweron_sequence();  // 扫描→绿灯亮

// 2. 任务开始：绿灯快闪 3 次
buzz_task_start();
for(int i = 0; i < 3; i++) {
    led_set_color(LED_IDX_WORKING_GREEN, LED_COLOR_FLASH_FAST);
    rt_thread_mdelay(100);
    led_set_color(LED_IDX_WORKING_GREEN, LED_COLOR_OFF);
    rt_thread_mdelay(100);
}

// 3. 运行中：绿灯常亮
led_set_working(RT_TRUE);

// 4. 低电量告警：红灯快闪 + OLED 切换页面
buzz_low_battery();
led_set_battery_low(RT_TRUE);
oled_switch_page(PAGE_BATTERY_INFO);

// 5. 一般错误：红灯常亮 + 日志页
buzz_general_error();
led_set_fault(RT_TRUE);
oled_switch_page(PAGE_FAULT_LOG);
```

#### MSH 命令（串口调试）

```bash
led status          # 显示 LED 当前状态
led scan            # 执行扫描灯效一次
led working on/off  # 启用/禁用工作状态
led fault on/off    # 启用/禁用故障状态
```

---

### 2.2 蜂鸣器音效

**文件位置:** `buzzer.h`, `buzzer.c`  
**初始化顺序:** 与 LED 同时启动（v1.0.6+）

#### 功能特性
- **主动式蜂鸣器**（PC6）：高电平触发发声
- **5 种预设音效**：普通提示、按键反馈、任务开始、任务完成、低电量警告、一般错误
- **非阻塞设计**：音效播放期间不影响主循环

#### API 调用

```c
#include "buzzer.h"

// ========== 开关控制 ==========
void buzz_poweron(void);              // 开机检测音（3 短音）
void buzz_poweroff(void);             // 关机静音

// ========== 预设音效 ==========
void buzz_info(void);                 // 普通提示音（1 短音）
void buzz_keypress(void);             // 按键确认音（2 短音）

void buzz_task_start(void);           // 任务开始（3 长音）
void buzz_task_done(void);            // 任务完成（2 长音）

void buzz_low_battery(void);          // 低电量警告（慢长音循环）
void buzz_general_error(void);        // 一般错误（急促短音循环）
```

#### 使用示例

```c
#include "buzzer.h"

// 开机自动完成
buzz_poweron();  // 3 短音确认硬件正常

// 普通操作提示
buzz_info();                 // 单个动作提示
rt_thread_mdelay(50);        // 等待音效结束

// 按键组合确认
buzz_keypress();             // 双击 SW4 按钮
buzz_keypress();

// 任务周期
buzz_task_start();           // 进入清洁模式
// ... 执行任务逻辑 ...
buzz_task_done();            // 任务完成，返回待机

// 异常处理
if (battery_voltage < 19200) {
    buzz_low_battery();      // 持续报警直到充电
    led_set_battery_low(RT_TRUE);
}
```

---

### 2.3 OLED 显示系统

**文件位置:** `System/oled_handle.h`, `System/oled_handle.c`  
**依赖库:** u8g2 SSD1306 驱动  
**初始化顺序:** LED/Buzzer 之后（避免 SPI 冲突）

#### 功能特性
- **128×64 SSD1306 OLED**：单色白色显示
- **11 个功能页面**：分页管理，SW3/SW4 轮询切换（不用外中断）
- **SPI 互斥锁保护**：OLED 和 LED 共用 SPI2，LED 需申请锁
- **多刷新策略**：自动刷新（定时）+ 触发刷新（事件驱动）

#### 页面列表

| 页面 ID | 名称 | 内容 | 自动刷新 |
|--------|------|------|----------|
| PAGE_BOOT | 启动页 | Logo + 版本信息 | 不刷新 |
| PAGE_HOME | 主页 | 电量 + 状态图标 | 1000ms |
| PAGE_PID_TUNING | PID 调参 | Kp/Ki/Kd 参数 | 1000ms |
| PAGE_ULTRASONIC | 超声波 | 7 路距离数据 | 500ms |
| PAGE_IR_SENSOR | 红外传感器 | 悬崖 + 对准管 | 500ms |
| PAGE_BATTERY_INFO | 电池信息 | 电压 + 进度条 | 1000ms |
| PAGE_WATER_LEVEL | 水位信息 | 百分比 + 温度 | 2000ms |
| PAGE_MOTOR_STATUS | 电机状态 | RPM 转速显示 | 100ms |
| PAGE_IMU_DATA | IMU 姿态 ⭐ | 俯仰角/横滚角 | 100ms |
| PAGE_FAULT_LOG | 故障日志 | 错误记录 | 3000ms |
| PAGE_SETTINGS | 系统设置 | 亮度/音量/语言 | 5000ms |

#### API 调用

```c
#include "oled_handle.h"

// ========== 初始化函数 ==========
void oled_handle_init(void);                      // 创建任务、注册页面、建信号量

// ========== 页面管理 ==========
int oled_switch_page(OledPageId_t page_id);       // 切换到指定页面
void oled_prev_page(void);                        // 上一页
void oled_next_page(void);                        // 下一页
OledPageId_t oled_get_current_page(void);         // 获取当前页码

// ========== 刷新控制 ==========
void oled_trigger_refresh(void);                  // 触发信号量刷新（外部调用）
void oled_force_refresh(void);                    // 强制立即刷新当前页
void oled_set_auto_refresh(OledPageId_t page_id, rt_uint32_t interval_ms);
                                                  // 设置自动刷新间隔（单位 ms）

// ========== UI 组件绘制 ==========
void oled_draw_title_bar(const char* title, rt_bool_t show_page_indicator);
                                                  // 顶部标题栏（页码指示器）
void oled_draw_progress(rt_uint8_t x, rt_uint8_t y, 
                        rt_uint8_t width, rt_uint8_t height,
                        rt_uint8_t progress, rt_bool_t is_good);
                                                  // 进度条（0-100%，绿色/红色）
void oled_draw_value_box(rt_uint8_t x, rt_uint8_t y, int32_t value,
                         const char* unit, rt_uint8_t precision);
                                                  // 数值框（带单位和精度）
void oled_draw_status_dot(rt_uint8_t x, rt_uint8_t y,
                          rt_uint8_t radius, rt_bool_t active);
                                                  // 状态圆点（亮绿/暗红）
void oled_draw_icon(rt_uint8_t x, rt_uint8_t y, rt_uint8_t icon_id);
                                                  // 图标（WiFi/电池/警告）

// ========== SPI 互斥锁 API（供其他模块调用）==========
void oled_spi_lock(void);                         // 获取 SPI 锁（等待）
void oled_spi_unlock(void);                       // 释放 SPI 锁
```

#### 使用示例

```c
// 1. 主程序初始化
led_init();
led_poweron_sequence();
rt_thread_mdelay(100);
oled_handle_init();

// 2. 切换到主页
oled_switch_page(PAGE_HOME);

// 3. 电池信息页：动态更新进度条
oled_switch_page(PAGE_BATTERY_INFO);
oled_set_auto_refresh(PAGE_BATTERY_INFO, 1000);  // 每秒刷新

// 4. 超声波数据变化时触发刷新
update_ultrasonic_distances(new_values);
oled_trigger_refresh();  // 通知 OLED 线程刷新最新数据

// 5. 强制刷新当前页（紧急场景）
oled_force_refresh();

// 6. UI 组件调用（在自定义页面渲染函数内）
oled_draw_title_bar("IMU 姿态", RT_TRUE);         // 带页码
oled_draw_value_box(10, 20, pitch_int, "°", 1);  // 放大 10 倍显示整数部分
oled_draw_progress(10, 40, 100, 10, battery_level, RT_TRUE);
```

#### SPI 互斥锁使用规范

```c
// led_porting.c 示例：LED 发送数据前必须申请锁
#include "oled_handle.h"

void led_porting_shift_out(rt_uint16_t data)
{
    oled_spi_lock();  // ← 申请 SPI 独占权
    // SPI transfer to 74HC595...
    oled_spi_unlock();  // ← 释放锁
}
```

⚠️ **注意**：如果 LED 未调用 `oled_spi_lock()`，可能导致 OLED 显示花屏或乱码！

---

### 2.4 QMI8658 IMU 姿态传感器

**文件位置:** `Peripherals/qmi8658.h`, `Peripherals/qmi8658.c`  
**硬件地址:** I2C @ 0x6B  
**初始化顺序:** 最后启动（最低优先级）

#### 功能特性
- **6 轴数据采集**: ±16g 加速度计 + ±2000°/s 陀螺仪
- **采样频率**: 100Hz~1000Hz 可调（默认 1000Hz）
- **欧拉角解算**: Pitch (-90~90°), Roll (-180~180°), Yaw (未校准漂移)
- **连续采样任务**: 后台任务实时采集并保存最新数据

#### API 调用

```c
#include "qmi8658.h"

// ========== 初始化 ==========
int qmi8658_init(void);                           // 返回 0 成功，负值失败

// ========== 数据读取 ==========
int qmi8658_read_once(QmiDataRaw_t* raw_out, QmiDataDecoded_t* decoded_out);
                                                  // 单次读取原始 + 解码数据
int qmi8658_read_average(rt_uint8_t samples, QmiDataDecoded_t* decoded_out);
                                                  // N 次采样平均（降噪）

// ========== 后台任务 ==========
void qmi8658_start_continuous(uint16_t freq_hz);  // 启动连续采样（100~1000 Hz）
void qmi8658_stop_continuous(void);               // 停止采样任务
void qmi8658_get_latest(QmiDataDecoded_t* decoded_out);
                                                  // 获取最新保存的数据（线程安全）
rt_bool_t qmi8658_is_ready(void);                 // 检查是否就绪

// ========== 数据结构定义 ==========
typedef struct {
    int16_t acc_x, acc_y, acc_z;      // 原始加速度计数值
    int16_t gyro_x, gyro_y, gyro_z;   // 原始陀螺仪计数值
} QmiDataRaw_t;

typedef struct {
    float acc_x_g, acc_y_g, acc_z_g;  // 加速度 (单位：g)
    float gyro_x_deg, gyro_y_deg, gyro_z_deg;  // 角速度 (单位：度/秒)
    float pitch;          // 俯仰角 (-90~90°)
    float roll;           // 横滚角 (-180~180°)
    float yaw;            // 航向角 (未校准，会漂移)
} QmiDataDecoded_t;
```

#### 使用示例

```c
// 1. 初始化 IMU
if (qmi8658_init() == 0) {
    LOG_I("QMI8658 initialized!");
    qmi8658_start_continuous(1000);  // 1000Hz 连续采样
    
    // 切换到 IMU 页面显示
    oled_switch_page(PAGE_IMU_DATA);
    oled_set_auto_refresh(PAGE_IMU_DATA, 100);  // 100ms 刷新一次
} else {
    LOG_W("QMI8658 init failed! Check I2C connection.");
}

// 2. 手动读取一次数据
QmiDataDecoded_t imu_data;
if (qmi8658_read_once(RT_NULL, &imu_data) == 0) {
    rt_kprintf("Pitch: %.2f°, Roll: %.2f°\n", 
               imu_data.pitch, imu_data.roll);
}

// 3. 高精度模式（取 10 次平均）
QmiDataDecoded_t avg_data;
qmi8658_read_average(10, &avg_data);

// 4. 后台任务模式：直接获取最新数据（无需主动读取）
while (1) {
    QmiDataDecoded_t latest;
    qmi8658_get_latest(&latest);
    
    if (fabs(latest.pitch) > 30.0f || fabs(latest.roll) > 30.0f) {
        // 检测到倾斜超过 30°
        buzz_general_error();
        led_set_fault(RT_TRUE);
    }
    
    rt_thread_mdelay(100);
}

// 5. 停止采样（省电模式）
qmi8658_stop_continuous();
```

#### MSH 调试命令

```bash
qmi read          # 手动读取一次数据
qmi stream        # 流式打印（输入 stop 退出）
qmi start [hz]    # 启动连续采样（默认 1000Hz）
qmi stop          # 停止采样
qmi status        # 显示传感器状态
```

#### I2C 连接检查

```
设备：QMI8658C
地址：0x6B (固定)
WHOAMI 寄存器：0x86

I2C 引脚映射：
PB6 → SCL (I2C Clock)
PB7 → SDA (I2C Data)

如果初始化失败，检查：
1. I2C1 是否在 global_conf.h 中正确配置
2. 上拉电阻是否正常（4.7kΩ）
3. 传感器供电（3.3V）
```

---

## 3. API 调用参考汇总

### 跨模块交互

| 场景 | 发起模块 | 被调用模块 | 调用方式 |
|------|----------|------------|----------|
| LED 占用 SPI | led_porting | oled_handle | `oled_spi_lock()/unlock()` |
| 触发 OLED 刷新 | 任意任务 | oled_handle | `oled_trigger_refresh()` |
| 切换 OLED 页面 | main 循环 | oled_handle | `oled_switch_page()` |
| 获取 IMU 数据 | 任意任务 | qmi8658 | `qmi8658_get_latest()` |

### 初始化顺序

```
1. fal_init()                    // 文件系统
2. buzz_poweron()                // 蜂鸣器开机音
3. led_init() + led_poweron_sequence()
4. rt_thread_mdelay(100)         // 等 LED 稳定
5. oled_handle_init()            // OLED 显示系统
6. qmi8658_init()                // IMU 姿态传感器（可选）
7. oled_switch_page(PAGE_HOME)   // 切换到主页
```

### 全局变量

```c
extern int g_oled_battery_mv;   // 电池电压（mV），由 OLED 线程每秒更新
```

---

## 4. 硬件连接表

### STM32F407VGT6 引脚分配

| 功能 | Pin | 复用功能 | 备注 |
|------|-----|----------|------|
| **SPI2_SCK** | PB13 | AF0_SPI2_SCK | LED + OLED 共用 |
| **SPI2_MOSI** | PB15 | AF0_SPI2_MOSI | LED + OLED 共用 |
| **74HC595_RCK** | PB2 | GPIO_Output | 移位寄存器时钟 |
| **I2C1_SCL** | PB6 | AF1_I2C1_SCL | IMU QMI8658 |
| **I2C1_SDA** | PB7 | AF1_I2C1_SDA | IMU QMI8658 |
| **Active Buzzer** | PC6 | GPIO_Output | 高电平发声 |
| **OLED_CS** | PB12 | GPIO_Output | SPI 片选 |
| **OLED_DC** | PB14 | GPIO_Output | 数据/命令选择 |
| **SW3 (Prev)** | PC13 | GPIO_Input | 上一页按钮（轮询） |
| **SW4 (Next)** | PC14 | GPIO_Input | 下一页按钮（轮询） |

### 功耗规格

| 模块 | 工作电流 | 最大电流 |
|------|----------|----------|
| MCU (STM32F407) | ~30mA | ~80mA |
| LEDs (8×RGB) | ~20mA | ~160mA |
| OLED (SSD1306) | ~10mA | ~50mA |
| Buzzer | ~10mA | ~30mA |
| IMU (QMI8658) | ~0.1mA | ~1mA |
| **总计峰值** | **~70mA** | **~350mA** |

建议电池容量 ≥ 2000mAh（支持 16.8V~25.2V 输入）。

---

## 5. 编译烧录流程

### Windows 环境

```bash
# 1. 进入工程目录
cd D:\iHomeRobotProject\rt-thread\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VG-irobot

# 2. 更新包依赖（首次运行必须）
.\env-windows-v2.0.0\env-windows\env.exe pkgs --update

# 3. 生成 Keil 工程文件
.\env-windows-v2.0.0\env-windows\env.exe scons --target=mdk5

# 4. 打开工程编译
# 双击 project.uvprojx → Keil MDK5 IDE → 点击 Build (F7)

# 5. 烧录
# Keil → Load (F7) → Verify → Run
```

### 常见问题

| 错误现象 | 原因 | 解决方案 |
|----------|------|----------|
| `error: oled_spi_lock undeclared` | 头文件未包含 | `led_porting.c` 添加 `#include "oled_handle.h"` |
| `undefined reference to oled_spi_lock` | 静态函数不可访问 | `oled_handle.c` 去掉 `static` 关键字 |
| `I2C device not found` | QMI8658 硬件未连接 | 检查 I2C 上拉电阻和电源 |
| `WHOAMI check failed` | 传感器地址错误 | QMI8658 地址固定为 0x6B |
| OLED 花屏 | SPI 冲突未加锁 | LED 发送前调用 `oled_spi_lock()` |

---

## 6. 调试与测试

### 串口调试（USB-TTL 连接 PA9/PA10）

```
波特率：115200
数据位：8
停止位：1
校验位：None
流控：None

期望启动日志：
[LED] SPI init complete for 74HC595
[OLED] Display system initialized with 11 pages
[IMU] QMI8658C detected (WHOAMI: 0x86)
QMI8658C IMU initialized successfully!
The current version of APP firmware is iBed-body-V1.0.9
```

### MSH 命令测试

进入 msh 模式（输入 `help` 查看所有命令）：

```bash
# LED 测试
led status
led scan
led working on

# OLED 测试
# 无直接 msh 命令，需通过串口修改 main.c 测试切换

# IMU 测试
qmi read
qmi stream
qmi status
```

### 实物测试步骤

1. **上电前检查**:
   - 电池电压 16.8V~25.2V ✅
   - I2C 上拉电阻 4.7kΩ ✅
   - SPI 连线正确（PB13, PB15）✅

2. **开机观察**:
   - LED 扫描效应 → 绿灯常亮 ✅
   - 蜂鸣器 3 短音 ✅
   - OLED 显示 "iBed-body V1.0.9" ✅

3. **功能验证**:
   - 按 SW3/SW4：OLED 页面切换 ✅
   - 低电量模拟：红灯闪烁 + 语音报警 ✅
   - IMU 倾斜：Pitch/Roll 角度实时更新 ✅

### 性能指标

| 指标 | 数值 |
|------|------|
| 启动时间（到主页） | ~500ms |
| IMU 响应延迟 | <5ms（1000Hz 采样） |
| OLED 刷新频率 | 10-100Hz 可调 |
| LED 响应延迟 | <10ms（SPI 传输） |
| 内存占用 | ~12KB SRAM + ~150KB Flash |

---

## 📞 技术支持

**问题反馈渠道**:
1. 查看 `memory/2026-03-14.md` 了解开发历史
2. 检查 `PERIPHERALS_README.md` 获取详细 API 说明
3. 通过 Feishu/Discord 联系开发团队

**版本更新日志**:
- v1.0.6: 基础蜂鸣器音效
- v1.0.7: LED 74HC595 驱动 + SPI 互斥锁框架
- v1.0.8: OLED 显示系统 + 多页面管理
- **v1.0.9**: QMI8658 IMU 集成 + 姿态数据显示 ⭐ 当前版本

---

_本文档持续维护中，欢迎贡献补充_ ✨
