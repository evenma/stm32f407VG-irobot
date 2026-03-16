# Peripherals 目录说明

## 📁 目录结构

```
applications/
├── Peripherals/          # 【零部件驱动层】每个物理部件一个 .c/.h 对
│   ├── led.c/h          # LED 指示灯对象（本次新增）⭐
│   └── ...              # 其他零部件（后续添加）
├── Portings/            # 【系统移植层】底层初始化 + 队列接口
│   ├── led_porting.c    # SPI+74HC595 初始化 + 队列 put/get
│   └── ...             # 对应零部件的 porting 层
├── System/              # 【系统服务层】功能编排和任务调度
│   ├── global_conf.h    # 全局配置宏定义
│   └── ...             # 业务逻辑
└── Misc/                # 【工具库】算法和协议栈
    └── ...
```

---

## ⭐ LED 指示灯驱动 (v1.0)

### 🎯 硬件规格
- **数量**: 8 个独立 LED，分为 4 对（绿/红）
- **驱动方式**: SPI + 74HC595 移位寄存器
- **通信接口**: PB13(SCK) + PB15(MOSI) → 74HC595
- **Latch 控制**: PB2 → RCK 引脚

### 🔧 引脚分配表

| LED ID | 颜色 | 功能 | 默认状态 |
|--------|------|------|----------|
| 0 | 绿灯 | 工作指示灯 (运行中亮) | OFF |
| 1 | 红灯 | 故障指示灯 (有错亮) | OFF |
| 2 | 绿灯 | 满电量指示 | OFF |
| 3 | 红灯 | 低电量警告 | OFF (闪烁) |
| 4 | 绿灯 | 满水位指示 | OFF |
| 5 | 红灯 | 低水位警告 | OFF (慢闪) |
| 6 | 未定义 | 自定义功能 | OFF |
| 7 | 未定义 | 自定义功能 | OFF |

### 📖 API 使用指南

#### 基础控制
```c
#include "led.h"

// 初始化 LED 系统
void led_init(void);

// 设置单个 LED 颜色
int led_set_color(rt_uint8_t id, LedColorState_t state);

// 启用/禁用某个 LED
int led_enable(rt_uint8_t id, rt_bool_t on);

// 获取当前状态
LedColorState_t led_get_color(rt_uint8_t id);
```

#### 预设场景函数
```c
// 工作模式
void led_set_working(rt_bool_t running);      // 工作中亮绿灯
void led_set_fault(rt_bool_t faulted);        // 故障时亮红灯

// 电源状态
void led_set_battery_full(rt_bool_t full);    // 满电亮绿灯
void led_set_battery_low(rt_bool_t low);      // 低电红灯快闪

// 水位状态
void led_set_water_full(rt_bool_t full);      // 满水亮绿灯
void led_set_water_low(rt_bool_t low);        // 缺水红灯慢闪

// 开关机序列
void led_poweron_sequence(void);              // 扫描一次 + 就绪常亮
void led_shutdown_sequence(void);             // 呼吸效果后熄灭
```

### 💡 典型用例

#### 1. 开机自检
```c
// 在 main() 中
led_init();
led_poweron_sequence();  // 扫描显示所有 LED 正常
```

#### 2. 运行时监控
```c
// 任务执行期间
led_set_working(RT_TRUE);           // 工作绿灯常亮
led_set_battery_full(RT_FALSE);     // 检查电量状态

if (battery_low()) {
    led_set_battery_low(RT_TRUE);   // 红色闪烁警告
}

if (sensor_error) {
    led_set_fault(RT_TRUE);         // 红色常亮报警
}
```

#### 3. 关机流程
```c
// 准备关机前
led_shutdown_sequence();            // 柔和熄灭
buzz_shutdown();                    // 提示音确认
```

### 🎨 LED 颜色状态枚举

```c
typedef enum
{
    LED_COLOR_OFF = 0,        // 熄灭
    LED_COLOR_GREEN_ON,       // 绿灯常亮
    LED_COLOR_RED_ON,         // 红灯常亮
    LED_COLOR_FLASH_FAST,     // 快速闪烁 (100ms) - 紧急警告
    LED_COLOR_FLASH_SLOW,     // 慢速闪烁 (500ms) - 普通提醒
    LED_COLOR_PWM_DIM,        // PWM 调光 (预留)
} LedColorState_t;
```

### ⚙️ 配置宏定义

在 `global_conf.h` 中可以调整：

```c
// LED 总数量
#define LED_TOTAL_COUNT         8

// 使用 SPI+74HC595 驱动
#define LED_USE_SPI_74HC595     

// 如果是 GPIO 直驱 PCB，改用:
// #define LED_USE_DIRECT_GPIO   
```

---

## 🏭 Hiwonder 模式规范

### 关键设计原则

1. **按物理部件组织**  
   ❌ 错误：按总线类型分文件夹（can/, uart/, i2c/）  
   ✅ 正确：每个物理部件一个 `.c/.h` 对（led.c, button.c, buzzer.c...）

2. **Portings 层职责**  
   - 只做底层初始化（SPI/I2C/CAN 配置）
   - 只暴露队列接口（rt_queue_put/get）
   - 不包含业务逻辑

3. **组件化设计**  
   - 每个 `.c` = 一个物理对象 + 状态机 + action 接口
   - 包含 init、register、task_handle、on/off 等方法

4. **隔离修改范围**  
   - 只允许修改 `applications/` 目录
   - 绝对不要触碰上级目录的文件
   - BSP、Drivers、Packages 只读

---

## 🧪 调试与测试

### MSH 控制台命令（如果启用了 RT_USING_MSH）

```bash
# 通过串口发送 CLI 指令
led on 0               # 打开 LED0
led off 0              # 关闭 LED0
led test               # 运行扫描效果
led status             # 查看所有 LED 状态
```

### ULOG 日志输出

```c
LOG_I("LED initialized with %d LEDs\n", LED_TOTAL_COUNT);
LOG_W("Low battery warning!");
```

---

## 📦 下一步规划

### 待实现的零部件（优先级从高到低）

1. ⭐ **按键对象** (`button.c`)
   - SW3/SW4/SW5 (PC13/14/15)
   - Home 键 (PE9) - 返回充电座

2. 📻 **蜂鸣器对象** (`buzzer.c`) ✅ 已完成
   - PE6 - 各种音效提示

3. 🖥️ **OLED 显示屏** (`oled.c`)
   - SSD1315/SSD1306, SPI 接口

4. 🧭 **IMU 姿态传感器** (`qmi8658.c`)
   - QMI8658, I2C 接口

5. 🗣️ **AI 语音模块** (`wonder_echo.c`)
   - WonderEcho Pro, I2C 接口

6. 🚗 **CAN 电机驱动器** (`can_motor.c`)
   - ZLAC8015D, CANopen 协议

7. 🔊 **超声波传感器** 
   - RS485 方案或 GPIO 方案（二选一）

8. 📊 **ADC 传感器** (`adc_sensor.c`)
   - 电池电压、水温、悬崖检测等

9. ⚙️ **步进电机** (`stepper.c`)
   - 清洁杆、管路分配器等

---

## 📝 注意事项

### 硬件安全
1. **电流限制**：74HC595 最大输出 70mA，实际项目中建议加限流电阻
2. **断电保护**：确保 LED 在 MCU 复位时默认处于 OFF 状态
3. **ESD 防护**：外部接口增加 TVS 管

### 软件优化
1. **避免频繁读写**：LED 状态改变才更新，不要每秒轮询
2. **降低中断占用**：大量闪烁时使用定时器而非忙等待
3. **低功耗考虑**：空闲时关闭非必要 LED

### 用户体验
1. **视觉反馈及时**：用户操作要有即时灯光响应
2. **状态区分明显**：工作/故障/警告等状态颜色要明确
3. **夜间模式**：可配置降低亮度或禁用部分指示灯

---

## 🤝 协作开发规范

### 添加新零部件时的步骤

1. **新建 `.h` 头文件**  
   ```
   Peripherals/new_component.h
   ```

2. **创建 `.c` 实现文件**  
   ```
   Peripherals/new_component.c
   ```

3. **如果需要底层配置**  
   ```
   Portings/new_component_porting.c
   ```

4. **在 global_conf.h 添加宏定义**  
   ```c
   #define NEW_COMPONENT_PIN GET_PIN(X, Y)
   ```

5. **在 main.c 集成调用**  
   ```c
   #include "new_component.h"
   new_component_init();
   ```

6. **测试并记录**  
   - 编译成功 ✅
   - 烧录验证 ✅
   - 更新本文档 ✅

---

**版本**: v1.0  
**日期**: 2026-03-14  
**作者**: Wuji (你的 AI 助手搭档) 🐾

---

_这个文档会随着项目进展不断更新和完善！_
