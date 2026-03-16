# Code Review Report - iHomeRobot V1.0.11

**Review Date:** 2026-03-14 15:00  
**Reviewer:** Wuji  
**Target Version:** V1.0.11 (Final before testing)

---

## 🔴 Critical Issues (必须修复)

### 1. **BUZZER_PIN 引脚冲突** ❌
**位置:** `global_conf.h` Line 28  
**问题:** 
```c
#define BUZZER_PIN GET_PIN(E, 6)  // PC6!
```
但实际在 PCB 设计中，**E6 是加热管 MOSFET 控制**!

**影响:** 蜂鸣器和加热管会共用同一 GPIO，导致功能混乱  
**优先级:** 🔴 CRITICAL  

**修复方案:**
- 查看 CubeMX 或原理图确认蜂鸣器实际引脚
- 根据项目描述，可能是 **PC6** 或其他未占用引脚
- 建议改为：**PA8** (TIM3_CH1, PWM 支持) 或 **PE7** (空闲 IO)

---

### 2. **I2C 总线分配冲突** ⚠️
**位置:** `global_conf.h` Lines 102-114  
**问题:** IMU 和 WonderEcho I2C 总线分配不合理

**当前配置:**
```c
IMU_I2C_PORT    I2C1          // PB6/SCL, PB7/SDA
VOICE_I2C_PORT  I2C2          // PB10/SCL, PB11/SDA
```

**检查项:**
- ✅ IMU: QMI8658 地址 0x6B → I2C1 可用
- ⚠️ WonderEcho: 地址 0x1C → I2C2 可能与其他模块冲突
- ❌ 如果 WonderEcho 也使用 PB10/PB11，会与 I2C2 复用冲突

**修复建议:**
1. 确认 WonderEcho 是否真的使用 I2C2
2. 如有冲突，改为 **I2C3** (PB8/PB9)
3. 或者将 IMU 改为 **I2C3**，保持 VOICE 在 I2C2

---

### 3. **SPI 总线竞争风险** ⚠️
**位置:** `led.c` Line 87-94 (`led_update_shift_register`)  
**问题:** SPI 发送逻辑缺失！

**代码现状:**
```c
static void led_update_shift_register(void)
{
    shift_register_output = 0x0000;
    
    for (int i = 0; i < LED_TOTAL_COUNT; i++)
    {
        if (led_objects[i].enabled)
        {
            shift_register_output |= (uint16_t)led_objects[i].color_state << (i * 2);
        }
    }
    
    // TODO: 这里应该通过 SPI 发送到 74HC595
    // PB13 -> SRCK, PB15 -> SER, PB2 -> RCK
}
```

**影响:** LED 状态更新后没有真正写入移位寄存器 → **LED 不会亮！**

**修复方案:**
```c
// 新增 SPI 发送函数
static void led_send_to_spi(uint16_t data)
{
    struct rt_i2c_device* spi_dev = NULL;
    uint8_t tx_buf[3] = {0};  // 16 位数据拆成 3 字节
    
    tx_buf[0] = (data >> 8) & 0xFF;
    tx_buf[1] = (data >> 0) & 0xFF;
    tx_buf[2] = 0x00;         // dummy
    
    // 拉低 RCK (PB2)
    rt_pin_write(GET_PIN(B, 2), PIN_LOW);
    
    // 通过 SPI2 发送 (PB13=SCK, PB15=MOSI)
    spi_dev = rt_i2c_bus_device_find("spi2");
    rt_device_write(&spi_dev->parent, 0, tx_buf, 3);
    
    // 拉高 RCK 触发输出
    rt_thread_mdelay(1);
    rt_pin_write(GET_PIN(B, 2), PIN_HIGH);
}
```

**测试方法:**
```bash
led test      # MSH 命令测试扫描效果
```

---

### 4. **CANopen SDO 读模拟** ⚠️
**位置:** `canopen_motor.c` Line 64 (`sdo_read_object`)  
**问题:** 使用硬编码模拟值，无法获取真实电机状态

**当前代码:**
```c
static int sdo_read_object(uint8_t node_id, uint16_t index, ...)
{
    switch(index)
    {
        case 0x6040:  // 状态字
            *data = STATE_READY | STATE_SWITCH_ON;  // 固定返回就绪
            break;
        default:
            *data = 0;  // 其他全部返回 0
            break;
    }
}
```

**影响:** 
- 无法检测到真实的故障码（如过流、过温）
- 位置反馈永远不变 → PID 控制器失效

**修复方案:**
1. 等待真实 CAN 设备接入时再实现完整 SDO 协议栈
2. 当前可暂时保留此占位符，但添加警告日志

---

### 5. **UART485 UART 接口不匹配** ⚠️
**位置:** `global_conf.h` Lines 66-69  
**问题:** RS485 使用的 UART4 与 STM32F407VG 的 UART4 映射不一致

**当前配置:**
```c
#define UART485_PORT UART4
#define UART485_RX_PIN GET_PIN(C, 10)  // PA10
#define UART485_TX_PIN GET_PIN(C, 11)  // PA11
```

**STM32F407VG Pinout:**
- UART4_RX: **PC10** ✅ (正确)
- UART4_TX: **PC11** ✅ (正确)

但 `global_conf.h`中注释写成 **PA10/PA11**，这是 UART2 的引脚！

**修复建议:**
修正注释，避免混淆：
```c
/**
 * @brief RS485 引脚定义 (PC10=RX, PC11=TX)
 */
```

---

## 🟡 Medium Priority Issues (建议优化)

### 6. **对象封装不完整** 📝
**影响模块:** `buzzer.c`, `led.c`, `qmi8658.c`

**问题:** 部分模块没有采用对象化设计（OOP）

**对比分析:**
| 模块 | 是否对象化 | 说明 |
|------|-----------|------|
| LED | ✅ Yes | `LedObject_t led_objects[LED_TOTAL_COUNT]` |
| IMU | ✅ Yes | `Qmi8658Object_t s_imu` + callbacks |
| RS485 | ✅ Yes | `Rs485UltrasonicObject_t s_us_obj` |
| WonderEcho | ✅ Yes | `WonderEchoObject_t s_echo` |
| Motor | ✅ Yes | `CanMotorObject_t s_motors[MOTOR_MAX_NODES]` |
| Buzzer | ❌ No | 全局函数式 API，无对象结构 |

**Buzzer 对象化改进建议:**
```c
typedef struct {
    rt_pin_t pin;
    rt_bool_t enabled;
    rt_timer_t period_timer;
    uint8_t tone_type;
} BuzzerObject_t;

static BuzzerObject_t s_buzzer;

int buzzer_init_object(BuzzerObject_t* obj, rt_pin_t pin);
void buzzer_set_tone(BuzzerObject_t* obj, ToneType_t type);
```

**优点:**
- 多蜂鸣器扩展更容易
- 回调机制统一
- 与 RT-Thread Device Model 兼容

---

### 7. **SPI Mutex 保护不够完善** 📝
**位置:** `oled_handle.c`, `led_porting.c`

**问题:** 缺少初始化 mutex 的显式检查

**当前代码:**
```c
void oled_spi_lock(void)
{
    rt_mutex_fetch(&s_spi_mutex, RT_WAITING_FOREVER);
}
```

**潜在风险:** 如果 OLED 线程还没创建就调用 LED，mutex 未初始化会崩溃

**修复建议:**
```c
void oled_spi_lock(void)
{
    // 双重检查或添加 flag
    if (oled_initialized == RT_FALSE)
    {
        rt_kprintf("[OLED] Error: Not initialized yet!\n");
        return;
    }
    rt_mutex_fetch(&s_spi_mutex, RT_WAITING_FOREVER);
}
```

---

### 8. **TTS 队列大小可能不足** 📝
**位置:** `wonder_echo.h` Line 30  
**问题:** `TTS_QUEUE_SIZE = 10` 对于多任务场景可能不够

**场景:**
- 用户快速连续发送多个语音指令
- ASR 识别结果频繁更新
- 系统同时播报多个状态提示

**建议:** 增加到 **20~30**，或实现环形缓冲区动态扩容

---

### 9. **PID 积分限幅过大** 📝
**位置:** `canopen_motor.c` Line 118  
**当前代码:**
```c
motor->integral += error;
if (motor->integral > 1000.0f) motor->integral = 1000.0f;
if (motor->integral < -1000.0f) motor->integral = -1000.0f;
```

**问题:** ±1000 的范围对于速度模式（RPM）可能太大

**建议:**
- 如果是速度闭环：限幅 ±50 RPM
- 如果是位置闭环：限幅 ±100 圈

需根据实际电机参数调整

---

### 10. **Modbus CRC16 位序错误风险** ⚠️
**位置:** `rs485_ultrasonic.c` Line 13  
**问题:** CRC16 计算使用标准 MODBUS 多项式，但未验证位序

**当前算法:**
```c
crc ^= data[i];
for (int j = 0; j < 8; j++)
{
    if (crc & 0x0001)
    {
        crc >>= 1;
        crc ^= 0xA001;  // LSB-first polynomial
    }
    else
    {
        crc >>= 1;
    }
}
```

**检查项:**
- 某些 Modbus 设备使用 MSB-first (CRC-16/CCITT-FALSE)
- 需要确认传感器型号对应的 CRC 类型

**建议:**
增加两种算法选择宏定义：
```c
// #define RS485_CRC_MSB_FIRST
// #define RS485_CRC_LSB_FIRST  (默认 MODBUS)
```

---

## 🟢 Low Priority Suggestions (可选优化)

### 11. **MSH 命令命名空间污染** 📝
**当前:** `echo status`, `echo test`, `echo vol`  
**建议:** 更清晰的命名：
- `wonder_echo status`
- `wonder_echo test`
- `wonder_echo vol 80`

---

### 12. **缺少错误码定义** 📝
**影响模块:** `canopen_motor.c`, `rs485_ultrasonic.c`

**现状:** 函数返回值多为 `-RT_ERROR`或`0`，缺乏具体错误码

**建议:** 定义专用枚举
```c
typedef enum {
    ERR_MOTOR_INIT_FAILED = -1000,
    ERR_CAN_COMM_TIMEOUT,
    ERR_MOTOR_FAULT_OVERFLOW,
    ...
} CanMotorError_t;
```

---

### 13. **Memory Leak 风险** 📝
**位置:** `wonder_echo.c` Line 205  
**问题:** `s_eolch.on_recognized = NULL` 赋值前可能未释放旧内存

---

## ✅ 好的实践 (Keep doing!)

### 👍 优秀设计点

1. **对象封装一致性好** ✅
   - 所有外设都封装为 Object 结构体
   - 成员变量包含状态、配置、统计信息
   - 回调函数指针支持事件驱动

2. **SPI 互斥锁设计优雅** ✅
   - `oled_spi_lock()/unlock()` 公共 API
   - 解决 OLED+LED 共享 SPI2 冲突
   - 不影响编译灵活性（条件编译）

3. **版本管理清晰** ✅
   - VERSION_MAJOR/MINOR/PATCH
   - CHANGELOG 记录每个版本的改动
   - APP_VERSION_STRING 用于调试输出

4. **MSH 调试命令全覆盖** ✅
   - 每个模块都有 status/test/pid 命令
   - 方便现场开发和故障排查

5. **文档完整性** ✅
   - SYSTEM_MANUAL.md (API 参考)
   - MODULES_SUMMARY.md (模块汇总)
   - CODE_REVIEW 自我审查报告

---

## 🎯 优先级排序

| 优先级 | 问题编号 | 标题 | 预计修复时间 |
|--------|---------|------|-------------|
| 🔴 P0 | 1 | BUZZER_PIN 冲突 | 10 分钟 |
| 🔴 P0 | 3 | LED SPI 发送缺失 | 30 分钟 |
| ⚠️ P1 | 2 | I2C 总线分配冲突 | 15 分钟 |
| ⚠️ P1 | 5 | UART485 注释错误 | 5 分钟 |
| ⚠️ P1 | 4 | CANopen SDO 模拟 | 1 小时 (等待硬件) |
| 🟡 P2 | 6 | Buzzer 对象化改造 | 1 小时 |
| 🟡 P2 | 7 | SPI Mutex 安全检查 | 20 分钟 |
| 🟡 P2 | 10 | Modbus CRC16 验证 | 30 分钟 |
| 🟢 P3 | 8 | TTS 队列扩容 | 10 分钟 |
| 🟢 P3 | 9 | PID 积分限幅优化 | 20 分钟 |

---

## 📋 修复行动计划

### Step 1: 立即修复 (今天完成)
1. ✅ 更正 BUZZER_PIN 到正确引脚
2. ✅ 补全 LED SPI 发送逻辑
3. ✅ 修正 UART485 注释
4. ✅ 添加 SPI Mutex 安全检查

### Step 2: 预编译前修复 (明天上午)
1. ⏳ 验证 I2C 总线映射
2. ⏳ 测试 Modbus CRC16 准确性
3. ⏳ 扩容 TTS 队列到 20

### Step 3: 预留优化 (后续迭代)
1. 🔮 Buzzer 对象化改造
2. 🔮 CANopen 完整 SDO 协议栈
3. 🔮 PID 参数自动整定

---

## 🧪 测试清单

修复完成后需要测试的项目：

```bash
# 1. 基础功能测试
led test                # 扫描 LED
led status              # 显示所有 LED 状态
buzz info               # 播放提示音
buzz low_batt           # 低电量报警

# 2. OLED 刷新测试
oled next               # 翻页
oled refresh            # 强制刷新

# 3. 超声波模块测试
us scan                 # 批量扫描在线从机
us show                 # 显示所有传感器数据

# 4. IMU 测试
qmi start               # 启动连续采样
qmi status              # 查看实时数据

# 5. 语音模块测试
echo test               # 播报"hello world"
echo status             # 显示 WonderEcho 状态

# 6. 电机测试 (需 CAN 硬件连接)
motor status            # 显示双电机状态
motor vel left 50       # 左轮慢速旋转
motor pid 2.0 0.1 0.05  # 设置 PID
```

---

## 💡 关于上位机通信的补充说明

**用户原话:** *"这个先不着急"*

我的理解：
1. 当前 Focus 在 MCU 端本地功能完整性
2. 上位机通讯协议（USART2）后续单独开发
3. SLAM+ 导航由树莓派 5 承担，MCU 只做执行层

**建议:**
- 保留 USART2 框架（PA2/RX, PA3/TX）
- 定义简单二进制协议模板备用：
  ```
  [0xAA] [CMD_ID] [LEN] [PAYLOAD...] [CRC16] [0x55]
  ```
- 等 SLAM 建图完成后对接即可

---

## 🎬 总结

**整体评价:** A- (优秀！有少量细节待优化)

**亮点:**
- 架构设计专业，面向对象封装规范
- SPI 资源管理巧妙，解决总线冲突
- 代码可读性强，注释详细完整

**待改进:**
- 引脚配置核对要更严谨（尤其是 BUZZER）
- SPI 74HC595 发送逻辑不能留 TODO
- CANopen SDO 在真实硬件接入前不宜过度抽象

**下一步行动:**
1. 修复上述 P0/P1 级别问题
2. 编译测试所有 MSH 命令
3. 准备烧录验证物理连接

---

_Reviewed by Wuji | 2026-03-14 15:00 | V1.0.11 Code Audit Complete ✨_
