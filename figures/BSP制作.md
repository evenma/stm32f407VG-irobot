![image-20260212104232200](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\image-20260212104232200.png)

<STM32系列BSP制作教程>

D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\docs

视频教程

https://www.bilibili.com/video/BV17J411V7xb/?vd_source=645d13a9720b7b8188faf7aa93c12af5

1. 复制模版工程并改名

   ![image-20260212110956115](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\image-20260212110956115.png)

2.修改board文件夹下的每个文件

​    1 cubeMX配置工程

![局部截取_20260212_141914](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260212_141914.png)

cubeMX升级 注意：不升级用不了  需要 账户登录  st账号密码

![局部截取_20260224_141700](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260224_141700.png)

配置管脚和功能

![局部截取_20260224_142143](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260224_142143.png)

配置时钟 分频

![局部截取_20260224_142945](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260224_142945.png)

工程设置  安装如下配置

![局部截取_20260224_143556](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260224_143556.png)

![局部截取_20260224_143755](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260224_143755.png)

配置board.c和board.h

flash  stm32f407ve 是512K    VG 是1M

#define STM32_FLASH_SIZE             (1024 * 1024)

#define STM32_FLASH_START_ADRESS     ((uint32_t)0x08000000)

注意：STM32F407 的 SRAM 构成

- **128KB 常规 SRAM**：起始地址 `0x2000 0000`，容量 128KB。这是系统默认使用的 RAM，可被所有总线主控（如 DMA、USB 等）访问。
- **64KB CCM RAM**：起始地址 `0x1000 0000`，容量 64KB。它**只能被 CPU 通过数据总线访问**，DMA 等外设无法直接访问。

所以，#define STM32_SRAM_SIZE           128    而 不是  192

RT-Thread 通常将 **常规 SRAM** 作为系统的默认内存堆（heap）和全局变量（.bss/.data）的存放区域

```
link.lds中，`.data`、`.bss`、`.stack 段的作用 它们通常都位于 SRAM 中，但用途和特性不同
```

### 1. `.data` 段（已初始化数据段）

- **内容**：存放已经初始化的全局变量和静态变量（例如 `int var = 100;`）。
- **存储位置**：
  - **初始值**：保存在 Flash（只读存储器）中。
  - **运行时**：在程序启动时，由启动代码（`startup_*.s`）从 Flash 复制到 SRAM 中。因此，`.data` 段最终在 SRAM 中占据空间。
- **为什么需要复制**：因为全局变量在程序运行期间可能被修改，所以必须放在可写的 RAM 中，但初始值需要从非易失的 Flash 加载。

### 2. `.bss` 段（未初始化数据段）

- **内容**：存放未初始化的全局变量和静态变量（例如 `int var;`），或者显式初始化为 0 的变量（通常编译器会将其归入 `.bss` 以节省 Flash 空间）。
- **存储位置**：
  - **初始值**：没有初始值，或者默认初始化为 0。
  - **运行时**：启动代码会将这段内存区域全部清零，因此它在 SRAM 中占据空间。
- **注意**：`.bss` 段不占用 Flash 空间（只记录大小），这有助于减小程序体积。

### 3. `.stack` 段（栈区）

- **内容**：用于函数调用时的局部变量、函数参数、返回地址以及 CPU 中断上下文等。
- **存储位置**：始终位于 SRAM 中。
- **特点**：
  - 栈是向下生长的（从高地址向低地址使用），由栈指针（SP）管理。
  - 栈的大小通常在链接脚本中固定（例如 `_estack` 定义栈顶，`_Min_Stack_Size` 定义大小）。
  - 如果栈溢出，可能会破坏其他数据（如 `.bss` 或堆），因此需要合理设置栈大小。

4. 堆（heap，用于动态内存分配，如 `malloc` 或 RT-Thread 的 `rt_malloc`）

**典型的内存布局（从低地址到高地址）**：

```
0x20000000  →  .data (已初始化变量)
              .bss  (未初始化变量)
              heap  (堆，向上增长)
              ...   (空闲内存)
              stack (栈，向下增长)
0x20020000  →  栈顶（例如 _estack，即 0x20020000）
```

注意：栈通常放在 RAM 的最高地址，以便向下生长时不会轻易与其他区域冲突。

配置 SConscript 中的启动文件路径：

```
rt-thread\bsp\stm32\stm32f407VE-irobot\board\CubeMX_Config\Drivers\CMSIS\Device\ST\STM32F4xx\Source\Templates\arm\startup_stm32f407xx.s
```

注意：在文件中表示为：

```
    src += [startup_path_prefix + '/STM32F4xx_HAL/CMSIS/Device/ST/STM32F4xx/Source/Templates/arm/startup_stm32f407xx.s']
```

并不是错误

Kconfig配置 

![deepseek_mermaid_20260225_da0526](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\deepseek_mermaid_20260225_da0526.png)

### 三个核心文件的分工

为了让你更清晰地理解，这里总结了这三个关键文件在整个流程中的角色：

| 文件                             | 角色/作用        | 关键点                                                       |
| :------------------------------- | :--------------- | :----------------------------------------------------------- |
| **`Kconfig`** (你正在编辑的文件) | **“菜单定义者”** | 定义了menuconfig界面里有什么选项、选项的类型（布尔、整型等）、默认值以及依赖关系。 |
| **`.config`** (在BSP根目录)      | **“配置存档”**   | 执行 `menuconfig` 并保存后自动生成。它以 `CONFIG_` 为前缀，记录了你的所有最终选择，是生成 `rtconfig.h` 的直接依据。 |
| **`rtconfig.h`** (在BSP根目录)   | **“最终执行者”** | 由系统根据 `.config` 文件**自动生成**。`#define BSP_USING_I2C1` 这样的宏就在这里。**永远不要手动修改它**，任何修改都会在下次运行 `menuconfig` 时被覆盖。 |



LVGL在RT-Thread中是以**软件包（package）**的形式存在的，而不是BSP的一部分。这意味着：

- **LVGL源码不在BSP目录下**：当你通过menuconfig使能LVGL软件包后，系统会自动从GitHub下载LVGL源码到`packages`目录下。这就是为什么你在`stm32f407-rt-spark\board\ports\lvgl`下只看到移植接口文件（如`lv_port_disp.c`），而没有LVGL系统文件——系统文件在`packages`文件夹里。
- **软件包有自己的Kconfig配置**：LVGL软件包本身带有一个完整的Kconfig文件，包含了版本选择、颜色深度、内存设置、功能裁剪等所有详细配置选项。当你选中`PKG_USING_LVGL`后，这些配置选项会自动出现在menuconfig中。
- **BSP只需要做两件事**：
  1. 使能软件包（`select PKG_USING_LVGL`）
  2. 提供显示和输入设备的移植接口（`lv_port_disp.c`、`lv_port_indev.c`）

LVGL需要消耗的资源过多，目前配置u8g2作为OLED屏界面

IROM1  的  Size  0x80000 改为  0x100000    

![局部截取_20260225_143419](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260225_143419.png)



menuconfig 多出来的RNG硬件真随机数生成以及UDID芯片序列号读取方法：

![局部截取_20260225_151853](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260225_151853.png)

```
#include "board.h"

#ifdef BSP_USING_RNG
    #include "hal_data.h"
    RNG_HandleTypeDef hrng;
    
    void rng_init(void) {
        __HAL_RCC_RNG_CLK_ENABLE();
        hrng.Instance = RNG;
        HAL_RNG_Init(&hrng);
    }
    
    uint32_t get_random(void) {
        uint32_t val;
        HAL_RNG_GenerateRandomNumber(&hrng, &val);
        return val;
    }
#endif

#ifdef BSP_USING_UDID
    void get_chip_id(uint32_t *id) {
        id[0] = *(uint32_t*)0x1FFF7A10;
        id[1] = *(uint32_t*)0x1FFF7A14;
        id[2] = *(uint32_t*)0x1FFF7A18;
    }
#endif

int main(void) {
    board_init();
    
    #ifdef BSP_USING_RNG
        rng_init();
        printf("RNG Test: %lu\n", get_random());
    #endif
    
    #ifdef BSP_USING_UDID
        uint32_t uid[3];
        get_chip_id(uid);
        printf("Chip ID: %08lx %08lx %08lx\n", uid[0], uid[1], uid[2]);
    #endif
    
    while(1) {
        rt_thread_mdelay(1000);
    }
}
```

应用场景：比如想用RNG为加密算法生成密钥，或者用UDID做设备认证

![局部截取_20260311_154452](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VG-irobot\figures\pics\局部截取_20260311_154452.png)

编译出现 `drv_crypto.c` 中 `RNG_HandleTypeDef` 未定义的错误。这是因为你启用了硬件随机数发生器（RNG）功能（`BSP_USING_RNG`），但新的工程配置中缺少 RNG 对应的 HAL 库支持，导致类型定义缺失。

## 🔍 错误原因

- `drv_crypto.c` 中使用了 RNG 的 HAL 类型和函数，但编译时没有找到对应的头文件和实现。这通常是因为宏 `HAL_RNG_MODULE_ENABLED` 未在 `stm32f4xx_hal_conf.h` 中定义，或者相应的 HAL 库源文件未被编译。

  

## SConstruct,SConscript,KConfig

STM32F4xx_HAL的驱动，直接在packages包中包含了最新版，原位置在board\CubeMX_Config\Drivers移植到rt-thread\bsp\stm32\libraries下创建STM32F4xx_HAL，在SConstruct中添加Libraries。

#老版本需要把board\CubeMX_Config\Drivers文件夹改名为"STM32F4xx_HAL",然后复制到.\libraries路径下。
#新版本的Library在升级包里已包含,不需要设置,packages文件夹下两个包stm32f4_cmsis_driver-latest,stm32f4_hal_driver-latest

图中左边是工程目录文件，右边是sconscript文件

![局部截取_20260227_090039](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260227_090039.png)

D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot路径下的几个重要配置文件

1. Kconfig  明确menuconfig需要配置的内容，BSP路径，RTT路径，更新包路径等

```
mainmenu "RT-Thread Configuration"

BSP_DIR := .

RTT_DIR := ../../..

PKGS_DIR := packages

config SOC_STM32F407VE
    bool
    select SOC_SERIES_STM32F4
    select RT_USING_COMPONENTS_INIT
    select RT_USING_USER_MAIN
    default y

config BOARD_STM32F407_QYiRobot
    bool
    default y

source "$(RTT_DIR)/Kconfig"
osource "$PKGS_DIR/Kconfig"
rsource "../libraries/Kconfig"

if !RT_USING_NANO
rsource "board/Kconfig"
endif

```

1.2 子文件夹board下的Kconfig ，配置了芯片用到的资源

```
menu "Hardware Drivers Config"

config SOC_STM32F407VE                   注意 ：SOC_STM32F407VG
    bool
    select SOC_SERIES_STM32F4
    select RT_USING_COMPONENTS_INIT
    select RT_USING_USER_MAIN
    default y

menu "Onboard Peripheral Drivers"
    config BSP_USING_USB_TO_USART
        bool "Enable USB TO USART (uart1)"
        select BSP_USING_UART
        select BSP_USING_UART1
        default y
		help
            UART1 is used for debugging via USB-to-serial.
		
    config BSP_USING_UART2_CTRL
        bool "Enable USB TO USART (uart2)"
        select BSP_USING_UART
        select BSP_USING_UART2
        default y
		help
            UART2 communicates with the host PC for command reception and data transmission.

    menuconfig BSP_USING_RS485
        bool "Enable UART4 for RS485 bus"
        select BSP_USING_UART
        select BSP_USING_UART4
        default y
        if BSP_USING_RS485
            config BSP_UART4_485_RE_PIN
                int "RS485 RE/DE control pin number (PD0 = 48)"
                range 0 143
                default 48
                help
                    Pin number for controlling RS485 transceiver direction.
                    High level enables transmission, low level enables reception.
                    PD0 corresponds to pin number 48 (port D, pin 0).

        endif

	menuconfig BSP_USING_UART5_BT
			bool "Enable UART5 for Bluetooth module (DX2002)"
			select BSP_USING_UART
			select BSP_USING_UART5
			default y
			if BSP_USING_UART5_BT
				 config BSP_UART5_BT_RST_PIN
					int "Bluetooth module reset pin number (PD1 = 49, -1 if unused)"
					range -1 143
					default 49
					help
						Pin number to control Bluetooth module reset (active low).
						Set to -1 if reset control is not needed.		
			endif		
endmenu

menu "On-chip Peripheral Drivers"

    config BSP_USING_GPIO
        bool "Enable GPIO"
        select RT_USING_PIN
        default y
		
	config BSP_USING_ON_CHIP_FLASH
        bool "Enable on-chip FLASH"
        default n 	
		
    menuconfig BSP_USING_UART
        bool "Enable UART"
        default y
        select RT_USING_SERIAL
        if BSP_USING_UART
            config BSP_STM32_UART_V1_TX_TIMEOUT
                int "UART TX timeout"
                default 2000
                depends on RT_USING_SERIAL_V1
                				
			menuconfig BSP_USING_UART1
                bool "Enable UART1"
                default y
                if BSP_USING_UART1
                    config BSP_UART1_RX_USING_DMA
                        bool "Enable UART1 RX DMA"
                        depends on BSP_USING_UART1 && RT_SERIAL_USING_DMA
                        default n

                    config BSP_UART1_TX_USING_DMA
                        bool "Enable UART1 TX DMA"
                        depends on BSP_USING_UART1 && RT_SERIAL_USING_DMA
                        default n

                    config BSP_UART1_RX_BUFSIZE
                        int "Set UART1 RX buffer size"
                        range 64 65535
                        depends on RT_USING_SERIAL_V2
                        default 256

                    config BSP_UART1_TX_BUFSIZE
                        int "Set UART1 TX buffer size"
                        range 0 65535
                        depends on RT_USING_SERIAL_V2
                        default 0

                    config BSP_UART1_DMA_PING_BUFSIZE
                        int "Set UART1 RX DMA ping-pong buffer size"
                        range 32 65535
                        depends on RT_USING_SERIAL_V2 && BSP_UART1_RX_USING_DMA
                        default 64    
                endif	
				
			menuconfig BSP_USING_UART2
                bool "Enable UART2"
                default y
                if BSP_USING_UART2
                    config BSP_UART2_RX_USING_DMA
                        bool "Enable UART2 RX DMA"
                        depends on BSP_USING_UART2 && RT_SERIAL_USING_DMA
                        default n

                    config BSP_UART2_TX_USING_DMA
                        bool "Enable UART2 TX DMA"
                        depends on BSP_USING_UART2 && RT_SERIAL_USING_DMA
                        default n

                    config BSP_UART2_RX_BUFSIZE
                        int "Set UART2 RX buffer size"
                        range 64 65535
                        depends on RT_USING_SERIAL_V2
                        default 256

                    config BSP_UART2_TX_BUFSIZE
                        int "Set UART2 TX buffer size"
                        range 0 65535
                        depends on RT_USING_SERIAL_V2
                        default 0

                    config BSP_UART2_DMA_PING_BUFSIZE
                        int "Set UART2 RX DMA ping-pong buffer size"
                        range 32 65535
                        depends on RT_USING_SERIAL_V2 && BSP_UART2_RX_USING_DMA
                        default 64                        
                endif
			
			menuconfig BSP_USING_UART4
                bool "Enable UART4"
                default y
                if BSP_USING_UART4
                    config BSP_UART4_RX_USING_DMA
                        bool "Enable UART4 RX DMA"
                        depends on BSP_USING_UART4 && RT_SERIAL_USING_DMA
                        default n

                    config BSP_UART4_TX_USING_DMA
                        bool "Enable UART4 TX DMA"
                        depends on BSP_USING_UART4 && RT_SERIAL_USING_DMA
                        default n

                    config BSP_UART4_RX_BUFSIZE
                        int "Set UART4 RX buffer size"
                        range 64 65535
                        depends on RT_USING_SERIAL_V2
                        default 256

                    config BSP_UART4_TX_BUFSIZE
                        int "Set UART4 TX buffer size"
                        range 0 65535
                        depends on RT_USING_SERIAL_V2
                        default 0

                    config BSP_UART4_DMA_PING_BUFSIZE
                        int "Set UART4 RX DMA ping-pong buffer size"
                        range 32 65535
                        depends on RT_USING_SERIAL_V2 && BSP_UART4_RX_USING_DMA
                        default 64                        
                endif
				
            menuconfig BSP_USING_UART5
                bool "Enable UART5"
                default y
                if BSP_USING_UART5
                    config BSP_UART5_RX_USING_DMA
                        bool "Enable UART5 RX DMA"
                        depends on BSP_USING_UART5 && RT_SERIAL_USING_DMA
                        default n

                    config BSP_UART5_TX_USING_DMA
                        bool "Enable UART5 TX DMA"
                        depends on BSP_USING_UART5 && RT_SERIAL_USING_DMA
                        default n

                    config BSP_UART5_RX_BUFSIZE
                        int "Set UART5 RX buffer size"
                        range 64 65535
                        depends on BSP_USING_UART5
                        default 256

                    config BSP_UART5_TX_BUFSIZE
                        int "Set UART5 TX buffer size"
                        range 0 65535
                        depends on BSP_USING_UART5
                        default 0

                    config BSP_UART5_DMA_PING_BUFSIZE
                        int "Set UART5 RX DMA ping-pong buffer size"
                        range 32 65535
                        depends on RT_USING_SERIAL_V2 && BSP_UART5_RX_USING_DMA
                        default 64                        
                endif				
        endif

	menuconfig BSP_USING_CAN
        bool "Enable CAN"
        default y
        select RT_USING_CAN
        if BSP_USING_CAN
            config BSP_CAN_USING_HDR
                bool "Enable CAN using hdr"
				select RT_CAN_USING_HDR
                default y	
			config BSP_USING_CAN1
				bool "Enable CAN1"
				default y
        endif

    menuconfig BSP_USING_SPI
        bool "Enable SPI BUS"
        default y
        select RT_USING_SPI
        if BSP_USING_SPI
            config BSP_USING_SPI2
                bool "Enable SPI2 BUS"
                default y
				
			config BSP_SPI2_TX_USING_DMA
                bool "Enable SPI2 TX DMA"
                depends on BSP_USING_SPI2
                default n
        endif
		
	menuconfig BSP_USING_TIM
        bool "Enable timer"
        default y
        select RT_USING_HWTIMER
        if BSP_USING_TIM
            config BSP_USING_TIM3
                bool "Enable TIM3"
                default y
			config BSP_USING_TIM5     
				bool "Enable TIM5"
				default y
        endif

	menuconfig BSP_USING_PWM
        bool "Enable PWM"
        default y
        select RT_USING_PWM
        if BSP_USING_PWM
			menuconfig BSP_USING_PWM5
				bool "Enable timer5 output PWM"
				depends on BSP_USING_TIM5 
				default y
				if BSP_USING_PWM5
					config BSP_USING_PWM5_CH1
						bool "Enable PWM5 channel1"
						default y
					config BSP_USING_PWM5_CH2
						bool "Enable PWM5 channel2"
						default y					
				endif
        endif

    menuconfig BSP_USING_ADC
        bool "Enable ADC"
        default y
        select RT_USING_ADC
        if BSP_USING_ADC
            config BSP_USING_ADC1
                bool "Enable ADC1"
                default y
        endif
		
    config BSP_USING_WDT
        bool "Enable Watchdog Timer"
        select RT_USING_WDT
        default y
		
	menuconfig BSP_USING_I2C
		bool "Enable I2C BUS (Hardware)"
		default y
		select RT_USING_I2C
		select STM32F4_HAL_I2C
		if BSP_USING_I2C
			 config BSP_USING_I2C1
				bool "Enable I2C1 BUS"
				default y
				if BSP_USING_I2C1
                    config BSP_I2C1_SCL_PIN
                        int "i2c1 scl pin number,PB6=22"
                        range 0 143
                        default 22
                    config BSP_I2C1_SDA_PIN
                        int "I2C1 sda pin number,PB7=23"
                        range 0 143
                        default 23
                endif
				
            config BSP_USING_I2C2
                bool "Enable I2C2 BUS"
                default y
				if BSP_USING_I2C2
                    config BSP_I2C2_SCL_PIN
                        int "i2c2 scl pin number, PB10=26"
                        range 0 143
                        default 26
                    config BSP_I2C2_SDA_PIN
                        int "I2C2 sda pin number, PB11=27"
                        range 0 143
                        default 27
                endif
		endif	
		
    source "$(BSP_DIR)/../libraries/HAL_Drivers/drivers/Kconfig"

endmenu

menu "Board extended module Drivers"

endmenu

endmenu

```



2. SConscript 生成编译需要的环境变量，以及添加子文件夹SConscript

```
# for module compiling
import os
Import('RTT_ROOT')
Import('env')
from building import *

cwd = GetCurrentDir()
objs = []
list = os.listdir(cwd)

env.Append(CPPDEFINES = ['STM32F407xx'])

for d in list:
    path = os.path.join(cwd, d)
    if os.path.isfile(os.path.join(path, 'SConscript')):
        objs = objs + SConscript(os.path.join(d, 'SConscript'))

Return('objs')

```

![局部截取_20260226_102021](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260226_102021.png)

2.1 子文件夹application的Sconscript，在Keil目录中添加Applications组以及.c文件

```
from building import *
import os

cwd = GetCurrentDir()
src = Glob('*.c')
CPPPATH = [cwd]

group = DefineGroup('Applications', src, depend = [''], CPPPATH = CPPPATH)

list = os.listdir(cwd)
for item in list:
    if os.path.isfile(os.path.join(cwd, item, 'SConscript')):
        group = group + SConscript(os.path.join(item, 'SConscript'))

Return('group')
```

![局部截取_20260226_102410](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260226_102410.png)

2.2 子文件夹board的Sconscript，在Keil目录中添加Drivers组以及.c文件

```
import os
from building import *

cwd = GetCurrentDir()

# add general drivers
src = Split('''
board.c
CubeMX_Config/Src/stm32f4xx_hal_msp.c
''')
#CubeMX_Config/MDK-ARM/startup_stm32f407xx.s  此文件在packages\stm32f4_cmsis_driver-latest\Source\Templates\arm中，是pkgs --update下载过来
path =  [cwd]
path += [os.path.join(cwd, 'CubeMX_Config', 'Inc')]


# STM32F405xx) || STM32F415xx) || STM32F407xx) || STM32F417xx)
# STM32F427xx) || STM32F437xx) || STM32F429xx) || STM32F439xx)
# STM32F401xC) || STM32F401xE) || STM32F410Tx) || STM32F410Cx)
# STM32F410Rx) || STM32F411xE) || STM32F446xx) || STM32F469xx)
# STM32F479xx) || STM32F412Cx) || STM32F412Rx) || STM32F412Vx)
# STM32F412Zx) || STM32F413xx) || STM32F423xx)
# You can select chips from the list above
CPPDEFINES = ['STM32F407xx']
group = DefineGroup('Drivers', src, depend = [''], CPPPATH = path, CPPDEFINES = CPPDEFINES)

list = os.listdir(cwd)
for item in list:
    if os.path.isfile(os.path.join(cwd, item, 'SConscript')):
        group = group + SConscript(os.path.join(item, 'SConscript'))

Return('group')

```

![局部截取_20260226_102925](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260226_102925.png)

其他配置，如模版文件的配置：

![局部截取_20260226_154324](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260226_154324.png)

还有kernel配置，发现编译报错，object name =20过短。名字长度修改为32

![局部截取_20260226_154429](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260226_154429.png)

最后编译通过，程序运行，LED灯闪烁(PB2)。

资源驱动的调用关系：如ADC1,drv_adc.c完成配置、初始化、注册等工作。

![局部截取_20260226_160503](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260226_160503.png)

![局部截取_20260226_160137](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260226_160137.png)

![局部截取_20260226_160633](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260226_160633.png)

代替了直接调用hal库配置，这样更适合框架结构，

![局部截取_20260226_160831](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260226_160831.png)

通过 **Kconfig 配置系统 + 宏定义（如你截图中的 `adc_config.h`）**来实现驱动的标准化配置。这种方式确实将硬件参数从驱动代码中分离出来，使用户无需直接操作 HAL 库的结构体，从而减少错误、提高代码复用性。

### 1. 标准化配置的实现方式

在 RT-Thread 的 BSP 中，通常采用以下分层设计：

- **Kconfig**：定义外设的使能选项（如 `BSP_USING_ADC1`）和可调参数（如采样时间、通道等）。用户在 `menuconfig` 中选择后，这些宏会被写入 `rtconfig.h`。

- **配置文件（如 `adc_config.h`）**：将 HAL 库需要的初始化参数抽象成宏。

- **驱动文件（如 `drv_adc.c`）**：通过条件编译包含这些宏，然后统一创建并注册设备

  驱动中遍历该数组，调用 `HAL_ADC_Init` 初始化，并通过 `rt_hw_adc_register` 注册到设备框架

### 2. 与直接操作 HAL 库的区别

- **直接操作 HAL**：用户需要在 `board.c` 或应用层编写 `MX_ADC1_Init` 这样的函数，手动填充结构体并处理错误。这种方式代码分散、不易维护，且更换硬件时需要大量修改。
- **RT-Thread 标准化驱动**：用户只需在 `menuconfig` 中勾选并配置参数，驱动框架自动完成初始化和注册。用户应用层通过设备文件接口（如 `rt_device_read`）操作 ADC，无需关心底层 HAL 细节。

### 4. 优点总结

- **配置与实现分离**：硬件参数集中在 `adc_config.h`，驱动逻辑集中在 `drv_adc.c`，修改参数无需改动驱动代码。
- **减少重复代码**：多个同类型外设（如 ADC1/2/3）共用一套驱动逻辑。
- **易移植**：更换芯片时，只需修改配置宏和底层 HAL 库，应用层代码完全不变。
- **符合 RT-Thread 设备框架**：应用通过标准接口访问外设，提高代码可读性和可维护性。

## ISP烧写程序

直接用烧写器ATK-XISP.exe烧写程序  xxx.hex文件

**DTR** **低电平复位，****RTS** **高电平进** ****  或者  按住boot0按键再按一下reset按键，进入ISP烧写

![局部截取_20260303_145306](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260303_145306.png)

但是，由于系统支持OTA,系统有bootloader,所以全片擦除会导致没有bootloader，系统起不来

## 关于扇区大小

- **扇区大小**：无论是 512KB 的 VET6 还是 1MB 的 VGT6，其扇区组成都是：**前 4 个扇区为 16KB，第 5 个扇区为 64KB，后面的扇区为 128KB** 。这是 F407 系列的硬件固定设计。

**扇区数量**：这是两者唯一的区别。

- **STM32F407VE** (512KB)：拥有 Sector 0 到 Sector 7，共 **8 个扇区** (4个16K + 1个64K + 3个128K) 。

- **STM32F407VG** (1MB)：拥有 Sector 0 到 Sector 11，共 **12 个扇区** (4个16K + 1个64K + 7个128K)

  简单来说，VG 就是在 VE 的基础上，在后面**追加了 4 个额外的 128KB 扇区**，将总容量从 512KB 扩展到了 1MB。

 STM32F407 系列的 Flash 的擦除按上面的扇区划分来擦写。不存在 2KB 或 4KB 的扇区



## 增加OTA

参考文档路径：https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/application-note/system/rtboot/an0028-rtboot

STM32F407VET5的flash有512K，从0x8000000 -0x807FFFF

bootloader在0x8000000-0x801FFFF内，最开始的128K空间,文件大小是18.3K,

制作该 app 固件有如下三个步骤：

- 为 BSP 添加下载器功能，下载需要的软件包并修改 FAL 分区表
- 修改 stm32 BSP 中断向量表跳转地址
- 修改 BSP 链接脚本

固件中使用的分区表如下所示：

| 分区名   | 起始地址  | 分区大小 | 分区位置   |
| -------- | --------- | -------- | ---------- |
| app      | 0x8020000 | 128k     | 片内 Flash |
| download | 0x8040000 | 128k     | 片内 Flash |
| factory  | 0x8060000 | 128k     | 片内 Flash |

1. 下载 ota_downloader 软件包，选中 Ymodem 功能。

   ![局部截取_20260227_101324](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260227_101324.png)

配置  config BSP_USING_ON_CHIP_FLASH        bool "Enable on-chip FLASH"        select RT_USING_FAL 

配置FAL分区表  board\ports\fal_cfg.h

```

#define RT_APP_PART_ADDR 0x08020000
#define NOR_FLASH_DEV_NAME             "norflash0"

#define FLASH_SIZE_GRANULARITY_16K (4*16*1024)
#define FLASH_SIZE_GRANULARITY_64K (64*1024)
#define FLASH_SIZE_GRANULARITY_128K (3*128*1024)

#define STM32_FLASH_START_ADRESS_16K STM32_FLASH_START_ADRESS
#define STM32_FLASH_START_ADRESS_64K (STM32_FLASH_START_ADRESS_16K + FLASH_SIZE_GRANULARITY_16K)
#define STM32_FLASH_START_ADRESS_128K (STM32_FLASH_START_ADRESS_64K + FLASH_SIZE_GRANULARITY_64K)

/* ===================== Flash device Configuration ========================= */
extern const struct fal_flash_dev stm32_onchip_flash_16k;
extern const struct fal_flash_dev stm32_onchip_flash_64k;
extern const struct fal_flash_dev stm32_onchip_flash_128k;
/* 调用libraries\HAL_Drivers\drivers\drv_flash\drv_flash_f4.c */

/* flash device table */
#define FAL_FLASH_DEV_TABLE                                          \
{                                                                    \
    &stm32_onchip_flash_16k,                                           \
	&stm32_onchip_flash_64k,                                           \
	&stm32_onchip_flash_128k,                                           \
}
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
/* partition table */
#define FAL_PART_TABLE                                                               \
{                                                                                    \
    {FAL_PART_MAGIC_WORD,"bl",            "onchip_flash_128k",    0,            128*1024,    0}, \
    {FAL_PART_MAGIC_WORD,"app",            "onchip_flash_128k",    128*1024,    128*1024,    0}, \
    {FAL_PART_MAGIC_WORD,"download",    "onchip_flash_128k",    256*1024,    128*1024,    0}, \
    {FAL_PART_MAGIC_WORD,"factory",    "onchip_flash_128k",    384*1024,    128*1024,    0}, \
}
#endif /* FAL_PART_HAS_TABLE_CFG */
```

第一次烧写app需要jtag仿真器烧写，因为芯片内没app代码，只有bootloader,没有msh命令行输入。

![局部截取_20260227_142328](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260227_142328.png)

## 重新生成工程文件

```
scons -c && scons
```

注意：uart4的DMA不要开启，根据 STM32F407 参考手册，UART4_TX 通常使用 **DMA1_Stream4**，通道 **4**，开启后报错。比较特殊，和其他的串口不同

```
/* UART4 TX DMA 配置 */
#define UART4_TX_DMA_INSTANCE              DMA1_Stream4
#define UART4_TX_DMA_CHANNEL                DMA_CHANNEL_4
#define UART4_TX_DMA_RCC                    RCC_AHB1Periph_DMA1
#define UART4_TX_DMA_IRQ                     DMA1_Stream4_IRQn
```

```
menuconfig
配置好后保存，再退出 
pkgs --update
更新packages文件夹 生成需要的包
scons --target=mdk5
生成Keil5工程
```

bootloader重新配置

https://iot.rt-thread.com/#/homePage

![局部截取_20260311_155152](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VG-irobot\figures\pics\局部截取_20260311_155152.png)

![局部截取_20260311_155438](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VG-irobot\figures\pics\局部截取_20260311_155438.png)

![局部截取_20260311_155839](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VG-irobot\figures\pics\局部截取_20260311_155839.png)

![局部截取_20260311_155916](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VG-irobot\figures\pics\局部截取_20260311_155916.png)

固件中使用的分区表如下所示：

| 分区名   | 起始地址  | 分区大小 | 分区位置   |
| -------- | --------- | -------- | ---------- |
| app      | 0x8020000 | 256k     | 片内 Flash |
| download | 0x8060000 | 256k     | 片内 Flash |
| factory  | 0x80a0000 | 256k     | 片内 Flash |

### 修改链接脚本（`.sct` 文件）

将 `0x00100000` 改为实际可用的 `0x000E0000`：

```
-LR_IROM1 0x08020000 0x00100000  {
+LR_IROM1 0x08020000 0x000E0000  {
   ER_IROM1 0x08020000 0x000E0000  {
    *.o (RESET, +First)
    *(InRoot$$Sections)
    .ANY (+RO)
   }
   RW_IRAM1 0x20000000 0x00020000  {
    .ANY (+RW +ZI)
   }
}
```



同时，建议将 Keil Target 对话框中的 IROM1 也改为匹配的值（虽然自定义链接脚本会覆盖，但保持一致性有助于避免误解）：

- **IROM1**：Start = `0x08020000`，Size = `0xE0000`（注意十六进制写法）

## 虚焊

注意：电路板串口芯片CH9102F出现TXD引脚虚焊导致键盘无法输入到串口，而且2套电路板同样的问题。引起程序以及配置大量的调整还是无法解决问题，最后通过示波器测量发现串口信息TXD脚没输出，低电平。根据手册，TXD和RXD没有数据是高电平的。

OTA升级包

![局部截取_20260228_103213](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260228_103213.png)

![局部截取_20260228_103256](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260228_103256.png)

传输完成后，自动复位，升级成功。

再打包factory 出厂包 烧写进去

## factory分区出厂恢复

![局部截取_20260228_103714](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260228_103714.png)

打包软件的固件分区名factory 这样子是错误的。应该还是app

![局部截取_20260228_113746](D:\rtt_v5\rt-thread-5.2.2\rt-thread\bsp\stm32\stm32f407VE-irobot\figures\pics\局部截取_20260228_113746.png)

在恢复出厂设置时，同时按住PC13和PC14对应的SW3和SW4按键10S，这样擦除的是app分区，而不是之前的factory分区

```
命令：
ymodem_ota -p factory
传输的固件和app相同，只有版本号不同或相同
正常升级：
ymodem_ota
```

到目前，编译后的代码大小：

```
Program Size: Code=107926 RO-data=11618 RW-data=2740 ZI-data=5388  
```

下一步，增加ulog以及lvgl

## 开启ulog日志功能

| 配置项                                     | 推荐状态 | 主要作用 / 使用说明                                          |
| :----------------------------------------- | :------- | :----------------------------------------------------------- |
| **[\*] Enable ulog**                       | **必选** | ulog功能的总开关。只有勾选此项，下面的子选项才会出现。       |
| **(X) The static output log level**        | **默认** | **静态输出级别**。在编译时决定最低的日志输出级别（如 `LOG_LVL_DBG`）。比设定级别低的日志（如调试日志）将不会被编译，有助于节省代码空间。 |
| **[ ] Enable ISR log.**                    | 可选     | **使能中断日志**。如果你需要在中断服务函数中输出日志，可以勾选此项。 |
| **[\*] Enable assert check.**              | 推荐     | **使能断言检查**。开启后，当程序触发 `ASSERT` 时会输出断言日志。 |
| **(128) The log's max width.**             | 默认     | **单行日志的最大长度**（字节）。建议保持默认或根据你的日志内容长度适当调整。 |
| **[ ] Enable async output mode.**          | 可选     | **异步输出模式**。开启后，日志不会立即输出，而是先放入缓存，由专门的线程负责输出，可以减少对业务代码的阻塞。如果开启此项，请注意合理配置缓冲区大小。 |
| **log format --->**                        | 推荐     | **日志格式配置**（**强烈推荐进入子菜单**）。                 |
| `[*] Enable color log.`                    | 推荐     | 使能颜色日志，不同级别的日志在终端会显示不同颜色，更易阅读。 |
| `[*] Enable time information.`             | 推荐     | 在日志前添加系统时间戳（如 tick 值），便于时序分析。         |
| `[*] Enable level information.`            | 推荐     | 显示日志级别（如 D/I/W/E），默认开启。                       |
| `[*] Enable tag information.`              | 推荐     | 显示日志的标签（Tag），方便按模块过滤，默认开启。            |
| `[ ] Enable thread information.`           | 可选     | 显示当前线程名，有助于多线程调试，但会略微增加日志长度和开销。 |
| `[ ] Enable float number support.`         | 可选     | 如果日志需要打印浮点数，可以开启此选项。它会占用更多线程栈空间。 |
| **[\*] Enable console backend.**           | **必选** | **使能控制台后端**。勾选后，日志会输出到控制台（即你正在使用的串口）。这是最常用的输出方式，建议保持开启。 |
| **[ ] Enable file backend.**               | 可选     | 使能文件后端。如果需要将日志保存到文件系统，可以开启。       |
| **[ ] Enable runtime log filter.**         | 推荐     | **使能运行时日志过滤**。开启后，可以在设备运行时通过 `ulog_tag_lvl`、`ulog_lvl` 等命令动态调整日志输出级别或按标签过滤，调试非常方便。 |
| **[\*] Enable syslog format log and API.** | 默认     | 使能兼容 syslog 格式的日志和 API。                           |

1. **基础配置**：如果你是初次使用，建议先勾选 **`Enable ulog`**、**`Enable console backend`**，并在 **`log format`** 子菜单中勾选**颜色、时间、级别、标签**这几项。保存并编译后，你的 `LOG_I()`、`LOG_E()` 等宏就会输出带颜色和格式的日志了。
2. **进阶调试**：在项目调试阶段，可以开启 **`Enable runtime log filter`**。这样，你可以在 `msh` 中动态地查看或过滤日志。例如：
   - 只查看 `wifi` 标签下警告级别及以上的日志：`ulog_tag_lvl wifi W`。
   - 全局只输出错误级别日志：`ulog_lvl error`。

1. **在代码中使用**：在你的源文件中，只要在包含头文件前定义好标签和默认级别即可：

   c

   ```
   #define LOG_TAG "your.tag"
   #define LOG_LVL LOG_LVL_DBG
   #include <ulog.h>
   
   void a_function(void) {
       LOG_D("This is a debug message, value: %d", 42);
       LOG_I("Initialization successful.");
   }
   ```

到目前，编译后的代码大小：

```
Program Size: Code=111176 RO-data=12524 RW-data=2788 ZI-data=5716  
```



## 工程框架设计：

在applications文件夹下创建自己的项目

1. main.c实现总的初始化以及任务开启,如can通讯任务开启，action任务开启， minitor监控任务开启
2. userAction.c  用户发送过来的任务，包括从遥控器通过蓝牙模块ble发送过来的任务，以及上位机通过typeC转串口发送过来的任务,调用ActionStartCmd(cmd,parm)实现相应的doAcion(parm)任务,任务包含开启或者关闭哪个部件，哪几个部件需要配合操作，这几个部件的调用时长，是否开启超时处理，任务完成时长根据任务来设定。
3. drvBoard.c  实现每个部件操作到具体io口或者中断或iic,spi等底层功能实现。
4. userCAN.c  实现智能小车的2个差速电机驱动器的通讯控制，
5. minitor.c 实现程序监控，以及采集所有上位机需要的信息、状态等。如电池电量、充电情况、任务工作状态、两个电机速度、传感器数据、IMU数据等

applications/
├── main.c                 // 系统初始化、创建线程
├── userAction.c           // 命令解析与任务调度
├── drvBoard.c             // 硬件抽象（电机、传感器、IO）
├── userCAN.c              // CAN 通讯（电机驱动器）
├── minitor.c              // 监控与数据采集
└── includes/              // 可选：头文件集中存放
    ├── app_config.h       // 应用配置宏（如线程优先级、堆栈大小）
    ├── userAction.h
    ├── drvBoard.h
    ├── userCAN.h
    └── minitor.h

### 串口接收处理

在 RT-Thread 的串口接收处理中，使用信号量或消息队列都是常见且合理的选择，而互斥锁**不直接适用于线程间通知**。下面详细分析几种机制的优劣，并解释为什么互斥锁不会丢包但也不适合。

------

## 1. 信号量（Semaphore）

**适用场景**：只需要通知“有数据来了”，不关心具体有多少字节。

- **工作原理**：
  - 在接收回调 `uart_input` 中释放一个信号量（`rt_sem_release`）。
  - 处理线程阻塞在 `rt_sem_take` 上，信号量可用后立即调用 `rt_device_read` 读取数据。
- **优点**：
  - 轻量，开销小。
  - 实时性好，线程能立刻响应。
- **缺点**：
  - 无法传递 `size` 信息。读取时可能需要使用较大的缓冲区一次性读尽可能多的数据，或依赖协议自行解析长度。

## 2. 消息队列（Message Queue）

**适用场景**：需要将 `size`（或设备句柄、数据指针）传递给处理线程。

- **工作原理**：
  - 回调中将 `dev` 和 `size` 打包成消息发送到消息队列。
  - 处理线程从队列中取出消息，获取 `size` 后再读取相应长度（或读取任意长度，但知道至少有多少数据）。
- **优点**：
  - 可传递附加信息，便于更精确的处理（如知道确切的接收长度）。
  - 支持多生产者（如多个串口共用同一个队列）。
- **缺点**：
  - 比信号量稍重，涉及内存拷贝。
  - 可能因队列满而丢消息（但驱动内部缓冲区还在，数据不会丢，仅通知丢失，可通过增大队列长度缓解）。





## OLED屏  

menuconfig 配置OLED屏  

注意：spi2不要用DMA

生成工程文件，全部在u8g2文件夹，修改三个文件u8g2_d_memory.c

```
/* u8g2_d_memory.c */
/* generated code, codebuild, u8g2 project */

#include "u8g2.h"

uint8_t *u8g2_m_16_8_f(uint8_t *page_cnt)
{
  #ifdef U8G2_USE_DYNAMIC_ALLOC
  *page_cnt = 8;
  return 0;
  #else
  static uint8_t buf[1024];
  *page_cnt = 8;
  return buf;
  #endif
}
/* end of generated code */

```

u8g2_fonts.c

```
仅保留
const uint8_t u8g2_font_synchronizer_nbp_tf[1805] U8G2_FONT_SECTION("u8g2_font_synchronizer_nbp_tf") 
const uint8_t u8g2_font_unifont_t_symbols[9015] U8G2_FONT_SECTION("u8g2_font_unifont_t_symbols") = 
const uint8_t u8g2_font_ncenB08_tr[1166] U8G2_FONT_SECTION("u8g2_font_ncenB08_tr") = 
三种字体
```

u8g2_d_setup.c

```
/* u8g2_d_setup.c */
/* generated code, codebuild, u8g2 project */

#include "u8g2.h"

/* ssd1306 f */
void u8g2_Setup_ssd1306_128x64_noname_f(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_msg_cb byte_cb, u8x8_msg_cb gpio_and_delay_cb)
{
  uint8_t tile_buf_height;
  uint8_t *buf;
  u8g2_SetupDisplay(u8g2, u8x8_d_ssd1306_128x64_noname, u8x8_cad_001, byte_cb, gpio_and_delay_cb);
  buf = u8g2_m_16_8_f(&tile_buf_height);
  u8g2_SetupBuffer(u8g2, buf, tile_buf_height, u8g2_ll_hvline_vertical_top_lsb, rotation);
}
```

u8g2_port.c不需要修改

最后就是用户使用：特别注意u8g2_InitDisplay(&u8g2);要调用2次中间要rt_thread_mdelay(10); 这样OLED屏幕才能正常显示。

```
#define OLED_SPI_PIN_CS                      GET_PIN(B, 12)  // PB12
#define OLED_SPI_PIN_DC                      GET_PIN(B, 14)  // PB14
#define OLED_SPI_PIN_RES                     U8X8_PIN_NONE  // RESET hardware 

// 全局 U8g2 对象（便于其他函数访问，如果需要）
static u8g2_t u8g2;

int oled_battery = 24000;
char oled_msg[24];
static void u8g2_ssd1306_12864_4wire_hw_spi_example(int argc,char *argv[])
{
//    u8g2_t u8g2;
	    // Initialization
		u8g2_Setup_ssd1306_128x64_noname_f( &u8g2, U8G2_R0, u8x8_byte_rtthread_4wire_hw_spi, u8x8_gpio_and_delay_rtthread);
		u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_CS, OLED_SPI_PIN_CS);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_DC, OLED_SPI_PIN_DC);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_RESET, OLED_SPI_PIN_RES);

		rt_kprintf("OLED display\r\n");
		u8g2_InitDisplay(&u8g2);
// 1. 上电延时，确保 OLED 稳定（关键！）
		rt_thread_mdelay(10);  
		u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
	

    // Draw Graphics
    /* full buffer example, setup procedure ends in _f */
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 1, 18, "U8g2 on RT-Thread");
    u8g2_SendBuffer(&u8g2);
	
	u8g2_SetFont(&u8g2, u8g2_font_synchronizer_nbp_tf);
	sprintf(oled_msg, "BAT:%dmv", oled_battery);
	u8g2_DrawStr(&u8g2, 20, 36, oled_msg);

    u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
    u8g2_DrawGlyph(&u8g2, 112, 56, 0x2603 );
    u8g2_SendBuffer(&u8g2);

}
```

编译后：

```
上一次：
Program Size: Code=111176 RO-data=12524 RW-data=2788 ZI-data=5716  
现在：
Program Size: Code=119184 RO-data=25020 RW-data=2776 ZI-data=7048  
```

