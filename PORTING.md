# USB CDC 移植包 · 车端 H723 → 主臂 H723

> 来源：`Engineering_Robot_H723_`（车端，已验证可用）
> 目标：`STM32H7232_UserController_1`（主臂板，**同款硬件**，当前固件未启用 USB）
> 生成日期：2026-09-26

---

## 0. 这个包是什么

车端工程里 USB CDC（虚拟串口）的**完整最小文件集，共 17 个文件**。
移植完成后，主臂板可以通过 USB 线以 **COM 口**的形式与 PC 双向通信。

**硬件前提（已确认）**：两块板同款、主臂板侧边有 USB 口。
车端的实现方式是 `USB_OTG_HS` + **HSI48 内部时钟** + **内置 FS PHY**，
`HAL_PCD_MspInit()` 里**没有任何 `HAL_GPIO_Init`**
⇒ 不需要外部晶振、不需要外部 PHY 芯片、不需要额外引脚。

---

## 1. 文件清单

### 1.1 `USB_DEVICE/`（8 个，应用层 + 底层配置）

| 路径 | 作用 |
|---|---|
| `App/usb_device.c` `.h` | `MX_USB_DEVICE_Init()`：初始化设备库、注册 CDC 类、启动 |
| `App/usbd_desc.c` `.h` | USB 描述符（VID/PID、产品字符串、`HS_Desc`） |
| `App/usbd_cdc_if.c` `.h` | **应用接口**：`CDC_Transmit_HS()` / `CDC_Receive_HS()` + 收发缓冲区 |
| `Target/usbd_conf.c` `.h` | 底层：`PCD_HandleTypeDef hpcd_USB_OTG_HS` 定义、`HAL_PCD_MspInit()`（时钟 + 中断） |

### 1.2 `Middlewares/ST/STM32_USB_Device_Library/`（9 个，ST 官方库）

| 路径 | 文件 |
|---|---|
| `Core/Src` | `usbd_core.c`、`usbd_ctlreq.c`、`usbd_ioreq.c` |
| `Core/Inc` | `usbd_core.h`、`usbd_ctlreq.h`、`usbd_def.h`、`usbd_ioreq.h` |
| `Class/CDC/Src` | `usbd_cdc.c` |
| `Class/CDC/Inc` | `usbd_cdc.h` |
| 根 | `LICENSE.txt` |

---

## 2. 使用步骤

### 第 1 步 · 复制文件（保持目录结构）

把本包内容整体覆盖到主臂工程根目录：

```
robocopy _usb_port_pack  "C:\...\STM32H7232_UserController_1"  /E
```

复制后主臂工程根下应出现 `USB_DEVICE\` 和 `Middlewares\ST\STM32_USB_Device_Library\`。

### 第 2 步 · 改 `CMakeLists.txt`

**① include 目录**（在 include_directories 里加 4 行）：

```cmake
        USB_DEVICE/App USB_DEVICE/Target
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
```

**② 源文件 GLOB**：在 `file(GLOB_RECURSE SOURCES ...)` 里加 `"USB_DEVICE/*.*"`：

```cmake
file(GLOB_RECURSE SOURCES "Core/*.*" "Middlewares/*.*" "Drivers/*.*"
        "USB_DEVICE/*.*"
        "User/Task/*.*" ...)   # 其余保持原样
```

> `Middlewares/*.*` 一般已存在，会自动覆盖 STM32_USB_Device_Library 里的 .c。

### 第 3 步 · 改 4 处 Core 文件

**① `Core/Src/stm32h7xx_it.c`** — 顶部 extern：

```c
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
```

**② `Core/Src/stm32h7xx_it.c`** — 文件末尾加中断处理：

```c
/**
  * @brief This function handles USB On The Go HS global interrupt.
  */
void OTG_HS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
}
```

**③ `Core/Inc/stm32h7xx_it.h`** — 声明：

```c
void OTG_HS_IRQHandler(void);
```

**④ `Core/Src/main.c`** — 在 `/* Initialize all configured peripherals */` 段落里调用（**只调一次**）：

```c
extern void MX_USB_DEVICE_Init(void);
...
MX_USB_DEVICE_Init();
```

> ✅ `Core/Startup/startup_stm32h723vgtx.s` **不用改** —— 向量表里已有 `OTG_HS_IRQHandler` 的 weak 定义。

---

## 3. 移植完成后的应用层接口

### 3.1 发送（板 → PC）

```c
#include "usbd_cdc_if.h"

uint8_t buf[64];
/* ...填充... */
if (CDC_Transmit_HS(buf, len) != USBD_OK)
{
    /* USBD_BUSY：上一包还没发完，需要重试或丢弃 */
}
```

⚠️ **注意**：`CDC_Transmit_HS` 在上一包未完成时返回 `USBD_BUSY`。
车端的 `ArmJointDataPack()` 就是"发失败就丢弃、无重试"，移植时可按同样策略。

### 3.2 接收（PC → 板）

`CDC_Receive_HS()` 已经在中断里把数据搬进全局缓冲区并置标志位，
应用层**轮询**即可：

```c
extern uint8_t  USB_Received_Data[APP_RX_DATA_SIZE];   /* 2048 */
extern uint32_t USB_Received_Len;
extern volatile uint8_t USB_Data_Ready_Flag;

if (USB_Data_Ready_Flag == 1)
{
    /* 解析 USB_Received_Data[0 .. USB_Received_Len-1] */
    USB_Received_Len = 0;
    USB_Data_Ready_Flag = 0;
}
```

### 3.3 缓冲区尺寸（可按需调整）

| 宏 | 默认值 | 位置 |
|---|---|---|
| `APP_RX_DATA_SIZE` | 2048 | `usbd_cdc_if.h:52` |
| `APP_TX_DATA_SIZE` | 2048 | `usbd_cdc_if.h:53` |

---

## 4. 注意事项

1. ⚠️ **车端工程里 `MX_USB_DEVICE_Init()` 被调用了两次**
   （`Core/Src/main.c:129` 和 `Core/Src/freertos.c:195`）。
   主臂移植时**只保留一处**，建议放 `main.c` 的外设初始化段。
   重复调用 `USBD_Init()` 会导致句柄被反复重置。

2. ⚠️ **主臂工程 `Core/Src/Backup/` 下有 `.bak` 文件**，
   说明 Core 目录被手改过 ⇒ **不要用 CubeMX 重新生成**，手工复制 + 手改这 4 处即可。

3. 📌 **主臂工程原有代码不受影响**：
   `huart1`（舵机总线）与 `huart10`（0x0302）保持原样，USB 是新增的第三条通道。

4. 📌 **移植后主臂板的能力**：
   - PC 可**直读**主臂舵机角度（省掉"车端转发"这一环）
   - PC 可**直写**主臂舵机目标（HIL 对齐的前提之一）
   - 但"写目标 + 使能扭矩"的应用逻辑**仍需另行开发** —— 当前主臂固件只读不写。

5. 📌 **建议先做的验证**（成本低、结论重要）：
   在动 USB 之前，先用一条临时命令验证**主臂舵机加上扭矩后能否带得动自己**。
   SO-ARM101 主臂无配重，靠减速比自锁；换过舵机或改过结构就可能带不动。
