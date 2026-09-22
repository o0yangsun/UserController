# 飞特 STS3215 舵机项目交接说明

> 用途：将当前对飞特（FEETECH）STS3215 串行总线舵机、FTServo STM32 HAL SDK，以及 `STM32H7232_UserController_1` 工程集成状态的已知信息交接给另一段 Codex 会话。
>
> 整理日期：2026-07-21（Asia/Shanghai）
>
> 最近一次源码交叉检查：2026-07-21 16:56（Asia/Shanghai），以 `STM32H7232_UserController_1` 当前源码和现有构建产物为准。
>
> 重要原则：本文区分“源码已确认”“用户已确认”和“待确认”。接手后应先重新读取最新源码，不能假设本文生成后工程没有继续变化。

## 1. 一页结论

- 用户已确认舵机型号为 **STS3215**，固件版本为 **3.10**。
- 当前具体工程使用 **STM32H723、FreeRTOS/CMSIS-RTOS2、USART10、1 Mbps、8N1** 与舵机通信。
- 当前舵机 ID 在业务代码中硬编码为 **1**。
- STS 使用小端字节顺序，初始化时调用 `setEnd(0)`。
- 读取单个舵机当前位置直接调用 `ReadPos(1)`；**不需要** `syncReadBegin()`。
- 当前工程选择常规中位校准：`CalibrationOfs(1)`，其本质是向地址 `40` 写入 `128`，把**当前位置定义为逻辑刻度 2048**；它不会驱动舵机转到中位。
- 固件 3.10 还支持专用 `0x0B` 校准指令，但当前工程没有使用该路径，也没有保留相应上层封装。
- 当前业务代码存在一个必须优先处理的问题：无限循环中只调用 `sts3215_angle_get()`，没有再次调用 `sts3215_encoder_get()`，因此编码器值不会持续更新；结构体又在进入循环前被清零，当前循环计算的角度实际保持为 0。
- `UartTask` 虽由 `freertos.c` 创建，但 `send_task.c` 的用户实现已整文件注释，最终链接的是 `freertos.c` 中的弱空循环。当前队列没有有效消费者，长度为 10 的队列写满后，后续 `xQueueSend(..., 0)` 会失败且返回值被忽略。
- `ReadPos()` 的失败返回处理存在 SDK 缺陷，业务代码必须以 `getLastError()` 判断通信是否成功，不能只判断 `pos == -1`。
- 硬件半双工 TTL 总线的收发电路/接线方式尚未从源码确认；USART10 配置为独立 TX/RX 引脚，需核对板级电路是否完成单线总线适配。
- 当前目录已有晚于最新关键源码的 Debug 构建产物：对象文件生成于 2026-07-21 16:56:31～32，ELF/HEX/BIN/MAP 生成于 16:56:33；MAP 中包含当前 STS3215 任务和驱动符号，说明该版本至少已成功编译并链接。

## 2. 工程与资料位置

### 2.1 当前具体工程

```text
D:\RoboMaster\Projects\STM32H7232_UserController_1
```

关键目录：

```text
Core/                              STM32CubeMX 生成的 HAL、时钟、USART、FreeRTOS 初始化
User/Module/FTSservo/              飞特舵机 SDK 及本项目 STS3215 封装
User/Task/Algorithm/               STS3215 采样、累计角度及队列发送任务
STM32H7232_UserController.ioc      CubeMX 配置
CMakeLists.txt                     CLion/arm-none-eabi-gcc 构建配置
```

### 2.2 SDK 参考工程及资料

```text
D:\RoboMaster\Projects\STM32H7232_UserController_Temp\FTServo_stm32HAL-main
```

其中包含：

```text
SCSLib/                            原始 FTServo STM32 HAL SDK
examples/SMS_STS/FeedBack.c        单个 STS 舵机反馈读取示例
examples/SMS_STS/SyncRead.c        多舵机同步读取示例
STS舵机SDK接口用户手册.md          已整理的本地接口说明
磁编码SMS&STS&HTS-十六进制指令生成表-250508.xlsx
```

## 3. 型号信息与尚未确认项

### 3.1 已确认

- 系列：飞特 STS，TTL 半双工异步串行总线。
- 型号：STS3215。
- 固件：3.10。
- 单圈绝对位置：`0～4095`，共 4096 个刻度。
- 每刻度角度：`360 / 4096 = 0.087890625°`，通常近似写为 `0.088°`。
- 单舵机 ID：`0～253`。
- 广播 ID：`254`（`0xFE`）；广播写不应被当作普通单舵机读取 ID。

### 3.2 待确认

STS3215 存在不同供电/扭矩版本。仅凭“STS3215”不能确定是 7.4 V 版本还是 12 V 版本，接手者必须核对舵机标签、料号和供电设计，不能根据本文猜测工作电压。

官方参考：

- 7.4 V STS3215 产品页：<https://www.feetechrc.com/74v-19-kgcm-plastic-case-metal-tooth-magnetic-code-double-axis-ttl-series-steering-gear.html>
- 12 V STS3215 产品页：<https://www.feetechrc.com/525603.html>
- STS3215 官方功能介绍：<https://www.feetechrc.com/20210430-56680.html>

## 4. 当前工程运行架构

### 4.1 模式与时钟

- 运行模式：**FreeRTOS 多线程**。
- MCU：STM32H723，Cortex-M7。
- HSE：24 MHz。
- SYSCLK/CPU：550 MHz。
- HCLK/AXI：275 MHz。
- APB2：137.5 MHz。
- USART1/USART10 内核时钟源：D2PCLK2。
- 构建工具链：`arm-none-eabi-gcc`，C11，Debug 默认 `-Og -g`。

主要启动链路：

```text
MPU_Config
  -> HAL_Init
  -> SystemClock_Config
  -> MX_GPIO_Init
  -> MX_DMA_Init
  -> MX_USART1_UART_Init
  -> MX_USART10_UART_Init
  -> robot_init
  -> osKernelInitialize
  -> MX_FREERTOS_Init
  -> osKernelStart
  -> AlgorithmTask_Entry
```

### 4.2 STS3215 任务

`AlgorithmTask` 配置：

- 创建位置：`Core/Src/freertos.c`。
- 优先级：`osPriorityHigh5`。
- 栈大小：`2048 * 4 = 8192` 字节。
- 实际强符号实现：`User/Task/Algorithm/algorithm_task.c`。

当前任务逻辑：

```text
sts3215_setup()
  -> setEnd(0)

循环读取 10 次 ReadPos(1)
  -> 每次 HAL_Delay(10)

STS3215_Init(&sts3215_encoder)
  -> 清零编码器值、累计值和角度
  -> 不会清零 is_received；此前10次读取已把该标志无条件设为1

CalibrationOfs(1)
  -> 常规中位校准

无限循环：
  -> sts3215_angle_get()
  -> angles = total_angle
  -> xQueueSend(xQueue, &angles, 0)
  -> vTaskDelay(2)
```

这里存在逻辑断裂：无限循环没有调用 `sts3215_encoder_get()`，所以 `encoder_value` 不会更新。

`UartTask` 同时由 `freertos.c` 创建，配置为普通优先级、4096 字节栈；但当前链接的是 CubeMX 生成的弱 `SendTask_Entry()` 空循环。`send_task.c` 中原本的队列接收和 DMA 发送代码全部处于注释状态，因此 `xQueue` 当前只有生产者、没有消费者。

## 5. UART 与总线移植

### 5.1 实际使用的 UART

尽管 `User/Module/FTSservo/FT_uart.c` 文件注释写着“USART1 实现”，函数实际使用的是 **`huart10`**：

```c
HAL_UART_Transmit(&huart10, nDat, nLen, 100);
HAL_UART_Receive(&huart10, nDat, nLen, 100);
```

USART10 当前配置：

| 项目 | 配置 |
| --- | --- |
| 波特率 | 1,000,000 bps |
| 数据位 | 8 bit |
| 停止位 | 1 bit |
| 校验 | None |
| 流控 | None |
| RX | PE2，AF4 |
| TX | PE3，AF11 |
| HAL 模式 | `UART_MODE_TX_RX` |
| 收发实现 | 阻塞式 HAL，超时 100 ms |

USART10 TX 虽配置了 DMA1 Stream0，但 FTServo 当前调用的是阻塞式 `HAL_UART_Transmit()`，没有使用 DMA。

### 5.2 SDK 串口链路

```text
SCS.c 协议函数
  -> SCSerail.c: writeSCS/readSCS/rFlushSCS/wFlushSCS
  -> FT_uart.c: ftUart_Send/ftUart_Read/ftBus_Delay
  -> STM32 HAL: HAL_UART_Transmit/HAL_UART_Receive
  -> USART10
```

`SCSerail.c` 使用全局发送缓冲区：

```c
uint8_t wBuf[128];
uint8_t wLen = 0;
```

注意事项：

- 没有互斥锁；多个任务同时访问同一舵机总线会破坏 `wBuf/wLen` 和协议状态。
- `ftUart_Read()` 最长阻塞 100 ms。由于 `AlgorithmTask` 优先级很高，舵机掉线时可能明显影响低优先级任务调度。
- `ftBus_Delay()` 当前固定调用 `HAL_Delay(1)`。
- 128 字节发送缓冲区会限制大规模同步写；SDK 上层虽声明最多 32 台，但 `32 * 7` 已超过该缓冲区，不能仅凭上层数组容量判断实际可发数量。
- USART10 使用独立 TX/RX 引脚，而 STS 是 TTL 半双工单总线；板上是否有收发器、二极管合线或方向控制电路仍需查原理图/实物。

## 6. SDK 文件职责

| 文件 | 职责 |
| --- | --- |
| `User/Module/FTSservo/INST.h` | 协议指令码，包括 `INST_OFSCAL = 0x0B` |
| `User/Module/FTSservo/SCS.h/.c` | 通用协议发包、读写、应答、错误状态、同步读写 |
| `User/Module/FTSservo/SCSerail.c` | SDK 缓冲区与硬件串口适配层 |
| `User/Module/FTSservo/FT_uart.c` | STM32H723 USART10 阻塞式硬件实现 |
| `User/Module/FTSservo/SMS_STS.h/.c` | SMS/STS 位置控制、同步写、常规校准、STS EPROM 锁 |
| `User/Module/FTSservo/SCSCL.h/.c` | 被 STS 复用的位置/速度/负载/电压/温度/电流反馈接口 |
| `User/Module/FTSservo/STS_Module.h/.c` | 当前项目自定义的 STS3215 编码器累计角度封装 |
| `User/Task/Algorithm/algorithm_task.c` | 初始化、校准、角度处理与队列发送 |

`CMakeLists.txt` 使用 `file(GLOB_RECURSE ... "User/*.*")`，并把 `User/Module/FTSservo` 加入头文件搜索路径，因此上述 `.c` 文件会自动进入构建。

## 7. 协议和关键寄存器

### 7.1 数据包基本格式

普通指令包：

```text
FF FF ID LENGTH INSTRUCTION [PARAMETERS...] CHECKSUM
```

校验：

```c
checksum = ~(ID + LENGTH + INSTRUCTION + 所有参数字节);
```

只取低 8 位发送。

### 7.2 SDK 中定义的 STS 地址

| 地址 | 宏 | 含义 |
| ---: | --- | --- |
| 3/4 | `SMS_STS_MODEL_L/H` | 型号 |
| 5 | `SMS_STS_ID` | ID |
| 6 | `SMS_STS_BAUD_RATE` | 波特率编号 |
| 9～12 | `MIN/MAX_ANGLE_LIMIT` | 角度限制 |
| 26/27 | `CW/CCW_DEAD` | 死区 |
| 31/32 | `SMS_STS_OFS_L/H` | 位置偏移 |
| 33 | `SMS_STS_MODE` | SDK 旧版模式地址；对当前磁编码 STS 有兼容性风险 |
| 40 | `SMS_STS_TORQUE_ENABLE` | 扭矩使能；值 128 被 SDK 用作常规中位校准 |
| 41 | `SMS_STS_ACC` | 加速度 |
| 42/43 | `SMS_STS_GOAL_POSITION_L/H` | 目标位置 |
| 44/45 | `SMS_STS_GOAL_TIME_L/H` | 目标时间 |
| 46/47 | `SMS_STS_GOAL_SPEED_L/H` | 目标速度 |
| 55 | `SMS_STS_LOCK` | STS EPROM 锁 |
| 56/57 | `PRESENT_POSITION_L/H` | 当前位置信息 |
| 58/59 | `PRESENT_SPEED_L/H` | 当前速度 |
| 60/61 | `PRESENT_LOAD_L/H` | 当前负载 |
| 62 | `PRESENT_VOLTAGE` | 当前电压 |
| 63 | `PRESENT_TEMPERATURE` | 当前温度 |
| 66 | `MOVING` | 移动状态 |
| 69/70 | `PRESENT_CURRENT_L/H` | 当前电流 |

### 7.3 固件/参考表差异

本地指令表和接口手册指出：

- 当前磁编码 STS 的工作模式地址应按参考表使用 `77`，不应直接依赖 SDK 的 `WheelMode()`（它写地址 33）。
- 恒速速度地址按参考表为 `78`，当前 SDK 的 `WriteSpe()` 从地址 41 开始连续写 7 字节，不能无条件认为兼容。
- STS 固件 `>= 3.10` 支持专用校准指令 `0x0B`。

## 8. 单舵机位置读取

### 8.1 正确的直接读取方式

初始化一次：

```c
setEnd(0);
```

读取 ID=1：

```c
int pos = ReadPos(1);

if (getLastError() == 0) {
    // pos 为有效位置
} else {
    // 通信失败，不得使用 pos
}
```

调用链：

```text
ReadPos(1)
  -> readWord(1, 56)
  -> Read(1, 56, buffer, 2)
  -> USART10 发读指令并接收两个位置字节
  -> SCS2Host() 按 setEnd(0) 组合
```

### 8.2 为什么返回 `int`

协议位置字段是 16 位，但 API 返回 `int`，因为它还需要表达：

- 16 位原始数据；
- 第 15 位方向标志转换出的负位置；
- 底层读取失败使用的 `-1`。

在本工程 ARM GCC ABI 下，`int` 为 32 位；用更宽的容器承载 16 位数据不会改变有效位置值。

### 8.3 不需要同步读缓冲区

直接 `ReadPos(1)` 使用 `readWord()` 的局部两字节数组，不依赖同步读全局缓冲区，因此不需要：

```c
syncReadBegin(...);
syncReadPacketTx(...);
syncReadPacketRx(...);
syncReadEnd();
```

只有真正执行多舵机同步读取时才使用这些接口。

### 8.4 `ReadPos(-1)` 的含义和风险

`ReadPos(-1)` 不是读取 ID=-1，而是从最近一次 `FeedBack(ID)` 填充的静态 `Mem` 缓冲区取值。

对当前 `setEnd(0)` 的 STS 使用场景，`SCSCL.c` 中缓存路径手工拼接位置字节的顺序与 `SCS2Host()` 不一致，存在字节反转风险。在修复并验证前，单舵机业务应优先使用 `ReadPos(实际ID)`，不要使用 `FeedBack + ReadPos(-1)` 路径。

## 9. 读取失败与状态判断

### 9.1 通信错误

`getLastError()` 返回最近一次 SDK 通信错误：

| 值 | 含义 |
| ---: | --- |
| 0 | 无通信错误 |
| 1 | 未收到应答 |
| 2 | 校验失败 |
| 3 | 返回 ID 不匹配 |
| 4 | 返回长度不匹配 |

### 9.2 舵机状态

`getState()` 返回最近一次有效应答包中的舵机状态字节。它来自：

```c
static uint8_t u8Status;

int getState(void)
{
    return u8Status;
}
```

`Ack()` 或 `Read()` 验证应答包后会更新 `u8Status`。

### 9.3 `ReadPos()` 当前实现的失败返回缺陷

`readWord()` 失败时确实返回 `-1`，但 `ReadPos()` 随后继续执行方向位解析：

```c
if (Pos & (1 << 15)) {
    Pos = -(Pos & ~(1 << 15));
}
```

在当前 32 位 `int` 环境中，输入 `-1` 会被转换为 `32769`，所以：

```c
if (pos == -1)     // 不可靠
```

必须使用：

```c
int pos = ReadPos(ID);
if (getLastError() != 0) {
    // 读取失败
}
```

接手者若修复 SDK，应在 `readWord()` 后、方向位解析前立即保留失败返回。

## 10. 中位校准与运动到中位

### 10.1 两个概念不能混淆

- **中位校准**：把当前机械位置重新定义为逻辑位置 2048；舵机不主动转动。
- **运动到中位**：控制舵机转到目标刻度 2048；应使用位置控制 API。

运动到中位示例：

```c
WritePosEx(1, 2048, speed, acc);
```

### 10.2 当前工程采用：常规中位校准

当前 `algorithm_task.c` 调用：

```c
CalibrationOfs(1);
```

SDK 实现：

```c
int CalibrationOfs(uint8_t ID)
{
    return writeByte(ID, SMS_STS_TORQUE_ENABLE, 128);
}
```

其含义是向地址 `40`（`0x28`）写入 `128`（`0x80`）。ID=1 时发送帧为：

```text
FF FF 01 04 03 28 80 4F
```

推荐检查：

```c
int ok = CalibrationOfs(1);
if (ok == 1 && getLastError() == 0 && getState() == 0) {
    // 应答包有效且舵机状态为0
}
```

随后应重新 `ReadPos(1)` 验证位置是否接近 2048。

### 10.3 固件 3.10 专用 `0x0B` 校准

用户已确认固件为 3.10，所以型号满足本地参考表中“STS 系列固件版本 >= 3.10”的条件。当前 SDK 只在 `INST.h` 中定义：

```c
#define INST_OFSCAL 0x0B
```

但没有现成上层函数。

将当前位置校准为 2048 的无参数帧，ID=1：

```text
FF FF 01 02 0B F1
```

将当前位置校准为自定义位置的帧：

```text
FF FF ID 04 0B POS_L POS_H CHECKSUM
```

本地指令表示例：ID=1、自定义位置=200：

```text
FF FF 01 04 0B C8 00 27
```

如果未来重新封装无参数 `0x0B`，可参考 `Reset()` 的发包结构：

```c
int STS3215_Calibrate2048_0B(uint8_t ID)
{
    rFlushSCS();
    writeBuf(ID, 0, NULL, 0, INST_OFSCAL);
    wFlushSCS();
    return Ack(ID);
}
```

需要直接包含：

```c
#include "INST.h"
#include <stddef.h>
```

当前工程已选择常规 `CalibrationOfs(1)`，接手者不要在没有明确需求时同时执行两种校准流程。

## 11. 控制与反馈 API 速查

### 11.1 当前可直接使用

| API | 作用 | 备注 |
| --- | --- | --- |
| `WritePosEx(ID, Position, Speed, ACC)` | 单舵机带加速度位置控制 | STS 主要位置控制接口 |
| `RegWritePosEx(...)` | 暂存单舵机运动参数 | 之后执行动作指令 |
| `SyncWritePosEx(...)` | 多舵机同步位置控制 | 注意发送缓冲区实际容量 |
| `CalibrationOfs(ID)` | 常规中位校准 | 当前项目选择 |
| `unLockEpromEx(ID)` | STS EPROM 解锁 | STS 锁地址55 |
| `LockEpromEx(ID)` | STS EPROM 加锁 | 修改ID后用新ID锁定 |
| `EnableTorque(ID, 0/1)` | 关闭/开启扭矩 | 位于复用的 SCSCL 接口 |
| `FeedBack(ID)` | 连续读取一组反馈 | 当前 STS 小端缓存解析存在风险 |
| `ReadPos(ID)` | 读取位置 | 当前单舵机使用方式 |
| `ReadSpeed/Load/Voltage/Temper/Move/Current` | 读取对应反馈 | `ID>=0` 直接读取 |
| `Ping(ID)` | 检测舵机在线 | 失败返回 `-1` |

### 11.2 暂不应直接使用

| API | 原因 |
| --- | --- |
| `WheelMode(ID)` | SDK 写模式地址33，参考表中当前磁编码 STS 地址为77 |
| `WriteSpe(ID, Speed, ACC)` | 当前实现从地址41写7字节，参考表恒速速度地址为78 |
| `unLockEprom/LockEprom` | SCSCL 锁地址48；STS 应使用带 `Ex` 的地址55版本 |
| `PWMMode/WritePWM` | SCSCL 专用，不属于本文 STS3215 接口范围 |

## 12. 当前项目自定义角度累计

### 12.1 数据结构

`STS3215_Encoder_t` 保存：

```c
int16_t encoder_value;
int16_t last_encoder_value;
int32_t total_encoder_value;
float last_total_angle;
float total_angle;
int is_initialized;
int is_received;
```

滤波系数：

```c
#define STS3215FILTER 0.3f
```

### 12.2 跨零点累计算法

```c
diff = current - last;

if (diff > 2048)
    diff -= 4096;
else if (diff < -2048)
    diff += 4096;

total_encoder_value += diff;
total_angle = total_encoder_value * 360.0f / 4096.0f;
filtered = 0.3f * total_angle + 0.7f * last_total_angle;
```

该算法用于把单圈 `0～4095` 的反馈展开为连续多圈累计角度。

## 13. 当前代码风险清单

### P0：必须优先处理

1. **无限循环没有更新编码器数据**

   `algorithm_task.c` 的循环只调用 `sts3215_angle_get()`，没有在它之前调用 `sts3215_encoder_get()`。当前角度不会随舵机转动更新。

2. **读取失败仍被当作有效数据**

   `sts3215_encoder_get()` 无条件执行：

   ```c
   pos = ReadPos(1);
   encoder_value = pos;
   is_received = 1;
   ```

   应先检查 `getLastError()`，失败时不得覆盖上一次有效值，也不得把 `is_received` 设为1。

3. **初始化读取被随后清零**

   任务先读取10次，然后调用 `STS3215_Init()` 把刚读取的 `encoder_value`、`last_encoder_value` 和累计量全部清零。代码注释写“记录首次原始刻度作为零点”，实际实现并没有保存该基准；同时 `STS3215_Init()` 没有清零 `is_received`，导致数据被清零后仍保持“已接收”标志。

4. **校准后没有重新读取并建立基准**

   `CalibrationOfs(1)` 后应重新读取当前位置，并把有效值同时写入 `encoder_value` 与 `last_encoder_value`，避免第一次差分产生错误跳变。

5. **`ReadPos()` 失败返回被方向解析覆盖**

   不能用 `pos == -1` 作为唯一失败条件，必须检查 `getLastError()`；建议修复 SDK。

6. **队列没有有效消费者**

   `freertos.c` 创建了 `UartTask`，但 `send_task.c` 的用户实现已全部注释，最终运行的是弱空循环。`AlgorithmTask` 每2个 RTOS tick 非阻塞写入长度为10的队列，队列写满后所有后续发送都会失败，且当前代码未检查 `xQueueSend()` 返回值。

### P1：应尽快核对

1. **半双工硬件链路未知**：确认 PE2/PE3 如何连接到 STS 单线 TTL 总线。
2. **阻塞式100 ms读取位于高优先级任务**：掉线可能拖慢其他任务。
3. **无总线互斥**：若其他任务也访问 FTServo，必须串行化协议事务。
4. **`AlgorithmTask_Entry` 函数签名不一致**：CubeMX 弱定义是 `void *argument`，用户实现是 `void const *argument`。建议统一为 `void *argument`。
5. **循环头文件依赖**：`STS_Module.c` 包含 `algorithm_task.h`，而 `algorithm_task.h` 又包含 `STS_Module.h`；当前依赖全局 `sts3215_encoder`，模块耦合较强。
6. **TIM2 句柄未初始化**：`robot.c` 自行定义了零初始化的 `htim2`，工程中不存在 `MX_TIM2_Init()`，但 `robot_init()` 仍调用 `HAL_TIM_Base_Start(&htim2)`。该调用会返回 `HAL_ERROR`，返回值当前被忽略；`TIM_Delay` 模块也尚未处于可用状态。

### P2：规模化时处理

1. `SCSerail.c` 发送缓冲区只有128字节，多舵机同步写可能静默截断。
2. 同步读使用 `malloc/free`；单舵机不需要，RTOS嵌入式环境使用前需评估堆和生命周期。
3. `HAL_Delay(10)` 的注释写“延时1s”，实际是10 ms，应避免交接误判。
4. USART10 TX DMA 已初始化但 FTServo 没有使用，配置和实现不一致但不影响当前阻塞发送。

## 14. 推荐的业务调用骨架（供后续修复参考）

以下只是交接建议，本文没有改动具体工程源码：

```c
static const uint8_t STS3215_ID = 1;

void STS3215_TaskInit(void)
{
    setEnd(0);
    STS3215_Init(&sts3215_encoder);

    if (!CalibrationOfs(STS3215_ID) ||
        getLastError() != 0 ||
        getState() != 0) {
        sts3215_encoder.is_initialized = -1;
        return;
    }

    int pos = ReadPos(STS3215_ID);
    if (getLastError() != 0) {
        sts3215_encoder.is_initialized = -1;
        return;
    }

    sts3215_encoder.encoder_value = (int16_t)pos;
    sts3215_encoder.last_encoder_value = (int16_t)pos;
    sts3215_encoder.is_received = 1;
    sts3215_encoder.is_initialized = 1;
}

void STS3215_TaskUpdate(void)
{
    int pos = ReadPos(STS3215_ID);
    if (getLastError() != 0) {
        sts3215_encoder.is_received = 0;
        return;
    }

    sts3215_encoder.encoder_value = (int16_t)pos;
    sts3215_encoder.is_received = 1;
    sts3215_angle_get();
}
```

注意：是否应在每次系统启动时执行中位校准取决于产品需求。中位校准改变逻辑零点，不应被当作普通通信初始化步骤反复调用。该决策需要项目负责人确认。

## 15. 构建历史与当前状态

曾尝试增加自定义 `STS3215_Calibrate2048()` 的 `0x0B` 封装，出现过以下编译错误：

- `NULL` 未声明：需要 `<stddef.h>` 或包含定义 `NULL` 的头文件。
- `INST_OFSCAL` 未声明：需要包含 `INST.h`。
- 错误调用 `STS3215_Calibrate2048(uint8_t ID)`：调用位置只能传值或变量，例如 `STS3215_Calibrate2048(1)`。

当前最新检查到的源码已经删除该自定义封装，并在 `algorithm_task.c` 中改用：

```c
CalibrationOfs(1)
```

2026-07-21 16:56 的工程状态已有一套完整 Debug 构建产物：

- `algorithm_task.c.obj`：16:56:32；
- `STS_Module.c.obj`：16:56:32；
- `freertos.c.obj`：16:56:31；
- `STM32H7232_UserController.elf/.hex/.bin/.map`：16:56:33。

链接 MAP 明确包含来自 `User/Task/Algorithm/algorithm_task.c.obj` 的强 `AlgorithmTask_Entry`、来自各 FTServo 对象文件的驱动符号，以及来自 `Core/Src/freertos.c.obj` 的弱 `SendTask_Entry`。因此，**现有证据表明该批最新源码已经成功编译并链接**。本次交叉检查本身仍为只读操作，没有再次执行构建，现有文件无法还原构建控制台中是否曾出现警告。

## 16. 接手 Codex 的建议工作顺序

1. 重新读取以下最新文件，确认它们没有在本文生成后变化：

   ```text
   User/Module/FTSservo/STS_Module.c
   User/Module/FTSservo/STS_Module.h
   User/Module/FTSservo/FT_uart.c
   User/Module/FTSservo/SCS.c
   User/Module/FTSservo/SCSCL.c
   User/Module/FTSservo/SMS_STS.c
   User/Task/Algorithm/algorithm_task.c
   Core/Src/usart.c
   Core/Src/freertos.c
   ```

2. 先修复 P0 数据更新、错误处理、校准后基准初始化和队列无消费者问题，不要同时重构无关模块。
3. 确认项目要使用“常规地址40写128校准”还是“固件3.10专用0x0B校准”；当前选择是前者。
4. 确认是否真的需要每次上电校准。若机械零点应持久保持，启动时反复校准可能不符合需求。
5. 查原理图确认 USART10 到 STS TTL 单线总线的硬件连接。
6. 修改后重新构建并记录结果；之后连接单个 ID=1 舵机验证：`Ping -> ReadPos -> Calibration -> ReadPos`。
7. 只有单舵机路径稳定后，再考虑同步读写、多舵机和 DMA/非阻塞改造。

## 17. 验收建议

### 17.1 通信

- `Ping(1)` 能稳定返回 ID=1。
- 连续 `ReadPos(1)` 时 `getLastError()==0`。
- 拔掉舵机后错误码为1，任务不会写入伪位置或阻塞整个系统不可接受的时间。

### 17.2 校准

- 手动将轴放到目标机械中位。
- 执行一次 `CalibrationOfs(1)`。
- 再读位置应接近2048。
- 确认校准动作不会在每次任务循环执行。

### 17.3 角度累计

- 缓慢跨越4095到0，累计角度连续增加而不发生约360°跳变。
- 反向跨越0到4095，累计角度连续减少。
- 通信失败期间保持上一次有效值，不产生大幅累计跳变。
- 恢复通信后的第一次采样不会因为错误基准产生半圈或多圈突变。

### 17.4 RTOS

- 舵机在线和离线两种情况下，低优先级任务都能按可接受周期运行。
- 确认队列存在有效消费者；持续运行时队列不会在10次发送后永久满载，且生产者检查发送失败。
- 若多个任务访问舵机总线，所有完整协议事务由同一任务或互斥锁串行化。

## 18. 信息来源

### 本地源码与资料

- 具体工程：`D:\RoboMaster\Projects\STM32H7232_UserController_1`
- SDK 参考工程：`D:\RoboMaster\Projects\STM32H7232_UserController_Temp\FTServo_stm32HAL-main`
- 本地接口手册：`STS舵机SDK接口用户手册.md`
- 本地指令表：`磁编码SMS&STS&HTS-十六进制指令生成表-250508.xlsx`

### 官方网页

- STS3215 官方功能介绍：<https://www.feetechrc.com/20210430-56680.html>
- 7.4 V STS3215：<https://www.feetechrc.com/74v-19-kgcm-plastic-case-metal-tooth-magnetic-code-double-axis-ttl-series-steering-gear.html>
- 12 V STS3215：<https://www.feetechrc.com/525603.html>

## 19. 可直接交给下一段 Codex 的提示词

```text
请先完整阅读 STS3215_Codex_Handoff.md，然后以实际工程最新源码为准进行复核。

项目根目录：
D:\RoboMaster\Projects\STM32H7232_UserController_1

当前目标：先修复 STS3215 单舵机 ID=1 的持续位置采样、通信错误处理、校准后基准初始化和累计角度更新。只修改与该目标直接相关的文件；不要顺手重构其他模块。修改前先列出将要修改的文件和依据，修改后构建验证并报告实际结果。
```
