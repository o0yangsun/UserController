# STM32H7232_UserController

> RoboMaster 工程机器人 —— **六轴机械臂操作手控制器固件**

操作手端固件。读取操作手上 6 个飞特 STS3215 串行总线舵机的绝对角度，解算成累计角度，
打包为 `0x0302` 帧，经 HC-12 无线串口发往车端 H723，由车端解析后驱动机械臂做 1:1 同构映射。

- 平台：STM32H723VGT6（Cortex-M7 @550 MHz）+ FreeRTOS / CMSIS-RTOS2
- 板卡：达妙 DM-MC-Board02（`DM-MC-Board02`）
- 状态：**真车联调通过**（六路角度、无线链路、方向一致性均已实机验证）

仓库地址：<https://github.com/o0yangsun/UserController>

---

## 1. 在整体链路中的位置

```
   操作手 6× STS3215 舵机（TTL 半双工总线，12-bit 磁编码器）
            │
            │  USART1 @ 1 Mbps, 8N1, 阻塞式（PA9 / PA10）
            ▼
 ┌───────────────────────────────────────────┐
 │  【本仓库】控制板 STM32H723 + FreeRTOS      │
 │   AlgorithmTask（3 ms 采样 → 累计角度）     │
 │        │  xQueue（长度 10）                 │
 │   UartTask（取最新帧 → 打包 → DMA 发送）   │
 └───────────────────────────────────────────┘
            │
            │  USART10 @ 115200, 8N1, DMA1_Stream0（PE3 TX / PE2 RX）
            ▼
      HC-12 无线串口（433 MHz 透传）
            │
            ▼
 ┌───────────────────────────────────────────┐
 │  车端 H723（Engineering_Robot_H723_）      │
 │   referee_system.c 解析 0x0302             │
 └───────────────────────────────────────────┘
            │
            ▼
   机械臂 6 关节（达妙电机，DMmotor_task.c）
```

> 注：本仓库只负责"操作手 → 无线"这一段。车端解析与机械臂驱动在另一个仓库。
>
> 另有一条**独立的 PC 调试通道**：`USB_OTG_HS`（内置 FS PHY + HSI48，无需外部晶振）虚拟串口，
> PC 可直读 6 关节角、下发目标角与使能命令 —— 见 **§14**。两条通道互不影响。

---

## 2. 硬件与资源分配

| 项目 | 配置 |
| --- | --- |
| MCU | STM32H723VGT6，Cortex-M7，C11 |
| 主频 | SYSCLK 550 MHz / HCLK 275 MHz / APB2 137.5 MHz（HSE 24 MHz） |
| 浮点 | `-mfpu=fpv5-d16 -mfloat-abi=hard` |
| **USART1（舵机总线）** | **1 000 000 bps, 8N1**，PA9 TX / PA10 RX，AF7，阻塞式 HAL |
| **USART10（无线上行）** | **115 200 bps, 8N1**，PE3 TX / PE2 RX，DMA1_Stream0 |
| **USB_OTG_HS（PC 调试口）** | **内置 FS PHY**，时钟源 **HSI48**（须使能），无需外部晶振/PHY/引脚；中断 `OTG_HS_IRQn` 优先级 5 |
| 对外电源 | **PC15 = 高 → 对外 5 V 使能**（达妙板为可控输出，必须软件使能）；PC13/PC14 = 两路 24 V，本控制器关闭 |
| 系统时基 | TIM6（`HAL_IncTick`）+ DWT 微秒级计时（`drv_dwt`） |
| 调试链 | OpenOCD + CMSIS-DAP / ST-Link + `arm-none-eabi-gdb`；可配合 OctoLink MCP Bridge 在线读写变量 |

---

## 3. 目录结构

```
Core/                                STM32CubeMX 生成的 HAL / 时钟 / 外设 / RTOS 初始化
  Src/main.c                         ★ PC13/14/15 对外电源控制、系统时钟、MPU
  Src/usart.c                        ★ USART1(舵机) + USART10(无线) + DMA + NVIC
  Src/freertos.c                     AlgorithmTask / UartTask 任务创建
  Src/stm32h7xx_it.c                 中断向量（含 USART10_IRQHandler）
User/
  Algorithm/Crc8_Crc16/              帧头 CRC8 + 帧尾 CRC16
  Module/Dwt/                        DWT 微秒级计时与延时（dwt_delay_us）
  Module/TIM_Delay/                  定时器延时
  Module/FTSservo/                   飞特 STS 舵机 SDK + 本项目封装
    INST.h                             协议指令码（含 INST_OFSCAL 0x0B）
    SCS.c/.h                           协议层：发包 / 读应答 / 错误状态 / 同步读写
    SCSCL.c/.h                         位置·速度·负载·电压·温度反馈接口（被 STS 复用）
    SMS_STS.c/.h                       STS 位置控制、同步写、中位校准、EPROM 锁
    SCSerail.c                         SDK 发送缓冲区 + 硬件适配入口（rFlushSCS / wFlushSCS）
    FT_uart.c/.h                     ★ 硬件层实现：USART1 阻塞收发 + ORE 清理 + 50 µs 总线延时
    STS_Module.c/.h                  ★ 本项目封装：六路角度累积 / 方向表 / 死区 / 基准确认
  Task/Algorithm/algorithm_task.c    ★ 采样任务（约 3 ms 节拍）
  Task/Usart_Send/send_task.c        ★ 发送任务（限速 30 ms → 打包 0x0302 → DMA）
  Task/Robot_Config/robot.c/.h       ★ 队列创建、ServoFeedback_t 定义
STM32H7232_UserController.ioc        CubeMX 工程配置
CMakeLists.txt                       CLion / arm-none-eabi-gcc 构建脚本
STS3215_Codex_Handoff.md             早期（2026-07-21）STS3215 技术交接文档，见文末说明
```

---

## 4. 运行架构

FreeRTOS 两个任务 + 一个队列，生产者/消费者模型。

| 任务 | 创建位置 | 优先级 | 栈 | 职责 |
| --- | --- | --- | ---: | --- |
| `AlgorithmTask` | `freertos.c` | `osPriorityHigh5` | 8 KB | 逐路采样 6 个舵机 → 角度解算 → 六路全有效才组帧入队，约 3 ms 一轮 |
| `UartTask`（`SendTask_Entry`） | `freertos.c` | `osPriorityNormal` | 4 KB | 取空队列只留最新帧 → `PackData` → DMA 发送，`send_period_ms` 限速 |

```
AlgorithmTask ──xQueueSend(长度10)──► xQueue ──xQueueReceive(取空)──► UartTask ──DMA──► USART10 ──► HC-12
```

**启动链路**

```
MPU_Config → HAL_Init → SystemClock_Config → MX_GPIO_Init → MX_DMA_Init
  → MX_USART1_UART_Init → MX_USART10_UART_Init
  → 使能对外 5V（PC15）→ robot_init（建队列 + dwt_init + TIM2）
  → osKernelInitialize → MX_FREERTOS_Init → osKernelStart
```

**队列**：`xQueue`，长度 10，单元类型 `ServoFeedback_t`（`float angle[6]` + `uint8_t valid`）。

---

## 5. 通信协议：`0x0302` 帧

总长 **39 字节**，小端，字段定义见 `User/Task/Usart_Send/send_task.h`。

| 偏移 | 长度 | 字段 | 说明 |
| ---: | ---: | --- | --- |
| 0 | 1 | `sof` | 固定 `0xA5` |
| 1 | 2 | `data_length` | 数据段长度，固定 30 |
| 3 | 1 | `seq` | 包序号，1~255 循环（0 保留） |
| 4 | 1 | `crc8` | 帧头 CRC8（覆盖前 5 字节） |
| 5 | 2 | `cmd_id` | 固定 `0x0302`（六轴机械臂控制器） |
| 7 | 30 | `data` | `data[0..23]` = 6 × `float` 角度（单位 °，小端）；其余 6 字节补 0 |
| 37 | 2 | `frame_tail` | CRC16（覆盖整帧 39 字节） |

> ⚠️ **数据段必须从 `data[0]` 起**。车端 `referee_system.c` 的解析口径是
> `custom_robot_data.data[i * 4]`（`i = 0..5`），即 `data[0..23]`。早年 F411 控制器在
> `data[0]` 处多放了一个字节，导致车端整体错位 1 字节 —— 本控制器已对齐 F411 口径。

---

## 6. 舵机链路与核心算法

### 6.1 SCS 协议（飞特 STS/SCS 系列通用）

```
FF FF  ID  LENGTH  INSTRUCTION  [PARAMETERS...]  CHECKSUM
checksum = ~(ID + LENGTH + INSTRUCTION + 所有参数) & 0xFF
```

- STS 系列为**小端**字节序，`sts3215_setup()` 中调用 `setEnd(0)`。
- `ReadPos()` 读失败时的返回值处理存在 SDK 缺陷，**必须**配合 `getLastError()` 或值域判断，不能只看 `pos == -1`。

### 6.2 STS3215 关键参数

| 项目 | 值 |
| --- | --- |
| 型号 / 固件 | STS3215 / 3.10 |
| 编码器 | 12-bit 磁编码器，单圈 `0 ~ 4095`（4096 刻度） |
| 角度分辨率 | `360 / 4096 = 0.0879°` / 刻度 |
| 单机 ID | `0 ~ 253`，广播 `254` |
| 中位校准 | `sts3215_calib_mid()`：向 `TORQUE_ENABLE`(地址 40) 写 `128`，把**当前位置**定义为逻辑刻度 2048（**不驱动**舵机转动） |

> 供电版本（7.4 V / 12 V）必须核对舵机标签与板上供电设计 —— 仅凭型号无法确定。

### 6.3 角度解算流程（`STS_Module.c`）

每路独立实例 `STS3215_Encoder_t`，`sts3215_angle_get(idx)` 依次执行：

| 步骤 | 逻辑 | 目的 |
| ---: | --- | --- |
| 1 | 读失败 → 直接 `return`，不累积、不更新 `last` | 防止垃圾值污染累计角度（"失联不发疯臂"） |
| 2 | 首次成功读到该路（`is_received` 由 0→1）→ 只同步基准，不产生增量 | 防止把 raw 本身（如归中后的 2048）当成转动量 |
| 3 | 基准确认期 `base_ready < 2`：相邻两次差值 ≤ `STS3215_BASE_TOL`(16 刻度) 才计数，否则清零重来 | 消除"中位校准生效时机 vs 首次读取"的时序竞争 |
| 4 | 过零修正：`diff > 2048 → -4096`；`diff < -2048 → +4096` | 处理 0 ↔ 4095 绕圈 |
| 5 | 方向修正：`diff *= sts3215_dir[idx]`（在过零修正**之后**） | 统一六路正转方向 |
| 6 | **累积式**死区：差值先入 `deadband_accum`，越界才一次性放行 | 抑制手部微颤，又不吞掉慢速转动 |
| 7 | `total_encoder_value += diff` → 换算角度 → 低通滤波 `α = 0.3` | 平滑输出 |

### 6.4 方向表 `sts3215_dir[]`

```c
static const int8_t sts3215_dir[STS3215_NUM] = {1, -1, -1, 1, -1, 1};
//  关节:   J1   J2   J3   J4   J5   J6
```

### 6.5 死区 `sts3215_deadband[]`

```c
static const int16_t sts3215_deadband[STS3215_NUM] = {8, 8, 8, 8, 8, 8};  // 单位：刻度
```

取值依据（固件内 3 ms 节拍统计器实测）：

| 状态 | 六路 raw 极差 |
| --- | --- |
| 完全静置（无人接触）35 s | **全部 0 刻度** —— 电气噪声为 0，无自重漂移 |
| 手接触但不主动转动 | **4 ~ 8 刻度（0.35 ~ 0.70°）** ← 这就是要过滤的量 |
| 主动转动 | 几十刻度以上，远超死区 |

> **必须用"累积式"而非"丢弃式"**：采样周期仅 3 ms，慢速操作每帧差值很小
> （30 °/s 时每帧仅约 1 刻度）。若用丢弃式阈值 8 刻度，等价于把 234 °/s 以下
> 的所有转动全部吞掉，机械臂会表现为"完全不动"。

### 6.6 上电初始化顺序（`algorithm_task.c`）

```
sts3215_setup()  → setEnd(0)
HAL_Delay(300)          // 纯延时等舵机就绪，【不读编码器】
for idx in 0..5:
     STS3215_Init(&enc[idx], base = 0)      // 输出基准统一 0°，与 F411/车端对齐
     sts3215_calib_mid(id) 或 Calibration(id)
HAL_Delay(100)          // 等六路校准全部生效
→ 进入主循环
```

两个"刻意不做的动作"，都是为了绕开实测踩到的坑：

- **不在延时里读编码器** —— 读成功会把 `is_received` 置 1，绕过"首次成功读只同步基准"的保护，实测出现过 ±2048 / ±85 刻度的凭空偏移。
- **校准后不补读** —— 若在校准尚未生效时把过期 raw 写进 `last_encoder_value`，主循环首帧就会把差值累加进去（实测 −85 刻度）。

---

## 7. 关键设计决策 / 踩坑记录

| # | 问题现象 | 根因 | 处理 | 位置 |
| ---: | --- | --- | --- | --- |
| 1 | 上电后各舵机只更新一次数据，随后**全部冻结**，且不自愈 | H7 版 HAL 移植**不清 ORE**，一旦置位则后续 `HAL_UART_Receive` 立即返回错误 | 每次总线事务前 + 读失败后调用 `ftUart_FlushRx()`（清 ORE + 排空 RX FIFO + 复位 `RxState`） | `FT_uart.c` |
| 2 | 采样周期被拖到 7 ms | `ftBus_Delay()` 固定 `HAL_Delay(1)`，六路轮询累计 6 ms | 半双工换向只需几十 µs → 改 `dwt_delay_us(50)` | `FT_uart.c` |
| 3 | 串口 **0 字节输出**，`dma_busy` 恒为 1，后续帧只打包不发送 | DMA 发送是"两段式"完成：DMA 完成中断 → 使能 USART TC 中断 → `TC 中断` 里才回调 `HAL_UART_TxCpltCallback`。缺 USART10 NVIC 使能，②永远等不到 | `usart.c` 中补 `HAL_NVIC_EnableIRQ(USART10_IRQn)` | `usart.c` |
| 4 | 发出的是**陈旧数据**，凭空多约 30 ms 延迟 | 采样 3 ms 入队一次，发送 30 ms 一次；队列满后 `xQueueSend(...,0)` 丢弃的是**最新**帧 | 发送端循环 `xQueueReceive` 取空队列，只保留最后一帧 | `send_task.c` |
| 5 | 上电时某几路一上来就是 **±180°** | 中位校准生效瞬间 raw 跳 2048 刻度，被当作真实转动累加 | 新增 `base_ready` 基准确认期，跳变只重新同步 | `STS_Module.c` |
| 6 | 机械臂慢速操作时**完全不动** | 丢弃式死区把慢速小增量整段吞掉 | 改累积式死区 | `STS_Module.c` |
| 7 | 外接无线模块**完全不工作**，量 VCC 只有约 2.1 V（幻电压） | 达妙 DM-MC-Board02 的对外 5 V 是**可控 DCDC 输出**，需 `Power_5V_EN`(PC15) 置高 | `main.c` 显式使能 PC15，并关掉两路 24 V | `main.c` |
| 8 | 车端解析角度整体错位 | 帧数据段起始位置不一致 | 6 个 float 统一从 `data[0]` 起，对齐 F411 口径 | `send_task.c` |
| 9 | 真车实测 **6 号关节方向相反**（操作手正转 → 机械臂反转） | 控制器侧方向表 J6 取 `-1`（按"读数符号对齐老控制器"推出），与车端 `smooth_motion_6` 的符号叠加后结果相反 | 控制器侧 `sts3215_dir[5]` 改为 `+1`（车端 `DMmotor_task.c` 的方案亦可，但需同时改运行时与上电初始化两处） | `STS_Module.c` |

---

## 8. 构建与烧录

### 8.1 构建（CLion / CMake + arm-none-eabi-gcc）

```bash
cmake -B cmake-build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build cmake-build-debug -j
```

- 编译宏：`-DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx`
- 优化：Debug 为 `-Og -g`；Release 为 `-Ofast`
- 源文件收集：`file(GLOB_RECURSE ... "Core/*.*" "Middlewares/*.*" "Drivers/*.*" "User/*.*")`
  → **在 `User/` 下新增 `.c` 后需重新执行 CMake 配置**，否则不会进构建。
- 链接脚本：`STM32H723VGTX_FLASH.ld`
- 产物：`<build>/STM32H7232_UserController.elf / .hex / .bin / .map`（全部不在 git 跟踪范围内）

### 8.2 烧录 / 调试

OpenOCD 运行配置见 `.idea/runConfigurations/OCD_STM32H7232_UserController.xml`。

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg
arm-none-eabi-gdb build/STM32H7232_UserController.elf
```

---

## 9. 在线调试技巧

| 手段 | 说明 |
| --- | --- |
| 抖动诊断 | 调用 `dbg_jitter_start()` 清零并开始统计；用手轻碰/轻转舵机数秒；读 `dbg_raw_min[i]` / `dbg_raw_max[i]` 看极差；`dbg_jitter_enable = 0` 停止。解决"调试器 halt 采样抓不到瞬时抖动"的问题 |
| 帧率在线调整 | `send_period_ms` 为 `volatile`，可直接用调试器改写：30 → ≈30 fps（对齐老控制器）、17 → ≈50 fps、7 → ≈100 fps。**不要设 0**，`vTaskDelay(0)` 不阻塞会饿死其他任务 |
| 反馈镜像 | `servo_feedback` 保存最近一次成功入队的帧，供 GDB / 上位机观察 |
| OctoLink | OctoLink MCP Bridge（`127.0.0.1:48731`，newline-delimited JSON-RPC）可在线读写上述变量，无需重新烧录 |

---

## 10. 排查手册

| 现象 | 优先检查 |
| --- | --- |
| 串口完全无输出 | `dma_busy` 是否恒 1 → 检查 `USART10_IRQn` 的 NVIC 使能（坑 #3） |
| 上电后舵机数据全部冻结 | ORE 是否置位 → `ftUart_FlushRx()` 是否被调用（坑 #1） |
| 上电某几路角度直接 ±180° | `STS3215_BASE_TOL` / `base_ready` 基准确认期（坑 #5） |
| 慢速操作机械臂不动 | 死区阈值与"累积式"实现（坑 #6） |
| 外接模块不工作、VCC 只有约 2.1 V | **PC15 是否置高**（对外 5 V 使能，坑 #7） |
| 操作手正转但机械臂反转 | 先看车端 `smooth_motion_N` 的 per-joint 符号，再看本仓 `sts3215_dir[]`（坑 #9） |
| 无线链路不通（车端收不到帧） | ① HC-12 的 **SET 引脚必须悬空**（接 GND 会进 AT 命令模式，不转发数据）；② 模块 VCC 取自板子可控 5 V；③ 两端波特率一致（115200） |

---

## 11. 分支策略

| 分支 | 用途 |
| --- | --- |
| `main` | 稳定分支，**真车验证通过**。只接纳已验证的合并 |
| `feature/*` | 后续测试与功能迭代，验证通过后再合回 `main` |

---

## 12. 已知限制 / TODO

- **舵机总线无互斥锁**：`SCSerail.c` 的 `wBuf[128]` / `wLen` 是全局共享的，多任务并发访问同一总线会破坏协议状态。当前只有 `AlgorithmTask` 一个使用者，暂无问题。
- **发送缓冲区上限 128 字节**：SDK 上层虽声明最多 32 台，但 `32 × 7 > 128`，不能仅凭上层数组容量判断实际可发数量。
- **半双工单总线收发电路**：USART1 配置为独立 TX/RX 引脚，STS 是 TTL 半双工单总线，板上合线方式需对照原理图确认。
- `//TODO:` 测试最大发送频率 —— `algorithm_task.c`
- `//TODO:` 舵机发送改用 DMA —— `FT_uart.c`（当前为阻塞式 `HAL_UART_Transmit`）
- STS3215 供电版本（7.4 V / 12 V）需与实物核对。
- 仓库暂无 `LICENSE` 文件；部分文件头部保留了原模板作者（刘嘉俊 / 杜尚泽）的版权与"仅供学习交流"声明，收录前请确认授权口径。

### 与早期交接文档的差异

`STS3215_Codex_Handoff.md`（2026-07-21）描述的是**更早**的工程状态，以下几条已经变化：

| 交接文档的说法 | 当前实际 |
| --- | --- |
| 舵机总线在 **USART10**（PE2/PE3） | 舵机总线已改为 **USART1**（PA9/PA10）；USART10 专用于 115200 无线上行 |
| 单舵机、ID 硬编码为 1 | 已扩展到 **6 路**，由 `motor_ids[6]` 表驱动 |
| `SendTask_Entry` 为空循环、队列无消费者 | `send_task.c` 已实现完整的取帧 / 打包 / DMA 发送 |
| `ftBus_Delay()` 为 `HAL_Delay(1)` | 已改为 `dwt_delay_us(50)`，并新增 ORE 清理 |
| 主循环未调用 `sts3215_encoder_get()` | 已由 `sts3215_angle_get()` 内部调用，并补齐丢包 / 基准保护 |

---

## 13. 相关仓库

- 车端固件：`Engineering_Robot_H723_`（团队仓库 `poppywork/Engineering_Robot_H723_`）
- 本控制器：<https://github.com/o0yangsun/UserController>

---

## 14. PC ↔ 主臂 USB CDC 链路（`0x60` / `0x61` / `0x62`）

移植自车端，用于 **PC 直连主臂板**：读 6 关节角（数据采集）、下发目标角与使能命令（HIL 干预）。
USB 是**第三条独立通道**，USART1（舵机总线）与 USART10（0x0302 上行）不受影响。

### 14.1 硬件与驱动

| 项 | 说明 |
| --- | --- |
| 外设 | `USB_OTG_HS` + **内置 FS PHY**（`HAL_PCD_MspInit()` 里没有任何 `HAL_GPIO_Init`） |
| 时钟 | **HSI48**（`RCC_USBCLKSOURCE_HSI48`）—— 见 `usbd_conf.c` |
| 驱动位置 | `USB_DEVICE/`（App + Target）、`Middlewares/ST/STM32_USB_Device_Library/`、HAL 的 `stm32h7xx_hal_pcd*` / `stm32h7xx_ll_usb*` |
| 中断 | `OTG_HS_IRQHandler()` 在 `stm32h7xx_it.c`（覆盖启动文件里的 weak 定义） |
| 初始化 | `MX_USB_DEVICE_Init()` 在 `main.c`，**全工程只调用这一处** |

> ⚠️ **两个移植时最容易漏、且症状隐蔽的点**（文档 `PORTING.md` 未列出）：
> 1. **HAL 的 PCD/LL_USB 驱动层**（6 个文件）—— 缺了直接链接失败。
> 2. **`HSI48` 必须在 `SystemClock_Config()` 里使能** —— 不开则 USB 的时钟源形同虚设，
>    PC 上不出现 COM 口，而 HAL 不一定报错。
> 另外 `stm32h7xx_hal_conf.h` 里的 `HAL_PCD_MODULE_ENABLED` 原本是注释状态。

### 14.2 帧格式（与车端完全一致）

```
偏移     字段        长度   说明
[0]     帧头         1     固定 0xFF
[1]     地址         1     固定 0x05
[2]     命名ID       1     见下表
[3]     数据长度     1     仅"数据段"字节数 N
[4..]   数据         N
[N+4]   sum_check    1     Σbuf[0..N+3] & 0xFF
[N+5]   addr_check   1     Σ(sum_check) & 0xFF（累加和的低 8 位）
```

帧总长 = `4 + N + 2`。

### 14.3 命名 ID

车端占用 `0x1x / 0x2x / 0x3x`，主臂走 **`0x6x`** 段，同一台 PC 抓包时一眼可分。

| ID | 方向 | 用途 | 数据长 | 帧长 |
| --- | --- | --- | ---: | ---: |
| `0x60` | 主臂 → PC | 6 关节角 + 使能状态 | 25 | 31 |
| `0x61` | PC → 主臂 | 6 个目标关节角 + 生效标志 | 25 | 31 |
| `0x62` | PC → 主臂 | 使能 / 失能 | 1 | 7 |

### 14.4 数据段

**`0x60` 上行**

| 偏移 | 类型 | 内容 |
| ---: | --- | --- |
| 0 / 4 / 8 / 12 / 16 / 20 | `int32` ×6 | J1~J6 关节角，**弧度 × 10000**（小端） |
| 24 | `int8` | 使能状态：1 = 锁死，0 = 自由 |

**`0x61` 下行**：偏移 0~23 = 6 × `int32`（**弧度 × 10000**），偏移 24 = 标志（1 = 立即生效）。
**`0x62` 下行**：偏移 0 = `int8`（1 = 使能，0 = 失能）。

> ★ **单位转换在主臂固件内部完成**（`User/Module/Usb_Pc/usb_pc_link.c`）：
> 上行打包时 `度 → 弧度×10000`，下行解析时 `弧度×10000 → 度`。
> 主臂内部一律用度，PC 侧一律用弧度 —— 这样 PC 上解析车端 `0x20` 与主臂 `0x60` 可以用同一套代码。

### 14.5 代码位置与行为

| 项 | 位置 / 说明 |
| --- | --- |
| 协议实现 | `User/Module/Usb_Pc/usb_pc_link.c/.h` |
| 挂载点 | `algorithm_task.c` 主循环、**组帧入队之后**（不影响 0x0302 节拍） |
| 上报周期 | `usb_pc_report_ms = 30`（`volatile`，可用调试器在线改） |
| 发送失败策略 | `CDC_Transmit_HS()` 返回 `USBD_BUSY` 时**丢弃本帧、不重试**（沿用车端策略） |
| `0x61` 目标角 | 解析存入 `usb_pc_target_deg[]`，再由 `usb_pc_arm_write_apply()` **过四道安全闸后执行**（见 14.7） |
| `0x62` 使能 | 复用按键那条 `sts3215_set_torque_all()` 通路，保证扭矩状态只有一个权威来源 |
| 调试计数 | 上行：`usb_pc_tx_cnt` / `tx_busy_cnt` / `rx_cnt` / `rx_bad_cnt` / `last_id` / `tick`<br>写位置：`usb_pc_write_cnt` / `write_reject_cnt` / `write_stale_cnt` / `last_write_deg[]` / `last_write_idx` |

### 14.7 `0x61` 写位置：安全闸与换算（`0x6x` 段唯一会动机械的通路）

**这是整条 PC 链路里唯一会让机械结构产生运动的通路**，所以专门做了多层闸门。

| # | 闸门 | 默认 | 行为 |
| --- | --- | --- | --- |
| ① | 必须已使能 | — | `sts3215_locked == 1` 才动；自由态下 PC 无法驱动（避免与操作手"抢"机械臂） |
| ② | 命令超时 | `usb_pc_write_timeout_ms = 200` | 距最后一次收到 `0x61` 超过它就停止。★ 防止 **PC 断连后残留目标持续驱动**（目标与标志都不会自己清零） |
| ③ | 单帧路数 | `usb_pc_write_max_joints = 6` | 一帧内"需要动作"的关节数超过它就整帧拒绝（记入 `write_reject_cnt`）。**默认 6 = 不限制**，理由见下 |
| ④ | **逐路软限位** | `usb_pc_soft_min/max_deg[6]` | PC 目标超程时**钳到该路机械行程内**（**J1 默认 ±85**，其余 ±170）。★ **只拒绝超程目标，不主动追赶已超限的读数**（见下方事故说明） |
| ⑤ | 相对当前角限幅 | `usb_pc_write_limit_deg = 10.0` | 目标与当前角之差超过 ±10° 就**钳到边界缓慢逼近**，不是拒绝 |
| ⑥ | **堵转保护** | `usb_pc_stall_limit = 20` | 连续 20 次（≈0.6s）"命令了却几乎没位移"就跳过该路，并累加 `write_stall_cnt` |
| ⑦ | 已到位阈值 | `usb_pc_write_eps_deg = 1.5` | 差值小于它就视为到位、不发命令。**必须大于舵机内部死区**（STS3215 约 1.3~1.8°），否则会"固件认为没到位、舵机认为已在死区内"地空转 |

另：只要六路里有一路读数不可信，**整帧不动** —— 限幅判断必须基于真实当前角，拿不到就不该盲动。

#### ⚠️ 软限位为什么必须"只拒绝、不追赶"（2026-09-27 实机事故）

原实现是「先把目标钳到 170°，再与当前值比较」⇒ 当某路**当前读数已经超程**时
（例如 J6 因绕圈累积报成 175.9°），每帧都会产生一个"把它拉回 170°"的命令。
**而这个命令在 raw 空间里可能对应错误方向** ⇒ 实测把 J6 一路驱动过去**撞到了底板**（真机械损伤）。

⇒ 现在改为：**用 PC 的【原始目标】判定"是否需要动作"**，
- PC 目标 == 当前值（哪怕当前值已超程）⇒ **不候选、不动**
- PC 目标本身超程（如 200° > 170°）⇒ 候选，钳位到 170° 后执行

#### ⚠️ 已到位阈值必须大于舵机内部死区（同一天的实测）

`eps` 原为 0.5°，太小：实测 J1 目标 3° 时停在离目标 **1.33°** 处（19 刻度）不动 ——
因为 STS3215 **内部有位置死区（约 15~20 刻度 ≈ 1.3~1.8°）**，残差落进死区后舵机不再动作，
而固件仍判"未到位"继续每 30ms 发命令。⇒ 调到 **1.5°**，让固件在死区范围内就认可到位。
**代价**：单关节定位精度约 ±1.5°（这是舵机本身的分辨率下限，不是软件问题）。

#### ⚠️ 修复"读数漏计"：短时读取失败不再丢弃位移（`STS_Module.c`）

同一次测试还暴露了**读数偏低**：J1 物理上已到 3°，但上报只有 1.667°。

根因在 `sts3215_angle_get()` 的失败处理 —— 旧实现是：

```c
if (sts3215_encoder_get(idx) != 0) return;   /* 读失败 */
if (!was_received) {                          /* 恢复后 */
    last_encoder_value = encoder_value;       /* ← 把当前位置当新基准 */
    base_ready = 0;                           /* ← 这段位移【永久丢失】 */
    return;
}
```

**舵机快速运动时 `ReadPos` 很容易失败**（总线只有 1Mbps 且是阻塞式 HAL，运动中舵机响应也变慢），
于是那段位移被丢掉 ⇒ **手感到位、读数偏低**。

修正后：
- **短时失败**（`fail_cnt ≤ STS3215_FAIL_TOL = 3`）⇒ **不动任何基准状态**，恢复后照常差分，**位移补回来**
- **长时失联**（超过阈值）⇒ 期间转了多少不可知，才重置基准
- 新增 `ever_ok` 字段，用于区分"上电首帧"（必须重置基准，否则归中偏移会被当成转动量）
  与"短时失败后的恢复帧"（绝不能重置）

全部参数都是 `volatile`，**可用调试器在线改**，不必重新烧录。

#### ⚠️ 闸门③的默认值为什么是 6 而不是 1（实机教训，2026-09-27）

最初默认设为 **1**（"单帧只准一路变化"），以为更安全。**实机结果是几乎完全不可用**：

> PC 发来的目标里，**只要任意一路与固件端"当前角"差超过 `eps`(0.5°)，它也算"需要动作"**。
> 而真实系统里这种小偏差几乎必然存在 —— 重力微动、机械耦合、USB 往返 30ms 期间的位置变化。
> 于是经常出现 2 路同时偏离 → **整帧被拒** → 表现为"命令完全没反应"。

实测计数器特征：**`write_reject_cnt` 疯涨，而 `write_cnt` 恒为 0**。

⇒ **安全应该由"限幅 + 超时 + 软限位 + 堵转保护"保证，而不是靠限制路数。**
想做单关节测试时，把 `usb_pc_write_max_joints` 临时改成 1（volatile 在线改），
并让 PC 侧把其余关节目标填成**各自当前值**。

#### ⚠️ 软限位：为什么必须有（同一天的教训）

底座 J1 的机械行程只有 **±90°**。实测中 PC 下发了一个朝限位方向的目标，结果：

> 舵机**满力顶在限位上持续堵转** → 某时刻机械滑脱 → **角度猛跳 113°**。

⇒ 加了**逐路软限位**：目标角先钳到该路允许范围内，最多走到边界就停。
默认已按已知信息把 **J1 限到 ±85°**（留 5° 余量），其余关节用 ±170° 占位 ——
**换机械臂或知道其他关节行程后，请按实机填写**（`usb_pc_soft_min_deg[]` / `usb_pc_soft_max_deg[]`，volatile 可在线改）。

#### ★ 行为约定：必须"持续发"才能走完全程

因为限幅是 10°/帧、超时是 200ms，所以：

- **PC 发一次 `0x61`** ⇒ 主臂最多走 10° 就停
- **PC 按周期持续发（如 30ms）** ⇒ 主臂按 10°/帧 的节奏一步步逼近，最终精确到位

这正是"上位机做闭环控制 / 数据采集"的自然语义。仿真验证：从 −160° 走到 +160° 的
**578 个组合全部收敛，最大偏差 0.742°**。

#### 角度 → 刻度的换算（`sts3215_goto_deg()`）

主臂的 `total_encoder_value` 是**增量累加**量，且累加时已乘过方向，所以不能用角度直接当刻度写：

```c
target_total = target_deg × (4096/360)            /* 度 -> 累计刻度 */
raw_target   = raw_now + sts3215_dir[i] × (target_total - enc->total_encoder_value)
```

两道硬保护（越界直接拒绝，**不做绕圈处理** —— 绕一圈会让舵机猛转 360°）：

- 单次位移 `|delta_total| > 2047` 刻度（半圈）⇒ 拒绝。否则下一轮采样会把它误判成
  "过零跳变"（`sts3215_angle_get` 的 ±2048 修正），角度彻底错乱
- 换算结果 `raw_target` 落在 0~4095 之外 ⇒ 拒绝（目标角超出该关节可达范围）

执行前还会**清该路死区累积器**，否则被暂扣的差值会叠加进本次位移，使上报角度滞后最多 0.7°。

### 14.8 PC 侧工具

```bash
pip install pyserial
python tools/pc_arm_monitor.py                 # 列出串口
python tools/pc_arm_monitor.py COM7 --deg      # 持续监视（角度制）
python tools/pc_arm_monitor.py COM7 --enable 1 # 发 0x62 使能

# 发 0x61 目标角：单纯发一次只能走 10°，要把目标走完必须 --hold 持续发
python tools/pc_arm_monitor.py COM7 --target-deg 10 0 0 0 0 0 --hold 3
```

`--hold` 模式下会一边按 `--rate`（默认 30ms）周期重发目标、一边打印六路实际角度与
**最大偏差**，到位后标 `← 已到位`，可以直观看到一步步逼近的过程。

```bash
# 只动 J1 到 10°，持续 3 秒（验证单关节跟踪）
python tools/pc_arm_monitor.py COM10 --enable 1
python tools/pc_arm_monitor.py COM10 --target-deg 10 0 0 0 0 0 --hold 3
```

内置的 `StreamParser` 是**字节流解析器**（能吃粘包/断包，校验失败只丢 1 字节重新同步），
不假设一次 `read()` 恰好等于一帧。
