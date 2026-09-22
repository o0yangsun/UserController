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

---

## 2. 硬件与资源分配

| 项目 | 配置 |
| --- | --- |
| MCU | STM32H723VGT6，Cortex-M7，C11 |
| 主频 | SYSCLK 550 MHz / HCLK 275 MHz / APB2 137.5 MHz（HSE 24 MHz） |
| 浮点 | `-mfpu=fpv5-d16 -mfloat-abi=hard` |
| **USART1（舵机总线）** | **1 000 000 bps, 8N1**，PA9 TX / PA10 RX，AF7，阻塞式 HAL |
| **USART10（无线上行）** | **115 200 bps, 8N1**，PE3 TX / PE2 RX，DMA1_Stream0 |
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
