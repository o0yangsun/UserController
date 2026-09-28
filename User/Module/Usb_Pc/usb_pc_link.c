/**
 * @file    usb_pc_link.c
 * @brief   PC ↔ 主臂板 USB CDC 通信协议的实现（详见 usb_pc_link.h）
 *
 * 依赖：
 *   - USB_DEVICE/App/usbd_cdc_if.c   收发原语 CDC_Transmit_HS() / 接收标志位
 *   - STS_Module.c                   关节角来源 sts3215_encoder[]、扭矩状态 sts3215_locked
 *
 * 设计要点：
 *   1. 发送失败一律丢弃、不重试 —— CDC_Transmit_HS() 在上一包未发完时返回 USBD_BUSY，
 *      等它只会把任务阻塞住（沿用车端策略）。
 *   2. 上报是【限速】的：本函数每轮都被调用（约 3ms），但只有到了 usb_pc_report_ms 才真发一帧。
 *   3. 角度单位转换只在本文件里做，外部（PC 侧）只认弧度，主臂内部只认度。
 */

#include "usb_pc_link.h"
#include "usbd_cdc_if.h"
#include "algorithm_task.h"   /* sts3215_encoder[] */
#include "STS_Module.h"       /* sts3215_locked、STS3215_NUM */
#include "main.h"             /* HAL_GetTick()、memcpy 所需的 HAL 头 */
#include <string.h>

/* 本协议的数据段固定按 6 关节设计，臂数变了这里必须同步改（含 PC 侧解析） */
#if (STS3215_NUM != 6)
#error "usb_pc_link 的 0x60/0x61 协议按 6 关节定义，STS3215_NUM 不为 6 时需同步修改"
#endif

/* ---------------- CDC 接收侧全局量 ----------------
 * 它们定义在 usbd_cdc_if.c 里，但 usbd_cdc_if.h 没有 extern 出来，
 * 所以这里按移植文档 §3.2 的说明自行声明。 */
extern uint8_t        USB_Received_Data[APP_RX_DATA_SIZE];
extern uint32_t       USB_Received_Len;
extern volatile uint8_t USB_Data_Ready_Flag;

/* ---------------- 对外可见的配置与状态 ---------------- */
/* 0x60 上报周期(ms)。
 * ★ 2026-09-27：由 30(≈33Hz) 改为 16(≈62.5Hz)。
 *   原因：PC 侧感受到的"读数滞后"主要来自本周期的量化误差
 *         （均值 T/2、最坏 T）—— 30ms 时均值 15ms、最坏 30ms。
 *         改成 16ms 后滞后降到均值 8ms、最坏 16ms。
 *   带宽核算：31B × 62.5 ≈ 1.94 kB/s，USB FS CDC 绰绰有余。
 *   下限约束：上报在 AlgorithmTask 主循环里（约 3ms/轮），16ms 远在其上，无压力。 */
volatile uint16_t usb_pc_report_ms = 16;

volatile float    usb_pc_target_deg[6] = {0};
volatile uint8_t  usb_pc_target_flag   = 0;
volatile int8_t   usb_pc_enable_req    = -1;
volatile uint8_t  usb_pc_hold_req      = 0;

volatile uint32_t usb_pc_tx_cnt      = 0;
volatile uint32_t usb_pc_tx_busy_cnt = 0;
volatile uint32_t usb_pc_rx_cnt      = 0;
volatile uint32_t usb_pc_rx_bad_cnt  = 0;
volatile uint32_t usb_pc_last_id     = 0;
volatile uint32_t usb_pc_tick        = 0;

/* ---------------- 0x61 写位置：安全闸 ---------------- */
/* max_joints 默认 1 的教训见 .h 说明：真实系统里"多路同时有小偏差"是常态，
 * 限制路数会导致整帧被拒、命令完全没反应（实机实测 reject_cnt 疯涨 write_cnt 恒 0）。 */
volatile uint8_t  usb_pc_write_max_joints = 6;      /* 默认全部允许，安全靠限幅+超时+软限位 */
volatile float    usb_pc_write_limit_deg  = 10.0f;  /* 单次限幅 10° */
/* 速度/加速度：单位 ≈ 0.088 °/s per unit（2026-09-27 实测标定，见 .h 说明）。
 * 默认 170 ≈ 12°/s：50° 行程约 4s 走完。0 = 舵机内部默认（最快）。 */
volatile uint16_t usb_pc_write_speed      = 170;
volatile uint8_t  usb_pc_write_acc        = 0;      /* 0 = 舵机内部默认加速度 */
volatile uint16_t usb_pc_write_timeout_ms = 200;    /* 命令有效期 200ms */
/* "已到位"阈值。
 * ★ 2026-09-27 实测：原为 0.5°，太小 —— STS3215 内部有位置死区（约 15~20 刻度
 *   ≈ 1.3~1.8°），当"目标与当前值"的差落到死区内时舵机不再动作，
 *   而固件仍判"未到位"继续发命令 ⇒ 每帧白跑一次总线、且表现为"命令了却不动"。
 *   实测 J1 目标 3° 时停在离目标 1.33° 处（正好落在死区边界）。
 *   ⇒ 调到 1.5°，让固件在死区范围内就认可"已到位"。 */
volatile float    usb_pc_write_eps_deg    = 1.5f;

/* 逐路软限位。★ 必须按实机机械行程填写。
 *
 * 实测来源（2026-09-28，用户用 OctoLink 读 `total_angle` 在机械两端取值）：
 *   J1 -90 ~ +90    J2 -170 ~ 0     J3    0 ~ +180
 *   J4 -180 ~ +180  J5  -25 ~ +90   J6 -180 ~ +180
 *
 * ⚠️ 这些值是【相对上电/复位基准】的角度 —— 所以每次上电（或 reset）时，
 *    机械臂必须停在【与测量时相同的参考姿态】，否则这组限位整体偏移，
 *    后果是"某一侧提前被限制"或"另一侧失去保护"。这是使用前提，不是 bug。
 *
 * 内缩余量：不直接把实测值当边界，而是向内收 USB_PC_SOFT_MARGIN_DEG。
 *   理由：软限位是"把目标钳到边界"，若边界=机械硬限位，它仍会一路走到并停在硬限位上；
 *         再叠加读数估计误差（舵机死区 ±1.5° 量级），可能在极限处形成持续微顶。
 *         内缩一点即彻底不接触。想用实测原值，把下面这个宏改成 0.0f 即可。
 * 两者都是 volatile，调试器可在线改，不必重烧。 */
#define USB_PC_SOFT_MARGIN_DEG      3.0f

volatile float    usb_pc_soft_min_deg[6] = {
    -90.0f  + USB_PC_SOFT_MARGIN_DEG,   /* J1 实测 -90  → -87   */
    -170.0f + USB_PC_SOFT_MARGIN_DEG,   /* J2 实测 -170 → -167  */
      0.0f  + USB_PC_SOFT_MARGIN_DEG,   /* J3 实测 0    → +3    */
    -180.0f + USB_PC_SOFT_MARGIN_DEG,   /* J4 实测 -180 → -177  */
    -25.0f  + USB_PC_SOFT_MARGIN_DEG,   /* J5 实测 -25  → -22   */
    -180.0f + USB_PC_SOFT_MARGIN_DEG,   /* J6 实测 -180 → -177  */
};
volatile float    usb_pc_soft_max_deg[6] = {
     90.0f  - USB_PC_SOFT_MARGIN_DEG,   /* J1 → +87   */
       0.0f - USB_PC_SOFT_MARGIN_DEG,   /* J2 → -3    */
    180.0f  - USB_PC_SOFT_MARGIN_DEG,   /* J3 → +177  */
    180.0f  - USB_PC_SOFT_MARGIN_DEG,   /* J4 → +177  */
     90.0f  - USB_PC_SOFT_MARGIN_DEG,   /* J5 → +87   */
    180.0f  - USB_PC_SOFT_MARGIN_DEG,   /* J6 → +177  */
};

/* 堵转保护：★ 2026-09-27 改为【按真实时间窗】判定，不再按"调用次数"。
 * 原写法"连续 20 次调用位移 < eps"是按 30ms/次估的 0.6s，但本函数实际被主循环
 * 每 3ms 调一次 ⇒ 真实窗口只有 60ms ⇒ 任何低于 50°/s 的正常运动都被误判成"卡住"，
 * 关节被周期性跳过（实测 stall_cnt 涨到 711，且速度卡在 ~7°/s 上不去）。 */
volatile uint8_t  usb_pc_stall_limit    = 20;     /* 已废弃，保留以兼容旧配置 */
volatile uint16_t usb_pc_stall_window_ms = 600;   /* 真实时间窗(ms)，0 = 关闭 */
volatile float    usb_pc_stall_eps_deg  = 0.30f;
/* ⚠️ eps 隐含一个下限：0.30° / 600ms = 0.5°/s，即"低于 0.5°/s 的运动会被判成堵转"。
 *    所以 usb_pc_write_speed 不要设得太低（按标定 ≈0.088°/s/unit，speed 需 ≳ 10）。
 *    量化噪声约 1 刻 = 0.088°，0.30° = 3.4 刻，已留足余量，不宜再调小。 */

/* ★★★ 2026-09-28 新增：堵转的两个修复（详见 usb_pc_arm_write_apply 内的长注释）★★★ */

/* 判定堵转时，是否写一次 goal = 当前角来【卸载】推力。默认 1（开）。
 * 为什么必须有：舵机是"持有目标"模型 —— 只是"不写新目标"并不会让它停，
 *   它会继续朝上次写入的 goal（越界方向）顶，机械应力一点没解除。
 * 关掉它（设 0）则退回"只跳过不卸载"的旧行为：不顶了，但顶限位的应力仍在。 */
volatile uint8_t  usb_pc_stall_unload    = 1;

/* 自动重试：距上次判定堵转超过本值(ms)才允许重试一次。默认 1000。
 * 限流的理由：不加限流的话，PC 每帧推进一点目标就会触发一次重试 ⇒ 变成持续顶限位。
 * 设为 0 = 【永不自动重试】（该路一直保持卸载态，直到发 0x63 或复位才解除）。 */
volatile uint16_t usb_pc_stall_retry_ms  = 1000;

/* 自动重试的第二个条件：PC 目标相对【判堵转时的目标】变化量超过本值(度)才重试。
 * 默认 3.0。理由：同一个目标重试没有意义（它已经被证明不可达）；
 * 目标真的变了才值得再试一次（可能是换了方向 / 换了姿态，限位不再挡路）。
 *   > 0  : 目标变化需超过该值（默认 3.0°）
 *   = 0  : 只要目标有任何变化即可
 *   < 0  : 忽略"目标变化"这一条，只按时间间隔重试（纯时间重试） */
volatile float    usb_pc_stall_retry_deg = 3.0f;

/* ★★★ 角度估计一致性护栏（安全关键，见 .h 说明）★★★ */
volatile float    usb_pc_consist_tol_ticks = 50.0f;   /* 容差(刻)，50 刻 ≈ 4.4° */
volatile uint32_t usb_pc_consist_err_cnt   = 0;
volatile float    usb_pc_consist_err_ticks = 0.0f;

volatile uint32_t usb_pc_write_cnt        = 0;
volatile uint32_t usb_pc_write_reject_cnt = 0;
volatile uint32_t usb_pc_write_stale_cnt  = 0;
volatile uint32_t usb_pc_write_stall_cnt  = 0;   /* 堵转【发生次数】（状态跃迁计数）
                                                  * ⚠️ 语义已变更：旧版是"被跳过的帧数"，
                                                  *    那会按 ~333/s 增长（实测 2140），看起来像
                                                  *    "一直在顶"，其实只是在每帧跳过一次而已。 */
volatile uint32_t usb_pc_write_stall_skip_cnt = 0;   /* 因堵转而跳过该路的【帧数】（旧语义） */
volatile uint32_t usb_pc_stall_unload_cnt     = 0;   /* 实际执行"卸载"(写 goal=当前角)的次数 */
volatile uint32_t usb_pc_stall_retry_cnt      = 0;   /* 自动重试放行的次数 */
volatile uint32_t usb_pc_hold_cnt         = 0;
volatile float    usb_pc_last_write_deg[6] = {0};
volatile uint8_t  usb_pc_last_write_idx   = 0;

/* ---------------- 内部状态 ---------------- */
static uint8_t  s_txbuf[USB_PC_UP_FRAME_LEN];
static uint32_t s_last_report_ms = 0;
static uint32_t s_last_target_ms = 0;   /* 最后一次收到 0x61 的时刻（供超时判定） */

/* 堵转检测状态（每路一份）—— 按时间窗判定：
 *   · 相对上次"确认有位移"的基准变化 > eps  ⇒ 更新基准、刷新时间戳、清"已卡住"
 *   · 距上次确认位移超过 stall_window_ms    ⇒ 判定该路卡住（本帧跳过）
 * 假堵转（慢速运动）会自己解除：它仍在动 → 位置变化 → 基准刷新 ✓ */
static float    s_stall_ref_deg[6] = {0};   /* 上次"确认有位移"时的角度 */
static uint32_t s_stall_ref_ms[6]  = {0};   /* 上次"确认有位移"的时刻 */
static uint8_t  s_stalled[6]       = {0};   /* 1 = 判定卡住中（持续跳过该路） */
static float    s_stall_tgt_deg[6] = {0};   /* 判定堵转时的 PC 目标角 —— 供"目标变了才重试" */
static uint32_t s_stall_ms[6]      = {0};   /* 判定堵转的时刻 —— 供"重试间隔限流" */

/* ---------------- 校验：sum + 累加和(addr)，与车端逐字节一致 ----------------
 * sum 覆盖 [0 .. 4+dlen-1]（即 帧头..数据末字节），两个校验值都取低 8 位。 */
static void usb_pc_put_check(uint8_t *b, uint8_t dlen)
{
    uint8_t s = 0, a = 0;
    const uint16_t n = (uint16_t)(4 + dlen);

    for (uint16_t i = 0; i < n; i++)
    {
        s = (uint8_t)(s + b[i]);
        a = (uint8_t)(a + s);
    }
    b[n]     = s;
    b[n + 1] = a;
}

static uint8_t usb_pc_check_ok(const uint8_t *b, uint8_t dlen)
{
    uint8_t s = 0, a = 0;
    const uint16_t n = (uint16_t)(4 + dlen);

    for (uint16_t i = 0; i < n; i++)
    {
        s = (uint8_t)(s + b[i]);
        a = (uint8_t)(a + s);
    }
    return ((b[n] == s) && (b[n + 1] == a)) ? 1u : 0u;
}

/* ---------------- 发送：0x60 主臂 → PC ---------------- */
static void usb_pc_send_report(void)
{
    s_txbuf[0] = USB_PC_HEAD;
    s_txbuf[1] = USB_PC_ADDR;
    s_txbuf[2] = USB_PC_ID_ARM_UP;
    s_txbuf[3] = USB_PC_UP_DATA_LEN;

    for (uint8_t i = 0; i < 6; i++)
    {
        /* ★ 度 → 弧度×10000（int32 小端，直接 memcpy；PC 侧用 struct '<6i'）
         * 乘 10000 是为了把浮点定点化成整数，避免 PC 侧再猜浮点格式。 */
        const int32_t v = (int32_t)(sts3215_encoder[i].total_angle * USB_PC_DEG2RAD * USB_PC_SCALE);
        memcpy(&s_txbuf[4 + i * 4], &v, sizeof(int32_t));
    }

    /* 使能状态：直接取按键那套的扭矩状态（1=锁死/使能，0=自由/失能） */
    s_txbuf[4 + 24] = sts3215_locked ? 1u : 0u;

    usb_pc_put_check(s_txbuf, USB_PC_UP_DATA_LEN);

    /* 上一包未发完 → 丢弃本帧，不重试（重试会阻塞任务） */
    if (CDC_Transmit_HS(s_txbuf, USB_PC_UP_FRAME_LEN) == USBD_OK)
    {
        usb_pc_tx_cnt++;
    }
    else
    {
        usb_pc_tx_busy_cnt++;
    }
}

/* ---------------- 接收：PC → 主臂 ---------------- */
static void usb_pc_handle_rx(void)
{
    const uint8_t *rx  = USB_Received_Data;
    const uint32_t len = USB_Received_Len;

    /* 先清标志再处理：避免处理期间被新一包覆盖时状态错乱 */
    USB_Data_Ready_Flag = 0;
    USB_Received_Len     = 0;

    if (len < 6u)                              { usb_pc_rx_bad_cnt++; return; }
    if (rx[0] != USB_PC_HEAD || rx[1] != USB_PC_ADDR) { usb_pc_rx_bad_cnt++; return; }

    const uint8_t nid  = rx[2];
    const uint8_t dlen = rx[3];

    if (len != (uint32_t)(4u + dlen + 2u))     { usb_pc_rx_bad_cnt++; return; }
    if (!usb_pc_check_ok(rx, dlen))            { usb_pc_rx_bad_cnt++; return; }

    usb_pc_rx_cnt++;
    usb_pc_last_id = nid;

    switch (nid)
    {
    case USB_PC_ID_ARM_TARGET:   /* 0x61：6 个目标关节角（弧度×10000）+ 生效标志 */
        if (dlen != USB_PC_DOWN_TARGET_LEN)
        {
            break;
        }
        for (uint8_t i = 0; i < 6; i++)
        {
            int32_t v = 0;
            memcpy(&v, &rx[4 + i * 4], sizeof(int32_t));
            /* ★ 弧度 → 度，存成主臂内部单位。
             *   本函数【只解析存储，不驱动舵机】—— 真正的执行在
             *   usb_pc_arm_write_apply() 里，且要过四道安全闸（见 .h）。
             *   这样"收包"与"动机械"彻底解耦：即使解析出了离谱的目标角，
             *   也不会在这里直接作用到舵机上。 */
            usb_pc_target_deg[i] = (float)v / USB_PC_SCALE * USB_PC_RAD2DEG;
        }
        usb_pc_target_flag = rx[4 + 24];
        s_last_target_ms   = HAL_GetTick();   /* 记录时刻，供 usb_pc_arm_write_apply 做超时判定 */
        break;

    case USB_PC_ID_ARM_ENABLE:   /* 0x62：使能 / 失能请求 */
        if (dlen != USB_PC_DOWN_ENABLE_LEN)
        {
            break;
        }
        usb_pc_enable_req = (rx[4] != 0u) ? 1 : 0;
        break;

    case USB_PC_ID_ARM_HOLD:   /* 0x63：立即原位保持（真正的"停"） */
        if (dlen != USB_PC_DOWN_HOLD_LEN)
        {
            break;
        }
        /* 这里只置请求标志，真正的舵机写入放到 usb_pc_service() 里按固定顺序执行：
         * 6 路 goal 写入约 12 次总线事务，集中在一处更好预期，也和 0x62 的处理方式一致。 */
        usb_pc_hold_req = 1u;
        break;

    default:
        break;
    }
}

/* ---------------- 服务函数 ---------------- */
void usb_pc_service(void)
{
    usb_pc_tick++;

    /* ① 收：CDC_Receive_HS 已在 USB 中断里搬运完毕，这里只查标志 */
    if (USB_Data_Ready_Flag != 0u)
    {
        usb_pc_handle_rx();
    }

    /* ①.5 0x63 HOLD 请求：把六路 goal 强制写成当前位置 ⇒ 机械臂真正停住。
     *     （普通的"停发"做不到这件事 —— 舵机会继续奔向旧目标，见 .h 说明） */
    if (usb_pc_hold_req != 0u)
    {
        usb_pc_hold_req = 0u;
        (void)usb_pc_arm_hold_now();
    }

    /* ② 发：按周期限速上报 0x60 */
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_last_report_ms) >= (uint32_t)usb_pc_report_ms)
    {
        s_last_report_ms = now;
        usb_pc_send_report();
    }
}

/* 软限位钳位：把目标角限制在该路允许的机械行程内（防止顶着限位堵转） */
static float clamp_soft(uint8_t i, float deg)
{
    if (deg > usb_pc_soft_max_deg[i])
    {
        return usb_pc_soft_max_deg[i];
    }
    if (deg < usb_pc_soft_min_deg[i])
    {
        return usb_pc_soft_min_deg[i];
    }
    return deg;
}

/* ---------------- 0x61 写位置：应用 PC 下发的目标角 ---------------- */
uint8_t usb_pc_arm_write_apply(void)
{
    /* ---------- 安全闸 ①：必须已锁死 ----------
     * 自由态(sts3215_locked == 0)下不驱动舵机。
     * 理由：自由态意味着操作手要用手拖动机械臂做遥操作，
     *      此时如果 PC 还能让它自己动，会与操作手"抢"机械臂，既危险又混乱。
     *      ⇒ 想用 PC 驱动，先按 PA15 或发 0x62 使能。 */
    if (sts3215_locked == 0u)
    {
        usb_pc_write_stale_cnt++;
        return 0;
    }

    /* ---------- 安全闸 ②：必须有生效请求且未超时 ---------- */
    if (usb_pc_target_flag == 0u)
    {
        usb_pc_write_stale_cnt++;
        return 0;
    }
    /* 超时保护：PC 断连后 flag 会一直保持 1、目标角也停在最后一帧，
     * 不设超时的话主臂会被"残留目标"持续驱动 —— 这是真实隐患，必须有。 */
    if ((uint32_t)(HAL_GetTick() - s_last_target_ms) > (uint32_t)usb_pc_write_timeout_ms)
    {
        usb_pc_write_stale_cnt++;
        return 0;
    }

    /* ---------- 安全闸 ②.5：★★★ 角度估计一致性护栏（2026-09-27 新增，安全关键）★★★
     * 校验 `raw == base_raw + dir×累积刻度`（mod 4096）。见 usb_pc_link.h 的详细说明。
     * 简述：0x61 是绝对角协议，角度估计一旦被污染，PC 发来的绝对目标就会被理解成
     *      大幅误动作（实测 J1 被污染 +1200 刻 ⇒ 发出 −117° 命令 ⇒ 冲出撞限位）。
     * ⚠️ 必须在【任何写舵机之前】判定 —— 漏过之后每步 ±10° 地朝错误目标走，
     *    2s 就能走 100°+，逐帧限幅完全挡不住。 */
    {
        float worst = 0.0f;

        for (uint8_t i = 0; i < STS3215_NUM; i++)
        {
            const int16_t e = sts3215_consist_err(i);

            if (e == (int16_t)0x7FFF)
            {
                usb_pc_write_stale_cnt++;      /* 最近一次读取不可信 */
                return 0;
            }

            const float ae = (e >= 0) ? (float)e : -(float)e;
            if (ae > worst)
            {
                worst = ae;
            }
        }

        if (worst > usb_pc_consist_tol_ticks)
        {
            /* 拒绝驱动：角度估计不可信，绝不据此发运动命令。
             * 计数器留给上位机/调试器看 —— 出现非 0 就说明"需要复位重建基准"。 */
            usb_pc_consist_err_cnt++;
            usb_pc_consist_err_ticks = worst;
            return 0;
        }
    }

    /* ---------- 先读六路当前角，挑出"需要动作"的关节 ---------- */
    float   cur[STS3215_NUM];
    uint8_t cand[STS3215_NUM];
    uint8_t n = 0;

    for (uint8_t i = 0; i < STS3215_NUM; i++)
    {
        if (sts3215_get_deg(i, &cur[i]) != 0)
        {
            /* 任一路读数不可信 → 整帧不动。
             * 理由：限幅判断必须基于真实当前角，拿不到就不该盲动
             *      （宁可这一帧不执行，也不要基于过期角度去算位移）。 */
            usb_pc_write_stale_cnt++;
            return 0;
        }

        /* ★★ 用【PC 原始目标】判定"是否需要动作"，而不是钳位后的目标。
         * 理由（2026-09-27 实测事故）：若某路当前读数已经"超程"
         *   （例如 J6 因绕圈累积报成 175.9°，而软限位上限是 170°），
         *   那么"钳位后目标(170) 与 当前值(175.9)"必然有偏差
         *   ⇒ 每帧都产生一个"把它拉回 170°"的命令
         *   ⇒ 而这个命令在 raw 空间里可能对应错误方向
         *   ⇒ 实测把 J6 一路驱动到撞底板（真机械损伤）。
         * 现在：只有 PC 真的要求该路变化时才动；PC 目标本身超程则钳位后再走。
         *    - PC 目标 == 当前值            -> 不候选 -> 不动（不再"主动追赶"）
         *    - PC 目标超程(如 200°>170°)    -> 候选 -> 钳位到 170° 再执行 */
        const float raw_d = usb_pc_target_deg[i] - cur[i];

        if (raw_d > usb_pc_write_eps_deg || raw_d < -usb_pc_write_eps_deg)
        {
            cand[n++] = i;
        }
    }

    if (n == 0u)
    {
        /* 六路都已在容差内 ⇒ 不产生任何总线事务。
         * 这正是"PC 持续重发同一目标也不会造成额外开销"的原因。 */
        return 0;
    }

    /* ---------- 安全闸 ③：单帧允许变化的路数上限 ----------
     * 默认 6（即不限制）—— 安全由限幅/超时/软限位保证，而不是靠限制路数。
     * 想做单关节测试时把这个值临时改成 1（volatile），PC 侧其余关节填当前值。 */
    if (n > usb_pc_write_max_joints)
    {
        usb_pc_write_reject_cnt++;
        return 0;
    }

    /* ---------- 安全闸 ④：逐路限幅 + 堵转保护，然后执行 ---------- */
    uint8_t done = 0;

    for (uint8_t k = 0; k < n; k++)
    {
        const uint8_t i = cand[k];
        float d = clamp_soft(i, usb_pc_target_deg[i]) - cur[i];

        /* 超限不是拒绝，而是"钳到边界缓慢逼近"：
         * 这样即使 PC 一次给了大目标，主臂也会按 10°/次 的节奏走过去，
         * 不会突然大幅甩动。 */
        if (d > usb_pc_write_limit_deg)
        {
            d = usb_pc_write_limit_deg;
        }
        else if (d < -usb_pc_write_limit_deg)
        {
            d = -usb_pc_write_limit_deg;
        }

        /* ================= 堵转保护（2026-09-28 重构：修两个真实缺陷） =================
         * 判定：距上次"确认有位移"超过 usb_pc_stall_window_ms，而位移仍小于 eps
         *       ⇒ 判定顶限位 / 卡住。
         *
         * ★ 修复 1「卸载」——旧实现只 `continue`（不再写新目标），但舵机是"持有目标"模型，
         *   它仍会继续朝【上次写入的 goal】顶，而那个 goal 正是越界方向 ⇒
         *   机械应力一点没解除。（stall_cnt 涨到 2140 并不代表"停止出力"，
         *   那只是"每帧跳过一次"的累计帧数。）
         *   现在：判定成立的那一刻，写一次 goal = 当前角（等价于对单路做一次 0x63），
         *   真正把推力卸掉。只在跃迁那一次写，之后每帧纯跳过 ——
         *   否则每 3ms 一次总线事务，代价太大。
         *
         * ★ 修复 2「自恢复」——旧实现里 s_stalled 只能靠"它自己动了"来清除，
         *   而它不动正是因为被跳过 ⇒ 一旦误判，该路在 0x61 通路上【永久失效】
         *   （之后怎么发都不动，极易误诊成机械问题）。
         *   现在改成"两个条件同时满足才放行一次重试"：
         *     a) 距上次判定堵转 ≥ usb_pc_stall_retry_ms   （限流，避免退化成持续顶限位）
         *     b) PC 目标相对【判堵转时的目标】变化 ≥ usb_pc_stall_retry_deg
         *   —— 同一目标重试没有意义（已被证明不可达），目标真变了才值得再试。
         *   注意：不再用"它动了"来解除 —— 卸载后关节会因重力回落，
         *   若用"动了就解除"会和"再推回去"形成往复振荡。 */
        if (usb_pc_stall_window_ms > 0u)
        {
            const uint32_t now_ms = HAL_GetTick();

            if (s_stalled[i] == 0u)
            {
                /* ★★ 2026-09-28 修正：首次评估只建立基准，不判堵转。
                 *
                 * 缺陷：`s_stall_ref_ms[]` / `s_stall_ref_deg[]` 都是 static，初值 0。
                 *   复位后六路都读 0.000，而 ref_deg 初值也是 0
                 *   ⇒ `moved = 0` 被判成"没动"，且 `now_ms - 0` = 上电至今几十秒 ≫ 观察窗
                 *   ⇒ 【上电后第一次用 0x61 驱动就被误判堵转并锁存】。
                 *   而锁存后需"目标变化 ≥3° + 间隔 ≥1s"才放行 ⇒ 若 PC 用【固定目标】驱动
                 *   （不逐步推进），该路就永远不动。
                 *   实测：烧录后复位、发固定目标 J5 +20°，rx_cnt=65 而 **write_cnt=0**，
                 *   stall_cnt=1 / unload=1（第一帧即跃迁）。
                 *
                 * 修法：用 `ref_ms == 0` 作为"本次上电还没建立基准"的哨兵 ——
                 *   HAL_GetTick() 上电后很快非 0，且首个 0x61 必在 USB 枚举之后到达，可靠。
                 *   首次只做同步（ref = 当前角、ref_ms = now），本帧不判堵转。 */
                if (s_stall_ref_ms[i] == 0u)
                {
                    s_stall_ref_deg[i] = cur[i];
                    s_stall_ref_ms[i]  = now_ms;
                }

                const float moved = cur[i] - s_stall_ref_deg[i];
                const int   moving = (moved > usb_pc_stall_eps_deg) ||
                                     (moved < -usb_pc_stall_eps_deg);

                if (moving)
                {
                    s_stall_ref_deg[i] = cur[i];      /* 确实在动：刷新基准与时间戳 */
                    s_stall_ref_ms[i]  = now_ms;
                }
                else if ((uint32_t)(now_ms - s_stall_ref_ms[i]) >
                         (uint32_t)usb_pc_stall_window_ms)
                {
                    /* ---------- 状态跃迁：判定堵转 ---------- */
                    s_stalled[i]       = 1u;
                    s_stall_ms[i]      = now_ms;
                    s_stall_tgt_deg[i] = usb_pc_target_deg[i];
                    usb_pc_write_stall_cnt++;         /* 发生【次数】，不是帧数 */

                    /* 修复 1：卸载推力（写 goal = 当前位置） */
                    if (usb_pc_stall_unload != 0u)
                    {
                        if (sts3215_goto_deg(i, cur[i],
                                             usb_pc_write_speed,
                                             usb_pc_write_acc) == 0)
                        {
                            usb_pc_last_write_deg[i] = cur[i];
                            usb_pc_last_write_idx    = (uint8_t)(i + 1u);
                            usb_pc_stall_unload_cnt++;
                        }
                    }
                }
            }
            else
            {
                /* ---------- 已在堵转态：判是否放行一次重试 ---------- */
                const float dt = usb_pc_target_deg[i] - s_stall_tgt_deg[i];

                /* retry_deg < 0 ⇒ 忽略目标变化条件（纯时间重试）；= 0 ⇒ 任何变化即可 */
                const int tgt_ok = (usb_pc_stall_retry_deg < 0.0f)
                                   ? 1
                                   : ((dt > usb_pc_stall_retry_deg) ||
                                      (dt < -usb_pc_stall_retry_deg));

                const int time_ok = (usb_pc_stall_retry_ms != 0u) &&
                                    ((uint32_t)(now_ms - s_stall_ms[i]) >=
                                     (uint32_t)usb_pc_stall_retry_ms);

                if (tgt_ok && time_ok)
                {
                    usb_pc_stall_retry_cnt++;         /* 放行一次：重新开始判定 */
                    s_stalled[i]       = 0u;
                    s_stall_ref_deg[i] = cur[i];
                    s_stall_ref_ms[i]  = now_ms;
                }
            }

            if (s_stalled[i] != 0u)
            {
                usb_pc_write_stall_skip_cnt++;        /* 本帧跳过这一路（不产生总线事务） */
                continue;
            }
        }

        const float tgt = cur[i] + d;

        if (sts3215_goto_deg(i, tgt, usb_pc_write_speed, usb_pc_write_acc) == 0)
        {
            usb_pc_last_write_deg[i] = tgt;
            usb_pc_last_write_idx    = (uint8_t)(i + 1u);
            usb_pc_write_cnt++;
            done++;
        }
    }

    return done;
}

/* ---------------- 0x63：立即原位保持（真正的"停"） ----------------
 * 背景（2026-09-27 实测）：位置伺服是"持有目标"模型 —— 停止下发新目标后，
 *   舵机仍会继续奔向最后写入的 goal 直到到达；脚本已停发 3s，J2 仍走了 9.49°、
 *   J3 走了 8.70°（J3 是往上走，可排除重力）。
 *   而想用普通 0x61 写"目标=当前位置"来刹车也不行：那会因差值 < eps 被判"已到位"，
 *   不产生任何总线事务（见 usb_pc_arm_write_apply 的候选筛选）。
 * ⇒ 本函数**绕过 eps 判定**，对六路逐个强制 `goal = 当前角`，并让残留目标失效。 */
uint8_t usb_pc_arm_hold_now(void)
{
    uint8_t done = 0;

    for (uint8_t i = 0; i < STS3215_NUM; i++)
    {
        float cur = 0.0f;

        /* 读不到当前位置就跳过这一路 —— 不知道位置就写不出安全的 goal，
         * 宁可这一路不写，也不要用过期角度看误差。 */
        if (sts3215_get_deg(i, &cur) != 0)
        {
            continue;
        }

        if (sts3215_goto_deg(i, cur, usb_pc_write_speed, usb_pc_write_acc) == 0)
        {
            /* 把残留目标同步成当前角：即使随后 write_apply 又被调用，
             * 目标与当前值相等也不会产生位移 */
            usb_pc_target_deg[i]     = cur;
            usb_pc_last_write_deg[i] = cur;
            usb_pc_last_write_idx    = (uint8_t)(i + 1u);
            done++;
        }
    }

    /* 关键：清掉"生效标志"并让超时判定失效 —— 否则旧的 0x61 目标会在超时窗口内
     * 被再次应用，等于白刹车。（PC 想继续驱动，重新发一帧 0x61 即可恢复） */
    usb_pc_target_flag = 0u;
    s_last_target_ms   = 0u;

    /* 顺带把堵转判定清零：刹车后位置基准变了，旧的"没动"判定已无意义。
     * ★ 这一步同时是堵转态【唯一的无条件解除口】—— 所以 0x63 既能刹车，
     *   也能把"被判堵转而永久跳过"的某一路重新放行。 */
    for (uint8_t i = 0; i < STS3215_NUM; i++)
    {
        s_stall_ref_deg[i] = usb_pc_target_deg[i];
        s_stall_ref_ms[i]  = HAL_GetTick();
        s_stalled[i]       = 0u;
        s_stall_tgt_deg[i] = usb_pc_target_deg[i];
        s_stall_ms[i]      = HAL_GetTick();
    }

    usb_pc_hold_cnt++;

    return done;
}
