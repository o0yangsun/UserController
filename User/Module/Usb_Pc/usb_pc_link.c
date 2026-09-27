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
volatile uint16_t usb_pc_report_ms = 30;

volatile float    usb_pc_target_deg[6] = {0};
volatile uint8_t  usb_pc_target_flag   = 0;
volatile int8_t   usb_pc_enable_req    = -1;

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
volatile uint16_t usb_pc_write_speed      = 0;      /* 0 = 舵机内部默认速度 */
volatile uint8_t  usb_pc_write_acc        = 0;      /* 0 = 舵机内部默认加速度 */
volatile uint16_t usb_pc_write_timeout_ms = 200;    /* 命令有效期 200ms */
volatile float    usb_pc_write_eps_deg    = 0.5f;   /* 小于 0.5° 视为已到位 */

/* 逐路软限位。★ 必须按实机机械行程填写。
 * 已知：底座(J1，索引 0) 的机械行程为 ±90°（2026-09-27 用户确认），
 *       故默认给它留 5° 余量、限到 ±85 —— 这样即使 PC 发来超程目标，
 *       也只走到 85° 就停，不会顶在限位上满力堵转。
 *       其余关节行程未知，暂用 ±170（几乎不限制），后续按实机填写。
 * 两者都是 volatile，调试器可在线改，不必重烧。 */
volatile float    usb_pc_soft_min_deg[6] = {-85.0f, -170.0f, -170.0f, -170.0f, -170.0f, -170.0f};
volatile float    usb_pc_soft_max_deg[6] = { 85.0f,  170.0f,  170.0f,  170.0f,  170.0f,  170.0f};

/* 堵转保护：连续 N 次"命令了却几乎没位移"就停止驱动该路（默认 20 次 ≈ 0.6s） */
volatile uint8_t  usb_pc_stall_limit    = 20;
volatile float    usb_pc_stall_eps_deg  = 0.15f;

volatile uint32_t usb_pc_write_cnt        = 0;
volatile uint32_t usb_pc_write_reject_cnt = 0;
volatile uint32_t usb_pc_write_stale_cnt  = 0;
volatile uint32_t usb_pc_write_stall_cnt  = 0;
volatile float    usb_pc_last_write_deg[6] = {0};
volatile uint8_t  usb_pc_last_write_idx   = 0;

/* ---------------- 内部状态 ---------------- */
static uint8_t  s_txbuf[USB_PC_UP_FRAME_LEN];
static uint32_t s_last_report_ms = 0;
static uint32_t s_last_target_ms = 0;   /* 最后一次收到 0x61 的时刻（供超时判定） */

/* 堵转检测状态（每路一份） */
static float    s_stall_ref_deg[6] = {0};   /* 上次"确认有位移"时的角度 */
static uint8_t  s_stall_cnt[6]     = {0};   /* 连续"命令了却没动"的次数 */

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

        /* ★ 软限位：先把目标钳到该路允许范围内，再判"是否需要动作"。
         *   这样即使 PC 发来超程目标，最多走到边界 —— 不会一直顶着机械限位出力
         *   （2026-09-27 实测：撞限位后机械滑脱、角度猛跳 113°）。 */
        const float d = clamp_soft(i, usb_pc_target_deg[i]) - cur[i];

        if (d > usb_pc_write_eps_deg || d < -usb_pc_write_eps_deg)
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

        /* ★ 堵转保护：命令了却几乎没位移 ⇒ 判定顶限位 / 卡住 ⇒ 跳过该路。
         * 判据用"本路角度相对上次'有位移'时的基准是否变化"，与命令是否发出无关，
         * 所以正常运动（哪怕是慢速）不会误触发。 */
        if (usb_pc_stall_limit > 0u)
        {
            const float moved = cur[i] - s_stall_ref_deg[i];

            if (moved > usb_pc_stall_eps_deg || moved < -usb_pc_stall_eps_deg)
            {
                s_stall_ref_deg[i] = cur[i];   /* 确实在动：更新基准、清零计数 */
                s_stall_cnt[i]     = 0u;
            }
            else if (s_stall_cnt[i] < 255u)
            {
                s_stall_cnt[i]++;              /* 没动：累计 */
            }

            if (s_stall_cnt[i] >= usb_pc_stall_limit)
            {
                usb_pc_write_stall_cnt++;      /* 卡住了：本帧跳过这一路 */
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
