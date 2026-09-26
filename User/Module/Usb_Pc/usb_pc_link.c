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

/* ---------------- 内部状态 ---------------- */
static uint8_t  s_txbuf[USB_PC_UP_FRAME_LEN];
static uint32_t s_last_report_ms = 0;

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
            /* ★ 弧度 → 度，存成主臂内部单位；是否驱动舵机由使用方决定
             *   （当前主臂固件只读不写 —— 写目标的应用逻辑尚未开发） */
            usb_pc_target_deg[i] = (float)v / USB_PC_SCALE * USB_PC_RAD2DEG;
        }
        usb_pc_target_flag = rx[4 + 24];
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
