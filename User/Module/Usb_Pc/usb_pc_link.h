/**
 * @file    usb_pc_link.h
 * @brief   PC ↔ 主臂板 的 USB CDC 通信协议（移植自车端 Engineering_Robot_H723_）
 *
 * 协议与车端【完全一致】，命名 ID 走 0x6x 段（车端占用 0x1x/0x2x/0x3x，同一台 PC 上抓包一眼可分）：
 *
 *   帧格式： [0]帧头=0xFF [1]地址=0x05 [2]命名ID [3]数据长度N [4..]数据 [N+4]sum [N+5]addr
 *            帧总长 = 4 + N + 2
 *   校验：   sum = Σbuf[0..N+3] ; addr = Σsum（累加和） ; 两者都取低 8 位
 *
 *   0x60  主臂 → PC   6 关节角(弧度×10000, int32 小端) + 使能状态(int8)   数据长 25，帧长 31
 *   0x61  PC → 主臂   6 个目标关节角(弧度×10000) + 生效标志(int8)          数据长 25
 *   0x62  PC → 主臂   使能/失能(int8)                                      数据长 1
 *
 * ★ 单位约定：PC 侧一律用【弧度】；主臂内部一律用【度】。
 *   所以 上行打包时 度→弧度，下行解析时 弧度→度，转换都在本模块内部完成。
 *   这样 PC 上解析车端(0x20)和主臂(0x60)用的是同一套代码。
 */

#ifndef USB_PC_LINK_H
#define USB_PC_LINK_H

#include <stdint.h>

/* ---- 帧基本字段 ---- */
#define USB_PC_HEAD                 0xFF
#define USB_PC_ADDR                 0x05

/* ---- 命名 ID ---- */
#define USB_PC_ID_ARM_UP            0x60  /* 主臂 → PC：关节角 + 使能状态 */
#define USB_PC_ID_ARM_TARGET        0x61  /* PC → 主臂：目标关节角 */
#define USB_PC_ID_ARM_ENABLE        0x62  /* PC → 主臂：使能 / 失能 */

/* ---- 数据长度 / 帧长 ---- */
#define USB_PC_UP_DATA_LEN          25
#define USB_PC_UP_FRAME_LEN         (4 + USB_PC_UP_DATA_LEN + 2)   /* = 31 */
#define USB_PC_DOWN_TARGET_LEN      25
#define USB_PC_DOWN_ENABLE_LEN      1

/* ---- 角度单位换算 ---- */
#define USB_PC_DEG2RAD              0.01745329252f   /* π/180 */
#define USB_PC_RAD2DEG              57.29577951f     /* 180/π */
#define USB_PC_SCALE                10000.0f         /* 弧度 × 10000 定点 */

/* ============ 上报周期 ============ */
/* 单位 ms。建议 30~50（30~33 Hz，与 0x0302 同级，见移植文档 §4.1）。
 * 用 volatile 修饰，可直接用调试器在线改写测试，无需重新烧录。 */
extern volatile uint16_t usb_pc_report_ms;

/* ============ PC 下发的数据（均已换算成【度】，与主臂内部单位一致） ============ */
extern volatile float    usb_pc_target_deg[6];   /* 0x61 解析结果 */
extern volatile uint8_t  usb_pc_target_flag;     /* 0x61 的"立即生效"标志 */
extern volatile int8_t   usb_pc_enable_req;      /* 0x62 请求：-1=无 / 0=失能 / 1=使能。使用方处理完应置回 -1 */

/* ============ 调试计数（供 OctoLink / GDB 观察） ============ */
extern volatile uint32_t usb_pc_tx_cnt;          /* 成功发出的 0x60 帧数 */
extern volatile uint32_t usb_pc_tx_busy_cnt;     /* 因 USBD_BUSY 丢弃的帧数 */
extern volatile uint32_t usb_pc_rx_cnt;          /* 校验通过的接收帧数 */
extern volatile uint32_t usb_pc_rx_bad_cnt;      /* 校验失败/长度不符的帧数 */
extern volatile uint32_t usb_pc_last_id;         /* 最近一次收到的命名 ID */
extern volatile uint32_t usb_pc_tick;            /* 本模块被调用的次数（≈任务循环次数） */

/**
 * @brief  USB PC 链路服务函数。需要主循环周期性调用（本工程挂在 AlgorithmTask）。
 * @note   每轮做两件事：
 *           ① 轮询接收：CDC_Receive_HS 已在 USB 中断里把数据搬进全局缓冲区并置标志，
 *              这里只做标志检查 + 解析（无数据时代价约等于一次 if 判断）；
 *           ② 限速上报：按 usb_pc_report_ms 周期发一帧 0x60。
 *         ⚠️ 不要把它塞进 send_task 的 0x0302 发送路径 —— 别拖慢 0x0302 的节拍。
 */
void usb_pc_service(void);

#endif /* USB_PC_LINK_H */
