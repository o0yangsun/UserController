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

/* ==========================================================================================
 *                      0x61 写位置：安全闸（全部 volatile，可在线调参）
 * ==========================================================================================
 * 设计原则：**默认最保守，靠显式调参放宽**。
 * 因为"PC 直接驱动舵机"是本模块里唯一会让机械结构产生运动的通路，
 * 所以默认值刻意压到最小（单帧只动 1 路、单次限幅 10°），先保证安全。
 * ========================================================================================== */

/* 单帧最多允许"发生变化"的关节数。默认 1。
 * 若一帧里需要动的关节数超过本值 → 整帧拒绝执行（记入 usb_pc_write_reject_cnt）。
 * 想一次跟踪整条臂时，把它放宽到 6。 */
extern volatile uint8_t  usb_pc_write_max_joints;

/* 每路相对【当前角】的限幅(度)。默认 10.0。
 * 目标与当前角之差超过它就被钳到 ±本值后执行（不是拒绝，是缓慢逼近）。 */
extern volatile float    usb_pc_write_limit_deg;

/* 透传给 WritePosEx 的速度/加速度（0,0 = 用舵机内部默认值）。
 * 想让它动得更慢/更柔，把速度调小（例如 100~300）。 */
extern volatile uint16_t usb_pc_write_speed;
extern volatile uint8_t  usb_pc_write_acc;

/* 命令有效期(ms)。默认 200。
 * 距最后一次收到 0x61 超过它 → 停止驱动。
 * ★ 必要性：PC 一旦断连，usb_pc_target_flag 会一直保持 1、目标角也是最后一帧的值，
 *   没有超时的话主臂会被"残留目标"持续驱动 —— 这是个真实的安全隐患。 */
extern volatile uint16_t usb_pc_write_timeout_ms;

/* "视为已到位"的角度阈值(度)。默认 0.5。
 * 角度差小于它的关节不产生任何总线事务，所以 PC 持续重发同一目标也无额外开销。 */
extern volatile float    usb_pc_write_eps_deg;

/* ---- 调试计数（供 OctoLink / GDB 观察） ---- */
extern volatile uint32_t usb_pc_write_cnt;         /* 实际执行移动的关节次数 */
extern volatile uint32_t usb_pc_write_reject_cnt;  /* 因超过单帧路数闸被拒的帧数 */
extern volatile uint32_t usb_pc_write_stale_cnt;   /* 因未使能 / 标志为0 / 超时而未执行的次数 */
extern volatile float    usb_pc_last_write_deg[6]; /* 最近一次实际写入的目标角(度) */
extern volatile uint8_t  usb_pc_last_write_idx;    /* 最近写入的通道号+1（0=从未写过） */

/**
 * @brief  0x61 写位置：把 PC 下发的目标角应用到舵机。由主循环调用。
 * @note   四项安全闸，任一不满足即不执行（并计入相应计数）：
 *           ① 必须已锁死（STS3215 的 sts3215_locked == 1）—— 自由态下不驱动舵机
 *           ② 必须带生效标志，且距最后一次收到 0x61 未超过 usb_pc_write_timeout_ms
 *           ③ 单帧内"需要动作"的关节数 ≤ usb_pc_write_max_joints（默认 1）
 *           ④ 每路相对当前角限幅 usb_pc_write_limit_deg（默认 10°）
 *         另：只要六路全景里有一路读数不可信，整帧不动（限幅判断必须基于真实当前角）。
 * @return 本次实际执行移动的关节数（0 = 未执行任何动作）
 */
uint8_t usb_pc_arm_write_apply(void);

#endif /* USB_PC_LINK_H */
