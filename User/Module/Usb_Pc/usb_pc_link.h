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
 *   0x63  PC → 主臂   立即原位保持 HOLD(int8，值忽略)                       数据长 1
 *
 * ★ 为什么需要 0x63（2026-09-27 实测教训）：
 *   位置伺服是"持有目标"模型 —— **不再写新目标，它仍会继续奔向最后写入的 goal**，
 *   没有任何"取消"手段（`usb_pc_write_timeout_ms` 只是不再写新目标，不是刹车）。
 *   实测：脚本已完全停发 3s，J2 仍走了 9.49°、J3 走了 8.70°（J3 是往上走，排除重力）。
 *   而想用"目标=当前位置"来刹车也写不进去 —— 普通 0x61 会因"差值 < eps"判定
 *   "已到位"而不产生任何总线事务。
 *   ⇒ 所以专门留这条命令：**绕过 eps 判定，强制把六路 goal 写成当前位置**，
 *     并让残留目标失效。这才是可靠的"停"。
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
#define USB_PC_ID_ARM_HOLD          0x63  /* PC → 主臂：立即原位保持（真正的"停"） */

/* ---- 数据长度 / 帧长 ---- */
#define USB_PC_UP_DATA_LEN          25
#define USB_PC_UP_FRAME_LEN         (4 + USB_PC_UP_DATA_LEN + 2)   /* = 31 */
#define USB_PC_DOWN_TARGET_LEN      25
#define USB_PC_DOWN_ENABLE_LEN      1
#define USB_PC_DOWN_HOLD_LEN        1

/* ---- 角度单位换算 ---- */
#define USB_PC_DEG2RAD              0.01745329252f   /* π/180 */
#define USB_PC_RAD2DEG              57.29577951f     /* 180/π */
#define USB_PC_SCALE                10000.0f         /* 弧度 × 10000 定点 */

/* ============ 上报周期 ============ */
/* 单位 ms。
 * ★ 2026-09-27：30(≈33Hz) → **16(名义 62.5Hz)**。
 *   原因：PC 侧感受到的"读数滞后"主要来自本周期的量化（均值 T/2、最坏 T）。
 *   实测 55.8~56.3 fps（不是 62.5）—— 因为上报判定发生在 ~3ms 的 AlgorithmTask 循环里，
 *   16ms 实际落到约 18ms ⇒ 55.6fps。想更贴近 60Hz 可设 14(≈66fps)。
 *   带宽：31B × 62.5 ≈ 1.94 kB/s，USB FS CDC 无压力。 */
extern volatile uint16_t usb_pc_report_ms;

/* ============ PC 下发的数据（均已换算成【度】，与主臂内部单位一致） ============ */
extern volatile float    usb_pc_target_deg[6];   /* 0x61 解析结果 */
extern volatile uint8_t  usb_pc_target_flag;     /* 0x61 的"立即生效"标志 */
extern volatile int8_t   usb_pc_enable_req;      /* 0x62 请求：-1=无 / 0=失能 / 1=使能。使用方处理完应置回 -1 */
extern volatile uint8_t  usb_pc_hold_req;        /* 0x63 请求：1=待执行"立即原位保持"，执行完自动清 0 */

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

/* 单帧最多允许"发生变化"的关节数。
 *
 * ★ 2026-09-27 实机教训：默认值曾设为 1，结果【几乎完全无法使用】——
 *   因为 PC 发来的目标里，只要任意一路与固件端"当前角"差超过 eps(0.5°)，
 *   它也算"需要动作"。而真实系统里这种小偏差几乎必然存在
 *   （重力微动、机械耦合、USB 往返 30ms 期间的位置变化），
 *   于是经常出现 2 路同时偏离 → 整帧被拒 → 表现为"命令完全没反应"。
 *   实测计数器：reject_cnt 疯涨而 write_cnt 恒为 0。
 *
 * ⇒ 默认改为 6（全部允许）：安全由"每路 10° 限幅 + 200ms 超时 + 逐路软限位"保证，
 *   而不是靠"限制路数"。
 * ⇒ 想做"单关节单独测试"时，把这个值临时改成 1（volatile，调试器可在线改），
 *   并让 PC 侧把其余关节的目标填成【各自当前值】。 */
extern volatile uint8_t  usb_pc_write_max_joints;

/* 每路相对【当前角】的限幅(度)。默认 10.0。
 * 目标与当前角之差超过它就被钳到 ±本值后执行（不是拒绝，是缓慢逼近）。 */
extern volatile float    usb_pc_write_limit_deg;

/* 透传给 WritePosEx 的速度/加速度。
 * ★ 2026-09-27 实测标定（仓库里没有单位文档，用"设定值 40 + 实测到位速度"反推）：
 *      设定 40 → 实测 2.5~3.1°/s ⇒ **单位 ≈ 0.088 °/s per unit**
 *      （即 steps/s 家族：1 步 = 1/4096 圈，0.0879°/步）
 *   ⇒ 角速度(°/s) ≈ 设定值 × 0.088 ；设定值 ≈ 目标角速度(°/s) × 11.4
 *   ⇒ 常用参考：50° 的行程想 4s 走完 → 12.5°/s → **≈170**
 *               想 8s 走完 → 6.25°/s → ≈70
 * 默认取 170（中速，约 12°/s）。0 = 舵机内部默认（最快，不建议）。
 * ⚠️ 只影响 0x61（PC 驱动主臂）这条通路，**不影响 0x0302 遥操作**。 */
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

/* ============================ 逐路软限位 ============================
 * 背景：各关节机械行程不同。若 PC 发来超程目标，舵机会一直顶在机械限位上
 *       满力堵转 —— 2026-09-27 实测因此出现过"撞限位后机械滑脱、角度猛跳 113°"。
 * 做法：把目标角先钳到这组边界内，再参与后面的限幅与执行。
 *       ⇒ 即使 PC 发来离谱目标，最多走到边界，不会顶着限位持续出力。
 *
 * ★ 已按实测填写（2026-09-28 用户用 OctoLink 读 total_angle 在机械两端取值）：
 *       J1 -90 ~ +90    J2 -170 ~ 0     J3    0 ~ +180
 *       J4 -180 ~ +180  J5  -25 ~ +90   J6 -180 ~ +180
 *   并统一向内收 3° 余量（见 usb_pc_link.c 的 USB_PC_SOFT_MARGIN_DEG）。
 *
 * ⚠️ 这些值是【相对上电/复位基准】的角度 —— 每次上电（或 reset）时机械臂必须停在
 *    【与测量时相同的参考姿态】，否则整组限位会偏移。这是使用前提，不是 bug。
 * 两者都是 volatile，调试器可在线改，不必重烧。 */
extern volatile float    usb_pc_soft_min_deg[6];
extern volatile float    usb_pc_soft_max_deg[6];

/* 堵转判定：连续多少次"命令了却几乎没位移"就停止驱动该路。默认 20（约 0.6s）。
 * 0 = 关闭该保护。触发时该路会被跳过，并累加 usb_pc_write_stall_cnt。
 * ⚠️ 2026-09-27 起【已废弃】：改为按真实时间窗判定（见 usb_pc_stall_window_ms）。
 *    原因：这个"次数"是按 30ms/次估的，而实际调用周期是 3ms ⇒ 真实窗口只有 60ms。 */
extern volatile uint8_t  usb_pc_stall_limit;
/* 堵转判定窗口(ms)。默认 600。
 * ★ 2026-09-27 重写：原来是"连续 N 次调用位移 < eps"，按"每 30ms 一次"估成 0.6s，
 *   但该函数实际被主循环【每 3ms】调一次（algorithm_task_delta = 3）⇒ 真实窗口只有
 *   20×3ms = 60ms ⇒ **任何低于 50°/s 的正常运动都被误判成"卡住"**，关节被周期性跳过，
 *   表现为"速度上不去、一顿一顿"，还容易被误认为"舵机速度档不够"（本项目白跑了一轮扫描）。
 *   ⇒ 现在按【真实时间窗】判定：某路在 usb_pc_stall_window_ms 内位移都没超过
 *     usb_pc_stall_eps_deg 才判堵转。0 = 关闭该保护。 */
extern volatile uint16_t usb_pc_stall_window_ms;
/* 判定"几乎没位移"的阈值(度)。默认 0.3（读数分辨率为 1 刻 = 0.0879°，故 0.3° 足够灵敏）。 */
extern volatile float    usb_pc_stall_eps_deg;

/* ============ 2026-09-28 新增：堵转的两个修复 ============
 * 背景：旧实现 (a) 判堵转后只 `continue`（不再写新目标），但舵机是"持有目标"模型，
 *       它仍会继续朝上次写入的 goal（越界方向）顶 ⇒ 机械应力没解除；
 *       (b) `s_stalled` 只能靠"它自己动了"来清除，而它不动正是因为被跳过
 *       ⇒ 一次误判 = 该路永久失效（之后怎么发都不动，易误诊成机械问题）。
 * 详见 usb_pc_link.c 内 usb_pc_arm_write_apply() 的长注释。 */

/* 1 = 判定堵转时写一次 goal=当前角，把顶限位的推力【卸载】掉。默认 1。
 * 设为 0 则退回旧行为（只跳过、不卸载）。 */
extern volatile uint8_t  usb_pc_stall_unload;

/* 自动重试的限流间隔(ms)，默认 1000。设为 0 = 永不自动重试
 * （该路一直保持卸载态，只能靠 0x63 或复位解除）。 */
extern volatile uint16_t usb_pc_stall_retry_ms;

/* 自动重试的目标变化阈值(度)，默认 3.0。
 *   > 0 : 需超过该值   = 0 : 任何变化即可   < 0 : 忽略变化条件，纯时间重试 */
extern volatile float    usb_pc_stall_retry_deg;

/* ==================== ★★★ 角度估计一致性护栏（2026-09-27 新增，安全关键）★★★ ====================
 *
 * 为什么必须有它（实测失控事故）：
 *   0x61 是【绝对角】协议，PC 发的是"它在读取时刻看到的角 + Δ"。正常情况下累积误差
 *   会在 `delta = target_enc − accumulated_enc` 里自动抵消，所以平时看不出问题。
 *   但一旦角度估计被污染（读取成串失败丢位移 / 过零修正误判 / 撞限位后机械滑脱），
 *   PC 仍在发同一帧里的绝对目标 ⇒ 固件算出的 delta 会突然变成几十上百度的【合法】命令
 *   （raw 仍在 0..4095、也不超半圈，所有量程检查全部通过）⇒ 机械臂朝错误方向猛冲。
 *   实测：J1 累积量被污染 +1200 刻(+105°) ⇒ 固件发出 −117° 命令 ⇒ 冲出撞到机械限位，
 *         用户手动失能才停住。
 *
 * 护栏：动手前校验 `raw == base_raw + dir×累积刻度`（mod 4096）。
 *   健康值：本项目实测 5/6 路残差仅 ±10 刻(≈0.9°，死区累积器残留)；
 *   污染量级是上百刻 ⇒ 50 刻的容差既能通过健康路、又能拦下污染路。
 * ⚠️ 必须在【任何写舵机之前】判定：一旦漏过，后面每步 ±10° 地朝错误目标走，
 *   2 秒就能走出 100°+，逐帧限幅根本挡不住。
 */
extern volatile float    usb_pc_consist_tol_ticks;   /* 容差(刻)，默认 50 */
extern volatile uint32_t usb_pc_consist_err_cnt;     /* 因一致性不合被拒绝驱动的帧数 */
extern volatile float    usb_pc_consist_err_ticks;   /* 最近一次的误差量(刻)，供诊断 */

/* ---- 调试计数（供 OctoLink / GDB 观察） ---- */
extern volatile uint32_t usb_pc_write_cnt;         /* 实际执行移动的关节次数 */
extern volatile uint32_t usb_pc_write_reject_cnt;  /* 因超过单帧路数闸被拒的帧数 */
extern volatile uint32_t usb_pc_write_stale_cnt;   /* 因未使能 / 标志为0 / 超时而未执行的次数 */
extern volatile uint32_t usb_pc_write_stall_cnt;      /* 堵转【发生次数】（状态跃迁计数）
                                                        * ⚠️ 语义已变更：旧版是"被跳过的帧数"，
                                                        *    那会按 ~333/s 增长（实测 2140），
                                                        *    看着像"一直在顶"，其实只是每帧跳过一次 */
extern volatile uint32_t usb_pc_write_stall_skip_cnt; /* 因堵转而跳过该路的【帧数】（旧语义） */
extern volatile uint32_t usb_pc_stall_unload_cnt;     /* 实际执行"卸载"(写 goal=当前角)的次数 */
extern volatile uint32_t usb_pc_stall_retry_cnt;      /* 自动重试放行的次数 */
extern volatile uint32_t usb_pc_hold_cnt;          /* 执行过多少次"立即原位保持" */
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

/**
 * @brief  0x63：立即原位保持 —— 让机械臂**真正停住**。
 * @note   为什么普通 0x61 做不到：位置伺服"持有目标"，停发新目标它仍会奔向旧 goal；
 *         而写"目标=当前位置"又会被 eps 判定为"已到位"、不产生任何总线事务。
 *         本函数**绕过 eps 判定**，对六路逐个 `goal = 当前角`，并让残留目标失效
 *         （同时把 usb_pc_target_deg[] 同步成当前角、清掉 target_flag）。
 *         不需要已使能 —— 自由态下调用是无害的（舵机不输出扭矩，只是写入 goal）。
 * @return 实际成功写入的路数（0~6）。读不到某路当前位置时该路跳过，不会盲写。
 */
uint8_t usb_pc_arm_hold_now(void);

#endif /* USB_PC_LINK_H */
