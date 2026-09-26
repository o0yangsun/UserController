#ifndef STS_MODULE_H
#define STS_MODULE_H

#include <stdint.h>
#include "SMS_STS.h"

#define STS3215_NUM   6      // 挂载的舵机数量
#define STS3215FILTER 0.3f   // 低通滤波系数

/* 上电"输出基准"(单位为 12-bit 刻度,4096=360°)
 * 0    = 对外上电即报 0°(当前统一采用:F411(AS5600)与车端 0x0302 解析均以 0° 起算,便于对齐)
 * 2048 = 上电即报 180°(仅当接收端希望从中位起算时才用)
 * 注意:"输出基准"与"舵机物理零点"是【两件不同的事】,二者解耦 ——
 *       物理零点由 algorithm_task.c 的 sts3215_use_mid_cal[] 决定(可让它居中、双向有余量),
 *       而输出基准由本宏/传参决定(可保持 0° 起算)。当前配置:物理居中 + 输出 0°。 */
#define STS3215_BASE_MID 2048

/* 上电基准"建立确认"容差(单位:刻度)
 * 用途:消除"中位校准生效时机"与"固件首次读取"之间的时序竞争 ——
 *       校准一生效,舵机 raw 会一次性跳到 2048(幅度可达 ±2048 刻度 = ±180°),
 *       若此时基准已经用校准前的 raw 建立,这一跳就会被当成"真实转动"累加进去,
 *       表现为"上电后某几路一上来就是 ±180°"(实测复现过)。
 * 做法:首次读到的值必须先"连续两次相差不超过本容差"才认可为基准;
 *       期间任何一次跳变都只重新同步基准、不产生增量。
 * 取值:正常静置时相邻 3ms 采样差值 ≤1 刻度,故 16 刻度(1.4°)足够宽裕;
 *       而校准跳变幅度是数百~2048 刻度,必然被判为"跳变"并重新建立。 */
#define STS3215_BASE_TOL 16

/* ============================ KEY 按键(PA15) 配置 ============================
 * 硬件：达妙 DM-MC-Board02 板载按键。官方 BSP 中定义为
 *       KEY__INPUT_Pin = GPIO_PIN_15 / KEY__INPUT_GPIO_Port = GPIOA
 *       (见 dm_bsp/damiao_mc02_bsp/dm02_mouse_test/Core/Inc/main.h)
 * 功能：按一次 → 六路舵机扭矩使能(锁死)；再按一次 → 失能(回到上电默认的自由态)。
 *
 * ★★ 极性开关 —— 整个改动只需要确认这一个宏 ★★
 *      1 = 按下时 PA15 为【低】电平（常见接法：外部上拉 + 按键对地）  ← 当前默认
 *      0 = 按下时 PA15 为【高】电平
 *
 *   为什么不能从代码推断：达妙 BSP 里这个引脚**只声明、从未读取过**，
 *   且配置为 GPIO_MODE_INPUT + GPIO_NOPULL，无法判断有效电平。
 *
 *   判定方法（不用万用表）：烧录后【不要碰按键】，用调试器读
 *       sts3215_key_dbg_high_cnt / sts3215_key_dbg_low_cnt
 *   计数大的那个电平 = 按键【松开】时的空闲电平 ⇒
 *       high 大 ⇒ 空闲为高 ⇒ 按下为低 ⇒ 本宏应为 1
 *       low  大 ⇒ 空闲为低 ⇒ 按下为高 ⇒ 本宏应为 0
 *
 *   注意：宏只在【复位/重新 init】时生效（决定内部上下拉方向）；若不想重烧，
 *   可直接用调试器改写运行期变量 sts3215_key_active_low（逻辑立刻切换）。 */
#define STS3215_KEY_ACTIVE_LOW      1

/* 按键去抖时间(ms)。与任务节拍解耦(用 HAL_GetTick 计时)，改任务周期不用动这里。 */
#define STS3215_KEY_DEBOUNCE_MS     20u

/* 舵机 ID 列表 (定义于 STS_Module.c) */
extern uint8_t motor_ids[STS3215_NUM];

// 定义STS3215编码器结构体（必须在函数声明之前）
typedef struct
{

    int16_t encoder_value;       // 当前编码器的值
    int16_t last_encoder_value;  // 上一次编码器的值
    int32_t total_encoder_value; // 编码器的累计值

    float last_total_angle; // 上一次角度结果，用来参与下一次的低通滤波
    float total_angle;      // 低通滤波得到角度结果

    int is_initialized; // 通信健康标志：1=已读到有效数据(含运行期自愈)，-1=初始化/通信失败
    int is_received;    // 最近一次读取的真实结果：1=读到有效刻度，0=读失败(超时/越界)

    int16_t deadband_accum; // 死区累积器(单位:刻度)。被死区暂扣的差值先在这里积攒，
                            // 累积量超过阈值后一次性放行。详见 sts3215_angle_get()。
                            // 必要性：采样周期仅 3ms，慢速操作的"每帧差值"远小于抖动幅度，
                            //         用"丢弃式"死区会把正常慢速转动整段吞掉(见 .c 注释)。

    uint8_t base_ready;     // 上电基准建立状态：0/1 = 尚未确认(只同步、不累加)，
                            // ≥2 = 已确认(正常累加)。详见 STS3215_BASE_TOL 与 sts3215_angle_get()。

} STS3215_Encoder_t;

/* ---- 函数声明 ---- */
void sts3215_setup(void);
// base_encoder_value: 上电基准刻度(0=标定位姿为0°;STS3215_BASE_MID=上电显示180°)
void STS3215_Init(STS3215_Encoder_t *encoder, int32_t base_encoder_value);
// 中位校准:把舵机当前位置设为量程中点(2048)。成功后该舵机 raw 上电即 ≈2048,
// 两个方向各留约 2048 刻度余量,避免零点压在 0/4095 边界导致单方向测不到变化。
int  sts3215_calib_mid(uint8_t id);
int  sts3215_encoder_get(uint8_t idx);   // 返回 0=成功读到有效刻度；-1=失败(舵机未接/握手失败/越界)
void sts3215_angle_get(uint8_t idx);

/* ---------------- KEY 按键(PA15)：锁死 / 自由 切换 ---------------- */

/* 运行期极性开关：初值 = STS3215_KEY_ACTIVE_LOW。
 * 可用调试器(OctoLink/GDB)直接改写做对比实验，不必重新烧录 ——
 * 这样一次烧录就能把两种极性都试出来。
 *   1 = 按下为低电平 ; 0 = 按下为高电平 */
extern volatile uint8_t  sts3215_key_active_low;

/* 极性自诊断计数器：上电起累计采样到的高/低电平次数。
 * 【不碰按键】时读这两个值，谁大谁就是"松开"时的空闲电平。
 * 注意：只在 sts3215_key_poll() 被调用时累加，所以要在主循环跑起来之后读。 */
extern volatile uint32_t sts3215_key_dbg_high_cnt;
extern volatile uint32_t sts3215_key_dbg_low_cnt;

/* 扭矩锁定状态：1 = 锁死(扭矩使能)，0 = 自由(失能)。
 * 上电默认 0 —— 与本次改动前的行为完全一致。 */
extern volatile uint8_t  sts3215_locked;

/* 最近一次切换实际成功的路数(0~6)，供调试观察。 */
extern volatile uint8_t  sts3215_lock_ok_count;

/* 配置 PA15 为输入(内部上下拉方向跟随 STS3215_KEY_ACTIVE_LOW)。上电时调用一次。
 * 注：GPIOA 时钟已在 MX_GPIO_Init() 里打开；PA15 复位默认是 JTDI，本工程调试口只用
 *     SWD(PA13/PA14)，故可安全当 GPIO 用。 */
void    sts3215_key_init(void);

/* 按键扫描：需在主循环里周期性调用。
 * 返回 1 = 本次检测到一次"按下"沿(已去抖)；0 = 无事件。
 * 【为什么轮询而不是外部中断】舵机总线是阻塞式 HAL(ftUart_Read 最坏阻塞 3ms)，
 * 一次扭矩切换要 6 路 ×(读位置+写目标+写扭矩) ≈ 18 次总线事务，
 * 在 ISR 里做会严重拖垮系统 —— 必须放在任务里。 */
uint8_t sts3215_key_poll(void);

/* 使能/失能单路舵机扭矩。enable: 1=锁死 0=自由。
 * 返回 0=成功，-1=失败(通信失败/读数越界/下标越界)。 */
int     sts3215_set_torque(uint8_t idx, uint8_t enable);

/* 六路一起切换。返回实际成功的路数(0~6)。
 * 只有 6 路**全部成功**才翻转 sts3215_locked —— 少一路就不翻，这样再按一次即重试；
 * 而写同样的值本身是幂等的，所以可自愈。 */
uint8_t sts3215_set_torque_all(uint8_t enable);

#endif
