#include "STS_Module.h"
#include "SCS.h"
#include "SCSCL.h"
#include "INST.h"
#include "algorithm_task.h"
#include "main.h"      /* HAL 库 + GPIO_PIN_x / GPIO_PULLx / HAL_Delay */
#include <stdbool.h>

/* 舵机 ID 表：对应总线上的 1~6 号舵机(顺序即数据帧里的通道顺序) */
uint8_t motor_ids[STS3215_NUM] = {1, 2, 3, 4, 5, 6};

/* 每路累计方向：+1 = 刻度增大时 total_angle 增大(默认同向)；
 *               -1 = 反向计数(该路机械安装方向与其余相反时用)。
 * 索引 = 通道号(0..STS3215_NUM-1)，与 motor_ids[] / 数据帧通道顺序一一对应。
 * 用途：让 6 路的"正转方向"统一，上位机不必为每个关节单独记符号。 */
static const int8_t sts3215_dir[STS3215_NUM] = {1, -1, -1, 1, -1, 1};
/* 依据：与老控制器（F411 + AS5600）做同口径方向比对后的结果。
 * 判据 = "每个关节第一个大幅动作的符号" 两侧是否一致：
 *   J1 一致(保持 +1) │ J2 需取反(-1) │ J3 取反后一致(保持 -1)
 *   J4 一致(保持 +1) │ J5 需取反(-1) │ J6 取反(-1)
 * 比对原始数据：老侧首段符号为 J1− J2− J3+ J4− J5+ J6+；
 *               新侧首段符号为 J1− J2+ J3+ J4− J5− J6−（J3 已含取反）。
 *
 * ⚠️ 2026-09-22 真车联调修正：J6 最终取 **+1**，不是上面比对推出的 -1。
 *    按"读数符号对齐老控制器"J6 应为 -1，但真车实测 6 号关节方向相反；
 *    改为 +1 后"操作手正转 ⇒ 机械臂 6 号正转"成立（改车端 DMmotor_task.c 亦可，
 *    但那里有"运行时"和"上电初始化"两条路径，必须同时改，否则上电瞬间会反向串一下）。
 *
 * ⚠️ 读数符号一致 ≠ 机械臂运动方向一致：车端 smooth_motion_N 另有一套 per-joint ± 符号会叠加，
 *    最终判据仍为"操作手正转 ⇒ 机械臂对应关节正转"。 */

/* 每路死区(单位:编码器刻度;4096 刻度 = 360°，故 1 刻度 = 0.0879°)
 * 用途：抑制"手搭在机械臂上"时传导进来的微颤(表现为读数在几刻度内摆动)。
 *
 * 取值依据(固件内 3ms 节拍统计器实测)：
 *   - 完全静置(无人接触) 35s → 六路 raw 极差 **全部为 0 刻度**(电气噪声为 0、无自重漂移)；
 *   - 手接触但不主动转动   → 六路极差 **4~8 刻度(0.35~0.70°)**  ← 这才是要过滤的量；
 *   - 主动转动             → 几十刻度以上，远超死区。
 *   故 8 刻度(0.70°)可覆盖实测最大的手部微颤。
 *
 * ⚠️ 必须配合"累积式"实现(见 sts3215_angle_get())，不能简单丢弃：
 *   采样周期仅 3ms，慢速操作每帧差值很小 —— 例如 30°/s 时每帧只有 30×0.003×11.4 ≈ 1 刻度，
 *   若"丢弃式"阈值设 8 刻度，等价于把 8/(0.003×11.4) ≈ 234°/s 以下的所有转动全部吞掉，
 *   机械臂会表现为"完全不动"。累积式则让双向微颤自然抵消、单向慢速转动积攒后放行。 */
static const int16_t sts3215_deadband[STS3215_NUM] = {8, 8, 8, 8, 8, 8};

static int pos = 0;

//  base_encoder_value: 上电基准刻度。
//  传 0    → 标定位姿即 0°(原行为);
//  传 STS3215_BASE_MID(2048) → 上电即显示 180°,两侧各留 2048 刻度余量。
//  同步初始化 last_total_angle,避免首帧被 0.3 低通从 0 慢慢爬上来。
void STS3215_Init(STS3215_Encoder_t *encoder, int32_t base_encoder_value)
{
    encoder->encoder_value = 0;
    encoder->last_encoder_value = 0;
    encoder->total_encoder_value = base_encoder_value;
    encoder->last_total_angle = (float)base_encoder_value * (360.0f / 4096.0f);
    encoder->total_angle = encoder->last_total_angle;
    encoder->is_initialized = -1;
    encoder->is_received = 0;
    encoder->deadband_accum = 0;
    encoder->base_ready = 0;   // 基准未经确认：头两次采样只同步、不累加
    encoder->fail_cnt = 0;
    encoder->ever_ok = 0;
    encoder->base_raw = 0;
    encoder->glitch_cnt = 0;
}

void sts3215_setup(void)
{
    setEnd(0); // SMS_STS舵机为小端存储结构
}

//  中位校准:把该舵机"当前位置"设为量程中点(2048)
//  等价于官方 SDK 的 CalibrationOfs(ID) —— 往扭矩使能寄存器写 128(厂商约定的中位校准触发值)。
//  用途:让 raw 上电落在量程中间,两个方向都有刻度余量。
//  注意:与 Calibration()(INST_OFSCAL,当前位姿=0)互斥,同一只舵机只用其中一个。
int sts3215_calib_mid(uint8_t id)
{
    uint8_t trig = 128; // 中位校准触发值

    rFlushSCS();
    writeBuf(id, SMS_STS_TORQUE_ENABLE, &trig, 1, INST_WRITE);
    wFlushSCS();
    return Ack(id);
}

//  读取指定舵机的当前刻度值并存入对应实例成员
//  返回 0=成功；-1=失败(未接舵机/握手失败/越界)
int sts3215_encoder_get(uint8_t idx)
{
    STS3215_Encoder_t *enc = &sts3215_encoder[idx];

    pos = ReadPos(motor_ids[idx]);

    // 12-bit 编码器合法值域 0~4095；ReadPos 读失败时经符号位处理后返回 -32767(或 -1)
    // 失败/越界的读数一律丢弃，防止垃圾值参与角度累积
    if (pos < 0 || pos > 4095)
    {
        enc->is_received = 0;   // 真实失败标志(供上层判断失联)
        return -1;
    }

    enc->encoder_value = pos; // 解码两个字节 bit15为方向位,参数=0表示无方向位
    enc->is_received = 1;
    return 0;
}

void sts3215_angle_get(uint8_t idx)
{
    STS3215_Encoder_t *enc = &sts3215_encoder[idx];

    /* ---------- ① 读取失败 ----------
     * ★ 2026-09-27 修正（原来这里会丢位移）：
     *   旧实现是"一次失败 → 恢复后把当前刻度当新基准" ⇒ 那一小段位移被【永久丢弃】。
     *   而舵机快速运动时 ReadPos 很容易失败（总线繁忙 + 运动中响应慢），
     *   实测导致"物理已到位、读数却偏低"（J1 目标 3° 只报 1.667°，用户手感是到位了）。
     * 现在：只累加失败计数、【不动任何基准状态】——
     *   恢复后照常做差分，那段位移就能补回来。 */
    if (sts3215_encoder_get(idx) != 0)
    {
        enc->fail_cnt++;   // 饱和于 255（uint8_t 自动回绕到 0 也有兜底：>TOL 即判失联）
        return;
    }

    /* ---------- ② 上电后首次成功读到：建立基准 ----------
     * 此时 total_encoder_value 还是初值，故 base_raw 直接取当前 raw。
     * 若不区分这一步：last_encoder_value 初值为 0 而 raw 已是 2048(归中后)，
     * 直接差分会把整个归中偏移当成"转动量" → total_angle 凭空多 180°。 */
    if (enc->ever_ok == 0)
    {
        enc->ever_ok            = 1u;
        enc->fail_cnt           = 0u;
        enc->last_encoder_value = enc->encoder_value;
        enc->base_raw           = enc->encoder_value;
        enc->deadband_accum     = 0;
        enc->base_ready         = 0;   /* 待确认：连续两次一致才认可 */
        return;
    }

    /* ★★★ 2026-09-27 关键修正：长时失联【不再重建基准 / 不再丢弃位移】★★★
     * 原实现在 fail_cnt > STS3215_FAIL_TOL 时把当前刻度当新基准 ⇒ 那一段位移被
     * 【永久丢弃】。高速运动期间读取会成串失败，这会让 total_encoder_value 系统性少算 ——
     * 而 0x61 是【绝对角】协议：少算 = 上报角整体偏掉 = PC 发来的绝对目标被固件
     * 理解成大幅误动作。实测：J1 累积量被污染 +1200 刻(+105°)，随即被命令 −117°
     * 冲出去撞到机械限位（用户手动失能才停住）。
     * 现在：失联只清计数，差分照常累加 —— raw 是绝对量，跨过失败窗口的差值仍然有效。 */
    enc->fail_cnt = 0u;

    /* 上电(或失联恢复)后的基准确认期：只同步、不累加。
     * —— 消除"中位校准生效时机 vs 固件首次读取"的时序竞争：
     *    校准一生效 raw 会一次性跳到 2048(幅度可达 ±2048 刻度 = ±180°)，
     *    若基准是用校准前的 raw 建立的，这一跳就会被当成真实转动累加进去，
     *    表现为"上电后某几路一上来就是 ±180°"(已在实机上复现)。
     * 判据：相邻两次采样差值 ≤ STS3215_BASE_TOL 才连续计数，累计 2 次即认可基准；
     *       期间任何一次跳变都清零计数并只做同步 —— 跳变不会被累加，慢速小位移(<容差)也几乎无损。 */
    if (enc->base_ready < 2)
    {
        int16_t d = (int16_t)(enc->encoder_value - (int16_t)enc->last_encoder_value);
        if (d > -STS3215_BASE_TOL && d < STS3215_BASE_TOL)
        {
            if (enc->base_ready < 2)
                enc->base_ready++;
        }
        else
        {
            enc->base_ready = 0; // 发生跳变(典型为校准生效)：重新计数
        }
        enc->last_encoder_value = enc->encoder_value;
        /* ★ 同步 base_raw 以【保持恒等式】raw == base_raw + dir*total。
         *   本块退出时 base_ready 会变成 2，之后一致性护栏就要依赖这条恒等式；
         *   校准生效造成的 raw 跳变是"坐标原点变了"，不是真实位移，
         *   所以这里要把 base_raw 重新锚定，而不是把跳变量累加进 total。 */
        enc->base_raw       = (int16_t)(enc->encoder_value - sts3215_dir[idx] * enc->total_encoder_value);
        enc->deadband_accum = 0;
        return;
    }

    int16_t diff = (int16_t)(enc->encoder_value - (int16_t)enc->last_encoder_value); // 计算差值
    if (diff > 2048)
    {
        // 逆时针跳变（0 → 4095，实际是减少了一圈）
        diff -= 4096; // 修正为负数（代表逆时针转）
    }
    else if (diff < -2048)
    {
        // 顺时针跳变（4095 → 0，实际是增加了一圈）
        diff += 4096; // 修正为正数（代表顺时针转）
    }

    /* 诊断：单次采样跳变过大（> STS3215_GLITCH_TOL 刻 ≈ 35°，对应 11667°/s）
     * 几乎不可能是真实运动，多为总线错帧或机械滑脱。
     * ★ 仍然按真实位移累加，只记计数 —— 宁可多算也不要少算：
     *   少算会让"绝对角"整体偏掉，而 0x61 是绝对角协议，那等于放大成大幅误动作。 */
    if (diff > STS3215_GLITCH_TOL || diff < -STS3215_GLITCH_TOL)
    {
        enc->glitch_cnt++;
    }

    // 方向修正：机械安装方向相反的通道在这里取反(配置见 sts3215_dir[])
    // 注意放在过零修正之后 —— 过零判断依据的是原始刻度绕圈方向，与输出符号无关
    diff = (int16_t)(diff * sts3215_dir[idx]);

    // 死区(累积式)：抑制"手搭在机械臂上"传导进来的微颤(实测 4~8 刻度)。
    // 原理：每帧差值先累加到 deadband_accum ——
    //   · 手部微颤双向(+2,-3,+1,…) → 累积量在 0 附近来回摆动，不触发放行 → 读数纹丝不动；
    //   · 真实转动单向 → 累积量持续增长，超过阈值后一次性放行 → 慢速微调不会被吞掉。
    //   (若改成"简单丢弃"，因采样周期仅 3ms、慢速操作每帧差值很小，会把正常转动整段吞掉。)
    // 阈值见 sts3215_deadband[]，单位是刻度(1 刻度 = 0.0879°)。
    enc->deadband_accum = (int16_t)(enc->deadband_accum + diff);
    if (enc->deadband_accum > sts3215_deadband[idx] ||
        enc->deadband_accum < -sts3215_deadband[idx])
    {
        diff = enc->deadband_accum; // 越界：把积攒的位移一次性放行
        enc->deadband_accum = 0;
    }
    else
    {
        diff = 0; // 仍在死区内：暂扣不丢，等后续帧继续积攒
    }

    // 刻度值累加
    enc->total_encoder_value += diff; // 累加增量到累计值

    // 计算角度值
    enc->total_angle = (float)(enc->total_encoder_value) * (360.0f / 4096.0f);
    enc->total_angle = STS3215FILTER * enc->total_angle + (1 - STS3215FILTER) * enc->last_total_angle;

    //  缓存当前刻度值与角度 供下次使用
    enc->last_encoder_value = enc->encoder_value; // 保存本次值供下次 diff 使用
    enc->last_total_angle = enc->total_angle;     // 保存值供下次滤波使用
}

/* ==========================================================================================
 *                          KEY 按键(PA15)：舵机 锁死 / 自由 切换
 * ==========================================================================================
 * 硬件：达妙 DM-MC-Board02 板载按键 → PA15（BSP 里叫 KEY__INPUT）。
 * 功能：按一次 → 六路舵机扭矩使能(锁死)；再按一次 → 失能(回到上电默认的自由态)。
 *
 * 为什么放在任务里轮询、不用外部中断：
 *   舵机总线是阻塞式 HAL（ftUart_Read 最坏阻塞 3ms），一次扭矩切换 = 6 路 × (读位置+写目标+
 *   写扭矩) ≈ 18 次总线事务。在 ISR 里做这个会严重拖垮系统。本任务循环已有约 3ms 节拍，
 *   轮询的额外代价约等于一次 GPIO 读，可忽略。
 * ========================================================================================== */

/* 运行期极性开关：初值取编译期宏，可用调试器在线改写做对比实验。
 * 2026-09-26 实测确认：松开=高、按下=低 ⇒ 应为 1（与 STS3215_KEY_ACTIVE_LOW 一致）。 */
volatile uint8_t  sts3215_key_active_low = STS3215_KEY_ACTIVE_LOW;

/* 极性自诊断：累计采样到的高/低电平次数。
 * 2026-09-26 用它实测出极性的判读依据 —— 松开期间只有 high 涨、按下期间只有 low 涨。 */
volatile uint32_t sts3215_key_dbg_high_cnt = 0;
volatile uint32_t sts3215_key_dbg_low_cnt  = 0;

/* 扭矩锁定状态。上电默认 0 = 自由/失能 —— 与本次改动前的行为一致 */
volatile uint8_t  sts3215_locked        = 0;
volatile uint8_t  sts3215_lock_ok_count = 0;

/* 去抖状态（文件内私有） */
static uint8_t  key_raw_last = 0;   /* 上一次采样到的"是否按下"(已按极性换算) */
static uint8_t  key_stable   = 0;   /* 去抖后确认的稳定状态 */
static uint32_t key_t_change = 0;   /* 原始状态最后一次发生变化的时刻 */
static uint8_t  key_inited   = 0;   /* 首次调用标志：只采纳当前状态，不产生事件 */

void sts3215_key_init(void)
{
    GPIO_InitTypeDef key = {0};

    key.Pin  = GPIO_PIN_15;             /* PA15 */
    key.Mode = GPIO_MODE_INPUT;
    /* 内部上下拉朝"空闲(松开)"方向拉，保证按键松开时引脚电平确定、不浮空。
     * 达妙 BSP 里配的是 GPIO_NOPULL，说明板上外部应该已有一只电阻；内部再拉只是并联，无害。
     *   active_low=1（按下为低）→ 空闲应为高 → 上拉
     *   active_low=0（按下为高）→ 空闲应为低 → 下拉
     * 注意：这里用编译期宏而非运行期变量 —— 因为上电后不会再来重配这个引脚；
     *       若想在线翻转逻辑，直接改 sts3215_key_active_low 即可（内部拉电阻方向不影响判定）。 */
    key.Pull  = STS3215_KEY_ACTIVE_LOW ? GPIO_PULLUP : GPIO_PULLDOWN;

    HAL_GPIO_Init(GPIOA, &key);
}

uint8_t sts3215_key_poll(void)
{
    /* 原始电平：1 = 高 */
    const uint8_t  lvl = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET) ? 1u : 0u;
    const uint32_t now = HAL_GetTick();

    /* 极性自诊断计数 */
    if (lvl) sts3215_key_dbg_high_cnt++;
    else     sts3215_key_dbg_low_cnt++;

    /* 换算成"是否按下"（极性由运行期变量决定，可在线改） */
    const uint8_t raw = sts3215_key_active_low ? (uint8_t)(!lvl) : lvl;

    /* 首次调用：只把当前状态采纳为初始状态，不产生任何事件。
     * 此时 AlgorithmTask 已经跑完 ~600ms 的舵机初始化，引脚电平早已稳定；
     * 这样也顺带避免了"上电瞬间引脚状态变化被误当成一次按键"。 */
    if (!key_inited)
    {
        key_raw_last = raw;
        key_stable   = raw;
        key_t_change = now;
        key_inited   = 1;
        return 0;
    }

    /* 状态刚变：重新计时，暂不认定（这就是去抖） */
    if (raw != key_raw_last)
    {
        key_raw_last = raw;
        key_t_change = now;
        return 0;
    }

    /* 状态已稳定超过去抖时间 → 认定为有效状态。只在"按下"这个沿产生事件，松手不触发。 */
    if ((now - key_t_change) >= STS3215_KEY_DEBOUNCE_MS && key_stable != raw)
    {
        key_stable = raw;
        if (raw)
        {
            return 1;   /* ★ 一次按下事件 */
        }
    }

    return 0;
}

int sts3215_set_torque(uint8_t idx, uint8_t enable)
{
    int pos;

    if (idx >= STS3215_NUM)
    {
        return -1;
    }

    const uint8_t id = motor_ids[idx];

    if (enable)
    {
        /* ★★ 使能扭矩前必须先把 GOAL_POSITION 写成【当前实际位置】 ★★
         * 原因：GOAL_POSITION(地址 42) 这个寄存器本控制器从来没写过（只读位置），
         *       里面可能还残留上电默认值或很久以前的旧目标。直接置 TORQUE_ENABLE=1，
         *       舵机会立刻朝那个陈旧目标猛冲一下 —— 这既危险又完全没必要。
         *       先把 goal 写成当前位置，才是真正的"原地锁死"。
         * 注：goal == 当前位置 ⇒ 舵机无需运动，后面那个速度参数取 0 不影响结果。 */
        pos = ReadPos(id);
        if (pos < 0 || pos > 4095)
        {
            return -1;      /* 读不到位置就绝不使能，宁可这路不锁，也不要它乱动 */
        }
        WritePosEx(id, (int16_t)pos, 0, 0);
    }

    /* 扭矩开关：地址 SMS_STS_TORQUE_ENABLE(40)，写 1=使能、0=失能。
     * ⚠️ 别和 128 混用 —— 128 是"中位校准"触发值（见 sts3215_calib_mid()）。
     * writeByte() 自带 rFlushSCS → writeBuf → wFlushSCS → Ack（SCS.c:185），
     * 用不着手工刷总线，和 sts3215_calib_mid() 的做法等价。 */
    return (writeByte(id, SMS_STS_TORQUE_ENABLE, enable ? 1u : 0u) != 0) ? 0 : -1;
}

uint8_t sts3215_set_torque_all(uint8_t enable)
{
    uint8_t ok = 0;
    uint8_t failed_mask = 0;

    for (uint8_t i = 0; i < STS3215_NUM; i++)
    {
        if (sts3215_set_torque(i, enable) == 0)
        {
            ok++;
        }
        else
        {
            failed_mask |= (uint8_t)(1u << i);
        }
        HAL_Delay(2);   /* 给舵机留一点处理时间，也避免总线上连发 */
    }

    /* ★★ 失能方向必须"尽力而为" —— 2026-09-27 实测教训 ★★
     * 原来是"六路全部成功(ok==6)才翻转 sts3215_locked"，看似稳妥，实则危险：
     *   运动过程中某一路写失败（总线忙/舵机响应慢）⇒ 状态一直保持 1（已使能）
     *   ⇒ `usb_pc_arm_write_apply()` 继续放行 ⇒ PC 的 0x61 继续驱动机械臂，
     *   **操作手按了 PA15 失能也停不下来**（用户实测反馈："好像没有用"）。
     * 失能的语义是"放开"，必须无条件生效；失败的路再重试一轮，仅用于报告实际成功数。 */
    if ((enable == 0u) && (failed_mask != 0u))
    {
        for (uint8_t i = 0; i < STS3215_NUM; i++)
        {
            if ((failed_mask & (uint8_t)(1u << i)) == 0u)
            {
                continue;
            }
            HAL_Delay(2);
            if (sts3215_set_torque(i, 0u) == 0)
            {
                ok++;
            }
        }
    }

    sts3215_lock_ok_count = ok;

    if (enable)
    {
        /* 使能方向保持严格：只有六路全部应答才算"真的锁死了"。
         * 少一路就不翻 → 再按一次即重试；而写同样的值幂等，故能自愈。 */
        if (ok == STS3215_NUM)
        {
            sts3215_locked = 1u;
        }
    }
    else
    {
        /* 失能方向：无论成功几路，都把状态置为"已放开" —— 用户/上位机的意图优先。 */
        sts3215_locked = 0u;
    }

    return ok;
}

/* ==========================================================================================
 *                    PC 下发目标角(0x61)：单路移动（执行层原语）
 * ==========================================================================================
 * 用途：把某个关节移动到指定的【绝对角度】(主臂内部单位：度，相对上电基准)。
 *       PC 侧(上位机 / 数据采集 / 推理)按 0x61 下发弧度，本模块先转成度，
 *       再由这里的换算把"目标角度"变成"舵机原始刻度"写进 GOAL_POSITION。
 *
 * ★ 为什么不能直接把角度当刻度写
 *   STS3215 的 12-bit 编码器读到的是 raw(0~4095)，而主臂上报的 total_angle 是
 *   【相对上电基准的增量累加量】(还乘过方向 dir)，两者不是同一个坐标系。
 *   必须用"当前 raw + 需要改变的累计刻度 × dir"反推目标 raw。
 *
 * ★ 本层不做安全策略：限幅/使能/单帧路数等闸门在 usb_pc_link.c 里统一实施。
 * ========================================================================================== */

int sts3215_get_deg(uint8_t idx, float *out_deg)
{
    if (idx >= STS3215_NUM || out_deg == 0)
    {
        return -1;
    }

    /* is_received 反映"最近一次读取是否真的成功"：
     * 为 0 时 total_angle 还是旧值(可能已过期几百 ms)，当作当前角用会算错位移。 */
    if (sts3215_encoder[idx].is_received == 0)
    {
        return -1;
    }

    *out_deg = sts3215_encoder[idx].total_angle;
    return 0;
}

/* 一致性校验：encoder_value 与 (base_raw + dir*total) 的差（刻）。
 * ★ 这是"绝对角估计是否可信"的唯一客观判据 —— 见 STS_Module.h 的说明，
 *   以及 usb_pc_link.c 里"安全闸 ②.5"记录的那次 J1 冲出撞限位事故。 */
int16_t sts3215_consist_err(uint8_t idx)
{
    if (idx >= STS3215_NUM)
    {
        return (int16_t)0x7FFF;
    }

    const STS3215_Encoder_t *e = &sts3215_encoder[idx];

    if (e->is_received == 0)
    {
        return (int16_t)0x7FFF;      /* 最近一次读取不可信 */
    }

    int32_t exp = (int32_t)e->base_raw
                + (int32_t)sts3215_dir[idx] * e->total_encoder_value;
    exp &= 0xFFF;                    /* 12-bit 绕圈，归一到 0..4095 */

    int16_t d = (int16_t)((int16_t)e->encoder_value - (int16_t)exp);
    if (d > 2048)       { d = (int16_t)(d - 4096); }
    else if (d < -2048) { d = (int16_t)(d + 4096); }
    return d;
}

int sts3215_goto_deg(uint8_t idx, float target_deg, uint16_t speed, uint8_t acc)
{
    if (idx >= STS3215_NUM)
    {
        return -1;
    }

    STS3215_Encoder_t *enc = &sts3215_encoder[idx];
    const uint8_t id = motor_ids[idx];

    /* ① 重读一次原始刻度：确认"这一路此刻通信正常" —— 读不到就绝不写。
     *    （已有 enc->encoder_value 是上一次采样的值，可能已过 3ms，不适合作为写入基准。） */
    const int raw_now = ReadPos(id);
    if (raw_now < 0 || raw_now > 4095)
    {
        return -1;
    }

    /* ② 目标角度 → 目标累计刻度（四舍五入到整数刻度，1 刻度 = 0.0879°） */
    const float   target_total_f = target_deg * (4096.0f / 360.0f);
    const int32_t target_total   = (int32_t)(target_total_f + (target_total_f >= 0.0f ? 0.5f : -0.5f));

    /* ③ 需要改变的累计刻度量 */
    const int32_t delta_total = target_total - enc->total_encoder_value;

    /* ④ 硬保护：单次位移不得超过半圈。
     *    理由：sts3215_angle_get() 用 ±2048 判"过零跳变"，
     *    若这次写入让 raw 跳超过半圈，下一轮采样会把它当成绕圈修正 → 角度彻底错乱。
     *    （正常调用方会先限幅到 10° 量级，这里只是最后一道保险。） */
    if (delta_total > 2047 || delta_total < -2047)
    {
        return -1;
    }

    /* ⑤ 累计刻度 → 原始刻度增量。累加时是 total += diff_raw * dir，
     *    故 diff_raw = delta_total * dir（dir 为 ±1，乘除等价）。 */
    const int32_t raw_target = (int32_t)raw_now + delta_total * (int32_t)sts3215_dir[idx];

    /* ⑥ 12-bit 量程检查：越界说明目标角超出该关节可达范围。
     *    直接拒绝，【不做绕圈处理】—— 绕一圈会让舵机猛转 360°，风险远大于收益。 */
    if (raw_target < 0 || raw_target > 4095)
    {
        return -1;
    }

    /* ⑦ 清死区累积器：否则"被暂扣的差值"会叠加进本次位移，
     *    使上报角度滞后最多 ±8 刻度(0.7°)，导致 PC 侧的闭环判断偏一点。 */
    enc->deadband_accum = 0;

    WritePosEx(id, (int16_t)raw_target, speed, acc);
    return 0;
}
