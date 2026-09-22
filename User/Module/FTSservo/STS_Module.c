#include "STS_Module.h"
#include "SCS.h"
#include "SCSCL.h"
#include "INST.h"
#include "algorithm_task.h"
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
 *   J4 一致(保持 +1) │ J5 需取反(-1) │ J6 需取反(-1)
 * 比对原始数据：老侧首段符号为 J1− J2− J3+ J4− J5+ J6+；
 *               新侧首段符号为 J1− J2+ J3+ J4− J5− J6−（J3 已含取反）。
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

    // 读失败(舵机未接/握手失败/通信中断)时：
    // 不执行差分累积与 last 更新，保证 total_encoder_value / total_angle 不被垃圾增量污染
    const int was_received = enc->is_received; // 记录本次读取【之前】的状态
    if (sts3215_encoder_get(idx) != 0)
        return;

    // 首次成功读到该路(含上电后的第一帧、以及失联后恢复的第一帧)：
    // 只把当前刻度同步为差分的基准，不产生增量。
    // 必要性：若 last_encoder_value 仍是初值 0 而 raw 已是 2048(归中后)，
    //         会把整个归中偏移当成"转动量"累加进去 → total_angle 凭空多 180°。
    //         之前只靠"校准后补读一次"来初始化 last，一旦那次读失败就会出现该问题。
    if (!was_received)
    {
        enc->last_encoder_value = enc->encoder_value;
        enc->deadband_accum = 0; // 基准重建：丢弃此前积攒的差值(失联期间的位移不可信)
        enc->base_ready = 0;     // 重新进入"待确认"状态，需连续两次一致才认可
        return;
    }

    // 上电(或失联恢复)后的基准确认期：只同步、不累加。
    // —— 消除"中位校准生效时机 vs 固件首次读取"的时序竞争：
    //    校准一生效 raw 会一次性跳到 2048(幅度可达 ±2048 刻度 = ±180°)，
    //    若基准是用校准前的 raw 建立的，这一跳就会被当成真实转动累加进去，
    //    表现为"上电后某几路一上来就是 ±180°"(已在实机上复现)。
    // 判据：相邻两次采样差值 ≤ STS3215_BASE_TOL 才连续计数，累计 2 次即认可基准；
    //       期间任何一次跳变都清零计数并只做同步 —— 跳变不会被累加，慢速小位移(<容差)也几乎无损。
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
