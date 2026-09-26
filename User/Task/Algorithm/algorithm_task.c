/**
 * @file algorithm_task.c
 * @author Snow Boreas (2969345759@qq.com)
 * @brief 工程自定义机械臂控制器舵机采样模块
 * @version 0.3
 * @date 2026-09-14
 *
 * @copyright Copyright (c) 2026
 *
 * @note v0.3 六舵机适配：STS3215_NUM=6，逐路校准/采样，六路全部有效才组帧入队
 *       (保持"失联不发疯臂"防护)。每路是否用中位校准由 sts3215_use_mid_cal[] 配置。
 *       v0.2 三舵机适配：单实例 → sts3215_encoder[STS3215_NUM] 数组。
 */

#include "algorithm_task.h"
#include <stdio.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "drv_dwt.h"
#include "robot.h"
#include "usb_pc_link.h"   /* PC ↔ 主臂 USB CDC 链路 */

/* 舵机编码器实例数组：每路保存独立的累计值与滤波状态 */
STS3215_Encoder_t sts3215_encoder[STS3215_NUM];

/* 调试监测：最近一次成功入队的反馈帧(供 GDB/上位机观察) */
ServoFeedback_t servo_feedback;

/* -------------------------------- 抖动诊断(调试期用,定死区阈值) ---------------------------------
 * 作用：以任务节拍(3ms)记录每路 raw 的最小/最大，解决"用调试器 halt 采样(约 2s 一轮)
 *       抓不到瞬时抖动"的问题。
 * 用法：
 *   ① 在 GDB 里调用 dbg_jitter_start()  → 清零统计并开始记录
 *   ② 用手轻碰 / 轻转舵机，维持几秒
 *   ③ 读 dbg_raw_min[i] 与 dbg_raw_max[i]，极差即抖动幅度
 *      单位是刻度(4096 刻度 = 360°，1 刻度 = 0.0879°)
 *   ④ 置 dbg_jitter_enable = 0 停止记录
 * 注意：统计只累加"读到有效值"的样本，丢包不会污染 min/max。                       */
volatile uint8_t dbg_jitter_enable = 0;
volatile int16_t dbg_raw_min[STS3215_NUM];
volatile int16_t dbg_raw_max[STS3215_NUM];

__attribute__((used)) void dbg_jitter_start(void)
{
    for (uint8_t i = 0; i < STS3215_NUM; i++)
    {
        dbg_raw_min[i] = 32767;
        dbg_raw_max[i] = -32768;
    }
    dbg_jitter_enable = 1;
}

/* 每路上电校准方式(按需调整，索引 = motor_ids 下标 = 数据帧通道号)
 *   1 = 中位校准:把舵机【自身零点】移到量程中点(写扭矩使能寄存器 40=128) → 上电 raw≈2048,
 *       两个方向各留约 2048 刻度余量(适合上电位姿靠近行程端点的关节);
 *   0 = 零点校准:把当前位姿设为舵机零点 → 上电 raw≈0(原行为)。
 * 注意:这张表只管"舵机物理零点"，【不影响对外输出的角度基准】——
 *       输出基准已统一为 0°(见下方 base=0)，既与 F411/车端对齐，又保证双向都有余量。 */
static const uint8_t sts3215_use_mid_cal[STS3215_NUM] = {1, 1, 1, 1, 1, 1}; // 六路物理零点全部居中

/* -------------------------------- 调试监测变量 --------------------------------- */
static float algorithm_task_dt = 0;
static float algorithm_task_delta = 0;
static float algorithm_task_start_dt = 0;
static uint32_t algorithm_task_dwt = 0;
/* -------------------------------- 调试监测变量 --------------------------------- */

/* -------------------------------- 线程入口 ------------------------------- */
void AlgorithmTask_Entry(void const *argument)
{

    sts3215_setup(); //  设置小段；分配串口缓冲区

    /* -------------------------------- 外设初始化段落 ------------------------------- */
    // 等待舵机上电就绪：用【纯延时】，不要在这里读编码器。
    // 原因：读成功会把 is_received 置 1，从而绕过 sts3215_angle_get() 的
    //       "首次成功读只同步基准、不产生增量"保护 —— 那道保护正是防止
    //       上电/复位后把 raw 本身当成转动量累加的（实测出现过 ±2048、±85 刻度）。
    HAL_Delay(300);

    //  逐路初始化STS3215的结构体配置并记录首次原始刻度作为零点
    for (uint8_t idx = 0; idx < STS3215_NUM; idx++)
    {
        // 上电基准/校准方式由 sts3215_use_mid_cal[] 决定:
        //   中位校准 → 基准 2048(上电显示 180°);零点校准 → 基准 0(上电显示 0°)。
        const uint8_t use_mid = sts3215_use_mid_cal[idx];

        // 输出基准统一为 0°(与 F411(AS5600)/车端 0x0302 解析保持一致)。
        // 重要:base 只是"输出零点"，与舵机自身的物理零点【解耦】——
        //       use_mid=1 时仍会执行中位校准(舵机 raw≈2048)，两个方向各留约 ±2048 刻度余量，
        //       所以 base=0 不会造成"往反方向转不累计"的问题。
        int32_t base = 0;
        STS3215_Init(&sts3215_encoder[idx], base);
        HAL_Delay(10);

        int cal_ok = use_mid ? sts3215_calib_mid(motor_ids[idx])
                             : Calibration(motor_ids[idx]);
        HAL_Delay(10);

        // 注意：这里【不再补读】。
        // 补读会在"中位校准尚未生效"时把一个过期 raw 写进 last_encoder_value，
        // 等校准生效后主循环首帧就把二者差值累加进去（实测出现过 -85 刻度）。
        // 基准改由 sts3215_angle_get() 的"首次成功读只同步、不累加"自动建立。

        if (cal_ok)
        {
            sts3215_encoder[idx].is_initialized = 1;
        }
        else
        {
            // 该路校准失败：is_initialized=-1，主循环内该路读不到数据时不发帧，避免疯臂
            sts3215_encoder[idx].is_initialized = -1;
        }
        HAL_Delay(10);
    }

    // 统一再等一段时间，确保六路的中位校准都已生效后再进入主循环，
    // 避免首帧读到"校准前"的刻度而使基准出现偏差。
    HAL_Delay(100);
    /* -------------------------------- 外设初始化段落 ------------------------------- */

    /* -------------------------------- 调试监测线程调度 --------------------------------- */
    algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
    algorithm_task_start_dt = dwt_get_time_ms();
    /* -------------------------------- 调试监测线程调度 --------------------------------- */
    for (;;)
    {
        /* -------------------------------- 调试监测线程调度 --------------------------------- */
        algorithm_task_delta = dwt_get_time_ms() - algorithm_task_start_dt;
        algorithm_task_start_dt = dwt_get_time_ms();
        algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
        /* -------------------------------- 调试监测线程调度 --------------------------------- */

        /* -------------------------------- 线程代码编写段落 ------------------------------- */
        ServoFeedback_t fb = {0};
        uint8_t all_valid = 1;

        // 逐路采样：读失败(未接舵机/握手失败/通信中断)时 angle_get 内部已跳过累积，
        // total_angle 保持上一帧值，不会出现"疯臂"乱跳。
        for (uint8_t idx = 0; idx < STS3215_NUM; idx++)
        {
            sts3215_angle_get(idx);

            // 抖动诊断：记录每路 raw 的历史最小/最大(由 dbg_jitter_start() 启动，
            // 见文件上方说明；只在读到有效值时统计，丢包不污染 min/max)
            if (dbg_jitter_enable && sts3215_encoder[idx].is_received)
            {
                const int16_t v = sts3215_encoder[idx].encoder_value;
                if (v < dbg_raw_min[idx]) dbg_raw_min[idx] = v;
                if (v > dbg_raw_max[idx]) dbg_raw_max[idx] = v;
            }

            if (sts3215_encoder[idx].is_received)
            {
                sts3215_encoder[idx].is_initialized = 1; // 通信恢复 → 标记可用(自愈)
                fb.angle[idx] = sts3215_encoder[idx].total_angle;
            }
            else
            {
                fb.angle[idx] = sts3215_encoder[idx].total_angle; // 失败通道保留旧值(不更新)
                all_valid = 0;
            }
        }

        // 通信健康闸门：六路全部有效才组帧下发；
        // 任一路失联 → 整帧丢弃，上位机收不到新帧即知异常(沿用"失联不发帧"语义)。
        if (all_valid)
        {
            fb.valid = 1;
            servo_feedback = fb; // 调试镜像
            xQueueSend(xQueue, &fb, 0);
        }

        /* ------------------------------ USB CDC：PC 链路 ------------------------------
         * usb_pc_service() 每轮处理两件事（内部已限速，见 usb_pc_link.c）：
         *   ① 轮询 PC 下发的命令（0x61 目标角 / 0x62 使能）
         *   ② 按 usb_pc_report_ms 周期上报 0x60（6 关节角 + 使能状态，单位弧度）
         * 放在【组帧入队】之后：不影响 0x0302 的节拍。
         *
         * 0x62 是 PC 请求使能/失能。这里复用按键那条通路（sts3215_set_torque_all），
         * 保证"扭矩状态只有一个权威来源"，也免得两套逻辑互相打架。
         * ⚠️ 0x61 的目标角只解析并存入 usb_pc_target_deg[]/usb_pc_target_flag，
         *    【不驱动舵机】—— 主臂当前只读不写，写目标的应用逻辑尚未开发。 */
        if (usb_pc_enable_req >= 0)
        {
            const uint8_t want = (uint8_t)usb_pc_enable_req;
            usb_pc_enable_req  = -1;              /* 消费掉请求 */
            if (want != sts3215_locked)
            {
                (void)sts3215_set_torque_all(want);
            }
        }
        usb_pc_service();

        /* ------------------------------ KEY 按键(PA15)：锁死 / 自由 ------------------------------
         * 按一次：六路舵机扭矩使能(锁死)。此时操作手被刚性固定 → 角度随之冻结 →
         *         车端机械臂停在同位置(0x0302 照常发，只是值不变)。
         * 再按一次：失能，回到上电默认的自由态，车端继续跟随操作手运动。
         *
         * 为什么不放在中断里：舵机总线是阻塞式 HAL(ftUart_Read 最坏阻塞 3ms)，
         * 一次切换 ≈ 18 次总线事务、耗时 15~40ms；在 ISR 里做会严重拖垮系统。
         * 本循环已有约 3ms 节拍，轮询代价可忽略（这个停顿是一次性的，可以接受）。
         *
         * 注意：这一句必须放在 [组帧入队] 之后 —— 让锁死/解锁前的最后一帧先发出去，
         *       避免扭矩切换那几十毫秒的总线占用拖后腿。 */
        if (sts3215_key_poll())
        {
            (void)sts3215_set_torque_all((uint8_t)(!sts3215_locked));
        }

        /* -------------------------------- 线程代码编写段落 ------------------------------- */
        //TODO: 测试最大发送频率
        vTaskDelay(2);
    }
}
