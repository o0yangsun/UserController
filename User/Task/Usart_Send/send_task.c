/**
 ******************************************************************************
 * @file    algorithm_task.c
 * @author  Liu JiaJun(187353224@qq.com)
 * @version V1.0.0
 * @date    2025-01-10
 * @brief   机器人算法任务线程，处理复杂算法，避免在其他线程中计算造成阻塞
 ******************************************************************************
 * @attention
 *
 * 本代码遵循GPLv3开源协议，仅供学习交流使用
 * 未经许可不得用于商业用途
 *
 ******************************************************************************
 */

#include "send_task.h"
#include "cmsis_os.h"
#include "drv_dwt.h"
#include <string.h>
#include <stdio.h>
#include "crc8_crc16.h"
#include "usart.h"
#include "robot.h"
#include "crc8_crc16.h"

/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t send_task_dwt = 0;   // 毫秒监测
static float send_task_dt = 0;       // 线程实际运行时间dt
static float send_task_delta = 0;    // 监测线程运行时间
static float send_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

static uint8_t dma_tx_buffer[2][FRAME_SIZE]; // 双缓冲区
static volatile uint8_t current_buffer = 0;  // 当前缓冲区索引
volatile uint8_t dma_busy = 0;                        // DMA状态标志位

/* 0x0302 帧发送周期(ms) —— 决定发送帧率；用 volatile 修饰，
 * 可直接用调试器(OctoLink/GDB)在线改写测试，无需重新烧录。
 *
 * 背景：老控制器(F411 + 6×AS5600 软件 IIC)实测 ≈29.4 fps(周期约 34ms) ——
 *       那个频率并不是"设定值"，而是被软件 IIC 读 6 路磁编码器的耗时自然限制出来的
 *       (两边的主循环结构完全相同：队列有数据就发 + vTaskDelay)。
 *       本控制器读舵机只需约 3ms，所以必须【显式限速】才能对齐老控制器的节拍。
 *
 * 换算：实际帧率 ≈ 1000 / (send_period_ms + 发送耗时)，39B@115200 ≈ 3.4ms
 *   send_period_ms = 30 → ≈30 fps  (对齐老控制器)
 *   send_period_ms = 17 → ≈50 fps
 *   send_period_ms = 7  → ≈100 fps
 *   send_period_ms = 1  → 约 200 fps 上限(原默认 4ms 时实测 223 fps)
 * ⚠️ 不要设 0：vTaskDelay(0) 不阻塞，会饿死其他任务。 */
volatile uint16_t send_period_ms = 30;

static ServoFeedback_t rx_fb;   // 从队列取出的舵机反馈帧
static float encoder_values[STS3215_NUM]; // 打包用的角度数组镜像

extern QueueHandle_t xQueue; // FreeRTOS 队列句柄

void PackData(float *values, uint16_t data_length, RobotArmController_t *tx_data)
{
    static uint32_t frame_seq = 0; // 帧序号
    // 设置帧头
    tx_data->frame_header.sof = 0xA5;                // 起始字节
    tx_data->frame_header.data_length = data_length; // 数据段长度
    tx_data->frame_header.seq = frame_seq++;         // 包序号
    if (frame_seq == 0)
        frame_seq = 1; // 防止包序号溢出，最多支持255个包序号

    // 计算帧头CRC8
    tx_data->frame_header.crc8 = 0;                                                  // 初始CRC8
    append_CRC8_check_sum((uint8_t *)(&tx_data->frame_header), FRAME_HEADER_LENGTH); // 添加CRC8校验

    // 设置命令码
    tx_data->cmd_id = ARM_CONTROLLER_CMD_ID;

    // 数据区：与 F411(自定义控制器)保持一致 —— 6 个 float 从 data[0] 起连续存放(24B)，
    // 其余字节补零。车端解析见 Engineering_Robot_H723_/referee_system.c：
    //   custom_robot_data.data[i * 4]  →  i = 0..5  即 data[0..23]
    uint8_t *src = (uint8_t *)values;
    uint8_t *dst = &tx_data->data[0];
    memcpy(dst, src, STS3215_NUM * sizeof(float));

    // 计算帧尾CRC16
    tx_data->frame_tail = 0;                                // 初始CRC16
    append_CRC16_check_sum((uint8_t *)tx_data, FRAME_SIZE); // 添加CRC16校验
}

void DMA_Send_Frame(void)
{
    if (dma_busy == 0)
    {                 // 判断DMA是否空闲
        dma_busy = 1; // 标志位置为忙
        // 启动DMA发送
        HAL_UART_Transmit_DMA(&huart10, dma_tx_buffer[current_buffer], FRAME_SIZE);
        // 切换到另一个缓冲区
        current_buffer = (current_buffer == 0) ? 1 : 0;
    }
}



// DMA发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART10) {
       dma_busy = 0; // 标志位置为空闲
   }
}

/* -------------------------------- 线程入口 ------------------------------- */
void SendTask_Entry(void const *argument)
{

   /* -------------------------------- 线程间Topics初始化 ------------------------------- */
    /* -------------------------------- 调试监测线程调度 --------------------------------- */
    send_task_dt = dwt_get_delta(&send_task_dwt);
    send_task_start_dt = dwt_get_time_ms();
    /* -------------------------------- 调试监测线程调度 --------------------------------- */
    for (;;)
    {
        /* -------------------------------- 调试监测线程调度 --------------------------------- */
        send_task_delta = dwt_get_time_ms() - send_task_start_dt;
        send_task_start_dt = dwt_get_time_ms();
        send_task_dt = dwt_get_delta(&send_task_dwt);
        /* -------------------------------- 调试监测线程调度 --------------------------------- */

        /* -------------------------------- 线程代码编写段落 ------------------------------- */

        // 从队列中获取编码器反馈帧
        // ★ 只取最新一帧：把队列里积压的帧全部取出、只保留最后一帧。
        //   必要性：算法任务约每 3ms 入队一次，而本任务按 send_period_ms 限速发送；
        //   若不取空，队列(长度10)很快填满，xQueueSend 会丢弃【最新】帧 ——
        //   结果发出去的反而是队列里最旧的数据，凭空引入 10×3ms≈30ms 的延迟。
        {
            ServoFeedback_t tmp;
            uint8_t got_new = 0;
            while (xQueueReceive(xQueue, &tmp, 0) == pdTRUE)
            {
                rx_fb = tmp;
                got_new = 1;
            }

            if (got_new)
            {
                // 更新打包镜像(仅拷贝角度部分)
                for (uint8_t i = 0; i < STS3215_NUM; i++)
                {
                    encoder_values[i] = rx_fb.angle[i];
                }

                // 打包数据到tx_data结构中
                RobotArmController_t tx_data = {0};     // 定义数据包结构体
                PackData(encoder_values, DATA_LENGTH, &tx_data); // 打包数据帧
                // 将打包后的数据写入DMA缓冲区
                memcpy(dma_tx_buffer[current_buffer], &tx_data, FRAME_SIZE);
                // 启动 DMA 发送
                DMA_Send_Frame();
            }
        }

        /* -------------------------------- 线程代码编写段落 ------------------------------- */

        // 限速：帧率 ≈ 1000 / (send_period_ms + 3.4ms)。见变量定义处说明。
        vTaskDelay(send_period_ms ? send_period_ms : 1);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */
