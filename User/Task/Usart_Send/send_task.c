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

static float angles_send = 0.0f;                // 用于临时存储队列中读取的编码器值
static float encoder_values = 0.0f; // 存储6个编码器值

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

    // 数据区：6个float（每个编码器值4字节）
    // 设置数据段

        uint8_t *src = (uint8_t *)values;
        uint8_t *dst = &tx_data->data[4];
        memcpy(dst, src, sizeof(float)); // 自动处理4个字节
    

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

        // 从队列中获取编码器值
        if (xQueueReceive(xQueue, &angles_send, 0) == pdTRUE)
        {
            // 更新全局的 encoder_values 数组（可选）
            
                encoder_values = angles_send;
            

            // 打包数据到tx_data结构中
            RobotArmController_t tx_data = {0};     // 定义数据包结构体
            PackData(&encoder_values, 30, &tx_data); // 打包数据帧
            // 将打包后的数据写入DMA缓冲区
            memcpy(dma_tx_buffer[current_buffer], &tx_data, FRAME_SIZE);
            // 启动 DMA 发送
            DMA_Send_Frame();

        }

        /* -------------------------------- 线程代码编写段落 ------------------------------- */

        vTaskDelay(4);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */
