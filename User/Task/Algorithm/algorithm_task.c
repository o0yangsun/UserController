/**
 * @file algorithm_task.c
 * @author Snow Boreas (2969345759@qq.com)
 * @brief 工程自定义机械臂控制器舵机采样模块
 * @version 0.1
 * @date 2026-07-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "algorithm_task.h"
#include <stdio.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "drv_dwt.h"
#include "robot.h"

STS3215_Encoder_t sts3215_encoder[6] = {0};

float angles[6] = {0};

/* -------------------------------- 调试监测变量 --------------------------------- */
static float algorithm_task_dt = 0;
static float algorithm_task_delta = 0;
static float algorithm_task_start_dt = 0;
static uint32_t algorithm_task_dwt = 0;
/* -------------------------------- 调试监测变量 --------------------------------- */

/* -------------------------------- 线程入口 ------------------------------- */
void AlgorithmTask_Entry(void const *argument)
{

    sts3215_setup();    //  设置小段；分配串口缓冲区

    /* -------------------------------- 外设初始化段落 ------------------------------- */
    // 先读取十次原始值在进行初始化和0点校准
    for (uint8_t i = 0; i < 10; i++)
    {
        sts3215_encoder_get();
        HAL_Delay(10); // 延时1s等待STS3215初始化完成
    }

    //  初始化STS3215的结构体配置
    //  延时10ms
    //  设定编码器2048中点值
    //  延时10ms
    //  初始化STS3215结构体并记录首次原始刻度作为零点
    for (uint8_t i = 0; i < 6; i++)
    {
        STS3215_Init(&sts3215_encoder[i]);
        HAL_Delay(10);
        if(CalibrationOfs(i+1))
        {
            sts3215_encoder[i].is_initialized = 1;
        }
        else
        {
            sts3215_encoder[i].is_initialized = -1;
        }
        HAL_Delay(10);
    }
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
        //  电机实例结构体更新
        sts3215_angle_get();

        for (int i = 0; i < 6; i++)
        {
            angles[i] = sts3215_encoder[i].total_angle;
        }

        // 将编码器数据放入队列
        xQueueSend(xQueue, angles, 0);

        /* -------------------------------- 线程代码编写段落 ------------------------------- */

        vTaskDelay(2);
    }
}
