//
// Created by 杜尚泽 on 25-7-16.
//

#include "robot.h"
#include "drv_dwt.h"
#include "main.h"

TIM_HandleTypeDef htim2;

QueueHandle_t xQueue = NULL;

#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(ServoFeedback_t)

void robot_init(void)
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用 dwt 进行延时
    //    __disable_irq();
    xQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if (xQueue == NULL)
    {
        Error_Handler();
    }

    dwt_init(); // 初始化 DWT 计时器

    HAL_TIM_Base_Start(&htim2);
    // 初始化完成,开启中断
    //    __enable_irq();
}