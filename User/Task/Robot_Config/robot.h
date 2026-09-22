//
// Created by 刘嘉俊 on 25-7-16.
//

#ifndef CTRBOARD_H7_ALL_ROBOT_H
#define CTRBOARD_H7_ALL_ROBOT_H

#define CPU_FREQUENCY 550 /* CPU主频(MHz) */

#include "stm32h7xx_hal.h" // 使用的芯片
#include "FreeRTOS.h"       // 提供 FreeRTOS 基础类型
#include "queue.h"           // 提供 QueueHandle_t
#include "cmsis_os.h"       // 使用的 OS 头文件
#include "STS_Module.h"     // 提供 STS3215_NUM

/* 舵机编码器反馈帧（algorithm → usart_send 队列传递的单元） */
typedef struct
{
    float   angle[STS3215_NUM]; // 各舵机累计角度(低通滤波后)
    uint8_t valid;              // 1=本轮全部通道读到有效刻度；0=至少一路失败(帧内失败通道保留旧值)
} ServoFeedback_t;

extern QueueHandle_t xQueue;    //algorithm和usart_send的队列
void robot_init(void);

#endif // CTRBOARD_H7_ALL_ROBOT_H
