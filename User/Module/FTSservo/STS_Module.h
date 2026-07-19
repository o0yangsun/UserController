#ifndef STS_MODULE_H
#define STS_MODULE_H

#include <stdint.h>
#include "SMS_STS.h"

#define STS3215FILTER 0.3f // 低通滤波系数

/* 舵机 ID 列表 (定义于 STS_Module.c) */
extern uint8_t motor_ids[6];

// 定义STS3215编码器结构体（必须在函数声明之前）
typedef struct
{

    int16_t encoder_value;       // 当前编码器的值
    int16_t last_encoder_value;  // 上一次编码器的值
    int32_t total_encoder_value; // 编码器的累计值

    float last_total_angle; // 上一次角度结果，用来参与下一次的低通滤波
    float total_angle;      // 低通滤波得到角度结果

    int is_initialized; // 初始化标志（判断是否已记录首次角度）
    int is_received;    // 作为标志位检查是否接收到数据

} STS3215_Encoder_t;

/* ---- 函数声明 ---- */
void sts3215_setup(void);
void STS3215_Init(STS3215_Encoder_t *encoder);
void sts3215_encoder_get(void);
void sts3215_angle_get(void);
#endif
