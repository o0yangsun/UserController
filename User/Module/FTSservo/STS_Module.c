#include "STS_Module.h"
#include "SCS.h"
#include "SCSCL.h"
#include "algorithm_task.h"
#include <stdbool.h>

static int pos = 0;

void STS3215_Init(STS3215_Encoder_t *encoder)
{
    encoder->encoder_value = 0;
    encoder->last_encoder_value = 0;
    encoder->total_encoder_value = 0;
    encoder->last_total_angle = 0.0f;
    encoder->total_angle = 0.0f;
    encoder->is_initialized = -1;
}

void sts3215_setup(void)
{
    setEnd(0); // SMS_STS舵机为小端存储结构
}

//  读取当前的刻度值并存入实例成员
void sts3215_encoder_get(void)
{

    pos = ReadPos(1);
    sts3215_encoder.encoder_value = pos; // 解码两个字节 bit15为方向位,参数=0表示无方向位
    sts3215_encoder.is_received = 1;

}

void sts3215_angle_get(void)
{
        sts3215_encoder_get();
        int16_t diff = (int16_t)(sts3215_encoder.encoder_value - (int16_t)sts3215_encoder.last_encoder_value); // 计算差值
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

        // 刻度值累加
        sts3215_encoder.total_encoder_value += diff; // 累加增量到累计值

        // 计算角度值
        sts3215_encoder.total_angle = (float)(sts3215_encoder.total_encoder_value) * (360.0f / 4096.0f);
        sts3215_encoder.total_angle = STS3215FILTER * sts3215_encoder.total_angle + (1 - STS3215FILTER) * sts3215_encoder.last_total_angle;

        //  缓存当前刻度值与角度 供下次使用
        sts3215_encoder.last_encoder_value = sts3215_encoder.encoder_value; // 保存本次值供下次 diff 使用
        sts3215_encoder.last_total_angle = sts3215_encoder.total_angle;     // 保存值供下次滤波使用
}