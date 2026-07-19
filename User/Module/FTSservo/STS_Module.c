#include "STS_Module.h"
#include "SCS.h"
#include "algorithm_task.h"
#include <stdbool.h>

uint8_t motor_ids[] = {1, 2, 3, 4, 5, 6};
static uint8_t rx_packet[2];


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
    setEnd(0);                                   // SMS_STS舵机为小端存储结构
    syncReadBegin(sizeof(motor_ids), sizeof(rx_packet)); // 分配串口缓冲区
}

//  读取当前的刻度值并存入实例成员
void sts3215_encoder_get(void)
{
    uint8_t rxPacket[2];

    syncReadPacketTx(motor_ids, sizeof(motor_ids), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket)); // 同步读指令包发送

    for (uint8_t i = 0; i < sizeof(motor_ids); i++)
    {
        // 接收ID[i]同步读返回包
        if (!syncReadPacketRx(motor_ids[i], rxPacket))
        {
            sts3215_encoder[i].is_received = -1; // 后续可以通过Linkscope读取该标志位
            continue;                            // 接收解码失败
        }
        sts3215_encoder[i].encoder_value = syncReadRxPacketToWrod(15); // 解码两个字节 bit15为方向位,参数=0表示无方向位
        sts3215_encoder[i].is_received = 1;
    }
}

void sts3215_angle_get(void)
{
    sts3215_encoder_get(); // 一次性得到六组数据
    for (int i = 0; i < 6; i++)
    {
        int16_t diff = (int16_t)(sts3215_encoder[i].encoder_value - (int16_t)sts3215_encoder[i].last_encoder_value); // 计算差值
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
        sts3215_encoder[i].total_encoder_value += diff; // 累加增量到累计值

        // 计算角度值
        sts3215_encoder[i].total_angle = (float)(sts3215_encoder[i].total_encoder_value) * (360.0f / 4096.0f);
        sts3215_encoder[i].total_angle = STS3215FILTER * sts3215_encoder[i].total_angle + (1 - STS3215FILTER) * sts3215_encoder[i].last_total_angle;

        //  缓存当前刻度值与角度 供下次使用
        sts3215_encoder[i].last_encoder_value = sts3215_encoder[i].encoder_value; // 保存本次值供下次 diff 使用
        sts3215_encoder[i].last_total_angle = sts3215_encoder[i].total_angle;     // 保存值供下次滤波使用
    }
}