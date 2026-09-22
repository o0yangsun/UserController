/**
 * @file  ft_uart_hw.c
 * @brief FTSservo 硬件抽象层 — USART1 实现
 */

#include "usart.h"
#include "drv_dwt.h"
#include <stdint.h>

//TODO: 修改为DMA串口发送
void ftUart_Send(uint8_t *nDat, int nLen)
{
    HAL_UART_Transmit(&huart1, nDat, nLen, 100);
}

/**
 * @brief 清理接收通路残留:清错误标志 + 排空 RX FIFO
 *
 * 本 HAL 移植不会清除 ORE(溢出)标志。一旦 ORE 置位,
 * HAL_UART_Receive 内部的 UART_WaitOnFlagUntilTimeout 会立刻返回 HAL_ERROR,
 * 之后每一次读都"秒失败",表现为:上电后各舵机只更新一次数据,随后全部冻结,
 * 且不会自愈(只能断电重启)。
 *
 * 因此需要在每次总线事务发起前(rFlushSCS)以及读失败之后都调用本函数。
 */
void ftUart_FlushRx(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart1);

    // 排空 RX FIFO 中残留的字节(H7: 读 RDR 清 RXNE)
    for (uint32_t i = 0; i < 64U; i++)
    {
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET)
        {
            break;
        }
        (void)huart1.Instance->RDR;
    }

    huart1.ErrorCode = HAL_UART_ERROR_NONE;

    // 上一次异常退出若把 RxState 卡在 BUSY_RX,后续 Receive 会直接返回 BUSY
    if (huart1.RxState == HAL_UART_STATE_BUSY_RX)
    {
        huart1.RxState = HAL_UART_STATE_READY;
    }
}

int ftUart_Read(uint8_t *nDat, int nLen)
{
    // 接收超时 3ms：1Mbps 下舵机应答远快于此(几十~几百 µs)；
    // 若该路舵机缺席,超时从 10ms 降到 3ms,减少对多路轮询节拍的拖累。
    if (HAL_OK != HAL_UART_Receive(&huart1, nDat, nLen, 3)) {
        ftUart_FlushRx(); // 清掉可能导致后续永久失败的 ORE 标志
        return 0;
    } else {
        return nLen;
    }
}

void ftBus_Delay(void)
{
    // 原实现 HAL_Delay(1)：每个总线事务前固定 1ms,三路轮询合计 3ms/轮,
    // 是采样周期(实测 7ms)的最大开销。半双工总线换向只需几十 µs,
    // 且事务开始前已由 ftUart_FlushRx() 排空残留,故改为 50µs 精确延时。
    dwt_delay_us(50);
}
