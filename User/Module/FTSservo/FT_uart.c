/**
 * @file  ft_uart_hw.c
 * @brief FTSservo 硬件抽象层 — USART1 实现
 */

#include "usart.h"
#include <stdint.h>

void ftUart_Send(uint8_t *nDat, int nLen)
{
    HAL_UART_Transmit(&huart1, nDat, nLen, 100);
}

int ftUart_Read(uint8_t *nDat, int nLen)
{
    if (HAL_OK != HAL_UART_Receive(&huart1, nDat, nLen, 100)) {
        return 0;
    } else {
        return nLen;
    }
}

void ftBus_Delay(void)
{
    HAL_Delay(1);
}
