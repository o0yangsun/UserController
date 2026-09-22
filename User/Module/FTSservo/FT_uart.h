#ifndef FT_UART_H
#define FT_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef enum
{
    FT_UART_TRANSACTION_IDLE = 0,
    FT_UART_TRANSACTION_BUSY,
    FT_UART_TRANSACTION_COMPLETE,
    FT_UART_TRANSACTION_ERROR
} FT_UartTransactionState_t;

void ftUart_Init(void);
void ftUart_FlushRx(void);
bool ftUart_StartTransaction(const uint8_t *tx_data,
                             uint16_t tx_length,
                             uint8_t *rx_data,
                             uint16_t rx_length);
FT_UartTransactionState_t ftUart_GetTransactionState(void);
void ftUart_ClearTransaction(void);
void ftUart_CancelTransaction(void);
void ftUart_TxCpltCallback(UART_HandleTypeDef *huart);

#endif
