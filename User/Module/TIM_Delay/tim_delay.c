//
// Created by Áõ¼Î¿¡ on 25-7-15.
//

#include "tim_delay.h"
#include "main.h"
#include "cmsis_os.h"


void delayUs(uint32_t us) {
   // taskENTER_CRITICAL();
    uint32_t start = TIM2->CNT;
    uint32_t elapsed;
    do {
        uint32_t current = TIM2->CNT;
        elapsed = (current >= start) ? (current - start) : (0xFFFFFFFF - start + current);
    } while (elapsed < us);
  // taskEXIT_CRITICAL();
}

void delay_us_safe(uint32_t us) {
    taskENTER_CRITICAL();
    uint32_t start = TIM2->CNT;
    while ((TIM2->CNT - start) < us);
    taskEXIT_CRITICAL();
}
