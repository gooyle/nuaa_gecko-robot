#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"

void TIM3_Init(uint16_t arr,uint16_t psc);
void TIM3_IRQHandler(void);
#endif