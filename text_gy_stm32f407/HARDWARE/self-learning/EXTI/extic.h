#ifndef __EXTIC_H
#define __EXTIC_H
#include "sys.h"


void EXTIC_Init(void);
void EXTI0_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI3_IRQHandler(void);


#endif