#ifndef __PWM_INIT_H
#define __PWM_INIT_H
#include "sys.h"
void PWMConfig(uint32_t arr,uint32_t psc);
void PWM_Enable(void);
void Angle(int32_t angle,int8_t footnumber);
#define LF_J1 0
#define LR_J1 1
#define RF_J1 2
#define RR_J1 3
#define LF_J2 4
#define LR_J2 5
#define RF_J2 6
#define RR_J2 7
#define LF_J3 8
#define LR_J3 9
#define RF_J3 10
#define RR_J3 11
#define ARM_J1 12
#define ARM_J2 13
#define ARM_J3 14
#define WAIST  15 
#endif
