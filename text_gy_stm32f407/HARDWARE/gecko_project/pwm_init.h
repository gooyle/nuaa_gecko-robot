#ifndef __PWM_INIT_H
#define __PWM_INIT_H
#include "sys.h"
void PWMConfig(uint32_t arr,uint32_t psc);
void PWM_Enable(void);
void Angle(int32_t angle,int8_t footnumber);
/*-----------------------------------------------------------------------*
*@LF_J1-T2CH1:PA15 @LR_J1-T2CH2:PB3  @RF_J1-T2CH3:PB10 @RR_J1-T2CH4:PB11
*@LF_J2-T3CH1:PC6  @LR_J2-T3CH2:PC7  @RF_J2-T3CH3:PB0  @RR_J2-T3CH4:PB1
*@LF_J3-T4CH1:PD12 @LR_J3-T4CH2:PD13 @RF_J3-T4CH3:PD14 @RR_J3-T4CH4:PD15
*@ARM_J1-T5CH1:PA0 @ARM_J2-T5CH2:PA1 @ARM_J3-T5CH3:PA2 @WAIST-T5CH4:PA3
*------------------------------------------------------------------------*/
#define RF_J1 0
#define LF_J1 1
#define RR_J1 2
#define LR_J1 3

#define RF_J2 4
#define LF_J2 5
#define RR_J2 6
#define LR_J2 7

#define RF_J3 8
#define LF_J3 9
#define RR_J3 10
#define LR_J3 11

#define ARM_J1 12
#define ARM_J2 13
#define ARM_J3 14
#define WAIST  15 
#endif
