#ifndef __LED_H
#define __LED_H
#include "sys.h"
/****步进电机结构体定义*****/
typedef struct
{
	int32_t PluseNumber;
	int8_t PluseFinished;
}StepperMotor;
/********/
void DCMotorPin_Init(void);
void TTMotorState_Init(void);
void StepperMotor_Init(void);
#endif
