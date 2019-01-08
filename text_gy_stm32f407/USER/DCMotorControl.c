/********************************************************************************************************
*date£∫2019-1-7
*author£∫NUAA gooyle
*function:DCMotorControl()
*input:none
*output:none
*remarks£∫GPIO…œ¿≠ ‰≥ˆ≈‰÷√
********************************************************************************************************/ 
#include "DCMotorControl.h"
#include "led.h"
void DCMotorControl(uint16_t Mpwm,uint8_t Dir,uint8_t Enable)
{
	if(Enable != ENABLE)
	GPIO_ResetBits(GPIOF,GPIO_Pin_9);
	else if(Dir == PDIR)
	{
		GPIO_SetBits(GPIOF,GPIO_Pin_9);
		GPIO_SetBits(GPIOF,GPIO_Pin_10);
		TIM_SetCompare1(TIM2,Mpwm);	
	}
	else if(Dir == NDIR)
	{
		GPIO_SetBits(GPIOF,GPIO_Pin_9);
		GPIO_ResetBits(GPIOF,GPIO_Pin_10);
		TIM_SetCompare1(TIM2,Mpwm);		
	}

		
}