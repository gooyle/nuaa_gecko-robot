/********************************************************************************************************
*date：2019-1-7
*author：NUAA gooyle
*function:DCMotorControl()
*input:none
*output:none
*remarks：@PF9-----BRAKE-PIN:START:置1；STOP：置0
*					@PA15----PWM:建议20K-30K
*					@PF10----dir 
********************************************************************************************************/ 
#include "DCMotorControl.h"
#include "DCMotorInit.h"
extern StepperMotor TTMotor;

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

/********************************************************************************************************
*date：2019-1-7
*author：NUAA gooyle
*function:StepperMotorControl()
*input:none
*output:none
*remarks：
********************************************************************************************************/ 
void StepperMotorControl(uint32_t Pluse,uint8_t Dir,uint8_t Enable)
{
	if(TTMotor.PluseFinished == 1)
	{
		TTMotor.PluseNumber = Pluse;
		if(Enable != ENABLE)
		{
			TIM_CCxCmd(TIM2,TIM_Channel_1, TIM_CCx_Disable);
			GPIO_ResetBits(GPIOF,GPIO_Pin_9);
		}
		else if(Dir == PDIR)
		{
			GPIO_SetBits(GPIOF,GPIO_Pin_9);
			GPIO_SetBits(GPIOF,GPIO_Pin_10);
			TIM_CCxCmd(TIM2,TIM_Channel_1, TIM_CCx_Enable);
			TIM_Cmd(TIM2, ENABLE);
			TTMotor.PluseFinished = 0;
		}
		else if(Dir == NDIR)
		{
			GPIO_SetBits(GPIOF,GPIO_Pin_9);
			GPIO_ResetBits(GPIOF,GPIO_Pin_10);
			TIM_CCxCmd(TIM2,TIM_Channel_1, TIM_CCx_Enable);
			TIM_Cmd(TIM2, ENABLE);
			TTMotor.PluseFinished = 0;
		}

	}
	else if(TTMotor.PluseNumber == 0)//这部分是考虑所有脉冲发送完成后，关闭时钟
	{
		TIM_Cmd(TIM2, DISABLE);
	}
}
