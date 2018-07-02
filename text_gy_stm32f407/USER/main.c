/********************************************************************************************************
*date£º2018-6-19
*author£ºNUAA google
*function:gecko motion control
*input:none
*output:none
*remarks£ºnone
********************************************************************************************************/ 
#include "rcc_config.h"
#include "pwm_init.h"

int main(void)
{
	int inc = 0;
	RCC_Config();
	PWMConfig(200-1,8400-1);//84MHZ/(8400*200) = 50HZ
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	
	while(1)
	{
		if(inc <= 150)
		{
			inc++;
			TIM_SetCompare1(TIM2,inc);
			TIM_SetCompare2(TIM2,inc);
			TIM_SetCompare3(TIM2,inc);
			TIM_SetCompare4(TIM2,inc);
			
			TIM_SetCompare1(TIM3,inc);
			TIM_SetCompare2(TIM3,inc);
			TIM_SetCompare3(TIM3,inc);
			TIM_SetCompare4(TIM3,inc);
			
			TIM_SetCompare1(TIM4,inc);
			TIM_SetCompare2(TIM4,inc);
			TIM_SetCompare3(TIM4,inc);
			TIM_SetCompare4(TIM4 ,inc);
		}
		else inc =0;
		

	}
	//while(1);
	return 0;
}