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
	PWMConfig(10000-1,168-1);//84MHZ/(10000*168) = 50HZ
	PWM_Enable();
	
	//while(1)
	{
		Angle(0,LF_J1);
		Angle(0,ARM_J2);
	}
	while(1);
	return 0;
}