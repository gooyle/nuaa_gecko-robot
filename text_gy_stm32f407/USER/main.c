/********************************************************************************************************
*date：2018-6-19
*author：NUAA google
*function:gecko motion control
*input:none
*output:none
*remarks：none
********************************************************************************************************/ 
#include "rcc_config.h"
#include "pwm_init.h"
#include "usart_gy.h"
#include "delay.h"
//#include "stdio.h"

int main(void)
{
	/**********************----初始化函数-------*****************************/
	RCC_Config();//程序第一步：时钟树配置
	uart_init(115200);//波特率配置；
	PWMConfig(10000-1,168-1);//84MHZ/(10000*168) = 50HZ
	PWM_Enable();//pwm使能
	delay_init(168);//延时函数初始化配置，系统时钟为168Mhz
	
	/***********************----流程函数-----***********************************/
	
	while(1)
	{
		Angle(60,LR_J1);
		delay_ms(500);
		Angle(-60,LR_J10);
		delay_ms(500);
	}
	return 0;
}