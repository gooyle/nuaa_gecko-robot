#include "timer.h"
#include "led.h"
#include "delay.h"

/********************************************************************************************************
*date：2018-5-14
*author：NUAA gooyle
*function:TIM3_Init()
*input:uint16_t arr,uint16_t psc
*output:none
*remarks：TIM3定时器定时设置
********************************************************************************************************/ 
void TIM3_Init(uint16_t arr,uint16_t psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = arr;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
	//TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);//定时器更新触发
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM3,ENABLE);
}

/********************************************************************************************************
*date：2018-5-14
*author：NUAA gooyle
*function:TIM3_IRQHandler()
*input:none
*output:none
*remarks：TIM3定时器中断服务程序
********************************************************************************************************/
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!= RESET)//溢出中断
	{
		LED0 = !LED0;
		//delay_ms(200);
	}
		
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}