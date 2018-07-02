#include "extic.h"
#include "led.h"
#include "delay.h"
#include "key.h"

/********************************************************************************************************
*date：2018-5-13
*author：NUAA gooyle
*function:EXTIC_Init()
*input:none
*output:none
*remarks：外部中断设置
********************************************************************************************************/ 
void EXTIC_Init(void)
{
	
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	KEY_Init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource4);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource3);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_LineCmd =ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line3|EXTI_Line4;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
	
	//PA0
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0X00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0X02;
	NVIC_Init(&NVIC_InitStructure);
	//pe3
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0X02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0X02;
	NVIC_Init(&NVIC_InitStructure);

	//PE4
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_Init(&NVIC_InitStructure);

}

/********************************************************************************************************
*date：2018-5-13
*author：NUAA gooyle
*function:EXTI0_IRQHandler()
					EXTI4_IRQHandler()
					EXTI3_IRQHandler()
*input:none
*output:none
*remarks：中断服务程序
********************************************************************************************************/ 

void EXTI0_IRQHandler(void)
{
	delay_ms(10);
	if(EXTI_GetITStatus(EXTI_Line0)!= RESET)
	{
		if(WK_UP == 1)
		LED0 = !LED0;	
	}
	EXTI_ClearITPendingBit(EXTI_Line0);	
}

void EXTI3_IRQHandler(void)
{
	delay_ms(10);
	if(EXTI_GetITStatus(EXTI_Line3)!= RESET)
	{
		if(KEY_0 == SET)
		LED1 = !LED1;	
	}
		EXTI_ClearITPendingBit(EXTI_Line3);
}

void EXTI4_IRQHandler(void)
{
	delay_ms(10);
	if(EXTI_GetITStatus(EXTI_Line4)!= RESET)
	{
		if(KEY_1 == SET)
		{
			LED0 = !LED0;	
			LED1 = !LED1;
		}
		
	}
	EXTI_ClearITPendingBit(EXTI_Line4);

}	