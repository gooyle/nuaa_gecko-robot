/********************************************************************************************************
*date：2018-6-22
*author：NUAA google
*function:pwm configuration
*input:uint32_t arr,uint32_t psc
*output:none
*remarks：Fout = Fclk/(arr * psc)
*the Fclk here is 84Mhz
********************************************************************************************************/ 
#include "stm32f4xx_tim.h"
#include "pwm_init.h"
void PWMConfig(uint32_t arr,uint32_t psc)
{
	/*-----------------------寄存器结构体定义-------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	/*------------------------------打开对应时钟----------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|
													RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4,ENABLE);  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|
													RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE); 
	/*-----------------------------------------------------------------------*
	*@LF_J1_T2CH1:PA15 @LR_J1_T2CH2:PB3  @RF_J1_T2CH3:PB10 @RR_J1_T2CH4:PB11
	*@LF_J2_T3CH1:PC6  @LR_J2_T3CH2:PC7  @RF_J2_T3CH3:PB0  @RR_J2_T3CH4:PB1
	*@LF_J3_T4CH1:PD12 @LR_J3_T4CH2:PD13 @RF_J3_T4CH3:PD14 @RR_J3_T4CH4:PD15
	*
	*------------------------------------------------------------------------*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);//TIM2复用配置
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);//TIM2复用配置
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);//TIM2复用配置
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);//TIM2复用配置
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);//TIM3复用配置
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);//TIM3复用配置
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);//TIM3复用配置
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);//TIM3复用配置

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);//TIM4复用配置
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);//TIM4复用配置
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);//TIM4复用配置
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);//TIM4复用配置
	
	/**-------------------------GPIOA配置---------------------------**/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
	GPIO_Init(GPIOA,&GPIO_InitStructure);      
	
	/**-------------------------GPIOB配置---------------------------**/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|
																GPIO_Pin_10|GPIO_Pin_11;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	/**-------------------------GPIOC配置---------------------------**/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	/**-------------------------GPIOD配置---------------------------**/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	/*----------------------------TIM2基本配置------------------------------*/

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=arr;                     
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure); 

	/*-----------------------------TIM3基本配置------------------------------*/
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	/*-----------------------------TIM4基本配置------------------------------*/
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	/*-------------------------------PWM配置--------------------------------------*/
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	/*---------------------TIM2-4 通道1 预装载值配置--------------------------------*/
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/*---------------------TIM2-4 通道2 预装载值配置--------------------------------*/
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	/*---------------------TIM2-4 通道3 预装载值配置--------------------------------*/
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  
	
	/*---------------------TIM2-4 通道4 预装载值配置--------------------------------*/
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  
	
	/*----------------TIM2-4 计数最大值使能配置-------------------------------------*/
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	//TIM_Cmd(TIM3, ENABLE);  //TIM3时钟总使能
}  