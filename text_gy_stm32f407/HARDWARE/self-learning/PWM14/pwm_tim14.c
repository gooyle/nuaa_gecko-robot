#include "pwm_tim14.h"

/********************************************************************************************************
*date：2018-5-14
*author：NUAA gooyle
*function:PWM_TIM14_Init()
*input:uint16_t arr,uint16_t psc
*output:none
*remarks：TIM14单通道pwm设置函数
********************************************************************************************************/

void PWM_TIM14_Init(uint16_t arr,uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStrture;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = arr;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
	//TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 
	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseInitStructure);
	
	/**注释掉的程序属于高级计数器**/
	//TIM_OCInitStrture.TIM_OCIdleState = 
	TIM_OCInitStrture.TIM_OCMode = TIM_OCMode_PWM1;
	//TIM_OCInitStrture.TIM_OCNIdleState = 
	//TIM_OCInitStrture.TIM_OCNPolarity = 
	TIM_OCInitStrture.TIM_OCPolarity = TIM_OCPolarity_Low;
	//TIM_OCInitStrture.TIM_OutputNState = 
	TIM_OCInitStrture.TIM_OutputState = TIM_OutputState_Enable;
	//TIM_OCInitStrture.TIM_Pulse = /**这里是CCR寄存器的数值，用于输出比较,外部函数进行操作**/
	TIM_OC1Init(TIM14,&TIM_OCInitStrture);
	
	//这里需要使能CCR1上的预装载寄存器以及ARR上的预分频寄存器
	TIM_OC1PreloadConfig(TIM14,TIM_OCPreload_Enable);//使能使得CCR寄存器的数值可以更改；
	TIM_ARRPreloadConfig(TIM14,ENABLE);//实时更新装载值；
	
	//最终使能时钟
	TIM_Cmd(TIM14,ENABLE);
}