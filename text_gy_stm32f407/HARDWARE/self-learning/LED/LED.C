/********************************************************************************************************
*date：2018-5-11
*author：NUAA gooyle
*function:LED_Init()
*input:none
*output:none
*remarks：GPIO上拉输出配置
********************************************************************************************************/ 
#include "led.h"

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	//初始化，一开始将f9和f10置为1
	//GPIO_SetBits(GPIOF,GPIO_Pin_9|GPIO_Pin_10);
	LED0 = 1;
	LED1 = 1;
}