/********************************************************************************************************
*date：2018-5-12
*author：NUAA gooyle
*function:KEY_Init()
*input:none
*output:none
*remarks：按键GPIO输入配置
********************************************************************************************************/ 
#include "key.h"

void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOA,
	ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure); //pe4,pe3端口配置
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);//pa0配置为下拉
	
	//初始化三个端口,使其都为不亮状态
	GPIO_SetBits(GPIOE,GPIO_Pin_3|GPIO_Pin_4);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
}

/********************************************************************************************************
*date：2018-5-12
*author：NUAA gooyle
*function:KEY_Scan()
*input:none
*output:按键数值
*remarks：通过返回值确定按键键位
********************************************************************************************************/ 
uint8_t KEY_Scan(void)
{
	if(WK_UP == 1)
		return WK_UP_PRESS;
	else if(KEY_0 == 0)
		return KEY_0_PRESS;
	else if(KEY_1 == 0)
		return KEY_1_PRESS;
	else 
		return ALL_KEY_UP;
}