/********************************************************************************************************
*date：2019-1-7
*author：NUAA gooyle
*function:DCMotorPin_Init()
*input:none
*output:none
*remarks：@Brake引脚——PF9:高为启动，低为停止；
*					@Pwm引脚———PA15:建议为20K-30K——Timer2CH2
*					@Dir引脚———PF10
********************************************************************************************************/ 
#include "DCMotorInit.h"
#include "delay.h"
StepperMotor TTMotor;

void DCMotorPin_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	//初始化，一开始将f9和f10置为0
	GPIO_ResetBits(GPIOF,GPIO_Pin_9|GPIO_Pin_10);
}

/********************************************************************************************************
*date：2019-1-8
*author：NUAA gooyle
*function:StepperMotor_Init()
*input:none
*output:none
*remarks：引用DCMotorInit的配置，这里注意不要冲突，程序依靠中断实现脉冲计数，此程序含有中断配置
**				@EN引脚——---PF9:高为启动，低为停止；
*					@Pluse引脚———PA15:建议为1k之内——Timer2CH1
*					@Dir引脚———PF10
**********************************************************************************************/ 
void StepperMotor_Init(void)
{
	    /**Dir及EN引脚配置***/
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	//初始化，一开始将f9和f10置为0
	GPIO_ResetBits(GPIOF,GPIO_Pin_9|GPIO_Pin_10);
		     
	/* 设置 NVIC 参数 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //打开 TIM2_IRQn 的全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级为 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1; //响应优先级为 1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能
	NVIC_Init(&NVIC_InitStructure);
	
	 /*开Timer2中断*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);	//清除TIM2中断标志
	delay_ms(1);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);	//使能TIM2更新中断
	

}
/********************************************************************************************************
*date：2019-1-8
*author：NUAA gooyle
*function:void TTMotorState_Init(void)
*input:none
*output:none
*remarks：脉冲数置0，脉冲未发送完成
**********************************************************************************************/ 
void TTMotorState_Init(void)
{
	TTMotor.PluseFinished = 1;
	TTMotor.PluseNumber = 0;
	//TIM_CCxCmd(TIM2,TIM_Channel_1, TIM_CCx_Disable);

}