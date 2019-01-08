/********************************************************************************************************
*date：2018-6-19
*author：NUAA google
*function:gecko motion control
*input:none
*output:none
*remarks：none
//		Angle(KMGecko.theta3[0],LF_J3);
//		Angle(-KMGecko.theta1[0],LF_J1);
//		Angle(KMGecko.theta2[0],LF_J2);
//		
//		Angle(KMGecko.theta1[0],RF_J1);
//		Angle(-KMGecko.theta2[0],RF_J2);		
//		Angle(KMGecko.theta3[0],RF_J3);
//		
//		Angle(KMGecko.theta1[0],RR_J1);
//		Angle(-KMGecko.theta2[0],RR_J2);
//		Angle(KMGecko.theta3[0],RR_J3);

//		Angle(-KMGecko.theta1[0],LR_J1);
//		Angle(KMGecko.theta2[0],LR_J2);
//		Angle(KMGecko.theta3[0],LR_J3);
********************************************************************************************************/ 
#include "rcc_config.h"
#include "pwm_init.h"
#include "usart.h"
#include "delay.h"
#include "stdio.h"
#include "DCMotorInit.h"
#include "DCMotorControl.h"

int main(void)
{
	float i = 0;
	int j = 0;
	/**********************----初始化函数-------*****************************/
	RCC_Config();//程序第一步：时钟树配置
	uart_init(115200);//波特率配置；
	PWMConfig(PWMCOUNT-1,168-1);//84MHZ/(50*168) = 1000HZ
	delay_init(168);//延时函数初始化配置，系统时钟为168Mhz
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	TTMotorState_Init();
	StepperMotor_Init();
	TIM_SetCompare1(TIM2, HALFPWMCOUNT);
	TIM_Cmd(TIM2,DISABLE);//使能Timer2,此步流程在控制函数中失能
	delay_ms(10);
	TIM_Cmd(TIM2,ENABLE);//使能Timer2,此步流程在控制函数中使能
	/***********************----流程函数-----***********************************/
									/*******DCMotorTest********/
	while(1)
	{
		StepperMotorControl(20,PDIR,ENABLE);
	}
	return 0;
}