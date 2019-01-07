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
#include "usart_gy.h"
#include "delay.h"
#include "stdio.h"
#include "Matrix_Inverse_solution.h"

extern KinematicsArm KMGecko;
int main(void)
{
	int i = 0;
	int j = 0;
	/**********************----初始化函数-------*****************************/
	RCC_Config();//程序第一步：时钟树配置
	uart_init(115200);//波特率配置；
	PWMConfig(10000-1,168-1);//84MHZ/(10000*168) = 50HZ
	PWM_Enable();//pwm使能
	delay_init(168);//延时函数初始化配置，系统时钟为168Mhz
	/******--------------------矩阵解算-------------------********/
	InitRobotPosion();//robot位置初始化
	FullStepCycle();//generate  px py pz Array	
	Inverse_Kinematic(LegRF);
	Inverse_Kinematic(LegLF);
	Inverse_Kinematic(LegRR);
	Inverse_Kinematic(LegLR);	
	/***********************----流程函数-----***********************************/
									/*******DCMotorTest********/

	return 0;
}