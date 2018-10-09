/********************************************************************************************************
*date��2018-6-19
*author��NUAA google
*function:gecko motion control
*input:none
*output:none
*remarks��none
********************************************************************************************************/ 
#include "rcc_config.h"
#include "pwm_init.h"
#include "usart_gy.h"
#include "delay.h"
//#include "stdio.h"
#include "Matrix_Inverse_solution.h"

extern KinematicsArm KMGecko;
int main(void)
{
	/**********************----��ʼ������-------*****************************/
	RCC_Config();//�����һ����ʱ��������
	uart_init(115200);//���������ã�
	PWMConfig(10000-1,168-1);//84MHZ/(10000*168) = 50HZ
	PWM_Enable();//pwmʹ��
	delay_init(168);//��ʱ������ʼ�����ã�ϵͳʱ��Ϊ168Mhz
	/******--------------------�������-------------------********/
	Inverse_Kinematic();
	
	/***********************----���̺���-----***********************************/
	
	while(1)
	{
//		Angle(KMGecko.theta3[0],LF_J3);
//		Angle(-KMGecko.theta1[0],LF_J1);
//		Angle(KMGecko.theta2[0],LF_J2);
		
//		Angle(KMGecko.theta1[0],RF_J1);
//		Angle(-KMGecko.theta2[0],RF_J2);		
//		Angle(KMGecko.theta3[0],RF_J3);
//		
//		Angle(KMGecko.theta1[0],RR_J1);
//		Angle(-KMGecko.theta2[0],RR_J2);
//		Angle(KMGecko.theta3[0],RR_J3);

		Angle(-KMGecko.theta1[0],LR_J1);
		Angle(KMGecko.theta2[0],LR_J2);
		Angle(KMGecko.theta3[0],LR_J3);
	}
	return 0;
}