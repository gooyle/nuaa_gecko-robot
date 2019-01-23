/********************************************************************************************************
*date��2018-6-19
*author��NUAA google
*function:gecko motion control
*input:none
*output:none
*remarks��none
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
	/**********************----��ʼ������-------*****************************/
	RCC_Config();//�����һ����ʱ��������
	uart_init(115200);//���������ã�
	PWMConfig(10000-1,168-1);//84MHZ/(10000*168) = 50HZ
	PWM_Enable();//pwmʹ��
	delay_init(168);//��ʱ������ʼ�����ã�ϵͳʱ��Ϊ168Mhz
	/******--------------------�������-------------------********/
	StartAngleInit();//�ؽڽ���
	InitRobotPosion();//robotλ�ó�ʼ��
	FullStepCycle();//generate  px py pz Array	
	Inverse_Kinematic(LegRF);
	Inverse_Kinematic(LegLF);
	Inverse_Kinematic(LegRR);
	Inverse_Kinematic(LegLR);	
	StartAngleInit();
	delay_ms(1000);
	/***********************----���̺���-----***********************************/
	
	while(1)
	{

		/**-----------��ʽ���г���-----------**/
		for(j = 0; j<(4*STEPNUM); j++)	
		{
			Angle(KMGecko.theta3[1][j]+KMGecko.StartAngle[LF_J3],LF_J3);
			Angle(-KMGecko.theta1[1][j]+KMGecko.StartAngle[LF_J1],LF_J1);
			Angle(KMGecko.theta2[1][j]+KMGecko.StartAngle[LF_J2],LF_J2);
			
			Angle(KMGecko.theta1[0][j]+KMGecko.StartAngle[RF_J1],RF_J1);
			Angle(-KMGecko.theta2[0][j]+KMGecko.StartAngle[RF_J2],RF_J2);		
			Angle(-KMGecko.theta3[0][j]+KMGecko.StartAngle[RF_J3],RF_J3);
			
			Angle(KMGecko.theta1[2][j]+KMGecko.StartAngle[RR_J1],RR_J1);
			Angle(-KMGecko.theta2[2][j]+KMGecko.StartAngle[RR_J2],RR_J2);
			Angle(KMGecko.theta3[2][j]+KMGecko.StartAngle[RR_J3],RR_J3);

			Angle(-KMGecko.theta1[3][j]+KMGecko.StartAngle[LR_J1],LR_J1);
			Angle(KMGecko.theta2[3][j]+KMGecko.StartAngle[LR_J2],LR_J2);
			Angle(-KMGecko.theta3[3][j]+KMGecko.StartAngle[LR_J3],LR_J3);
			delay_ms(15);
		}


	}
	return 0;
}