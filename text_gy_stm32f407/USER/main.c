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
#include "usart.h"
#include "delay.h"
#include "stdio.h"
#include "DCMotorInit.h"
#include "DCMotorControl.h"

int main(void)
{
	float i = 0;
	int j = 0;
	/**********************----��ʼ������-------*****************************/
	RCC_Config();//�����һ����ʱ��������
	uart_init(115200);//���������ã�
	PWMConfig(PWMCOUNT-1,168-1);//84MHZ/(50*168) = 1000HZ
	delay_init(168);//��ʱ������ʼ�����ã�ϵͳʱ��Ϊ168Mhz
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	TTMotorState_Init();
	StepperMotor_Init();
	TIM_SetCompare1(TIM2, HALFPWMCOUNT);
	TIM_Cmd(TIM2,DISABLE);//ʹ��Timer2,�˲������ڿ��ƺ�����ʧ��
	delay_ms(10);
	TIM_Cmd(TIM2,ENABLE);//ʹ��Timer2,�˲������ڿ��ƺ�����ʹ��
	/***********************----���̺���-----***********************************/
									/*******DCMotorTest********/
	while(1)
	{
		StepperMotorControl(20,PDIR,ENABLE);
	}
	return 0;
}