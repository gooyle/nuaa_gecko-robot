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
#include "led.h"
#include "DCMotorControl.h"

int main(void)
{
	float i = 0;
	int j = 0;
	/**********************----��ʼ������-------*****************************/
	RCC_Config();//�����һ����ʱ��������
	uart_init(115200);//���������ã�
	MotorPin_Init();
	PWMConfig(50-1,168-1);//84MHZ/(25*168) = 2000HZ
	PWM_Enable();//pwmʹ��
	delay_init(168);//��ʱ������ʼ�����ã�ϵͳʱ��Ϊ168Mhz
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	/***********************----���̺���-----***********************************/
									/*******DCMotorTest********/
	while(1)
	{
		DCMotorControl(3,PDIR,ENABLE);
		delay_ms(1000);
		DCMotorControl(3,PDIR,DISABLE);
		delay_ms(800);
		DCMotorControl(3,NDIR,ENABLE);
		delay_ms(1000);
		DCMotorControl(3,PDIR,DISABLE);
		delay_ms(800);

	}
	return 0;
}