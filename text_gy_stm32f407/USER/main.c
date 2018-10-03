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

int main(void)
{
	/**********************----��ʼ������-------*****************************/
	RCC_Config();//�����һ����ʱ��������
	uart_init(115200);//���������ã�
	PWMConfig(10000-1,168-1);//84MHZ/(10000*168) = 50HZ
	PWM_Enable();//pwmʹ��
	delay_init(168);//��ʱ������ʼ�����ã�ϵͳʱ��Ϊ168Mhz
	
	/***********************----���̺���-----***********************************/
	
	printf("12.99\n");
	delay_ms(100);
	while(1)
	{
		Angle(0,LF_J1);
		//delay_ms(500);
		//Angle(-45,LF_J1);
		//delay_ms(2000);
	}
	return 0;
}