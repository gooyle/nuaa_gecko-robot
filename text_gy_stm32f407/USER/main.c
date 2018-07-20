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
#include "usart.h"
#include "delay.h"

int main(void)
{
	int inc = 0;
	uint16_t res_gy = 0;//�������ݼĴ�����
	uart_init(115200);//���������ã�
	RCC_Config();
	PWMConfig(10000-1,168-1);//84MHZ/(10000*168) = 50HZ
	PWM_Enable();//pwmʹ��
	delay_init(168);//SYSTEMCLOCK Ϊ168Mhz��
	
	while(1)
	{
		Angle(0,LF_J1);
		Angle(0,ARM_J1);
	}
	//while(1);
	return 0;
}