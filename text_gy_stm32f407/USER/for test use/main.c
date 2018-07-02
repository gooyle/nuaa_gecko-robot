/********************************************************************************************************
*date：2018-5-10
*author：NUAA google
*function:test code
*input:none
*output:none
*remarks：for test 
********************************************************************************************************/ 

/*************************************************/
//#include "led.h"
//#include "delay.h"
//#include "key.h"
//#include "usart.h"
//#include "extic.h"
//#include "timer.h"
//#include "pwm_tim14.h"
//#include "stm32f4xx_conf.h"
/**int main(void)
{
	//uint8_t len = 0;//数据长度len、寄存器res调用原子文件，不用时可以注释
	//uint8_t res;
	//int t = 0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组，组二，两位抢占优先级，两位响应优先级
	delay_init(168);
	uart_init(115200);
	//LED_Init();
	KEY_Init();
	EXTIC_Init();
	TIM3_Init(5000-1,8400-1);
	PWM_TIM14_Init(500-1,84-1);
	while(1)
	{
		uint16_t change_led = 0;
		
		for(change_led = 0; change_led <= 450; change_led++)
		{
			TIM_SetCompare1(TIM14,change_led);
			delay_ms(10);
		}
	}
return 0;
}
******************///
