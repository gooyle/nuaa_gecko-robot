#ifndef __USART_GY_H
#define __USART_GY_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
typedef struct
{
	char DateReceive;//为1：正在接受数据包；为0：还为收到开头
	char PackFinish;//为1:已经完成包的接受；为0:还未完成包的接收
	char DateCNT;//字节计数，收到几个计数值,初始化为0
	u8 DateBuffer[100];//缓存区为100
	char EndFlag;//结束标志为1：接收到两个结束字符
}DateStatE;

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define EN_YZ_USART       1    //原子自带例程
#define EN_GY_USART       0    //谷雨撰写的USART解码包程序,支持数据包形式，头为0xAE 86 ；尾为0x68 EA
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
extern DateStatE DateState;
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
#endif


