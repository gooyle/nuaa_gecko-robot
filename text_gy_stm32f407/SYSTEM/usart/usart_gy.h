#ifndef __USART_GY_H
#define __USART_GY_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
typedef struct
{
	char DateReceive;//Ϊ1�����ڽ������ݰ���Ϊ0����Ϊ�յ���ͷ
	char PackFinish;//Ϊ1:�Ѿ���ɰ��Ľ��ܣ�Ϊ0:��δ��ɰ��Ľ���
	char DateCNT;//�ֽڼ������յ���������ֵ,��ʼ��Ϊ0
	u8 DateBuffer[100];//������Ϊ100
	char EndFlag;//������־Ϊ1�����յ����������ַ�
}DateStatE;

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define EN_YZ_USART       1    //ԭ���Դ�����
#define EN_GY_USART       0    //����׫д��USART���������,֧�����ݰ���ʽ��ͷΪ0xAE 86 ��βΪ0x68 EA
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
extern DateStatE DateState;
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
#endif


