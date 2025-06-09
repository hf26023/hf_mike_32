#ifndef _MY_USART_H
#define _MY_USART_H

#include "main.h"

#define RXBUFFER_LEN 30

typedef struct User_USART
{
		uint8_t Rx_flag;									//������ɱ�־
		uint8_t Rx_len;										//���ճ���
		uint8_t frame_head;						//֡ͷ
		uint8_t frame_tail;								//֡β
		int aileron,elevator,throttle,rudder,x,y,claw,mode,z,m4mode;
		int ch[8];//ģ���������������Լ���д��app -127d��128
		uint8_t sw[8];//�����������������Լ���д��app 0��1
		int BTcnt; //���ռ������������ж�ʧ��
		uint8_t RxBuffer[RXBUFFER_LEN];		//���ݴ洢
}User_USART;

typedef struct Customize_USART //�Զ��崮�ڣ�Ԥ�����Ӿ�or ROSʹ��
{
		uint8_t Rx_flag;									//������ɱ�־
		uint8_t Rx_len;										//���ճ���
		uint8_t frame_head;						//֡ͷ
		uint8_t frame_tail;								//֡β
		uint8_t key;   //�ؼ���
		int RXcnt; //���ռ�����
		uint8_t RxBuffer[RXBUFFER_LEN];		//���ݴ洢
}Customize_USART;

extern User_USART BT_Data;
extern Customize_USART myData;
void BTData_Process(uint8_t *RxBuffer);
void User_USART_Init(User_USART *Data);
void myData_Init(Customize_USART *Data);
void myData_Process(uint8_t *RxBuffer);
#endif


