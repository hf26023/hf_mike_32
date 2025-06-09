#ifndef _MY_USART_H
#define _MY_USART_H

#include "main.h"

#define RXBUFFER_LEN 30

typedef struct User_USART
{
		uint8_t Rx_flag;									//接收完成标志
		uint8_t Rx_len;										//接收长度
		uint8_t frame_head;						//帧头
		uint8_t frame_tail;								//帧尾
		int aileron,elevator,throttle,rudder,x,y,claw,mode,z,m4mode;
		int ch[8];//模拟量，适配我们自己编写的app -127d到128
		uint8_t sw[8];//开关量，适配我们自己编写的app 0或1
		int BTcnt; //接收计数量，用于判断失控
		uint8_t RxBuffer[RXBUFFER_LEN];		//数据存储
}User_USART;

typedef struct Customize_USART //自定义串口，预留给视觉or ROS使用
{
		uint8_t Rx_flag;									//接收完成标志
		uint8_t Rx_len;										//接收长度
		uint8_t frame_head;						//帧头
		uint8_t frame_tail;								//帧尾
		uint8_t key;   //关键词
		int RXcnt; //接收计数量
		uint8_t RxBuffer[RXBUFFER_LEN];		//数据存储
}Customize_USART;

extern User_USART BT_Data;
extern Customize_USART myData;
void BTData_Process(uint8_t *RxBuffer);
void User_USART_Init(User_USART *Data);
void myData_Init(Customize_USART *Data);
void myData_Process(uint8_t *RxBuffer);
#endif


