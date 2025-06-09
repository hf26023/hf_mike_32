#include "my_usart.h"
#include "stdlib.h"
User_USART BT_Data;		//��������
Customize_USART myData; //�Զ��崮�ڵĽ�������
#define BtMode 1 //����ģʽ��0Ϊʹ������������ 1Ϊʹ�������Լ�������app
//��ʼ������
void User_USART_Init(User_USART *Data)
{
    for(uint8_t i=0; i < RXBUFFER_LEN; i++)	Data->RxBuffer[i] = 0;
    Data->frame_head = 0xA5;
    Data->frame_tail = 0x5A;
    Data->Rx_flag = 0;
    Data->Rx_len = 0;//aileron,elevator,throttle,rudder,x,y,claw,mode,z;
		if(!BtMode){
			Data->aileron = 0;
			Data->elevator = 0;
			Data->throttle = 0;
			Data->rudder = 0;
			Data->x = 0;
			Data->y = 0;
			Data->claw = 128;
			Data->mode = 0;
			Data->z = -120;
			Data->m4mode = 0 ;//5���˵�һ���ϵ��⣬������������ʼֵΪ0
		}
    else{
			for(int i= 0;i<8;i++){
			Data->ch[i] = 0;//ģ������0
		}
			for(int i = 0;i<7;i++){
				Data->sw[i] = 0; //��������0
			}
			
		}
		
		
}

void myData_Init(Customize_USART *Data){
    for(uint8_t i=0; i < RXBUFFER_LEN; i++)	Data->RxBuffer[i] = 0;//����ȫ����0
    Data->frame_head = 0xA5;
    Data->frame_tail = 0x5A;//֡ͷ֡β ���ÿɲ���
    Data->Rx_flag = 0;
    Data->Rx_len = 0;
	Data->key = 0xFF;
	Data->RXcnt = 0;
}

void BTData_Process(uint8_t *RxBuffer)
{

    //���֡ͷ֡β
    if(RxBuffer[0] != BT_Data.frame_head) return;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
    if(RxBuffer[12] != BT_Data.frame_tail) return;
		if(BtMode){//ʹ�����ǵ�app
			uint8_t temp = 0;
			for(int i = 1;i<=10;i++){
				temp+=RxBuffer[i];
			}
			if(temp!=RxBuffer[11]) return;
			else{//У����ȷ
				uint8_t mask = 1;
				for(int i=0;i<8;i++){
					BT_Data.ch[i] = RxBuffer[i+1];//��ģ��ͨ����ֵ
					BT_Data.sw[i] = (RxBuffer[9]&mask)>>i;//�����ظ�ֵ
					mask = mask<<1;
					
				}
				//Ϊ�˼�����ǰ�ĳ����������֮ǰ�õı�����ֵ
				BT_Data.aileron = BT_Data.ch[2]-128;
				BT_Data.elevator =BT_Data.ch[3]-128;
				BT_Data.throttle =BT_Data.ch[1]-128;
				BT_Data.rudder = BT_Data.ch[0]-128;
				BT_Data.x = BT_Data.ch[4]-128;
				BT_Data.y = BT_Data.ch[5]-128;
				BT_Data.z = BT_Data.ch[6]-128;
				BT_Data.claw = BT_Data.ch[7];
				if(BT_Data.sw[0] == 1){//���Ͽ�
					if(BT_Data.sw[1] ==1){//���ȿ�
						BT_Data.m4mode = 1;//����ģʽ
					}
					else{//���ȹ�
						BT_Data.m4mode = 4;
					}
					
				}
				else{//���Ϲ�
					if(BT_Data.sw[1] ==1){//���ȿ�
						BT_Data.m4mode = 2;
					}
					else{//���ȹ�
						BT_Data.m4mode = 0;
					}
					
				}
				if(BT_Data.sw[2]==1){//���sw3���£����붨�����ģʽ
					BT_Data.m4mode = 3;
				}
				if(BT_Data.sw[3] == 1){//���sw4Ϊ1�����밲װģʽ
					BT_Data.m4mode = 6;
				}
				//BT_Data.mode = 1;//ʧ�ر�ʶ
				BT_Data.BTcnt++;
				
			}
	}else{//ʹ��һ��ʼ������������
		
		BT_Data.aileron = RxBuffer[1];
    BT_Data.elevator = RxBuffer[2];
    BT_Data.throttle = RxBuffer[3];
    BT_Data.rudder = RxBuffer[4];
    BT_Data.x = RxBuffer[5];
    BT_Data.y = RxBuffer[6];
    BT_Data.claw = RxBuffer[7];
    BT_Data.mode = RxBuffer[8];
    BT_Data.z = RxBuffer[9];
    BT_Data.m4mode = RxBuffer[10];
		
		if (BT_Data.aileron>127) BT_Data.aileron-=255;
    if (abs(BT_Data.aileron)<10) BT_Data.aileron = 0;//����ת�򶶶�
    if (BT_Data.elevator>127) BT_Data.elevator-=255;
    if (abs(BT_Data.elevator)<10) BT_Data.elevator = 0;//����ǰ������
    if (BT_Data.throttle>127) BT_Data.throttle-=255;
    if (abs(BT_Data.throttle)<10) BT_Data.throttle = 0;//����ǰ������
    if (BT_Data.rudder>127) BT_Data.rudder-=255;
    if (abs(BT_Data.rudder)<10) BT_Data.rudder = 0;//����ǰ������
    if (BT_Data.x>127) BT_Data.x-=255;
    if (BT_Data.y>127) BT_Data.y-=255;
    if (BT_Data.z>127) BT_Data.z-=255;
    if (BT_Data.claw>127) BT_Data.claw-=255;
    BT_Data.claw+=128;
		BT_Data.BTcnt++;
		
	}
    



    

}


void myData_Process(uint8_t *RxBuffer){
	myData.key = RxBuffer[0];


}
/*������μӵ�main�ĳ�ʼ����Ѿ��ӹ��ˣ�

	//��ʼ�����սṹ��
	User_USART_Init(&BT_Data);
	// ����DMA�����ж�
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,BT_Data.RxBuffer,RXBUFFER_LEN);

*/
