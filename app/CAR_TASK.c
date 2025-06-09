#include "CAR_TASK.h"
#include "tim.h"
#include "mpu6050.h"
#include "echo.h"
#include "stdio.h"
#include "inv_mpu.h"
#include "contrl.h"
#include "delay.h"
#include "my_usart.h"
#include "oled.h"
#include "adc.h"
#include "traceModule.h"

struct mpu6050_data outMpu;
int Velocity_Pwm,Turn_Pwm;//�����pwm
int Motor1=0,Motor2=0,Motor3=0,Motor4=0;//���ҵ�����������pwm
float  Movement = 0;//Ŀ���ٶ�
float taget_yaw = 0;//Ŀ��Ƕ�
int Contrl_Turn = 64;//
int hf_Mode = 1;
int s1=1620,s2=1820,s3=1800,s4 = 1150;
int s1_raw = 1620,s2_raw = 1850,s3_raw = 1820,s4_raw = 1120;
float alpha,beta,gama=0,x=60,y=120;
int rx_cnt = 0;//ʧ�ر��
int rxloss_flag = 1,avoidance_flag = 0,off_flag =0;
int ex_dis = 200;//mm
float distrack_Kp = 0.5;

//�������ݲɼ�����
void Car_Task_200HZ(void)
{
    //static struct mpu6050_data Last_Data;
    mpu_dmp_get_data(&outMpu.pitch,&outMpu.roll,&outMpu.yaw);
    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_15);

}


/**************************************************************************************************************
*������:Car_Task_100HZ(void)
*���ܣ�100hz����
*�β�:��
*����ֵ:��
*ȷ��ת���Լ����õ��pwm
**************************************************************************************************************/
void Car_Task_100HZ(void)
{
		rc_smooth(&BT_Data.aileron, &BT_Data.elevator, &BT_Data.throttle,&BT_Data.rudder);//ƽ��ң������
    taget_yaw -= BT_Data.rudder*0.015;
    if(taget_yaw >180) taget_yaw -= 360;
    if(taget_yaw <-180) taget_yaw +=360;
    Turn_Pwm = Vertical_turn_PD(taget_yaw,outMpu.yaw,outMpu.gyro_z);
    //����С��ģʽ�л�
		GettraceModule();
    switch(BT_Data.m4mode) {
    case 0://��yaw�Ƕȱջ�
        Turn_Pwm = 20*BT_Data.rudder;
        break;
    case 1://�нǶȱջ����б���
        if(dis<250 && BT_Data.elevator>0) {
            BT_Data.elevator=0;
            avoidance_flag = 1;
        }
        else if(dis<200)  {
            BT_Data.elevator = -(200-dis)*1.3;
            avoidance_flag = 1;
        }
        else avoidance_flag = 0;
        break;

    case 2://�нǶȱջ����ޱ���
        break;
    case 3://�������
        if(dis>350) {
            BT_Data.elevator=0;
        }
        else {
            BT_Data.elevator = (dis - ex_dis)*distrack_Kp;
        }
        break;
		case 4://�ޱջ����б���
			Turn_Pwm = 20*BT_Data.rudder;
		  if(dis<250 && BT_Data.elevator>0) {
            BT_Data.elevator=0;
            avoidance_flag = 1;
        }
        else if(dis<200)  {
            BT_Data.elevator = -(200-dis)*1.3;
            avoidance_flag = 1;
        }
        else avoidance_flag = 0;
        break;
			
			
		case 5://�������ǵ�Ѱ��
				
				if(la == 1){
					Turn_Pwm = -500;
				}
				
				if(lb == 1){
					Turn_Pwm = -200;
				}
				
				if(ra == 1){
					Turn_Pwm = 200;
				}
				
				if(rb == 1){
					Turn_Pwm = 500;
				}
				
				
				if((la == 1)&&(lb == 1)&&(ra == 1)&&(rb == 1)){//����ֹͣ��ʱͣ��
					Turn_Pwm = 0;
					BT_Data.elevator=0;
				}
				else{
					BT_Data.elevator=15;//����������ʻ
				}
    default:
        break;

    }


		//�����Ҫ�����ڶ�Ӧ���ǰ������Ӹ���
    Motor4 = -BT_Data.aileron*30+BT_Data.elevator*20-Turn_Pwm;//����
    Motor1 = BT_Data.aileron*30+BT_Data.elevator*20+Turn_Pwm;//����
    Motor2 = -BT_Data.aileron*30+BT_Data.elevator*20+Turn_Pwm;//����
    Motor3 = BT_Data.aileron*30+BT_Data.elevator*20-Turn_Pwm;//����
    PWM_Limiting(&Motor1,&Motor2,&Motor3,&Motor4);//pwm�޷�

    if( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)) off_flag = 1;
    else off_flag = 0;//��ؼ�⣬�����õ��Ǻ���ģ�飬���ع��ܺ����ٽ�


    //ʧ�ػ�����ر������رյ��
    //if (rxloss_flag == 0 && off_flag == 0) {
		if (rxloss_flag == 0) {//��Ҫ��ؼ������һ�д����滻������
        Set_PWM(Motor1,Motor2,Motor3,Motor4);
    }
    else {
        Set_PWM(0,0,0,0);
    }

    x+= (float)BT_Data.x*25/2560;
    y+= (float)BT_Data.y*25/2560;
    gama = (float)BT_Data.z*90/127;

    //��е���˶���Χ����
    if(y<-90) {
        y=-90;
        if(x<95) x=95;
        if(x>110) x=110;
    }
    else if(y<-30) {
        if(x<95) x=95;
        if(x>110) x=110;
    }
    else if(y<100) {
        if(x<95) x=95;
        if(x>140) x=140;
    }
    else if (y<140) {
        if(x<60) x=60;
        if(x>100) x =100;
    }
    else {
        y=140;
        if(x<60) x=60;
        if(x>100) x =100;
    }
    //��е�ۿ��ƣ���һ����װ���·�5�д���ע�͵���Ĭ��ע�͵���
    arm_calculating( x,y,&alpha,&beta,&gama);
		if(BT_Data.m4mode!=6){//���modeֵΪ6�������ģʽ��ֻ��Ը�����ʱ��Ч
			s1=s1_raw-(90-alpha)*2000/180;//�����ֵΪ��е����ֱ����ʱ��Ӧ������ֵ����λus
			s2=s2_raw-(alpha-beta)*2000/180;//�����ֵΪ��е����ֱ����ʱ��Ӧ������ֵ����λus
			s3=s3_raw+(-beta+gama)*2000/180;//�����ֵΪ��е����ֱ����ʱ��Ӧ������ֵ����λus
			s4 = BT_Data.claw*(500)/256+s4_raw;//���Ҳ�ֵΪ��צ�ر�ʱ��Ӧ������ֵ����λus 1050
			
		}
		else{//��װģʽ
			s1=s1_raw;
			s2=s2_raw;
			s3=s3_raw;
			s4 = s4_raw;
		}
    if(rxloss_flag == 1){
			s4 = 1400;//ʧ���ɿ� ��ֹ��צ�����ת
		}
		
    Set_Servo(s1,s2,s3,s4);

}



/**************************************************************************************************************
*������:Car_Task_10HZ(void)
*���ܣ�10hz����
*�β�:��
*����ֵ:��
*�ش����ݺ͵��
**************************************************************************************************************/
int cnt =0;
float bat_value;
void Car_Task_10HZ(void)
{
    Getdistance();//���������ݸ���
    //���������ݻش�
    printf("acc_x = %d\n",outMpu.acc_x);
    printf("acc_y = %d\n",outMpu.acc_y);
    printf("acc_z = %d\n",outMpu.acc_z);
    printf("gyro_x = %d\n",outMpu.gyro_x);
    printf("gyro_y = %d\n",outMpu.gyro_y);
    printf("gyro_z = %d\n",outMpu.gyro_z);
    printf("pitch = %f\n",outMpu.pitch);
    printf("roll = %f\n",outMpu.roll);
    printf("yaw = %f\n",outMpu.yaw);
    printf("\r\n");
    printf("dis = %f\n",dis);
    //led��˸����Ӧ����mos�ܣ�
    switch(cnt) {
    case 0:
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
        cnt++;
        break;
    case 1:
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
        cnt++;
        break;
    case 2:
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
        cnt++;
        break;
    case 3:
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
        cnt=1;
        break;
    }
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);


    HAL_ADC_Start(&hadc1);
    if(HAL_OK == HAL_ADC_PollForConversion(&hadc1,100))
        bat_value =(float)HAL_ADC_GetValue(&hadc1)*0.005076;//4096*3.3*6*1.05������ϵ�����ݲ���ֵУ��
}

/**************************************************************************************************************
*������:Car_Interaction_begin(void)
*���ܣ�������������
*�β�:��
*����ֵ:��
*
**************************************************************************************************************/
void Car_Task_Interaction_begin(void) {
    //��ʾ�����ʼ��
    OLED_CLS();
    OLED_ShowStr (1,1,"pitch:",1);
    OLED_ShowStr (1,2,"roll:",1);
    OLED_ShowStr (1,3,"yaw:",1);
    OLED_ShowStr (1,4,"dis:",1);
    OLED_ShowStr (1,5,"bat:",1);
    OLED_ShowStr (70,5,"key:",1);

}


/**************************************************************************************************************
*������:Car_Interaction(void)
*���ܣ���������
*�β�:��
*����ֵ:��
*������ʾ
**************************************************************************************************************/
void Car_Task_Interaction(void)
{
    //ʧ���ж�
		if(BT_Data.BTcnt == 0){
			rx_cnt ++;
		}
		else {
       rx_cnt =0;
       BT_Data.BTcnt  = 0;
    }

    if (rx_cnt >3) {
        if(rxloss_flag ==0) {

            OLED_ShowStr(30,6,"rxloss",2);
            rxloss_flag = 1;
        }
    }
    else {
        if(rxloss_flag ==1) {
            rxloss_flag = 0;

        }
				if(BT_Data.m4mode == 6){
						OLED_ShowStr(30,6,"factory",2);
				}
				else if(BT_Data.m4mode == 0){
						OLED_ShowStr(30,6,"manual ",2);
				}
				else if(BT_Data.m4mode == 1){
						OLED_ShowStr(30,6,"stabil ",2);
				}
				else
            OLED_ShowStr(30,6,"       ",2);//�������
    }
		


    //�����ʾ
    if(off_flag == 1) {
        OLED_ShowStr(90,7,"off",1);
    }
    else {
        OLED_ShowStr(90,7,"   ",1);
    }
    //ŷ������ʾ
    if(outMpu.pitch<0)
    {
        OLED_ShowChar(46,1,'-',2);
        OLED_ShowFloat(50,1,-outMpu.pitch,3,2);
    }
    else
    {
        OLED_ShowChar(46,1,' ',2);
        OLED_ShowFloat(50,1,outMpu.pitch,3,2);
    }

    if(outMpu.roll<0)
    {
        OLED_ShowChar(46,2,'-',2);
        OLED_ShowFloat(50,2,-outMpu.roll,3,2);
    }
    else
    {
        OLED_ShowChar(46,2,' ',2);
        OLED_ShowFloat(50,2,outMpu.roll,3,2);
    }

    if(outMpu.yaw<0)
    {
        OLED_ShowChar(46,3,'-',2);
        OLED_ShowFloat(50,3,-outMpu.yaw,3,2);
    }
    else
    {
        OLED_ShowChar(46,3,' ',2);
        OLED_ShowFloat(50,3,outMpu.yaw,3,2);
    }

    OLED_ShowChar(46,4,' ',2);
    //��������������ʾ
    OLED_ShowNum(56,4,dis,4,2);
    OLED_ShowStr(80,4,"mm",1);
    
    //��ص�ѹ��ʾ
    OLED_ShowFloat(20,5,bat_value,4,2);
    // if(bat_value<7.8) {
    //     OLED_ShowStr(90,5,"lowbat",1);
    //     // HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
    // }
    // else {
    //     OLED_ShowStr(90,5,"      ",1);
    //     // HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
    // }
		int tempNum = myData.key;
    OLED_ShowNum(90,5,tempNum,4,2);
    //������ʾ
    if (avoidance_flag == 1) OLED_ShowStr(95,4,"!!",1);
    else OLED_ShowStr(95,4,"  ",1);


}
