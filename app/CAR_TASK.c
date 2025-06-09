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
int Velocity_Pwm,Turn_Pwm;//计算的pwm
int Motor1=0,Motor2=0,Motor3=0,Motor4=0;//左右电机最终输出的pwm
float  Movement = 0;//目标速度
float taget_yaw = 0;//目标角度
int Contrl_Turn = 64;//
int hf_Mode = 1;
int s1=1620,s2=1820,s3=1800,s4 = 1150;
int s1_raw = 1620,s2_raw = 1850,s3_raw = 1820,s4_raw = 1120;
float alpha,beta,gama=0,x=60,y=120;
int rx_cnt = 0;//失控标记
int rxloss_flag = 1,avoidance_flag = 0,off_flag =0;
int ex_dis = 200;//mm
float distrack_Kp = 0.5;

//环境数据采集任务
void Car_Task_200HZ(void)
{
    //static struct mpu6050_data Last_Data;
    mpu_dmp_get_data(&outMpu.pitch,&outMpu.roll,&outMpu.yaw);
    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_15);

}


/**************************************************************************************************************
*函数名:Car_Task_100HZ(void)
*功能；100hz任务
*形参:无
*返回值:无
*确定转向环以及设置电机pwm
**************************************************************************************************************/
void Car_Task_100HZ(void)
{
		rc_smooth(&BT_Data.aileron, &BT_Data.elevator, &BT_Data.throttle,&BT_Data.rudder);//平滑遥控数据
    taget_yaw -= BT_Data.rudder*0.015;
    if(taget_yaw >180) taget_yaw -= 360;
    if(taget_yaw <-180) taget_yaw +=360;
    Turn_Pwm = Vertical_turn_PD(taget_yaw,outMpu.yaw,outMpu.gyro_z);
    //麦轮小车模式切换
		GettraceModule();
    switch(BT_Data.m4mode) {
    case 0://无yaw角度闭环
        Turn_Pwm = 20*BT_Data.rudder;
        break;
    case 1://有角度闭环，有避障
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

    case 2://有角度闭环，无避障
        break;
    case 3://定距跟随
        if(dis>350) {
            BT_Data.elevator=0;
        }
        else {
            BT_Data.elevator = (dis - ex_dis)*distrack_Kp;
        }
        break;
		case 4://无闭环，有避障
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
			
			
		case 5://无陀螺仪的寻迹
				
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
				
				
				if((la == 1)&&(lb == 1)&&(ra == 1)&&(rb == 1)){//碰到停止线时停下
					Turn_Pwm = 0;
					BT_Data.elevator=0;
				}
				else{
					BT_Data.elevator=15;//正常低速行驶
				}
    default:
        break;

    }


		//如果需要反向在对应电机前面整体加负号
    Motor4 = -BT_Data.aileron*30+BT_Data.elevator*20-Turn_Pwm;//右上
    Motor1 = BT_Data.aileron*30+BT_Data.elevator*20+Turn_Pwm;//左上
    Motor2 = -BT_Data.aileron*30+BT_Data.elevator*20+Turn_Pwm;//左下
    Motor3 = BT_Data.aileron*30+BT_Data.elevator*20-Turn_Pwm;//右下
    PWM_Limiting(&Motor1,&Motor2,&Motor3,&Motor4);//pwm限幅

    if( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)) off_flag = 1;
    else off_flag = 0;//离地检测，这里用的是红外模块，隐藏功能后续再讲


    //失控或者离地保护，关闭电机
    //if (rxloss_flag == 0 && off_flag == 0) {
		if (rxloss_flag == 0) {//需要离地检测用上一行代码替换掉此行
        Set_PWM(Motor1,Motor2,Motor3,Motor4);
    }
    else {
        Set_PWM(0,0,0,0);
    }

    x+= (float)BT_Data.x*25/2560;
    y+= (float)BT_Data.y*25/2560;
    gama = (float)BT_Data.z*90/127;

    //机械臂运动范围限制
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
    //机械臂控制，第一次组装把下方5行代码注释掉（默认注释掉）
    arm_calculating( x,y,&alpha,&beta,&gama);
		if(BT_Data.m4mode!=6){//如果mode值为6进入调试模式，只针对刚启动时有效
			s1=s1_raw-(90-alpha)*2000/180;//最左侧值为机械臂竖直向上时对应的脉宽值，单位us
			s2=s2_raw-(alpha-beta)*2000/180;//最左侧值为机械臂竖直向上时对应的脉宽值，单位us
			s3=s3_raw+(-beta+gama)*2000/180;//最左侧值为机械臂竖直向上时对应的脉宽值，单位us
			s4 = BT_Data.claw*(500)/256+s4_raw;//最右侧值为夹爪关闭时对应的脉宽值，单位us 1050
			
		}
		else{//安装模式
			s1=s1_raw;
			s2=s2_raw;
			s3=s3_raw;
			s4 = s4_raw;
		}
    if(rxloss_flag == 1){
			s4 = 1400;//失控松开 防止夹爪舵机堵转
		}
		
    Set_Servo(s1,s2,s3,s4);

}



/**************************************************************************************************************
*函数名:Car_Task_10HZ(void)
*功能；10hz任务
*形参:无
*返回值:无
*回传数据和点灯
**************************************************************************************************************/
int cnt =0;
float bat_value;
void Car_Task_10HZ(void)
{
    Getdistance();//超声波数据更新
    //陀螺仪数据回传
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
    //led闪烁（对应两个mos管）
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
        bat_value =(float)HAL_ADC_GetValue(&hadc1)*0.005076;//4096*3.3*6*1.05，最后的系数根据测量值校正
}

/**************************************************************************************************************
*函数名:Car_Interaction_begin(void)
*功能；交互进程启动
*形参:无
*返回值:无
*
**************************************************************************************************************/
void Car_Task_Interaction_begin(void) {
    //显示界面初始化
    OLED_CLS();
    OLED_ShowStr (1,1,"pitch:",1);
    OLED_ShowStr (1,2,"roll:",1);
    OLED_ShowStr (1,3,"yaw:",1);
    OLED_ShowStr (1,4,"dis:",1);
    OLED_ShowStr (1,5,"bat:",1);
    OLED_ShowStr (70,5,"key:",1);

}


/**************************************************************************************************************
*函数名:Car_Interaction(void)
*功能；交互进程
*形参:无
*返回值:无
*数据显示
**************************************************************************************************************/
void Car_Task_Interaction(void)
{
    //失控判断
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
            OLED_ShowStr(30,6,"       ",2);//清空这行
    }
		


    //离地显示
    if(off_flag == 1) {
        OLED_ShowStr(90,7,"off",1);
    }
    else {
        OLED_ShowStr(90,7,"   ",1);
    }
    //欧拉角显示
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
    //超声波检测距离显示
    OLED_ShowNum(56,4,dis,4,2);
    OLED_ShowStr(80,4,"mm",1);
    
    //电池电压显示
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
    //避障显示
    if (avoidance_flag == 1) OLED_ShowStr(95,4,"!!",1);
    else OLED_ShowStr(95,4,"  ",1);


}
