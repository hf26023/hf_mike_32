#include "math.h"
#include "stdlib.h"
#include "stm32f1xx_hal.h"
#include "contrl.h"



//需要在两个文件里改成你的编码器（.c）和pwm(.h)对应的寄存器

int   Dead_Zone=4000;    //电机死区
int   control_turn=64;                             //转向控制


//PID调节参数
struct pid_arg PID = {
    .Velocity_Kp=-120,//-48 70 100
    .Velocity_Ki=-2.9,//-0.24 0.21 0.325
    .Turn_Kp = 115,
    .Turn_Kd = 0.02,
};







/**************************************************************************************************************
*函数名:Vertical_turn_PD()
*功能:转向环PD
*形参:taget_yaw 目标yaw， yaw 陀螺仪yaw ， gyro 陀螺仪yaw方向角速度
*返回值:无
***************************************************************************************************************/
int Vertical_turn_PD(float taget_yaw,float yaw,float gyro)
{
    float Turn;
    float Bias_yaw;
    Bias_yaw=taget_yaw-yaw;
    if (Bias_yaw<-180) Bias_yaw+=360;
    if (Bias_yaw>180) Bias_yaw-=360;

    Turn=-Bias_yaw*PID.Turn_Kp-gyro*PID.Turn_Kd;
    return Turn;
}



/**************************************************************************************************************
*函数名:PWM_Limiting()
*功能:PWM限幅函数
*形参:无
*返回值:无
***************************************************************************************************************/
void PWM_Limiting(int *motor1,int *motor2,int *motor3,int *motor4)
{
    int Amplitude=7199-Dead_Zone;
    if(*motor1<-Amplitude) *motor1=-Amplitude;
    if(*motor1>Amplitude)  *motor1=Amplitude;
    if(*motor2<-Amplitude) *motor2=-Amplitude;
    if(*motor2>Amplitude)  *motor2=Amplitude;
    if(*motor3<-Amplitude) *motor2=-Amplitude;
    if(*motor3>Amplitude)  *motor2=Amplitude;
    if(*motor4<-Amplitude) *motor2=-Amplitude;
    if(*motor4>Amplitude)  *motor2=Amplitude;
}


/**************************************************************************************************************
*函数名:Turn_off()
*功能:关闭电机
*形参:(const float Angle):x轴角度值
*返回值:1:小车当前处于停止状态/0:小车当前处于正常状态
***************************************************************************************************************/
u8 FS_state;

u8 Turn_off(const float Angle)
{
    u8 temp;
    if(fabs(Angle)>68) {
        FS_state=1;
        temp=1;
        PWMA1=0;
        PWMA2=0;
        PWMB1=0;
        PWMB2=0;
        PWMC1=0;
        PWMC2=0;
        PWMD1=0;
        PWMD2=0;
    }
    else
        temp=0;
    FS_state=0;
    return temp;
}

/**************************************************************************************************************
*函数名:arm_acalculating()
*功能:二连杆机械臂解算
*形参；x，y增量
*返回值:两个连杆相对于水平面的角度
*************************************************************************************************************/

void arm_calculating(float x,float y,float *alpha,float *beta,float *gama) {
    const float a = 90,b = 90,c = 90;
    float alpha_temp,beta_temp;
//		x-=c*cos(*gama*3.1415926/180);
//		y-=c*sin(*gama*3.1415926/180);
    alpha_temp = atan(y/x)+acos ( (a*a+x*x+y*y-b*b)/2/a/sqrt(x*x+y*y));
    beta_temp = asin((y-a*sin(alpha_temp))/b);
    *alpha = alpha_temp/3.1415926*180;
    *beta = beta_temp/3.1415926*180;
}



/**************************************************************************************************************
*函数名:Set_PWM()
*功能:输出PWM控制电机
*形参；(int motor1):电机1对应的PWM值/(int motor2):电机2对应的PWM值
*返回值:无
*************************************************************************************************************/
void Set_PWM(int motor1,int motor2,int motor3,int motor4)
{
    if(abs(motor1) <5  && abs(motor2) <5 && abs(motor3) <5 &&abs(motor4) < 5) {
        PWMA1 = 0;
        PWMA2 = 0;
        PWMB1 = 0;
        PWMB2 = 0;
        PWMC1 = 0;
        PWMC2 = 0;
        PWMD1 = 0;
        PWMD2 = 0;

    }
    else {
        //a
        if(motor1>0)
        {
            PWMA1=Dead_Zone+(abs(motor1));
            PWMA2 =0;
        }
        else
        {
            PWMA1=0;
            PWMA2=Dead_Zone+(abs(motor1));
        }
        //b
        if(motor2<0)
        {
            PWMB1=Dead_Zone+(abs(motor2));
            PWMB2 =0;
        }
        else
        {
            PWMB1=0;
            PWMB2=Dead_Zone+(abs(motor2));
        }
        //c
        if(motor3<0)
        {
            PWMC1=Dead_Zone+(abs(motor3));
            PWMC2 =0;
        }
        else
        {
            PWMC1=0;
            PWMC2=Dead_Zone+(abs(motor3));
        }
        //d
        if(motor4<0)
        {
            PWMD1=Dead_Zone+(abs(motor4));
            PWMD2 =0;
        }
        else
        {
            PWMD1=0;
            PWMD2=Dead_Zone+(abs(motor4));
        }
    }



}

void Set_Servo(int s1,int s2,int s3, int s4) {
    //Servo_smooth(&s1,&s2,&s3,&s4);//1620 1850 1800
    int s1_max=2300,s1_min =800;
    if(s1>s1_max)
        s1=s1_max;
    if(s1<s1_min)
        s1=s1_min;

    int s2_max=2500,s2_min =600;
    if(s2>s2_max)
        s2=s2_max;
    if(s2<s2_min)
        s2=s2_min;

    int s3_max=2500,s3_min =500;
    if(s3>s3_max)
        s3=s3_max;
    if(s3<s3_min)
        s3=s3_min;

    int s4_max=2000,s4_min =600;
    if(s4>s4_max)
        s4=s4_max;
    if(s4<s4_min)
        s4=s4_min;

    servo1=s1;
    servo2=s2;
    servo3=s3;
    servo4=s4;

}
static int servo_last[4] = {1620,1850,1800,1500};

void Servo_smooth(int *s1,int *s2,int *s3,int *s4) {

    int servo_now[4];
    servo_now[0] = *s1;
    servo_now[1] = *s2;
    servo_now[2] = *s3;
    servo_now[3] = *s4;
    int servo_step = 20;
    //平滑处理
    for(int i =0; i<4; i++) {
        if(servo_now[i]>(servo_last[i]+servo_step)) servo_last[i] +=servo_step;
        else if(servo_now[i]<(servo_last[i]-servo_step)) servo_last[i] -=servo_step;
        else servo_last[i] = servo_now[i];
    }
    *s1 = servo_last[0];
    *s2 = servo_last[1];
    *s3 = servo_last[2];
    *s4 = servo_last[3];

}

/*
遥控摇杆平滑函数
传入参数4个摇杆数据
参数 step 最大增长步长
*/
int step = 10;//最大增长步长
int zone = 8;//摇杆操控死区
void rc_smooth(int *x,int *y,int *j,int *k){
	static int num[4];
	static int num_last[4];
	num[0] = *x;
	num[1] = *y;
	num[2] = *j;
	num[3] = *k;
	for (int i = 0;i<4;i++){
		if(num[i]-num_last[i]>step){
			num[i] = num_last[i]+step;
		}
		else if(abs(num[i]) < zone){
			num[i] = 0.6*num_last[i];
		}
		else if(num[i]-num_last[i]<-step){
			num[i] = num_last[i]-step;
		}
		num_last[i] = num[i];
	}
	*x = num[0];
	*y = num[1];
	*j = num[2];
	*k = num[3];
}

/*直接把下面的复制到main里*/
//void Motor_PWM_init(){
//	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
//  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
//}
