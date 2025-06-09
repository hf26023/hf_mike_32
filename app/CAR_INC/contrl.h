#ifndef _CONTRIL_H_
#define _CONTRIL_H_

#include "sys.h"


//机械0点
#define Mechanical_balance -3


#define PWMA1   TIM2->CCR1//左上
#define PWMA2   TIM2->CCR2

#define PWMB1   TIM2->CCR3//左下
#define PWMB2   TIM2->CCR4

#define PWMC1   TIM3->CCR1//右下
#define PWMC2   TIM3->CCR2

#define PWMD1   TIM3->CCR3//右上
#define PWMD2   TIM3->CCR4

#define servo1 TIM4->CCR1
#define servo2 TIM4->CCR2
#define servo3 TIM4->CCR3
#define servo4 TIM4->CCR4



extern volatile int Encoder_Left,Encoder_Right;		      //编码器左右速度值

struct pid_arg{
	
	float Velocity_Kp;
	float Velocity_Ki;
	float Velocity_Kd;
	
	float  Turn_Kp;
	float  Turn_Ki;
	float  Turn_Kd;

};
extern struct pid_arg PID;


int Vertical_turn_PD(float taget_yaw,float yaw,float gyro);
void arm_calculating(float x ,float y ,float *alpha,float *beta,float *gama );

void PWM_Limiting(int *motor1,int *motor2,int *motor3,int *motor4);
u8 Turn_off(const float Angle);

void Set_PWM(int motor1,int motor2,int motor3,int motor4);
void Set_Servo(int s1,int s2,int s3, int s4);
//void Servo_smooth(int s1,int s2,int s3,int s4);
void Servo_smooth(int *s1,int *s2,int *s3,int *s4);
void rc_smooth(int *x,int *y,int *j,int *k);
#endif
