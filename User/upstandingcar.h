#ifndef __UPSTANDINGCAR_H
#define __UPSTANDINGCAR_H
#include "stm32f10x.h"

/**********�Ƕȿ��ƺ궨��**********/									
#define    CAR_ZERO_ANGLE (0)		 //��ʼС������ṹ����С�����ڴ�ֱ����Ϊ��Ƕȵ����������Ҫ������������ֱ�����Ƕ�ֵ��

/******�ٶȿ�����غ궨��******/
#define CAR_POSITION_SET      0
#define CAR_SPEED_SET         0
#define MOTOR_LEFT_SPEED_POSITIVE  (BST_fLeftMotorOut >0)
#define MOTOR_RIGHT_SPEED_POSITIVE (BST_fRightMotorOut>0)
#define OPTICAL_ENCODE_CONSTANT  448	//������̶̿Ȳ�
#define SPEED_CONTROL_PERIOD	 50	    //�ٶȻ���������
#define CAR_SPEED_CONSTANT		(1000.0/(float)SPEED_CONTROL_PERIOD/(float)OPTICAL_ENCODE_CONSTANT)
#define CAR_POSITION_MAX	(MOTOR_OUT_MAX*10)//500////20
#define CAR_POSITION_MIN	(MOTOR_OUT_MIN*10) //-500//
/******���������غ궨��******/
#define MOTOR_OUT_DEAD_VAL       0	   //����ֵ8
#define MOTOR_OUT_MAX           250	   //ռ�ձ������ֵ
#define MOTOR_OUT_MIN         (-250)   //ռ�ձȸ����ֵ

#define	MOTOR_LEFT_AIN1_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_15))  //PB15��Ӧ�������оƬ���ƽŶ��壬AIN1Ϊ��ʱ��PB15��Ϊ0
#define	MOTOR_LEFT_AIN1_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_15))	  //PB15��Ӧ�������оƬ���ƽŶ��壬AIN1Ϊ��ʱ��PB15��Ϊ1
#define	MOTOR_LEFT_AIN2_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_14))  //PB14��Ӧ�������оƬ���ƽŶ��壬AIN2Ϊ��ʱ��PB14��Ϊ0
#define	MOTOR_LEFT_AIN2_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_14))	  //PB14��Ӧ�������оƬ���ƽŶ��壬AIN2Ϊ��ʱ��PB14��Ϊ1

#define	MOTOR_RIGHT_BIN1_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_12))//PB12��Ӧ�������оƬ���ƽŶ��壬BIN1Ϊ��ʱ��PB12��Ϊ0
#define	MOTOR_RIGHT_BIN1_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_12))		//PB12��Ӧ�������оƬ���ƽŶ��壬BIN1Ϊ��ʱ��PB12��Ϊ1
#define	MOTOR_RIGHT_BIN2_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_13))//PB13��Ӧ�������оƬ���ƽŶ��壬BIN2Ϊ��ʱ��PB13��Ϊ0
#define	MOTOR_RIGHT_BIN2_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_13))		//PB13��Ӧ�������оƬ���ƽŶ��壬BIN2Ϊ��ʱ��PB13��Ϊ1

extern float BST_fCarAngle;					//extern���ڱ������ߺ���ǰ���Ա�ʾ�������ߺ����Ķ����ڱ���ļ��У���ʾ�����������˱�������ʱ��������ģ����Ѱ���䶨�塣
extern float BST_fBluetoothSpeed;
extern float BST_fBluetoothDirectionR;
extern float BST_fBluetoothDirectionL;
extern u8 BST_u8MainEventCount;
extern u8 BST_u8SpeedControlCount;
extern float BST_fSpeedControlOut;
extern float  BST_fAngleControlOut;
extern u8 BST_u8SpeedControlPeriod;
extern u8 BST_u8DirectionControlPeriod;
extern u8 BST_u8DirectionControlCount;
extern u8 BST_u8LEDCount; 
extern u8 BST_u8trig;
extern u8 btcount;
extern u8 BST_u8turnPeriod;
extern u8 BST_u8turnCount;
extern int chose;


void delay_nms(u16 time);
void CarUpstandInit(void);
//void SampleInputVoltage(void);
void AngleControl(void)	 ;
void MotorOutput(void);
void SpeedControl(void);
void BluetoothControl(void)	;
void GetMotorPulse(void);
void SpeedControlOutput(void);
void DirectionControlOutput(void);
void DirectionControl(void); 
void chaoshengbo(void);
void gfcsbOutput(void);
void csbcontrol(void);
void turn(void);
void turnfliteroutput(void);
void InitMPU6050(void);
#endif
