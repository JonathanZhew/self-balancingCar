/******************** (C) COPYRIGHT (2015)BST BALANCECAR **************************
 * �ļ���  ��main.c
**********************************************************************************/
#include "stm32f10x.h"
#include "mpu6050.h"
#include "i2c_mpu6050.h"
#include "motor.h"
#include "upstandingcar.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "i2c.h"
#include "outputdata.h"
#include "timer.h"
#include "UltrasonicWave.h"

/*
 * ��������main
 * ����  ��������
 */
int main(void)
{	
       
	
	
	Timerx_Init(5000,7199);				   //��ʱ��TIM1
	UltrasonicWave_Configuration(); 	   //��������ʼ������ IO�ڼ��ж�����			    

	USART1_Config();						//����1��ʼ�� ��λ��
	USART3_Config();						//����3��ʼ�� ������USART3������ͬIO��
	
	TIM2_PWM_Init();					   //PWM�����ʼ��
	MOTOR_GPIO_Config();				  //���IO�ڳ�ʼ��

	TIM3_External_Clock_CountingMode();	   //������������ⲿ�жϿ�PA7ʹ��TIM3��ʱ������Ϊ����������
	TIM4_External_Clock_CountingMode();	   //�ҵ����������ⲿ�жϿ�PB7ʹ��TIM4��ʱ������Ϊ����������
	
	i2cInit();							   //IIC��ʼ�� ���ڹҿ��������ϵ��豸ʹ��
	delay_nms(10);						   //��ʱ10ms
	MPU6050_Init();						   //MPU6050�����ǳ�ʼ��

	SysTick_Init();						  //SysTick������ʼ��
	
	CarUpstandInit();					  //С��ֱ��������ʼ��
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);	  
	 
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;	 //ʹ�����㷨ʱ��

	while (1)
	{
	  //chaoshengbo();
	  //gyro_z=(GetData(GYRO_ZOUT_H));
	  //	printf("\r\n---------������Z��ԭʼ����---------%d \r\n",GetData(GYRO_ZOUT_H));
      BluetoothControl();			 //��������
										    
   	}

}
