/******************** (C) COPYRIGHT (2015)BST BALANCECAR **************************
 * 文件名  ：main.c
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
 * 函数名：main
 * 描述  ：主函数
 */
int main(void)
{	
       
	
	
	Timerx_Init(5000,7199);				   //定时器TIM1
	UltrasonicWave_Configuration(); 	   //超声波初始化设置 IO口及中断设置			    

	USART1_Config();						//串口1初始化 上位机
	USART3_Config();						//串口3初始化 蓝牙与USART3公用相同IO口
	
	TIM2_PWM_Init();					   //PWM输出初始化
	MOTOR_GPIO_Config();				  //电机IO口初始化

	TIM3_External_Clock_CountingMode();	   //左电机脉冲输出外部中断口PA7使用TIM3定时器用作为脉冲数计算
	TIM4_External_Clock_CountingMode();	   //右电机脉冲输出外部中断口PB7使用TIM4定时器用作为脉冲数计算
	
	i2cInit();							   //IIC初始化 用于挂靠在总线上的设备使用
	delay_nms(10);						   //延时10ms
	MPU6050_Init();						   //MPU6050陀螺仪初始化

	SysTick_Init();						  //SysTick函数初始化
	
	CarUpstandInit();					  //小车直立参数初始化
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);	  
	 
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;	 //使能总算法时钟

	while (1)
	{
	  //chaoshengbo();
	  //gyro_z=(GetData(GYRO_ZOUT_H));
	  //	printf("\r\n---------陀螺仪Z轴原始数据---------%d \r\n",GetData(GYRO_ZOUT_H));
      BluetoothControl();			 //蓝牙控制
										    
   	}

}
