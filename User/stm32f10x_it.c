/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
 #include "stm32f10x_it.h"
 #include <stdio.h>
 #include "upstandingcar.h"
 #include "outputdata.h"
 #include "mpu6050.h"
 #include "UltrasonicWave.h"
 #include "stm32f10x_exti.h"
 

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

#if 1
void SysTick_Handler(void)				 //1ms��ʱ��
{  

	BST_u8MainEventCount++;				   //��ѭ������ֵ

	BST_u8SpeedControlPeriod++;     		 //�ٶ�ƽ������������ֵ
	SpeedControlOutput();   				 //�ٶ�PWM�������
	

	BST_u8DirectionControlPeriod++;		   //ת��ƽ������������ֵ
    DirectionControlOutput();			  //ת��PWM�������

	BST_u8turnPeriod++;					  //��תƽ������������ֵ
	turnfliteroutput();					  //��תPWM�������

	BST_u8trig++;						   //������ѭ������ֵ
	 
	if(BST_u8MainEventCount>=5)					 //���ܼ���ֵ=5������ѭ��ʱ��Ϊ5msʱ������������㺯��
	{
		BST_u8MainEventCount=0;					//ÿ5ms����ѭ�������ֵ����
		GetMotorPulse();						//������㺯��

		BST_u8trig++;							//������ѭ������ֵ
		 
		if(BST_u8trig>=12)	                    //6*5=30ms ÿ30ms��һ�γ��������
	  {
		  
		
		UltrasonicWave_StartMeasure();	   //���ó��������ͳ��� ��Trig�� <10us �ߵ�ƽ		 
		chaoshengbo();			       //���㳬����������  juli
		BST_u8trig=0;						   //ѭ������ 

	  }
					 
	}
	else if(BST_u8MainEventCount==1)		 //���ܼ���ֵ=1������ѭ��ʱ��Ϊ5msʱ����ȡ�����ǵĽǶ�״̬
	{
		MPU6050_Pose();						 //��ȡMPU6050�Ƕ�״̬

	}
	else if(BST_u8MainEventCount==2)	   //���ܼ���ֵ=2������ѭ��ʱ��Ϊ5msʱ���Ƕ�PD����PWM�����ÿ5ms����һ�Σ�����С��ƽ�⣩��С����PWM���
	{
	 
		AngleControl();					  //�Ƕ�PD����PWNM���

		MotorOutput();					  //С����PWM���
	
		
	}
	else if(BST_u8MainEventCount==3)		  //���ܼ���ֵ=3������ѭ��ʱ��Ϊ5msʱ����ģ���������ж�
	{
		BST_u8SpeedControlCount++;			  //С���ٶȿ��Ƶ��ü���ֵ
        if(BST_u8SpeedControlCount>=10)       //������ֵΪ10ʱ������ϵͳ����50msʱ��(ÿ10���Ƕ�PWM���������1���ٶ�PWM����������ܱ����ٶ�PID��������ŽǶ�PID������Ӷ�Ӱ��С��ƽ��)
        {
		
          SpeedControl();                     //��ģ�ٶȿ��ƺ���   ÿ50ms����һ��
          BST_u8SpeedControlCount=0;		  //С���ٶȿ��Ƶ��ü���ֵ����
          BST_u8SpeedControlPeriod=0;		  //ƽ���������ֵ����
        }
	}
	else if(BST_u8MainEventCount==4)		    //���ܼ���ֵ=3������ѭ��ʱ��Ϊ5msʱ������ת�����ת����
	{	 
	     	BST_u8DirectionControlCount++;		 //ת�������ü���ֵ
	   		BST_u8turnCount++;					 //��ת�������ü���ֵ

		      if(BST_u8turnCount>=3)            //ÿ15ms������ת����
		      {
			    BST_u8turnCount=0;			   //����ֵ����
		        BST_u8turnPeriod=0;			   //ƽ���������ֵ����
			     turn();					   //��ת����
			  }

			if(BST_u8DirectionControlCount>=5)//ÿ25ms����ת����
	        {
			  BST_u8DirectionControlCount=0;   //����ֵ����
	          BST_u8DirectionControlPeriod=0;  //ƽ���������ֵ����	
		      DirectionControl();			   //ת����
			 
		    }
	}	
	      
}	   
		


  

 
 


#endif
#if 0
void SysTick_Handler(void)
{  	
	
	//SampleInputVoltage();
	MPU6050_Pose();
	AngleControl();
	GetMotorPulse();
	SpeedControl();                  
	DirectionControl();
	MotorOutput();
	#if 1

		g_u8LEDCount++;
		if(g_u8LEDCount>=100)
		{
			g_u8LEDCount=0;
			GPIO_ResetBits(GPIOB, GPIO_Pin_3);
		}
		else 
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_3);
		}
#endif         
#if 0  /*������ Ԥ��������*/
   OutData[0] = g_fCarAngle;
   OutData[1] = g_fGravityAngle;
   OutData[2] = g_fGyroAngleSpeed ;
   OutData[3] = g_iAccelInputVoltage_X_Axis;
   
   OutPut_Data();
#endif	  
	
}
#endif
void USART3_IRQHandler(void)
{
/*	
	u8 ucBluetoothValue;
	if(USART3->SR&(1<<5))//���յ�����
	{	 
		ucBluetoothValue=USART1->DR; 

    }
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 	
	    ucBluetoothValue = USART_ReceiveData(USART3);
	  	switch (ucBluetoothValue)
	{
	  case 0x02 : g_fBluetoothSpeed =   500 ; break;	   //ǰ��
	  case 0x01 : g_fBluetoothSpeed = (-500);  break;	   //����
	  case 0x03 : g_fBluetoothDirection =   100 ;  break;//��ת
	  case 0x04 : g_fBluetoothDirection = (-100);  break;//��ת
	  case 0x05 : g_fBluetoothSpeed =   1000 ; break ;
	  case 0x06 : g_fBluetoothSpeed = (-1000); break ;
	  case 0x07 : g_fBluetoothDirection =   500 ;  break;
	  case 0x08 : g_fBluetoothDirection = (-500);  break;
	  default : g_fBluetoothSpeed = 0; g_fBluetoothDirection = 0;break;
	}
	USART_ClearITPendingBit(USART3, USART_IT_RXNE); //����жϱ�־
	} 
*/	 
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
