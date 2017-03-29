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
void SysTick_Handler(void)				 //1ms定时器
{  

	BST_u8MainEventCount++;				   //总循环计数值

	BST_u8SpeedControlPeriod++;     		 //速度平滑输出计算比例值
	SpeedControlOutput();   				 //速度PWM输出函数
	

	BST_u8DirectionControlPeriod++;		   //转向平滑输出计算比例值
    DirectionControlOutput();			  //转向PWM输出函数

	BST_u8turnPeriod++;					  //旋转平滑输出计算比例值
	turnfliteroutput();					  //旋转PWM输出函数

	BST_u8trig++;						   //超声波循环计数值
	 
	if(BST_u8MainEventCount>=5)					 //若总计数值=5，即总循环时间为5ms时，调用脉冲计算函数
	{
		BST_u8MainEventCount=0;					//每5ms，总循环体计数值清零
		GetMotorPulse();						//脉冲计算函数

		BST_u8trig++;							//超声波循环计数值
		 
		if(BST_u8trig>=12)	                    //6*5=30ms 每30ms做一次超声波检测
	  {
		  
		
		UltrasonicWave_StartMeasure();	   //调用超声波发送程序 给Trig脚 <10us 高电平		 
		chaoshengbo();			       //计算超声波测距距离  juli
		BST_u8trig=0;						   //循环清零 

	  }
					 
	}
	else if(BST_u8MainEventCount==1)		 //若总计数值=1，即总循环时间为5ms时，获取陀螺仪的角度状态
	{
		MPU6050_Pose();						 //获取MPU6050角度状态

	}
	else if(BST_u8MainEventCount==2)	   //若总计数值=2，即总循环时间为5ms时，角度PD控制PWM输出（每5ms调用一次，保持小车平衡），小车总PWM输出
	{
	 
		AngleControl();					  //角度PD控制PWNM输出

		MotorOutput();					  //小车总PWM输出
	
		
	}
	else if(BST_u8MainEventCount==3)		  //若总计数值=3，即总循环时间为5ms时，车模车速融入判断
	{
		BST_u8SpeedControlCount++;			  //小车速度控制调用计数值
        if(BST_u8SpeedControlCount>=10)       //当计数值为10时，即总系统运行50ms时候(每10个角度PWM输出中融入1个速度PWM输出，这样能保持速度PID输出不干扰角度PID输出，从而影响小车平衡)
        {
		
          SpeedControl();                     //车模速度控制函数   每50ms调用一次
          BST_u8SpeedControlCount=0;		  //小车速度控制调用计数值清零
          BST_u8SpeedControlPeriod=0;		  //平滑输出比例值清零
        }
	}
	else if(BST_u8MainEventCount==4)		    //若总计数值=3，即总循环时间为5ms时，调用转向和旋转函数
	{	 
	     	BST_u8DirectionControlCount++;		 //转向函数调用计数值
	   		BST_u8turnCount++;					 //旋转函数调用计数值

		      if(BST_u8turnCount>=3)            //每15ms调用旋转函数
		      {
			    BST_u8turnCount=0;			   //计数值清零
		        BST_u8turnPeriod=0;			   //平滑输出比例值清零
			     turn();					   //旋转函数
			  }

			if(BST_u8DirectionControlCount>=5)//每25ms调用转向函数
	        {
			  BST_u8DirectionControlCount=0;   //计数值清零
	          BST_u8DirectionControlPeriod=0;  //平滑输出比例值清零	
		      DirectionControl();			   //转向函数
			 
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
#if 0  /*调试用 预编译命令*/
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
	if(USART3->SR&(1<<5))//接收到数据
	{	 
		ucBluetoothValue=USART1->DR; 

    }
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 	
	    ucBluetoothValue = USART_ReceiveData(USART3);
	  	switch (ucBluetoothValue)
	{
	  case 0x02 : g_fBluetoothSpeed =   500 ; break;	   //前进
	  case 0x01 : g_fBluetoothSpeed = (-500);  break;	   //后退
	  case 0x03 : g_fBluetoothDirection =   100 ;  break;//左转
	  case 0x04 : g_fBluetoothDirection = (-100);  break;//右转
	  case 0x05 : g_fBluetoothSpeed =   1000 ; break ;
	  case 0x06 : g_fBluetoothSpeed = (-1000); break ;
	  case 0x07 : g_fBluetoothDirection =   500 ;  break;
	  case 0x08 : g_fBluetoothDirection = (-500);  break;
	  default : g_fBluetoothSpeed = 0; g_fBluetoothDirection = 0;break;
	}
	USART_ClearITPendingBit(USART3, USART_IT_RXNE); //清除中断标志
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
