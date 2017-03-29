
#include "upstandingcar.h"
#include "I2C_MPU6050.h"
#include "MOTOR.h"
#include "led.h"
#include "USART.H"
#include "MPU6050.H"
#include "UltrasonicWave.h"

u8 BST_u8LEDCount; 
u8 BST_u8MainEventCount;						  //��ѭ���жϼ���  ��SysTick_Handler(void)��ʹ�� ÿ1ms��1
u8 BST_u8SpeedControlCount;						  //�ٶȿ���ѭ������  ��SysTick_Handler(void)��ʹ�� ÿ5ms��1
u8 BST_u8SpeedControlPeriod;
u8 BST_u8DirectionControlPeriod;
u8 BST_u8DirectionControlCount;					  //ת�����ѭ������  ��SysTick_Handler(void)��ʹ�� ÿ5ms��1 
u8 BST_u8trig;
u8 BST_u8turnCount;								  //��ת����ѭ������  ��SysTick_Handler(void)��ʹ�� ÿ5ms��1	 
u8 BST_u8turnPeriod;							  //��תƽ���������ֵ
u8   btcount;
//s32 g_liAccAdd;
//s32 g_liGyroAdd;
/******������Ʋ���******/
float BST_fSpeedControlOut;						   //�ٶȿ���PWM
float BST_fSpeedControlOutOld;
float BST_fSpeedControlOutNew;
float BST_fAngleControlOut;
float BST_fLeftMotorOut;
float BST_fRightMotorOut;

float BST_fCarAngle;						 //�Ƕȿ���PWM
float gyro_z;

/******�ٶȿ��Ʋ���******/
s16   BST_s16LeftMotorPulse;					  //����������
s16	  BST_s16RightMotorPulse;					   //�ҵ��������

s32   BST_s32LeftMotorPulseOld;
s32   BST_s32RightMotorPulseOld;
s32   BST_s32LeftMotorPulseSigma;				  //50ms��������ֵ
s32   BST_s32RightMotorPulseSigma;				 //50ms�ҵ������ֵ

float BST_fCarSpeed;							 //�������̵ó��ĳ���
float BST_fCarSpeedOld;
//float BST_fCarSpeedOld;
float BST_fCarPosition;						   //��������ͨ������õ���С��λ��
//float fSpeedCtrlPeriod=100.0;
//���²ο�Ϊ�ص���Բο���ͬ��ص�ѹ�йأ������õ��ٵ���
/*-----�ǶȻ����ٶȻ�PID���Ʋ���-----*/
float  BST_fCarAngle_P =29;//2 4 8 10 15 20 14   	 //����Сʱ�����Ұڣ�����ʱ����  ����������ܹ�վ�� P=20�����ڸ�С�����˶�����ʹ��
float  BST_fCarAngle_D = 0.0389;	// 0.001 0.002 0.004 0.008 0.0010 0.011	 ��Сʱ��Ӧ��������ʱ�����
float  BST_fCarSpeed_P = 5.1;//3.7;// 1 2 4 8 10 �ص�8 5 	 ���ֵ�Сһ���ȶ� 5.5����õ�
float  BST_fCarSpeed_I = 0.0898;//0.081;//0.002 0.004 0.010 0.02  0.08 0.10 0.2 �ص�����	��ѹ��ʱ��С�������������Ʋ��ȣ��ᵹ
/******�������Ʋ���******/																	
float BST_fBluetoothSpeed;						//�������Ƴ���
float BST_fBluetoothDirectionOld;			    //����ƽ���������ʹ��
float BST_fBluetoothDirectionNew;			    //����ƽ���������ʹ��
float BST_fBluetoothDirectionOut;				 //����ƽ���������ʹ��
float BST_fBluetoothDirectionSL;			    //��ת��־λ  ����PWMת�����ʹ���ж������ʽ ����Ҫ��־λ
float BST_fBluetoothDirectionSR;			   //��ת��־λ	  ����PWMת�����ʹ���ж������ʽ ����Ҫ��־λ
/************��ת*****************/
float BST_fBluetoothDirectionL;				   //����ת��־λ  ����PWM��ת���ʹ���ж������ʽ ����Ҫ��־λ
float BST_fBluetoothDirectionR;				   //����ת��־λ  ����PWM��ת���ʹ���ж������ʽ ����Ҫ��־λ
float BST_speednow ;						   //����ƽ���������ʹ��
float BST_speedold;							  //����ƽ���������ʹ��
float BST_speedout;							  //����ƽ���������ʹ��
int chose;									   //��ת��־λ
/******������********/
float BST_fchaoshengbooutput;				  //������PWM���
float fchaoshengbo;							   //�����������
float fchaoshengboOld;						  //����ƽ���������ʹ��
float fchaoshengboNew;						  //����ƽ���������ʹ��
float fchaoshengboPeriod;					 //����ƽ���������ʹ��
float juli;									 //����������


/**********��ʱ�Ӻ���****************/
void delay_nms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  //�Լ�����
      while(i--) ;    
   }
}
/***********************************/


/***************************************************************
** ��������: CarUpstandInit
** ��������: ȫ�ֱ�����ʼ������

***************************************************************/
void CarUpstandInit(void)
{
	
	
	BST_s16LeftMotorPulse = BST_s16RightMotorPulse = 0;					  //��������ֵ   ��ʼ��
//	BST_s32LeftMotorPulseOld = BST_s32RightMotorPulseOld = 0;			  //����		 ��ʼ��
	BST_s32LeftMotorPulseSigma = BST_s32RightMotorPulseSigma = 0;		  //����������	 ��ʼ��


	BST_fCarSpeed = BST_fCarSpeedOld = 0;								   //ƽ��С������	��ʼ��
	BST_fCarPosition = 0;												  //ƽ��С��λ����	��ʼ��
	BST_fCarAngle    = 0;												  //ƽ��С������	��ʼ��


	BST_fAngleControlOut = BST_fSpeedControlOut = BST_fBluetoothDirectionOut = 0;	//�Ƕ�PWM������PWM����������PWM	 ��ʼ��
	BST_fLeftMotorOut    = BST_fRightMotorOut   = 0;								//���ҳ���PWM���ֵ 			 ��ʼ��
	BST_fBluetoothSpeed  = 0;														//�������Ƴ���ֵ                 ��ʼ��
	BST_fBluetoothDirectionL =BST_fBluetoothDirectionR= 0;						    //��������������ת��־λ         ��ʼ��
	BST_fBluetoothDirectionSL =BST_fBluetoothDirectionSR= 0;						//������������ת���־λ         ��ʼ��
	BST_fBluetoothDirectionNew = BST_fBluetoothDirectionOld = 0;				    //����������                     ��ʼ��
	chose=0;																		//����������תPWM�����־λ      ��ʼ��        

    btcount=0;																		//��������ת��PWM�����־λ      ��ʼ��
	BST_speednow=BST_speedout=BST_speedold=0;									    //����������תƽ��PWM�������ֵ  ��ʼ��
	//BST_u8turnPeriod= BST_u8turnCount=0;

  	BST_u8MainEventCount=0;															//����5ms��ʱ���ӳ���SysTick_Handler(void)�����жϼ���λ
	BST_u8SpeedControlCount=0;														//����5ms��ʱ���ӳ���SysTick_Handler(void)��50ms�ٶ�ƽ���������λ
    BST_u8SpeedControlPeriod=0;														//����5ms��ʱ���ӳ���SysTick_Handler(void)��50ms�ٶ�ƽ���������λ

	BST_fchaoshengbooutput=fchaoshengbo=0;											//����5ms��ʱ���ӳ���SysTick_Handler(void)�г�����ƽ���������λ
	fchaoshengboOld=0;
	fchaoshengboNew=0;
	fchaoshengboPeriod=30;
	

}

/***************************************************************
** ��������: SampleInputVoltage
** ��������: ��������             

***************************************************************/

/***************************************************************
** ��������: AngleControl
** ��������: �ǶȻ����ƺ���

***************************************************************/
void AngleControl(void)	 
{

	BST_fCarAngle = Roll - CAR_ZERO_ANGLE;													   //DMP ROLL��������Ƕ���Ԥ��С����б�Ƕ�ֵ�Ĳ�ó��Ƕ�
	BST_fAngleControlOut =  BST_fCarAngle * BST_fCarAngle_P + gyro[0] * BST_fCarAngle_D ;	  //�Ƕ�PD����							   
	  
																							  
	 
}
#define MAX_PWM 255
/***************************************************************
** ��������: SetMotorVoltageAndDirection
** ��������: ���ת��             

***************************************************************/
void SetMotorVoltageAndDirection(s16 s16LeftVoltage,s16 s16RightVoltage)
{
	u16 u16LeftMotorValue;
	u16 u16RightMotorValue;
	
    if(s16LeftVoltage<0)										 //������PWM���Ϊ��ʱ PB14��Ϊ�� PB15��Ϊ�� ��PB14 15 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
    {	
	  GPIO_SetBits(GPIOB, GPIO_Pin_15 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 );
      s16LeftVoltage = (-s16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    	 //������PWM���Ϊ��ʱ PB14��Ϊ�� PB15��Ϊ�� ��PB14 15 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 ); 
      s16LeftVoltage = s16LeftVoltage;
    }

    if(s16RightVoltage<0)
    {															 //���ҵ��PWM���Ϊ��ʱ PB12��Ϊ�� PB13��Ϊ�� ��PB12 13 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
      GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );
      s16RightVoltage = (-s16RightVoltage);
    }
    else														//���ҵ��PWM���Ϊ��ʱ PB12��Ϊ�� PB13��Ϊ�� ��PB12 13 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
    {
	GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );	
      
      s16RightVoltage = s16RightVoltage;
    }
	 u16RightMotorValue= (u16)s16RightVoltage;
	 u16LeftMotorValue = (u16)s16LeftVoltage;

	if(u16RightMotorValue>MAX_PWM)  							   //�趨���ҵ��PWM����������Ϊ255
	{
		u16RightMotorValue=MAX_PWM;
	}
	if(u16LeftMotorValue>MAX_PWM)									//�趨���ҵ��PWM����������Ϊ255
	{
	   u16LeftMotorValue=MAX_PWM;
	}
    //TIM2_PWM_CHANGE(u16RighttMotorValue,u16LeftMotorValue) ;
	//TIM2->CCR3 = 	u16RightMotorValue ; 
   	//TIM2->CCR4 =    u16LeftMotorValue; 
	TIM_SetCompare3(TIM2,u16RightMotorValue);			  //TIM2�� u16RightMotorValue�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
	TIM_SetCompare4(TIM2,u16LeftMotorValue);			  //TIM3�� u16LeftMotorValue�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�

#if 1	 /*�жϳ����Ƿ������������*/

	if(BST_fCarAngle > 180 || BST_fCarAngle < (-180))
	{
		TIM_SetCompare3(TIM2,0);
		TIM_SetCompare4(TIM2,0);  
	}

#endif
}

/***************************************************************
** ��������: MotorOutput
** ��������: ����������
             ��ֱ�����ơ��ٶȿ��ơ�������Ƶ���������е���,����
			 �����������������������������

***************************************************************/
void MotorOutput(void)																					  //���PWM�������
{	   
	if(btcount==1)																						  //��ת�ж� btcountΪ1ʱ ������תPWMƽ�����
	{
    BST_fLeftMotorOut  = BST_fAngleControlOut - BST_fSpeedControlOut -BST_speedout;
	 BST_fRightMotorOut = BST_fAngleControlOut - BST_fSpeedControlOut+BST_speedout;
	btcount=0;
	}

	else if(chose==1)																						//ת���ж� btcountΪ1ʱ ����ת��PWMƽ�����
	{
	//delay_nms(1000)	;
     BST_fLeftMotorOut  = BST_fAngleControlOut - BST_fSpeedControlOut +BST_fBluetoothDirectionNew;			//����ת��PWM�����ں�ƽ��Ƕȡ��ٶ����	
     BST_fRightMotorOut = BST_fAngleControlOut - BST_fSpeedControlOut-BST_fBluetoothDirectionNew;			//�ҵ��ת��PWM�����ں�ƽ��Ƕȡ��ٶ����
	 chose=0;
	 }
	else{																									//ƽ��PWM���
	   BST_fLeftMotorOut  = BST_fAngleControlOut - BST_fSpeedControlOut ;								   //������תPWM�����ں�ƽ��Ƕȡ��ٶ����
     BST_fRightMotorOut = BST_fAngleControlOut - BST_fSpeedControlOut;										//�ҵ����תPWM�����ں�ƽ��Ƕȡ��ٶ����
	
	} 
	/*������ʹ�����ֹ����PWM��Χ*/
		
	if((s16)BST_fLeftMotorOut  > MOTOR_OUT_MAX)	BST_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((s16)BST_fLeftMotorOut  < MOTOR_OUT_MIN)	BST_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((s16)BST_fRightMotorOut > MOTOR_OUT_MAX)	BST_fRightMotorOut = MOTOR_OUT_MAX;
	if((s16)BST_fRightMotorOut < MOTOR_OUT_MIN)	BST_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((s16)BST_fLeftMotorOut,(s16)BST_fRightMotorOut);
}

void GetMotorPulse(void)              //�ɼ�����ٶ�����
{ 
  uint16_t u16TempLeft;
  uint16_t u16TempRight;

  u16TempLeft = TIM_GetCounter(TIM3);   //  TIM3��ʱ���������
  u16TempRight= TIM_GetCounter(TIM4);	//	 TIM4��ʱ���������
  TIM3->CNT = 0;
  TIM4->CNT = 0;   //����
  BST_s16LeftMotorPulse=u16TempLeft;
  BST_s16RightMotorPulse=u16TempRight;

  if(!MOTOR_LEFT_SPEED_POSITIVE)  										   //�����ж�	��������ж�
  {
  	BST_s16LeftMotorPulse  = (-BST_s16LeftMotorPulse) ; 				  //��������ת��������Ϊ����
  }
  if(!MOTOR_RIGHT_SPEED_POSITIVE) 
  {
  	BST_s16RightMotorPulse = (-BST_s16RightMotorPulse);					 //���ҵ����ת��������Ϊ����
  }

  BST_s32LeftMotorPulseSigma +=  BST_s16LeftMotorPulse;		 //����ֵ���� 50ms����ֵ
  BST_s32RightMotorPulseSigma += BST_s16RightMotorPulse; 	 //����ֵ���� 50ms����ֵ
}
/***************************************************************
** ��������: SpeedControl
** ��������: �ٶȻ����ƺ���
***************************************************************/

void SpeedControl(void)
{

	BST_fCarSpeed = (BST_s32LeftMotorPulseSigma  + BST_s32RightMotorPulseSigma ) * 0.5 ;		  //���ҵ��������ƽ��ֵ��ΪС����ǰ����
    BST_s32LeftMotorPulseSigma =BST_s32RightMotorPulseSigma = 0;	  //ȫ�ֱ��� ע�⼰ʱ����
	BST_fCarSpeed = 0.7 * BST_fCarSpeedOld + 0.3 * BST_fCarSpeed ;		//�ٶ�һ���˲�
	BST_fCarSpeedOld = BST_fCarSpeed;
	BST_fCarSpeed *= CAR_SPEED_CONSTANT;	 //��λ��ת/��
	BST_fCarPosition += BST_fCarSpeed; 		 //·��  ���ٶȻ���	
	BST_fCarPosition += BST_fBluetoothSpeed;   //�ں����������ٶ�
	BST_fCarPosition +=	fchaoshengbo;		   //�ںϳ����������ٶ�
	//������������//
	if((s32)BST_fCarPosition > CAR_POSITION_MAX)    BST_fCarPosition = CAR_POSITION_MAX;
	if((s32)BST_fCarPosition < CAR_POSITION_MIN)    BST_fCarPosition = CAR_POSITION_MIN;
	
	BST_fSpeedControlOutOld = BST_fSpeedControlOutNew;
																								  //�ٶ�PI�㷨 �ٶ�*P +λ��*I=�ٶ�PWM���
	BST_fSpeedControlOutNew = (CAR_SPEED_SET - BST_fCarSpeed) * BST_fCarSpeed_P +\
		(CAR_POSITION_SET - BST_fCarPosition) * BST_fCarSpeed_I; 		  

}
void SpeedControlOutput(void)																	 //�ٶ�ƽ��������� 
{
  float fValue;
  fValue = BST_fSpeedControlOutNew - BST_fSpeedControlOutOld ;
  BST_fSpeedControlOut = fValue * (BST_u8SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + BST_fSpeedControlOutOld; 	//ת��ƽ��������㹫ʽ
  
}


/***************************************************************
** ��������: BluetoothControl
** ��������: �������ƺ���
             �ֻ���������
			 ǰ��0x01    ��0x02
             ��0x04    �ң�0x03
             ֹͣ��0x07
             ���ܼ�������ת��
             ����ת:0x05      ����ת��0x06
           	 ͣת��0x07
** �䡡��:   
** �䡡��:   
***************************************************************/
void BluetoothControl(void)	 								 //��������
{
	u8 u8BluetoothValue;
	
	while(USART_GetFlagStatus(USART3,USART_FLAG_RXNE)!=SET);	//�ȴ�USART3�Ľ�������Ϊ��

	u8BluetoothValue =USART3->DR;
	USART_ClearFlag(USART3,USART_FLAG_RXNE); 
		
	switch (u8BluetoothValue)
	{
	  case 0x01 : BST_fBluetoothSpeed =   100 ; break;	   //��ǰ�ٶ� 250 
	  case 0x02 : BST_fBluetoothSpeed = (-90);  break;	   //�����ٶ� -250
	  case 0x04 : BST_fBluetoothDirectionSL = 1; break;//��ת
	  case 0x03 : BST_fBluetoothDirectionSR = 1; break;//��ת
	  case 0x05 : BST_fBluetoothDirectionL = 1; break ;//����
	  case 0x06 : BST_fBluetoothDirectionR = 1; break ;//����ת
	  case 0x07 : BST_fBluetoothDirectionL  =0; BST_fBluetoothDirectionR = 0;  break; //ͣ
	  case 0x08 : BST_fBluetoothDirectionSL =0; BST_fBluetoothDirectionSR = 0; break; //ͣ��ת
	  case 0x09 :  BST_fBluetoothSpeed =   0 ;  break;
	  default : BST_fBluetoothSpeed = 0; BST_fBluetoothDirectionL=BST_fBluetoothDirectionR = 0;BST_fBluetoothDirectionSR=BST_fBluetoothDirectionSL=0;break;
	}
	
}


/***********************����PWM��� ת��*************************/
void DirectionControl(void)											  //ת����
{


static float temp ,turnconvert,speedtarget,temp2;
static  int turncount,speedlimit=80;

	BST_fBluetoothDirectionNew=BST_fBluetoothDirectionOld ;

   if(gyro[2]>32768) gyro[2]-=65536;						//ǿ��ת��������ת�� 2Gת����1G
	chose=0;

   if(BST_fBluetoothDirectionSL==1|BST_fBluetoothDirectionSR==1)	  //����ң���Ƿ���ָ���ж�
{	
   chose=1;
    if(++turncount==1)
    { 
	temp  = BST_s16LeftMotorPulse+ BST_s16RightMotorPulse;		//��ʼ�ٶ� ��ת��ǰ�����ݳ�ʼ�ٶȣ��ٸ�������ٶȡ�
	if(temp<0) temp2=-temp;									   //��ǰС���˶������ϵĲ�������
    turnconvert=400/temp2;									   //��ʵ�ʵ��Թ�����ȷ������ת��ʱ�����ȶ��Ծ�����
    }

	if(turnconvert<3)turnconvert=5;
    if(turnconvert>5)turnconvert=10;	
   
}
  else
{																												 	
 	turnconvert=1;
	BST_speedout=0;
    turncount=0;
	speedtarget=0; 
	temp=0;
}

 if(BST_fBluetoothDirectionSL==1) speedtarget+=turnconvert;

	  
 else if(BST_fBluetoothDirectionSR==1) speedtarget-=turnconvert;
 
      else speedtarget=0; 	 
    
if(speedtarget>speedlimit) speedtarget=speedlimit;						//ת���ٶȷ�ֵ���� 
if(speedtarget<-speedlimit) speedtarget=-speedlimit;	   
   	 
	BST_fBluetoothDirectionNew=-speedtarget*19-gyro[2]*0.4;			   //ת��PD���ơ�


}

void DirectionControlOutput(void)									   //ת��ƽ�����
{
  float fValue;
 

  fValue = BST_fBluetoothDirectionNew - BST_fBluetoothDirectionOld;
  BST_fBluetoothDirectionOut = fValue *(BST_u8DirectionControlPeriod + 1) /25 + BST_fBluetoothDirectionOld; //ƽ��������㹫ʽ 


}
/***************************��ת����************************************/



void turn(void)								                        // ��ת����
{
   static float temp ,turnconvert,speedtarget,temp1,temp2,temp3;
   int turncount,speedlimit=100;

   btcount=0;	
   BST_speedold=BST_speednow;
   if(gyro[2]>32768) gyro[2]-=65536;

   if(BST_fBluetoothDirectionL==1|BST_fBluetoothDirectionR==1)
{	
    btcount=1;
    if(++turncount==1)										     //��ʼ�ٶ� ����תǰ�����ݳ�ʼ�ٶȣ��ٸ�������ٶȡ�
    { 
	temp  = BST_s16LeftMotorPulse+ BST_s16RightMotorPulse;
	temp1 = BST_s16LeftMotorPulse-BST_s16RightMotorPulse;
	if(temp<0) temp2=-temp;
	if(temp1<0) temp3=-temp1;

    turnconvert=temp3/temp2;								   //��ʵ�ʵ��Թ�����ȷ������ת��ʱ�����ȶ��Ծ�����
    }
	if(turnconvert<1)turnconvert=1;							
    if(turnconvert>5)turnconvert=5;	
}
  else
{
 	turnconvert=1;
	BST_speedout=0;
    turncount=0;
	speedtarget=0; 
	temp=0;
}

 if(BST_fBluetoothDirectionL==1) speedtarget+=turnconvert;	  
 else if(BST_fBluetoothDirectionR==1) speedtarget-=turnconvert;
      else speedtarget=0; 	 
	      
if(speedtarget>speedlimit) speedtarget=speedlimit;
if(speedtarget<-speedlimit) speedtarget=-speedlimit;	   
   	 
	BST_speednow=-speedtarget*19-gyro[2]*0.4;					//��תPD�����㷨	
}



void turnfliteroutput(void)											  //��תPWMֵƽ�����
{
float turnvalue;
turnvalue=BST_speednow-BST_speedold;
BST_speedout=turnvalue*(BST_u8turnPeriod + 1) / 15  +BST_speedold;		  //��תƽ��������㹫ʽ
}

/**********************�������������***************************/
void chaoshengbo(void)
{  
	   if(TIM_GetCounter(TIM1)>0)				   //�����⵽������ģ�鱻���룬���߳�������⵽�������ò��
	   {
	juli=TIM_GetCounter(TIM1)*5*34/200.0;		  //��ʼ������8�������źź󣬿���TIM1��ʱ�����������лز�ʱ��ֹͣ������������l=T *SPEED/2
	if(juli<=8.00)								  //�ж�������С��8cm��С��������PWMֵ��
     {
	  fchaoshengbo= (-100);
	}
	else fchaoshengbo=0;						 //�������8cm ��������PWM���Ϊ0


 	}

 }


