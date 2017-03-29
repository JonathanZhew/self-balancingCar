
#include "upstandingcar.h"
#include "I2C_MPU6050.h"
#include "MOTOR.h"
#include "led.h"
#include "USART.H"
#include "MPU6050.H"
#include "UltrasonicWave.h"

u8 BST_u8LEDCount; 
u8 BST_u8MainEventCount;						  //主循环判断计数  在SysTick_Handler(void)中使用 每1ms加1
u8 BST_u8SpeedControlCount;						  //速度控制循环计数  在SysTick_Handler(void)中使用 每5ms加1
u8 BST_u8SpeedControlPeriod;
u8 BST_u8DirectionControlPeriod;
u8 BST_u8DirectionControlCount;					  //转向控制循环计数  在SysTick_Handler(void)中使用 每5ms加1 
u8 BST_u8trig;
u8 BST_u8turnCount;								  //旋转控制循环计数  在SysTick_Handler(void)中使用 每5ms加1	 
u8 BST_u8turnPeriod;							  //旋转平滑输出比例值
u8   btcount;
//s32 g_liAccAdd;
//s32 g_liGyroAdd;
/******电机控制参数******/
float BST_fSpeedControlOut;						   //速度控制PWM
float BST_fSpeedControlOutOld;
float BST_fSpeedControlOutNew;
float BST_fAngleControlOut;
float BST_fLeftMotorOut;
float BST_fRightMotorOut;

float BST_fCarAngle;						 //角度控制PWM
float gyro_z;

/******速度控制参数******/
s16   BST_s16LeftMotorPulse;					  //左电机脉冲数
s16	  BST_s16RightMotorPulse;					   //右电机脉冲数

s32   BST_s32LeftMotorPulseOld;
s32   BST_s32RightMotorPulseOld;
s32   BST_s32LeftMotorPulseSigma;				  //50ms左电机叠加值
s32   BST_s32RightMotorPulseSigma;				 //50ms右电机叠加值

float BST_fCarSpeed;							 //测速码盘得出的车速
float BST_fCarSpeedOld;
//float BST_fCarSpeedOld;
float BST_fCarPosition;						   //测速码盘通过计算得到的小车位移
//float fSpeedCtrlPeriod=100.0;
//以下参考为重点调试参考，同电池电压有关，建议充好电再调试
/*-----角度环和速度环PID控制参数-----*/
float  BST_fCarAngle_P =29;//2 4 8 10 15 20 14   	 //调大小时会左右摆，调大时会振动  请调到基本能够站立 P=20是用于给小车在运动过程使用
float  BST_fCarAngle_D = 0.0389;	// 0.001 0.002 0.004 0.008 0.0010 0.011	 调小时反应慢，调大时会干扰
float  BST_fCarSpeed_P = 5.1;//3.7;// 1 2 4 8 10 回调8 5 	 发现调小一点稳定 5.5是最好的
float  BST_fCarSpeed_I = 0.0898;//0.081;//0.002 0.004 0.010 0.02  0.08 0.10 0.2 回调后发现	电压低时调小，调大，蓝牙控制不稳，会倒
/******蓝牙控制参数******/																	
float BST_fBluetoothSpeed;						//蓝牙控制车速
float BST_fBluetoothDirectionOld;			    //用于平缓输出车速使用
float BST_fBluetoothDirectionNew;			    //用于平缓输出车速使用
float BST_fBluetoothDirectionOut;				 //用于平缓输出车速使用
float BST_fBluetoothDirectionSL;			    //左转标志位  由于PWM转向输出使用判断输出方式 固需要标志位
float BST_fBluetoothDirectionSR;			   //右转标志位	  由于PWM转向输出使用判断输出方式 固需要标志位
/************旋转*****************/
float BST_fBluetoothDirectionL;				   //左旋转标志位  由于PWM旋转输出使用判断输出方式 固需要标志位
float BST_fBluetoothDirectionR;				   //右旋转标志位  由于PWM旋转输出使用判断输出方式 固需要标志位
float BST_speednow ;						   //用于平缓输出车速使用
float BST_speedold;							  //用于平缓输出车速使用
float BST_speedout;							  //用于平缓输出车速使用
int chose;									   //旋转标志位
/******超声波********/
float BST_fchaoshengbooutput;				  //超声波PWM输出
float fchaoshengbo;							   //超声波输出量
float fchaoshengboOld;						  //用于平缓输出车速使用
float fchaoshengboNew;						  //用于平缓输出车速使用
float fchaoshengboPeriod;					 //用于平缓输出车速使用
float juli;									 //超声波距离


/**********延时子函数****************/
void delay_nms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }
}
/***********************************/


/***************************************************************
** 函数名称: CarUpstandInit
** 功能描述: 全局变量初始化函数

***************************************************************/
void CarUpstandInit(void)
{
	
	
	BST_s16LeftMotorPulse = BST_s16RightMotorPulse = 0;					  //左右脉冲值   初始化
//	BST_s32LeftMotorPulseOld = BST_s32RightMotorPulseOld = 0;			  //用于		 初始化
	BST_s32LeftMotorPulseSigma = BST_s32RightMotorPulseSigma = 0;		  //叠加脉冲数	 初始化


	BST_fCarSpeed = BST_fCarSpeedOld = 0;								   //平衡小车车速	初始化
	BST_fCarPosition = 0;												  //平衡小车位移量	初始化
	BST_fCarAngle    = 0;												  //平衡小车车速	初始化


	BST_fAngleControlOut = BST_fSpeedControlOut = BST_fBluetoothDirectionOut = 0;	//角度PWM、车速PWM、蓝牙控制PWM	 初始化
	BST_fLeftMotorOut    = BST_fRightMotorOut   = 0;								//左右车轮PWM输出值 			 初始化
	BST_fBluetoothSpeed  = 0;														//蓝牙控制车速值                 初始化
	BST_fBluetoothDirectionL =BST_fBluetoothDirectionR= 0;						    //蓝牙控制左右旋转标志位         初始化
	BST_fBluetoothDirectionSL =BST_fBluetoothDirectionSR= 0;						//蓝牙控制左右转向标志位         初始化
	BST_fBluetoothDirectionNew = BST_fBluetoothDirectionOld = 0;				    //蓝牙控制量                     初始化
	chose=0;																		//蓝牙控制旋转PWM输出标志位      初始化        

    btcount=0;																		//蓝牙控制转向PWM输出标志位      初始化
	BST_speednow=BST_speedout=BST_speedold=0;									    //蓝牙控制旋转平滑PWM输出过度值  初始化
	//BST_u8turnPeriod= BST_u8turnCount=0;

  	BST_u8MainEventCount=0;															//用于5ms定时器子程序SysTick_Handler(void)中总中断计数位
	BST_u8SpeedControlCount=0;														//用于5ms定时器子程序SysTick_Handler(void)中50ms速度平衡融入计数位
    BST_u8SpeedControlPeriod=0;														//用于5ms定时器子程序SysTick_Handler(void)中50ms速度平衡融入计数位

	BST_fchaoshengbooutput=fchaoshengbo=0;											//用于5ms定时器子程序SysTick_Handler(void)中超声波平衡融入计数位
	fchaoshengboOld=0;
	fchaoshengboNew=0;
	fchaoshengboPeriod=30;
	

}

/***************************************************************
** 函数名称: SampleInputVoltage
** 功能描述: 采样函数             

***************************************************************/

/***************************************************************
** 函数名称: AngleControl
** 功能描述: 角度环控制函数

***************************************************************/
void AngleControl(void)	 
{

	BST_fCarAngle = Roll - CAR_ZERO_ANGLE;													   //DMP ROLL滚动方向角度与预设小车倾斜角度值的差得出角度
	BST_fAngleControlOut =  BST_fCarAngle * BST_fCarAngle_P + gyro[0] * BST_fCarAngle_D ;	  //角度PD控制							   
	  
																							  
	 
}
#define MAX_PWM 255
/***************************************************************
** 函数名称: SetMotorVoltageAndDirection
** 功能描述: 电机转速             

***************************************************************/
void SetMotorVoltageAndDirection(s16 s16LeftVoltage,s16 s16RightVoltage)
{
	u16 u16LeftMotorValue;
	u16 u16RightMotorValue;
	
    if(s16LeftVoltage<0)										 //当左电机PWM输出为负时 PB14设为正 PB15设为负 （PB14 15 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
    {	
	  GPIO_SetBits(GPIOB, GPIO_Pin_15 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 );
      s16LeftVoltage = (-s16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    	 //当左电机PWM输出为正时 PB14设为负 PB15设为正 （PB14 15 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 ); 
      s16LeftVoltage = s16LeftVoltage;
    }

    if(s16RightVoltage<0)
    {															 //当右电机PWM输出为负时 PB12设为正 PB13设为负 （PB12 13 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
      GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );
      s16RightVoltage = (-s16RightVoltage);
    }
    else														//当右电机PWM输出为正时 PB12设为负 PB13设为正 （PB12 13 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
    {
	GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );	
      
      s16RightVoltage = s16RightVoltage;
    }
	 u16RightMotorValue= (u16)s16RightVoltage;
	 u16LeftMotorValue = (u16)s16LeftVoltage;

	if(u16RightMotorValue>MAX_PWM)  							   //设定左右电机PWM最大输出幅度为255
	{
		u16RightMotorValue=MAX_PWM;
	}
	if(u16LeftMotorValue>MAX_PWM)									//设定左右电机PWM最大输出幅度为255
	{
	   u16LeftMotorValue=MAX_PWM;
	}
    //TIM2_PWM_CHANGE(u16RighttMotorValue,u16LeftMotorValue) ;
	//TIM2->CCR3 = 	u16RightMotorValue ; 
   	//TIM2->CCR4 =    u16LeftMotorValue; 
	TIM_SetCompare3(TIM2,u16RightMotorValue);			  //TIM2与 u16RightMotorValue对比，不相同则翻转波形，调节PWM占空比
	TIM_SetCompare4(TIM2,u16LeftMotorValue);			  //TIM3与 u16LeftMotorValue对比，不相同则翻转波形，调节PWM占空比

#if 1	 /*判断车辆是否跌倒，调试用*/

	if(BST_fCarAngle > 180 || BST_fCarAngle < (-180))
	{
		TIM_SetCompare3(TIM2,0);
		TIM_SetCompare4(TIM2,0);  
	}

#endif
}

/***************************************************************
** 函数名称: MotorOutput
** 功能描述: 电机输出函数
             将直立控制、速度控制、方向控制的输出量进行叠加,并加
			 入死区常量，对输出饱和作出处理。

***************************************************************/
void MotorOutput(void)																					  //电机PWM输出函数
{	   
	if(btcount==1)																						  //旋转判断 btcount为1时 进入旋转PWM平衡输出
	{
    BST_fLeftMotorOut  = BST_fAngleControlOut - BST_fSpeedControlOut -BST_speedout;
	 BST_fRightMotorOut = BST_fAngleControlOut - BST_fSpeedControlOut+BST_speedout;
	btcount=0;
	}

	else if(chose==1)																						//转向判断 btcount为1时 进入转向PWM平衡输出
	{
	//delay_nms(1000)	;
     BST_fLeftMotorOut  = BST_fAngleControlOut - BST_fSpeedControlOut +BST_fBluetoothDirectionNew;			//左电机转向PWM控制融合平衡角度、速度输出	
     BST_fRightMotorOut = BST_fAngleControlOut - BST_fSpeedControlOut-BST_fBluetoothDirectionNew;			//右电机转向PWM控制融合平衡角度、速度输出
	 chose=0;
	 }
	else{																									//平衡PWM输出
	   BST_fLeftMotorOut  = BST_fAngleControlOut - BST_fSpeedControlOut ;								   //左电机旋转PWM控制融合平衡角度、速度输出
     BST_fRightMotorOut = BST_fAngleControlOut - BST_fSpeedControlOut;										//右电机旋转PWM控制融合平衡角度、速度输出
	
	} 
	/*输出饱和处理，防止超出PWM范围*/
		
	if((s16)BST_fLeftMotorOut  > MOTOR_OUT_MAX)	BST_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((s16)BST_fLeftMotorOut  < MOTOR_OUT_MIN)	BST_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((s16)BST_fRightMotorOut > MOTOR_OUT_MAX)	BST_fRightMotorOut = MOTOR_OUT_MAX;
	if((s16)BST_fRightMotorOut < MOTOR_OUT_MIN)	BST_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((s16)BST_fLeftMotorOut,(s16)BST_fRightMotorOut);
}

void GetMotorPulse(void)              //采集电机速度脉冲
{ 
  uint16_t u16TempLeft;
  uint16_t u16TempRight;

  u16TempLeft = TIM_GetCounter(TIM3);   //  TIM3定时器计算调用
  u16TempRight= TIM_GetCounter(TIM4);	//	 TIM4定时器计算调用
  TIM3->CNT = 0;
  TIM4->CNT = 0;   //清零
  BST_s16LeftMotorPulse=u16TempLeft;
  BST_s16RightMotorPulse=u16TempRight;

  if(!MOTOR_LEFT_SPEED_POSITIVE)  										   //极性判断	电机极性判断
  {
  	BST_s16LeftMotorPulse  = (-BST_s16LeftMotorPulse) ; 				  //若左电机反转，脉冲数为负数
  }
  if(!MOTOR_RIGHT_SPEED_POSITIVE) 
  {
  	BST_s16RightMotorPulse = (-BST_s16RightMotorPulse);					 //若右电机反转，脉冲数为负数
  }

  BST_s32LeftMotorPulseSigma +=  BST_s16LeftMotorPulse;		 //脉冲值叠加 50ms叠加值
  BST_s32RightMotorPulseSigma += BST_s16RightMotorPulse; 	 //脉冲值叠加 50ms叠加值
}
/***************************************************************
** 函数名称: SpeedControl
** 功能描述: 速度环控制函数
***************************************************************/

void SpeedControl(void)
{

	BST_fCarSpeed = (BST_s32LeftMotorPulseSigma  + BST_s32RightMotorPulseSigma ) * 0.5 ;		  //左右电机脉冲数平均值作为小车当前车速
    BST_s32LeftMotorPulseSigma =BST_s32RightMotorPulseSigma = 0;	  //全局变量 注意及时清零
	BST_fCarSpeed = 0.7 * BST_fCarSpeedOld + 0.3 * BST_fCarSpeed ;		//速度一阶滤波
	BST_fCarSpeedOld = BST_fCarSpeed;
	BST_fCarSpeed *= CAR_SPEED_CONSTANT;	 //单位：转/秒
	BST_fCarPosition += BST_fCarSpeed; 		 //路程  即速度积分	
	BST_fCarPosition += BST_fBluetoothSpeed;   //融合蓝牙给定速度
	BST_fCarPosition +=	fchaoshengbo;		   //融合超声波给定速度
	//积分上限设限//
	if((s32)BST_fCarPosition > CAR_POSITION_MAX)    BST_fCarPosition = CAR_POSITION_MAX;
	if((s32)BST_fCarPosition < CAR_POSITION_MIN)    BST_fCarPosition = CAR_POSITION_MIN;
	
	BST_fSpeedControlOutOld = BST_fSpeedControlOutNew;
																								  //速度PI算法 速度*P +位移*I=速度PWM输出
	BST_fSpeedControlOutNew = (CAR_SPEED_SET - BST_fCarSpeed) * BST_fCarSpeed_P +\
		(CAR_POSITION_SET - BST_fCarPosition) * BST_fCarSpeed_I; 		  

}
void SpeedControlOutput(void)																	 //速度平滑输出函数 
{
  float fValue;
  fValue = BST_fSpeedControlOutNew - BST_fSpeedControlOutOld ;
  BST_fSpeedControlOut = fValue * (BST_u8SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + BST_fSpeedControlOutOld; 	//转向平滑输出计算公式
  
}


/***************************************************************
** 函数名称: BluetoothControl
** 功能描述: 蓝牙控制函数
             手机发送内容
			 前：0x01    后：0x02
             左：0x04    右：0x03
             停止：0x07
             功能键：（旋转）
             左旋转:0x05      右旋转：0x06
           	 停转：0x07
** 输　入:   
** 输　出:   
***************************************************************/
void BluetoothControl(void)	 								 //蓝牙控制
{
	u8 u8BluetoothValue;
	
	while(USART_GetFlagStatus(USART3,USART_FLAG_RXNE)!=SET);	//等待USART3的接受区不为空

	u8BluetoothValue =USART3->DR;
	USART_ClearFlag(USART3,USART_FLAG_RXNE); 
		
	switch (u8BluetoothValue)
	{
	  case 0x01 : BST_fBluetoothSpeed =   100 ; break;	   //向前速度 250 
	  case 0x02 : BST_fBluetoothSpeed = (-90);  break;	   //后退速度 -250
	  case 0x04 : BST_fBluetoothDirectionSL = 1; break;//左转
	  case 0x03 : BST_fBluetoothDirectionSR = 1; break;//右转
	  case 0x05 : BST_fBluetoothDirectionL = 1; break ;//左旋
	  case 0x06 : BST_fBluetoothDirectionR = 1; break ;//右旋转
	  case 0x07 : BST_fBluetoothDirectionL  =0; BST_fBluetoothDirectionR = 0;  break; //停
	  case 0x08 : BST_fBluetoothDirectionSL =0; BST_fBluetoothDirectionSR = 0; break; //停旋转
	  case 0x09 :  BST_fBluetoothSpeed =   0 ;  break;
	  default : BST_fBluetoothSpeed = 0; BST_fBluetoothDirectionL=BST_fBluetoothDirectionR = 0;BST_fBluetoothDirectionSR=BST_fBluetoothDirectionSL=0;break;
	}
	
}


/***********************蓝牙PWM输出 转向*************************/
void DirectionControl(void)											  //转向函数
{


static float temp ,turnconvert,speedtarget,temp2;
static  int turncount,speedlimit=80;

	BST_fBluetoothDirectionNew=BST_fBluetoothDirectionOld ;

   if(gyro[2]>32768) gyro[2]-=65536;						//强制转数据类型转换 2G转换成1G
	chose=0;

   if(BST_fBluetoothDirectionSL==1|BST_fBluetoothDirectionSR==1)	  //蓝牙遥控是否有指令判断
{	
   chose=1;
    if(++turncount==1)
    { 
	temp  = BST_s16LeftMotorPulse+ BST_s16RightMotorPulse;		//初始速度 在转向前，根据初始速度，再给出输出速度。
	if(temp<0) temp2=-temp;									   //当前小车运动方向上的测速正负
    turnconvert=400/temp2;									   //在实际调试过程中确定。（转向时车的稳定性决定）
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
    
if(speedtarget>speedlimit) speedtarget=speedlimit;						//转向速度幅值限制 
if(speedtarget<-speedlimit) speedtarget=-speedlimit;	   
   	 
	BST_fBluetoothDirectionNew=-speedtarget*19-gyro[2]*0.4;			   //转向PD控制。


}

void DirectionControlOutput(void)									   //转向平滑输出
{
  float fValue;
 

  fValue = BST_fBluetoothDirectionNew - BST_fBluetoothDirectionOld;
  BST_fBluetoothDirectionOut = fValue *(BST_u8DirectionControlPeriod + 1) /25 + BST_fBluetoothDirectionOld; //平滑输出计算公式 


}
/***************************旋转操作************************************/



void turn(void)								                        // 旋转函数
{
   static float temp ,turnconvert,speedtarget,temp1,temp2,temp3;
   int turncount,speedlimit=100;

   btcount=0;	
   BST_speedold=BST_speednow;
   if(gyro[2]>32768) gyro[2]-=65536;

   if(BST_fBluetoothDirectionL==1|BST_fBluetoothDirectionR==1)
{	
    btcount=1;
    if(++turncount==1)										     //初始速度 在旋转前，根据初始速度，再给出输出速度。
    { 
	temp  = BST_s16LeftMotorPulse+ BST_s16RightMotorPulse;
	temp1 = BST_s16LeftMotorPulse-BST_s16RightMotorPulse;
	if(temp<0) temp2=-temp;
	if(temp1<0) temp3=-temp1;

    turnconvert=temp3/temp2;								   //在实际调试过程中确定。（转向时车的稳定性决定）
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
   	 
	BST_speednow=-speedtarget*19-gyro[2]*0.4;					//旋转PD控制算法	
}



void turnfliteroutput(void)											  //旋转PWM值平滑输出
{
float turnvalue;
turnvalue=BST_speednow-BST_speedold;
BST_speedout=turnvalue*(BST_u8turnPeriod + 1) / 15  +BST_speedold;		  //旋转平滑输出计算公式
}

/**********************超声波距离计算***************************/
void chaoshengbo(void)
{  
	   if(TIM_GetCounter(TIM1)>0)				   //如果检测到超声波模块被插入，或者超声波检测到距离后调用测距
	   {
	juli=TIM_GetCounter(TIM1)*5*34/200.0;		  //初始化后发送8个脉冲信号后，开启TIM1定时计算器，当有回波时候，停止计算器，距离l=T *SPEED/2
	if(juli<=8.00)								  //判断若距离小于8cm，小车输出向后PWM值。
     {
	  fchaoshengbo= (-100);
	}
	else fchaoshengbo=0;						 //距离大于8cm ，超声波PWM输出为0


 	}

 }


