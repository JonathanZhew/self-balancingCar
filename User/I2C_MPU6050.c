/***********************************************
** �ļ����ƣ�i2c.c
** ��������: i2c Ӧ�ú�����            
** ʵ��ƽ̨��������ƽ��С��
** Ӳ�����ӣ�----------------- 
**	   		| PB8 - I2C1_SCL  |
**			| PB9 - I2C1_SDA  |
**			 ----------------- 

**********************************************************************************/
#include "I2C_MPU6050.h"
#include "upstandingcar.h"
/*
 * ��������I2C_GPIO_Config
 * ����  ��I2C1 I/O����
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
void I2C_Config(void)
{
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure; 
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
 
    /* PB8,9 SCL and SDA */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    /* I2C1_SCL on PB08, I2C1_SDA on PB09 */ 
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);   //ʹ����ӳ�书��
 

    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;

    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;//100K�ٶ�
    
    I2C_Cmd(I2C1, ENABLE);
    I2C_Init(I2C1, &I2C_InitStructure);
    /*����1�ֽ�1Ӧ��ģʽ*/
    I2C_AcknowledgeConfig(I2C1, ENABLE);
 
}

/*
 * ��������I2C_ByteWrite
 * ����  ��дһ���ֽڵ�I2C�豸�Ĵ�����
 * ����  ��REG_Address �������ݵ�IIC�豸�Ĵ����ĵ�ַ 
 *         REG_data ��д�������
 * ���  ����
 * ����  ����
 * ����  ���ڲ�����
 */	
void I2C_ByteWrite(uint8_t REG_Address,uint8_t REG_data)
{

I2C_GenerateSTART(I2C1,ENABLE);

while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

I2C_Send7bitAddress(I2C1,SlaveAddress,I2C_Direction_Transmitter);

while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

I2C_SendData(I2C1,REG_Address);

while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

I2C_SendData(I2C1,REG_data);

while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

I2C_GenerateSTOP(I2C1,ENABLE);

}


/*
 * ��������I2C_ByteRead
 * ����  ����IIC�豸�Ĵ����ж�ȡһ���ֽ�
 * ����  ��REG_Address ��ȡ���ݵļĴ����ĵ�ַ 
 * ���  ����
 * ����  ����
 * ����  ���ڲ����� 
*/
uint8_t I2C_ByteRead(uint8_t REG_Address)
{
uint8_t REG_data;

while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));

I2C_GenerateSTART(I2C1,ENABLE);//��ʼ�ź�

while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

I2C_Send7bitAddress(I2C1,SlaveAddress,I2C_Direction_Transmitter);//�����豸��ַ+д�ź�

while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//

I2C_Cmd(I2C1,ENABLE);

I2C_SendData(I2C1,REG_Address);//���ʹ洢��Ԫ��ַ����0��ʼ

while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

I2C_GenerateSTART(I2C1,ENABLE);//��ʼ�ź�

while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

I2C_Send7bitAddress(I2C1,SlaveAddress,I2C_Direction_Receiver);//�����豸��ַ+���ź�

while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

I2C_AcknowledgeConfig(I2C1,DISABLE);

I2C_GenerateSTOP(I2C1,ENABLE);

while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)));

REG_data=I2C_ReceiveData(I2C1);//�����Ĵ�������

return REG_data;

}

/*
 * ��������void InitMPU6050(void)
 * ����  ����ʼ��Mpu6050
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void InitMPU6050(void)
{
	I2C_ByteWrite(PWR_MGMT_1,0x00);//�������״̬
	delay_nms(100);
	I2C_ByteWrite(SMPLRT_DIV,0x07);	 //IICд��ʱ�ĵ�ַ�ֽ�����
	delay_nms(100);
	I2C_ByteWrite(CONFIG,0x06);	 //��ͨ�˲�Ƶ��
	delay_nms(100);
	I2C_ByteWrite(GYRO_CONFIG,0x18); //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
	delay_nms(100);
	I2C_ByteWrite(ACCEL_CONFIG,0x01);//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
	delay_nms(100);

}


/*
 * ��������GetData
 * ����  �����16λ����
 * ����  ��REG_Address �Ĵ�����ַ
 * ���  �����ؼĴ�������
 * ����  ���ⲿ����
 */

s16 GetData(unsigned char REG_Address)
{
	char H,L;
	H=I2C_ByteRead(REG_Address);
	L=I2C_ByteRead(REG_Address+1);
	return (H<<8)+L;   //�ϳ�����
}

/******************* (C) COPYRIGHT 2012  *****END OF FILE************/








