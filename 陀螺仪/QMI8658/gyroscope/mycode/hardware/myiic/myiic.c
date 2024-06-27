/*
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-01-18 13:34:25
 * @LastEditTime: 2024-01-25 17:12:24
 * @FilePath: \MDK-ARMd:\Desptop\icMatser_RTOS_guider_2\myCode\hardware\myiic\myiic.c
 */
#include "myiic.h"

#include "delay.h"

#define IIC_SCL_1() HAL_GPIO_WritePin(IIC_PORT,IIC_SCL_PIN, GPIO_PIN_SET)
#define IIC_SCL_0() HAL_GPIO_WritePin(IIC_PORT,IIC_SCL_PIN, GPIO_PIN_RESET)

#define IIC_SDA_1() HAL_GPIO_WritePin(IIC_PORT,IIC_SDA_PIN, GPIO_PIN_SET)
#define IIC_SDA_0() HAL_GPIO_WritePin(IIC_PORT,IIC_SDA_PIN, GPIO_PIN_RESET)
#define READ_SDA()  HAL_GPIO_ReadPin(IIC_PORT,IIC_SDA_PIN)


static void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    IIC_IO_ENABLE;
        
    // HAL_GPIO_WritePin(DHT11_IO_Init, DHT11_IO_DQ, GPIO_PIN_SET);
    /*Configure GPIO pin : PB9 */
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(IIC_PORT, &GPIO_InitStruct);
}


static void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    IIC_IO_ENABLE;

    // HAL_GPIO_WritePin(DHT11_IO_Init, DHT11_IO_DQ, GPIO_PIN_SET);

    /*Configure GPIO pins : PBPin PBPin */
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IIC_PORT, &GPIO_InitStruct);
}

//**************************************
//I2C��ʼ�ź�
//**************************************
void I2C_Start(void)
{
	SDA_OUT();//sda�����
    IIC_SDA_1();                    //����������
    IIC_SCL_1();                    //����ʱ����
    delay_us(5);                 //��ʱ
    IIC_SDA_0();                    //�����½���
    delay_us(5);                 //��ʱ
    IIC_SCL_0();                    //����ʱ����
    delay_us(5);
}

//**************************************
//I2Cֹͣ�ź�
//**************************************
void I2C_Stop(void)
{
	SDA_OUT();//sda�����
    IIC_SDA_0();                    //����������
    IIC_SCL_1();                    //����ʱ����
    delay_us(5);                 //��ʱ
    IIC_SDA_1();                    //����������
    delay_us(5);                 //��ʱ						   	
}
//**************************************
//I2C����Ӧ���ź�
//��ڲ���:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(uint8_t ack)
{
	SDA_OUT();//sda�����
    if(ack) IIC_SDA_1();           //����NAK�ź�
    else IIC_SDA_0();           //����ACK�ź�
    // IIC_SDA = ack;                  //дӦ���ź�
    IIC_SCL_1();                    //����ʱ����
    delay_us(5);                 //��ʱ
    IIC_SCL_0();                    //����ʱ����
    delay_us(5);                 //��ʱ
}
//**************************************
//I2C����Ӧ���ź� 0Ӧ��
//**************************************
uint8_t I2C_RecvACK(void)
{
	uint8_t CY;
	
	SDA_IN();
    IIC_SCL_1();                    //����ʱ����
    delay_us(5);                 //��ʱ
    CY = READ_SDA();                   //��Ӧ���ź�
    IIC_SCL_0();                    //����ʱ����
    delay_us(5);                 //��ʱ
    return CY;
}
//**************************************
//��I2C���߷���һ���ֽ�����
//**************************************
void I2C_SendByte(uint8_t dat)
{
    uint8_t i;
	SDA_OUT();//sda�����
    for (i=0; i<8; i++)         //8λ������
    {
        if(dat>>7) IIC_SDA_1();
        else IIC_SDA_0();
		// IIC_SDA = (dat>>7);
        dat <<= 1;              //�Ƴ����ݵ����λ
        IIC_SCL_1();                //����ʱ����
        delay_us(5);             //��ʱ
        IIC_SCL_0();                //����ʱ����
        delay_us(5);             //��ʱ
    }	 
}
//**************************************
//��I2C���߽���һ���ֽ�����
//**************************************
uint8_t I2C_RecvByte(uint8_t a)
{
    uint8_t i;
    uint8_t dat = 0;
    IIC_SDA_1();                    //ʹ���ڲ�����,׼����ȡ����,
	SDA_IN();
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        IIC_SCL_1();                //����ʱ����
        delay_us(5);             //��ʱ
        dat |= READ_SDA();             //������               
        IIC_SCL_0();                //����ʱ����
        delay_us(5);             //��ʱ
    }
    I2C_SendACK(!a);
    return dat;
}

