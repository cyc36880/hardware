#ifndef _MYIIC2_H_
#define _MYIIC2_H_

#include "main.h"

#define IIC_W 0
#define IIC_R 1 

// //IO方向设置
// #define SDA_IN()  {GPIOA->MODER&=~(3<<(12*2));GPIOA->MODER|=0<<12*2;}	//输入模式
// #define SDA_OUT() {GPIOA->MODER&=~(3<<(12*2));GPIOA->MODER|=1<<12*2;}   //输出模式

#define IIC2_PORT         GPIOA
#define IIC2_SCL_PIN      GPIO_PIN_4
#define IIC2_SDA_PIN      GPIO_PIN_5
#define IIC2_IO_ENABLE    __HAL_RCC_GPIOA_CLK_ENABLE()//

// //IO操作
// #define IIC_SCL   PBout(10) //SCL
// #define IIC_SDA   PBout(11) //SDA
// #define READ_SDA  PBin(11)  //输入SDA



void I2C2_Start(void);
void I2C2_Stop(void);
uint8_t I2C2_RecvACK(void);
void I2C2_SendByte(uint8_t dat);
uint8_t I2C2_RecvByte(uint8_t a);

#endif

