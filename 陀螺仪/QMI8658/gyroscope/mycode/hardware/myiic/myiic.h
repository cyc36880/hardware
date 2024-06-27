/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-01-18 13:34:25
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-01-25 17:04:50
 * @FilePath: \MDK-ARMd:\Desptop\icMatser_RTOS_guider\myCode\hardware\myiic\myiic.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _MYIIC_H_
#define _MYIIC_H_

#include "main.h"

#define IIC_W 0
#define IIC_R 1 

// //IO方向设置
// #define SDA_IN()  {GPIOA->MODER&=~(3<<(12*2));GPIOA->MODER|=0<<12*2;}	//输入模式
// #define SDA_OUT() {GPIOA->MODER&=~(3<<(12*2));GPIOA->MODER|=1<<12*2;}   //输出模式

#define IIC_PORT    GPIOA
#define IIC_SCL_PIN GPIO_PIN_11
#define IIC_SDA_PIN GPIO_PIN_12
#define IIC_IO_ENABLE    __HAL_RCC_GPIOA_CLK_ENABLE()//GPIOB使能

// //IO操作
// #define IIC_SCL   PBout(10) //SCL
// #define IIC_SDA   PBout(11) //SDA
// #define READ_SDA  PBin(11)  //输入SDA



void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_RecvACK(void);
void I2C_SendByte(uint8_t dat);
uint8_t I2C_RecvByte(uint8_t a);

#endif

