/*
 * spi.c:
 * Copyright (c) 2014-2021 Rtrobot. <admin@rtrobot.org>
 *  <http://rtrobot.org>
 ***********************************************************************
 * use keil5
 */

#include <main.h>
#include <spi.h>
#include "delay.h"
#include <stdio.h>

spi_device spi;

/*****************************************************************************
向SPI总线发送一个字节数据
*****************************************************************************/
uint8_t SPI_SendByte(uint8_t byte)
{
    uint8_t i, rev_data = 0;
    if (spi.CPOL == 0 && spi.CPHA == 0)
    {

        for (i = 0; i < 8; i++)
        {
            if (byte & 0x80)
                MOSI = 1;
            else
                MOSI = 0;
            byte <<= 1;
            Delay1us();
            SCK = 1;
            rev_data <<= 1;
            if (MISO)
                rev_data++;
            Delay1us();
            SCK = 0;
            Delay1us();
        }
    }
    else if (spi.CPOL == 0 && spi.CPHA == 1)
    {
        for (i = 0; i < 8; i++)
        {
            SCK = 1;
            if (byte & 0x80)
                MOSI = 1;
            else
                MOSI = 0;
            byte <<= 1;
            Delay1us();
            SCK = 0;
            rev_data <<= 1;
            if (MISO)
                rev_data++;
            Delay1us();
        }
    }
    else if (spi.CPOL == 1 && spi.CPHA == 0)
    {
        for (i = 0; i < 8; i++)
        {
            if (byte & 0x80)
                MOSI = 1;
            else
                MOSI = 0;
            byte <<= 1;
            Delay1us();
            SCK = 0;
            rev_data <<= 1;
            if (MISO)
                rev_data++;
            Delay1us();
            SCK = 1;
        }
    }
    else if (spi.CPOL == 1 && spi.CPHA == 1)
    {
        for (i = 0; i < 8; i++)
        {
            SCK = 0;
            if (byte & 0x80)
                MOSI = 1;
            else
                MOSI = 0;
            byte <<= 1;
            Delay1us();
            SCK = 1;
            rev_data <<= 1;
            if (MISO)
                rev_data++;
            Delay1us();
        }
    }
    return rev_data;
}

/***************************************************************************************************************
从SPI总线接收字符串
****************************************************************************************************************/
void SPI_ReadCommand(uint8_t reg_addr, uint8_t *rev_data, uint16_t length)
{
    CS = 0;
    SPI_SendByte(reg_addr);
    while (length)
    {
        *rev_data = SPI_SendByte(0xFF);
        rev_data++;
        length--;
    }

    CS = 1;
}

/***************************************************************************************************************
向SPI总线发送字符串
****************************************************************************************************************/
void SPI_WriteCommand(uint8_t reg_addr, uint8_t *send_data, uint16_t length)
{
    if (spi.cs_status == 1)
        CS = 1;
    else
        CS = 0;
    SPI_SendByte(reg_addr);
    while (length)
    {
        SPI_SendByte(*send_data);
        send_data++;
        length--;
    }
    if (spi.cs_status == 1)
        CS = 0;
    else
        CS = 1;
}
