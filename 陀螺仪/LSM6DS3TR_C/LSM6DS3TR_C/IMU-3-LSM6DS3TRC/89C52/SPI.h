/*
 * spi.h:
 * Copyright (c) 2014-2021 Rtrobot. <admin@rtrobot.org>
 *  <http://rtrobot.org>
 ***********************************************************************
 * use keil5
 */

#include <main.h>

sbit MISO = P1 ^ 0;
sbit MOSI = P1 ^ 1;
sbit SCK = P1 ^ 2;
sbit CS = P1 ^ 3;

typedef struct
{
    bool CPOL;
    bool CPHA;
    bool cs_status;
} spi_device;

extern spi_device spi;
void SPI_ReadCommand(uint8_t reg_addr, uint8_t *rev_data, uint16_t length);
void SPI_WriteCommand(uint8_t reg_addr, uint8_t *send_data, uint16_t length);