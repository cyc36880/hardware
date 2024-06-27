/*************************************************************
 * @LastEditors: 蔡雅超
 * @Date: 2024-06-27 14:35:04
 * @LastEditTime: 2024-06-27 14:40:40
 *************************************************************/
#ifndef _FLASH_H_
#define _FLASH_H_

#include "main.h"

#define MemoryAddr 0x800FC00
#define MemoryBank FLASH_BANK_1
#define MemoryPage 63



void flash_Init(void);

float *get_flash_buf(void);
void updata_flash(void);

// uint16_t Get_Flash_Data(void);
// void Write_Flash_Data(uint16_t data);

#endif
