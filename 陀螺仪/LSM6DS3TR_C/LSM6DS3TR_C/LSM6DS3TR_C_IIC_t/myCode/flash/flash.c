/*************************************************************
 * @LastEditors: ���ų�
 * @Date: 2024-06-27 14:34:17
 * @LastEditTime: 2024-06-27 14:50:00
 *************************************************************/
#include "flash.h"

#define FLASH_BUF_LEN  10

static float flash_buf[FLASH_BUF_LEN] = {0};


static void updata_flash_data(float *data, uint16_t len);

/**
 * 
 * uint16_t flash buf[] = {...}
 * 
 * [0]    �ӻ���ַ
 * [1-10]  �Ҷ�
 * [6-20] ��ֵ����
 * 
 * 
*/



/*************************************************************
 * @description: ��ȡflash����
 *  @note ��flash�ж�ȡһЩ����
 * @return {*}
 *************************************************************/
void flash_Init(void)
{
    for(uint8_t i=0; i<FLASH_BUF_LEN; i++)
    {
        flash_buf[i] = ((float*)MemoryAddr)[i];
    }
}

void updata_flash(void)
{
    updata_flash_data(flash_buf, FLASH_BUF_LEN);
}

static void updata_flash_data(float *data, uint16_t len)
{
    //1������FLASH
    HAL_FLASH_Unlock();

    //2������FLASH
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.Banks = MemoryBank;    //ѡ��洢��ַ���ڵ�����
    f.PageAddress = MemoryAddr;//ѡ��洢��ַ���ڵ�ҳ
    f.NbPages = 1;

    //����PageError
    uint32_t PageError = 0;
    //���ò�������
    HAL_FLASHEx_Erase(&f, &PageError);
    HAL_Delay(5);
    //3����FLASH��д

    for(uint8_t i=0; i<len; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MemoryAddr+i*4, *(uint32_t *)&data[i]);
    }
    //4����סFLASH
    HAL_FLASH_Lock();
}

float *get_flash_buf(void)
{
    return flash_buf;
}

