/*************************************************************
 * @LastEditors: ���ų�
 * @Date: 2024-06-27 14:34:17
 * @LastEditTime: 2024-07-07 18:06:00
 *************************************************************/
#include "flash.h"

#define FLASH_BUF_LEN  10

static float flash_buf[FLASH_BUF_LEN] = {0};

static uint32_t GetSector(uint32_t Address);

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
    len = len/2 + 1;
    //1������FLASH
    HAL_FLASH_Unlock();

    //2������FLASH
    FLASH_EraseInitTypeDef f;
    f.Banks = FLASH_BANK_1;
    f.Page = GetSector(MemoryAddr);
    f.NbPages = 1;

    //����PageError
    uint32_t PageError = 0;
    //���ò�������
    HAL_FLASHEx_Erase(&f, &PageError);
    HAL_Delay(5);
    //3����FLASH��д

    for(uint8_t i=0; i<len; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, MemoryAddr+i*8, *(uint64_t *)&data[i]);
    }
    //4����סFLASH
    HAL_FLASH_Lock();
}

float *get_flash_buf(void)
{
    return flash_buf;
}


/**
  * @brief  ��������ĵ�ַ���������ڵ�sector
  *					���磺
						uwStartSector = GetSector(FLASH_USER_START_ADDR);
						uwEndSector = GetSector(FLASH_USER_END_ADDR);	
  * @param  Address����ַ
  * @retval ��ַ���ڵ�sector
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  Address -= FLASH_BASE;

  sector = Address/FLASH_PAGE_SIZE;
	
  return sector;
}

