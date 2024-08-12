/**
  ******************************************************************************
  * @file    IAP_Main/Src/flash_if.c 
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    12-May-2017
  * @brief   ���ļ��ṩ�������ڴ���صĲ�������.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/** @addtogroup STM32F1xx_IAP
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_if.h"

static uint32_t GetSector(uint32_t Address);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  �����������д�����
  * @param  None
  * @retval None
  */
void FLASH_If_Init(void)
{
  /* ���������ڴ� */
  // HAL_FLASH_Unlock();

  /* ������������־ */
  // __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR);
  /* ���������ڴ� */
  // HAL_FLASH_Lock();
}

/**
  * @brief  �˺����������û�����������в���a
  * @param  start: �û���������Ŀ�ʼ
  * @retval FLASHIF_OK : �û���������ɹ�����
  *         FLASHIF_ERASEKO : �����˴���
  */
uint32_t FLASH_If_Erase(uint32_t start)
{
  uint32_t NbrOfPages = 0;
  uint32_t PageError = 0;
  FLASH_EraseInitTypeDef pEraseInit;
  HAL_StatusTypeDef status = HAL_OK;

  /* ��������������������ƼĴ������� *************/
  HAL_FLASH_Unlock();

  /* ��ȡ�����û�������������� */
  NbrOfPages = (USER_FLASH_END_ADDRESS - start)/FLASH_PAGE_STEP + 1;

  pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  pEraseInit.Banks = FLASH_BANK_1;
  pEraseInit.Page = GetSector(start);
  pEraseInit.NbPages = NbrOfPages;
  status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);

  /* ���������Խ���������ƼĴ������ʣ����鱣���������ܿ��ܲ���Ҫ�Ĳ�����*********/
  HAL_FLASH_Lock();

  if (status != HAL_OK)
  {
    /* ҳ�����ʱ�������� */
    return FLASHIF_ERASEKO;
  }

  return FLASHIF_OK;
}

/* Public functions ---------------------------------------------------------*/
/**
  * @brief  �˺�����������д�����ݻ����� (������32λ�����).
  * @note   д�����ݻ������󣬼����������
  * @param  destination: Ŀ��λ�õ���ʼ��ַ
  * @param  p_source: ָ��Ҫд�����ݵĻ�������ָ��
  * @param  length: ���ݻ������ĳ��ȣ���λ��32λ�֣�
  * @retval uint32_t 0: ���ݳɹ���д������
  *         1: ��������д������ʱ��������
  *         2: �����е�д�����ݲ�ͬ��Ԥ�ڵ�����
  */
uint32_t FLASH_If_Write(uint32_t destination, uint32_t *p_source, uint32_t length)
{
  uint32_t i = 0;


  /* ��������������������ƼĴ������� *************/
  HAL_FLASH_Unlock();

  for (i = 0; (i < length) && (destination <= (USER_FLASH_END_ADDRESS-8)); i+=2)
  {
    /* �豸��ѹ��ΧӦΪ[2.7V��3.6V]��������ͨ��Word���*/
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destination, *(uint64_t*)(p_source+i)) == HAL_OK)      
    {
     /* ���д���ֵ */
      if (*(uint64_t*)destination != *(uint64_t*)(p_source+i))
      {
        /* ����������SRAM���ݲ�ƥ�� */
        return(FLASHIF_WRITINGCTRL_ERROR);
      }
      /* ����FlashĿ�ĵص�ַ */
      destination += 8;
    }
    else
    {
      /* ��������д������ʱ�������� */
      return (FLASHIF_WRITING_ERROR);
    }
  }

  /* ���������Խ���������ƼĴ�������(���鱣���������ܿ��ܲ���Ҫ�Ĳ��� *********/
  HAL_FLASH_Lock();

  return (FLASHIF_OK);
}

/**
  * @brief  ����Ӧ�ó������������д����״̬
  * @param  None
  * @retval ���Ӧ�ó��������е�������д�����ģ��򷵻�ֵ��һ�����
                   ���ܵ�ֵ : FLASHIF_PROTECTION_WRPENABLED, FLASHIF_PROTECTION_PCROPENABLED, ...
  *        ���û��������д������  FLASHIF_PROTECTION_NONE ���˻�.
  */
uint32_t FLASH_If_GetWriteProtectionStatus(void)
{
  uint32_t ProtectedPAGE = FLASHIF_PROTECTION_NONE;
  FLASH_OBProgramInitTypeDef OptionsBytesStruct;

  /* ��������������������ƼĴ�������*************/
  HAL_FLASH_Unlock();

  /* ����û������������Ƿ���д�������� ****/
  HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);

  /* ���������Խ���������ƼĴ������ʣ����鱣���������ܿ��ܲ���Ҫ�Ĳ����� *********/
  HAL_FLASH_Lock();

  /* ��ȡ��д�뱣����ҳ�� ****************************************/
  // ProtectedPAGE = ~(OptionsBytesStruct.WRPPage) & FLASH_PAGE_TO_BE_PROTECTED;

  /* �������ҳ���Ƿ��Ѿ�д�뱣�� ***********************/
  if(ProtectedPAGE != 0)
  {
    /* �û����������ڵ�һЩ������д�뱣�� */
    return FLASHIF_PROTECTION_WRPENABLED;
  }
  else
  { 
    /* �û�����������û��д�������� */
    return FLASHIF_PROTECTION_NONE;
  }
}

/**
  * @brief  �����û����������д����״̬.
  * @param  protectionstate : FLASHIF_WRP_DISABLE or FLASHIF_WRP_ENABLE the protection
  * @retval uint32_t FLASHIF_OK if change is applied.
  */
uint32_t FLASH_If_WriteProtectionConfig(uint32_t protectionstate)
{
//  uint32_t ProtectedPAGE = 0x0;
  FLASH_OBProgramInitTypeDef config_new, config_old;
  HAL_StatusTypeDef result = HAL_OK;
  

  /* ��ȡҳ��д�뱣��״̬ ****************************************/
  HAL_FLASHEx_OBGetConfig(&config_old);

  /* ������ʾ�����Ƿ�򿪻�رձ��� */
  // config_new.WRPState = (protectionstate == FLASHIF_WRP_ENABLE ? OB_WRPSTATE_ENABLE : OB_WRPSTATE_DISABLE);

  /* ����ֻ���޸�д���� */
  config_new.OptionType = OPTIONBYTE_WRP;
  
  /* û�ж�ȡ����������BOR����������*/
  config_new.RDPLevel = OB_RDP_LEVEL_0;
  config_new.USERConfig = config_old.USERConfig;  
  /* ��ȡ��д�뱣����ҳ�� ****************************************/
  // ProtectedPAGE = config_old.WRPPage | FLASH_PAGE_TO_BE_PROTECTED;

  /* ��������������������ƼĴ������� *************/
  HAL_FLASH_Unlock();

  /* ����ѡ���ֽ�*************************************************/
  HAL_FLASH_OB_Unlock();
  
  /* ɾ������ѡ���ֽ� ***********************************************/
  // result = HAL_FLASHEx_OBErase();
    
  if (result == HAL_OK)
  {
    // config_new.WRPPage    = ProtectedPAGE;
    result = HAL_FLASHEx_OBProgram(&config_new);
  }
  
  return (result == HAL_OK ? FLASHIF_OK: FLASHIF_PROTECTION_ERRROR);
}
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




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
