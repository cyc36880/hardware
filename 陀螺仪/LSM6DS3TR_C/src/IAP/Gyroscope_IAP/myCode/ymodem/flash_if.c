/**
  ******************************************************************************
  * @file    IAP_Main/Src/flash_if.c 
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    12-May-2017
  * @brief   此文件提供所有与内存相关的操作功能.
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
  * @brief  解锁闪存进行写入访问
  * @param  None
  * @retval None
  */
void FLASH_If_Init(void)
{
  /* 解锁程序内存 */
  // HAL_FLASH_Unlock();

  /* 清除所有闪存标志 */
  // __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR);
  /* 锁定程序内存 */
  // HAL_FLASH_Lock();
}

/**
  * @brief  此函数对所有用户闪存区域进行擦除a
  * @param  start: 用户闪存区域的开始
  * @retval FLASHIF_OK : 用户闪存区域成功擦除
  *         FLASHIF_ERASEKO : 发生了错误
  */
uint32_t FLASH_If_Erase(uint32_t start)
{
  uint32_t NbrOfPages = 0;
  uint32_t PageError = 0;
  FLASH_EraseInitTypeDef pEraseInit;
  HAL_StatusTypeDef status = HAL_OK;

  /* 解锁闪存以启用闪存控制寄存器访问 *************/
  HAL_FLASH_Unlock();

  /* 获取启动用户闪存区域的扇区 */
  NbrOfPages = (USER_FLASH_END_ADDRESS - start)/FLASH_PAGE_STEP + 1;

  pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  pEraseInit.Banks = FLASH_BANK_1;
  pEraseInit.Page = GetSector(start);
  pEraseInit.NbPages = NbrOfPages;
  status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);

  /* 锁定闪存以禁用闪存控制寄存器访问（建议保护闪存免受可能不必要的操作）*********/
  HAL_FLASH_Lock();

  if (status != HAL_OK)
  {
    /* 页面擦除时发生错误 */
    return FLASHIF_ERASEKO;
  }

  return FLASHIF_OK;
}

/* Public functions ---------------------------------------------------------*/
/**
  * @brief  此函数在闪存中写入数据缓冲区 (数据是32位对齐的).
  * @note   写入数据缓冲区后，检查闪存内容
  * @param  destination: 目标位置的起始地址
  * @param  p_source: 指向要写入数据的缓冲区的指针
  * @param  length: 数据缓冲区的长度（单位是32位字）
  * @retval uint32_t 0: 数据成功地写入闪存
  *         1: 在闪存中写入数据时发生错误
  *         2: 闪存中的写入数据不同于预期的数据
  */
uint32_t FLASH_If_Write(uint32_t destination, uint32_t *p_source, uint32_t length)
{
  uint32_t i = 0;


  /* 解锁闪存以启用闪存控制寄存器访问 *************/
  HAL_FLASH_Unlock();

  for (i = 0; (i < length) && (destination <= (USER_FLASH_END_ADDRESS-8)); i+=2)
  {
    /* 设备电压范围应为[2.7V至3.6V]，操作将通过Word完成*/
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destination, *(uint64_t*)(p_source+i)) == HAL_OK)      
    {
     /* 检查写入的值 */
      if (*(uint64_t*)destination != *(uint64_t*)(p_source+i))
      {
        /* 闪存内容与SRAM内容不匹配 */
        return(FLASHIF_WRITINGCTRL_ERROR);
      }
      /* 增量Flash目的地地址 */
      destination += 8;
    }
    else
    {
      /* 在闪存中写入数据时发生错误 */
      return (FLASHIF_WRITING_ERROR);
    }
  }

  /* 锁定闪存以禁用闪存控制寄存器访问(建议保护闪存免受可能不必要的操作 *********/
  HAL_FLASH_Lock();

  return (FLASHIF_OK);
}

/**
  * @brief  返回应用程序闪存区域的写保护状态
  * @param  None
  * @retval 如果应用程序区域中的扇区是写保护的，则返回值是一个组合
                   可能的值 : FLASHIF_PROTECTION_WRPENABLED, FLASHIF_PROTECTION_PCROPENABLED, ...
  *        如果没有扇区是写保护的  FLASHIF_PROTECTION_NONE 被退回.
  */
uint32_t FLASH_If_GetWriteProtectionStatus(void)
{
  uint32_t ProtectedPAGE = FLASHIF_PROTECTION_NONE;
  FLASH_OBProgramInitTypeDef OptionsBytesStruct;

  /* 解锁闪存以启用闪存控制寄存器访问*************/
  HAL_FLASH_Unlock();

  /* 检查用户闪存区域内是否有写保护扇区 ****/
  HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);

  /* 锁定闪存以禁用闪存控制寄存器访问（建议保护闪存免受可能不必要的操作） *********/
  HAL_FLASH_Lock();

  /* 获取已写入保护的页面 ****************************************/
  // ProtectedPAGE = ~(OptionsBytesStruct.WRPPage) & FLASH_PAGE_TO_BE_PROTECTED;

  /* 检查所需页面是否已经写入保护 ***********************/
  if(ProtectedPAGE != 0)
  {
    /* 用户闪存区域内的一些扇区被写入保护 */
    return FLASHIF_PROTECTION_WRPENABLED;
  }
  else
  { 
    /* 用户闪存区域内没有写保护扇区 */
    return FLASHIF_PROTECTION_NONE;
  }
}

/**
  * @brief  配置用户闪存区域的写保护状态.
  * @param  protectionstate : FLASHIF_WRP_DISABLE or FLASHIF_WRP_ENABLE the protection
  * @retval uint32_t FLASHIF_OK if change is applied.
  */
uint32_t FLASH_If_WriteProtectionConfig(uint32_t protectionstate)
{
//  uint32_t ProtectedPAGE = 0x0;
  FLASH_OBProgramInitTypeDef config_new, config_old;
  HAL_StatusTypeDef result = HAL_OK;
  

  /* 获取页面写入保护状态 ****************************************/
  HAL_FLASHEx_OBGetConfig(&config_old);

  /* 参数表示我们是否打开或关闭保护 */
  // config_new.WRPState = (protectionstate == FLASHIF_WRP_ENABLE ? OB_WRPSTATE_ENABLE : OB_WRPSTATE_DISABLE);

  /* 我们只想修改写保护 */
  config_new.OptionType = OPTIONBYTE_WRP;
  
  /* 没有读取保护，保持BOR和重置设置*/
  config_new.RDPLevel = OB_RDP_LEVEL_0;
  config_new.USERConfig = config_old.USERConfig;  
  /* 获取已写入保护的页面 ****************************************/
  // ProtectedPAGE = config_old.WRPPage | FLASH_PAGE_TO_BE_PROTECTED;

  /* 解锁闪存以启用闪存控制寄存器访问 *************/
  HAL_FLASH_Unlock();

  /* 解锁选项字节*************************************************/
  HAL_FLASH_OB_Unlock();
  
  /* 删除所有选项字节 ***********************************************/
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
  * @brief  根据输入的地址给出它所在的sector
  *					例如：
						uwStartSector = GetSector(FLASH_USER_START_ADDR);
						uwEndSector = GetSector(FLASH_USER_END_ADDR);	
  * @param  Address：地址
  * @retval 地址所在的sector
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  Address -= FLASH_BASE;

  sector = Address/FLASH_PAGE_SIZE;
	
  return sector;
}
