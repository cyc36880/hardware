#include "control.h"

#include "usart.h"

#include "menu.h"
#include "flash_if.h"

#include <stdio.h>

#define BOOT_WAIT_BYTE '2'

// 
void jumpToBootloader(void);


void setup(void)
{    
    uint8_t usart_rx_data = 0;
    uint32_t now_Tick = HAL_GetTick();


    while(HAL_GetTick() - now_Tick < 200)
    {
        HAL_UART_Receive(&huart1, (uint8_t*)&usart_rx_data, 1, 20);
        if(usart_rx_data == BOOT_WAIT_BYTE) {
           /* Execute the IAP driver in order to reprogram the Flash */
//            FLASH_If_Init();
            /* Display main menu */
            Main_Menu();
        }
    }

    jumpToBootloader();
}

void loop(void)
{

}


static void sysSetDefault(void)
{
    HAL_RCC_DeInit(); //
    HAL_UART_DeInit(&huart1); //
}

void jumpToBootloader(void)
{
    pFunction JumpToApplication;
    uint32_t JumpAddress;

    sysSetDefault(); //
    SCB->VTOR = APPLICATION_ADDRESS; //设置中断向量偏移地址
    /* execute the new program */
    JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
    /* Jump to user application */
    JumpToApplication = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
    JumpToApplication();
}


