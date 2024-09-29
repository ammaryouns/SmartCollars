#include "FLASH_EEPROM.h"

/*

    Use the functions HAL_EEPROM_WriteQueue() and HAL_EEPROM_ReadQueue() to read from and write to 
    packet Queue in FLASH EEPROM.
    
    
*/ 
extern void HAL_IncTicks(uint32_t ticks);






/* Private function Definitions -----------------------------------------------*/

/*
    The destination address must be an offset from the EEPROM BASE ADDR, 
    not the absolute address in FLASH.
*/

HAL_StatusTypeDef Flash_EEPROM_Write(uint32_t destAddrOffset, uint8_t* srcAddr, uint32_t len){
    
    uint32_t tempAddr;
    HAL_StatusTypeDef retStatus = HAL_ERROR;
    uint32_t millisecondsElapsed = 0;
    union{
        
        uint32_t value;
        uint8_t  buffer[4];
        
    }tempData;
    
    HAL_DATA_EEPROMEx_Unlock();
    //    //stop systick.
    CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_ENABLE_Msk);
    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);


    SysTick->VAL = HAL_RCC_GetHCLKFreq() - 1;
    SysTick->LOAD = HAL_RCC_GetHCLKFreq() - 1;  //set Systick to count one second.
                                            //The flash write should never take more than this.
                                            //The counter value can then be divided by the HCLK frequency to get
                                            //elapsed number of milliseconds, which can then be added.
    //NOTE   :  Even though, the HAL_FLASH_Program function uses the HAL_Delay() function,
    //                it doesn't matter because systick interrupt wont be executed anyway until the flash is busy.
    //                When the flash is free, it will break out of the loop on it's own.

    //start Systick counter;
    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
    //FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
    
    if ((destAddrOffset % 4) != 0){            //Writing word instead of byte, because EEPROM write throws fault when writing 0x00 to a byte. 
    
        tempAddr = (destAddrOffset/4)*4;
        tempData.value = *(uint32_t*)(FLASH_EEPROM_BASE + tempAddr);
        
        for(int i = destAddrOffset%4 ; i < 4 && i <= len ; i++){
            tempData.buffer[i] = *(uint8_t*)(srcAddr);
            srcAddr++;
        }
        len -= 4 - (destAddrOffset - tempAddr);
        
        if (HAL_DATA_EEPROMEx_Program(FLASH_TYPEPROGRAMDATA_WORD, FLASH_EEPROM_BASE + tempAddr, tempData.value) != HAL_OK)
            goto error;
        
        destAddrOffset = tempAddr + 4;
    }
    
    
    if ((len % 4) != 0){            //Writing word instead of byte, because EEPROM write throws fault when writing 0x00 to a byte. 
    
        tempAddr = (len/4)*4;
        tempData.value = *(uint32_t*)(FLASH_EEPROM_BASE + destAddrOffset + tempAddr);
        
        
        for(int i = 0 ; i < len%4 ; i++){
            tempData.buffer[i] = *(uint8_t*)(srcAddr + tempAddr + i);
        }
        
        len = tempAddr;
        if (HAL_DATA_EEPROMEx_Program(FLASH_TYPEPROGRAMDATA_WORD, FLASH_EEPROM_BASE + destAddrOffset + len, tempData.value) != HAL_OK)
            goto error;
        
    }
    
    
    while (len > 0){
    
        len = len-4;
        if (HAL_DATA_EEPROMEx_Program(FLASH_TYPEPROGRAMDATA_WORD, FLASH_EEPROM_BASE + destAddrOffset + len, *(uint32_t*)(srcAddr + len)) != HAL_OK)
            goto error;
    }
    
    retStatus = HAL_OK;
    
    error:
    HAL_DATA_EEPROMEx_Lock();
    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);

    //get number of milliseconds.

    millisecondsElapsed += SysTick->VAL / (HAL_RCC_GetHCLKFreq() / 1000);
    millisecondsElapsed = 1000 - millisecondsElapsed;       //Systick counts downwards.
    millisecondsElapsed++;

    SysTick->VAL = (HAL_RCC_GetHCLKFreq()/1000) - 1;
    SysTick->LOAD = (HAL_RCC_GetHCLKFreq()/1000) - 1;

    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
    SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);

    HAL_IncTicks(millisecondsElapsed);      //to account for inaccuracy caused by systick being suspended.

    for (; millisecondsElapsed > 0; millisecondsElapsed--)
    {
        xTaskIncrementTick();
    }
    return retStatus;
   
}



HAL_StatusTypeDef Flash_EEPROM_Read(uint32_t srcAddrOffset, uint8_t* destAddr, uint32_t len){
 
    memcpy(destAddr, (uint8_t*)(srcAddrOffset + FLASH_EEPROM_BASE), len);
    
    return HAL_OK;
}
