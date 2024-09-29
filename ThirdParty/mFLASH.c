/**
 * @file   	mFLASH.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Oct 26, 2016
 * 
 * @brief   
 */

#include "mFLASH.h"


/**
 * @defgroup FLASH_Private_Functions		FLASH Private Functions
 * @{
 */
#if defined(STM32F205xx)

#define FLASH_SECTOR_COUNT_16K		((uint32_t)4)
#define FLASH_SECTOR_COUNT_64K		((uint32_t)1)
#define FLASH_SECTOR_COUNT_128K		((uint32_t)7)

/**
 * @brief 			Get the corresponding sector for the flash memory address.
 * @param flashAddr	Flash memory address.
 * @return          Sector number in flash memory according to the internal Flash map.
 */
uint32_t HAL_FLASH_get_Sector(uint32_t flashAddr)
{
    assert_param(IS_FLASH_ADDRESS(flashAddr));
    uint32_t sector = 0;
    
	flashAddr -= FLASH_BASE;

	if (flashAddr < (16*1024*FLASH_SECTOR_COUNT_16K))
    {
        return flashAddr / (16*1024);
    }

    sector = FLASH_SECTOR_COUNT_16K;
    flashAddr -= (16*1024*FLASH_SECTOR_COUNT_16K);
    
    if(flashAddr < (64*1024*FLASH_SECTOR_COUNT_64K))
    {
        return flashAddr / (64*1024);
    }
    
    sector += FLASH_SECTOR_COUNT_64K;
    flashAddr -= (64*1024*FLASH_SECTOR_COUNT_64K);
    
    return sector + flashAddr / (128*1024);
}

/**
 * @brief			Get the size of the specified sector in internal Flash
 * @param sector	Sector Number in internal Flash
 * @return			Size of the sector in bytes.
 */
uint32_t HAL_FLASH_get_SectorSize(uint32_t sector)
{
    assert_param(IS_FLASH_SECTOR(sector));
    
    if (sector < FLASH_SECTOR_COUNT_16K)
        return 16*1024;
    
    if(sector < FLASH_SECTOR_COUNT_64K)
        return 64*1024;
    

    return 128*1024;
}

#endif

/**
 * @defgroup FLASH_Public_Functions			FLASH Public Functions
 * @{
 */

extern uint32_t uwTick;

/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in Systick ISR.
  * @note This function is declared as __weak to be overwritten in case of other 
  *      implementations in user file.
  * @retval None
  */
__weak void HAL_IncTicks(uint32_t ticks)
{
  uwTick+= ticks;
}

/**
 * @brief			Write the given data to the Internal FLASH at the specified address.
 * @param destAddr	Absolute Address in Internal FLASH where the data is to be written
 * @param srcAddr	Pointer to data that is to be written to FLASH
 * @param len		Length of the data in bytes.
 * @return			#HAL_StatusTypeDef
 *
 * @note This function assumes that the FLASH area where data is to be written has already been erased.
 */
HAL_StatusTypeDef HAL_FLASH_Write(uint32_t destAddr, uint8_t* srcAddr, int32_t len){

    HAL_StatusTypeDef status;
    int32_t writeIndex = 0;
    uint32_t millisecondsElapsed = 0;
    uint64_t  readmemory = 0;

    if ((len%8) != 0)
    {
      len = ((len >> 3)+1 )<< 3;
    }




    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return status;


////    //stop systick.
//    CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_ENABLE_Msk);
//    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);


//    SysTick->VAL = HAL_RCC_GetHCLKFreq() - 1;
//    SysTick->LOAD = HAL_RCC_GetHCLKFreq() - 1;  //set Systick to count one second.
                                            //The flash write should never take more than this.
                                            //The counter value can then be divided by the HCLK frequency to get
                                            //elapsed number of milliseconds, which can then be added.
    //NOTE   :  Even though, the HAL_FLASH_Program function uses the HAL_Delay() function,
    //                it doesn't matter because systick interrupt wont be executed anyway until the flash is busy.
    //                When the flash is free, it will break out of the loop on it's own.

//    //start Systick counter;
//    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
    while (len > 0){


        #if defined(STM32L452xx) 
        if (((destAddr + writeIndex) % 8 == 0) && (len >= 8)) //  if is dest addres is 8 byte devisable 
        {
          memcpy(&readmemory, srcAddr + writeIndex, 8);
          status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destAddr + writeIndex, *(uint64_t*)(&readmemory));
          len = len-8;
          writeIndex = writeIndex+8;
        }
        else if (((destAddr+ writeIndex) % 8 == 0) && (len == 4)) //  if is dest addres is 8 byte devisable  and len is 4 byte
        {
          memcpy((uint8_t*)&readmemory, (uint8_t*) destAddr + writeIndex, 8);
          memcpy(&readmemory, srcAddr + writeIndex, 4);
          
          status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destAddr + writeIndex, *(uint64_t*)(&readmemory));
          len = len-4; 
          writeIndex = writeIndex+4;
        }
        else if ((destAddr + writeIndex) % 4 == 0) //  if is dest addres is 4 byte devisable (not 8 byte) and len is 4 byte 
        {
          // write only 4 bytes 
          memcpy((uint8_t*)&readmemory, (uint8_t*)destAddr + writeIndex - 4, 8);
          memcpy((uint8_t*)((uint8_t*)(&readmemory) + 4), (uint8_t*)(srcAddr + writeIndex), 4);
          status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destAddr + writeIndex - 4 , *(uint64_t*)(&readmemory));
          
//          memcpy((uint8_t*)((uint8_t*)(&readmemory)), (uint8_t*)(srcAddr + writeIndex), 4);
//          status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, destAddr + writeIndex , *(uint32_t*)(&readmemory));
          
          len = len-4; 
          writeIndex = writeIndex+4;
        }
        else
        {
          status = HAL_ERROR;
        }
        #elif defined(STM32L151xBA)
        len = len-4;
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, destAddr + len, *(uint32_t*)(srcAddr + len));
        #elif defined(STM32F205xx)
        len = len-4;
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, destAddr + len, *(uint32_t*)(srcAddr + len));
        #else
          #error "Please select first the target STM32L4xx device used in your application (in stm32l4xx.h file)"
        #endif
        //if counter overflowed, one second has elapsed.
//        if (HAL_IS_BIT_SET(SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk))
//        {
//            millisecondsElapsed += 1000;
//        }

        if (status != HAL_OK)
        {
            break;
        }

    }

    HAL_FLASH_Lock();

//    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);

    //get number of milliseconds.

//    millisecondsElapsed += SysTick->VAL / (HAL_RCC_GetHCLKFreq() / 1000);
//    millisecondsElapsed = 1000 - millisecondsElapsed;       //Systick counts downwards.
//    millisecondsElapsed++;

//    SysTick->VAL = (HAL_RCC_GetHCLKFreq()/1000) - 1;
//    SysTick->LOAD = (HAL_RCC_GetHCLKFreq()/1000) - 1;

//    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
//    SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);

    HAL_IncTicks(millisecondsElapsed);      //to account for inaccuracy caused by systick being suspended.

    for (; millisecondsElapsed > 0; millisecondsElapsed--)
    {
        xTaskIncrementTick();
    }
    return status;

}


/**
 * @brief				Erase Pages in the internal FLASH
 * @param startAddr		Absolute memory Address in the Internal FLASH. This value is then translated to
 * 						which from page is the data going to be erased. Does not need to be at boundary of
 * 						a page.
 * @param dataLen		Size of memory in bytes to be cleared. This is used to calculate the number of\
 * 						pages to be erased.
 * @return				#HAL_StatusTypeDef
 */
HAL_StatusTypeDef HAL_FLASH_ErasePages(uint32_t startAddr, uint32_t dataLen)
{
    FLASH_EraseInitTypeDef FLASHEraseStruct;
    uint32_t Error;
    uint32_t retries = 5;
    HAL_StatusTypeDef status;
      if((startAddr - FLASH_BASE) >= FLASH_SIZE)     // sector size is 4KB in this area // end of Flash                                
      {
          return HAL_ERROR;
      }

    #if defined(STM32L452xx) 

      FLASHEraseStruct.NbPages = dataLen + FLASH_PAGE_SIZE - 1;   // calculate the 
      FLASHEraseStruct.NbPages = (FLASHEraseStruct.NbPages / FLASH_PAGE_SIZE);    
      FLASHEraseStruct.TypeErase = FLASH_TYPEERASE_PAGES;
      FLASHEraseStruct.Page = ((startAddr & (FLASH_SIZE-1)) / FLASH_PAGE_SIZE);
      FLASHEraseStruct.Banks = ((startAddr & (FLASH_SIZE-1)) / FLASH_BANK_SIZE);
      
    #elif defined(STM32L151xBA)
      
      FLASHEraseStruct.NbPages = dataLen + FLASH_PAGE_SIZE - 1;   // calculate the 
      FLASHEraseStruct.NbPages = (FLASHEraseStruct.NbPages / FLASH_PAGE_SIZE);    
      FLASHEraseStruct.TypeErase = FLASH_TYPEERASE_PAGES;
      FLASHEraseStruct.PageAddress = startAddr;

    #elif defined(STM32F205xx)
      
      FLASHEraseStruct.NbSectors = 1 + (HAL_FLASH_get_Sector(startAddr + dataLen - 1) - HAL_FLASH_get_Sector(startAddr));
      FLASHEraseStruct.Sector    = HAL_FLASH_get_Sector(startAddr);

      FLASHEraseStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
      FLASHEraseStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;   
      
    #else
      #error "Please select first the target STM32L4xx device used in your application (in stm32l4xx.h file)"
    #endif
    
    do
    {
        HAL_FLASH_Unlock();
        status = HAL_FLASHEx_Erase(&FLASHEraseStruct, &Error);
        HAL_FLASH_Lock();

    }while (status != HAL_OK && Error != 0xFFFFFFFF && retries--);

    return status;

}


/** @}*/
