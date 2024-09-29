#ifndef __FLASH_EEPROM_H_
#define __FLASH_EEPROM_H_

#include "bsp.h"





HAL_StatusTypeDef Flash_EEPROM_Write(uint32_t destAddrOffset, uint8_t* srcAddr, uint32_t len);

HAL_StatusTypeDef Flash_EEPROM_Read(uint32_t srcAddrOffset, uint8_t* destAddr, uint32_t len);


#endif
