/**
 * @file   	mFLASH.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Oct 26, 2016
 * 
 * @brief   
 */

#ifndef MFLASH_H_
#define MFLASH_H_

#include "Flash_Memory_Map.h"
#include "bsp.h"

#ifdef __cplusplus
extern "C"{
#endif 

HAL_StatusTypeDef HAL_FLASH_Write		(uint32_t destAddr, uint8_t* srcAddr, int32_t len);
HAL_StatusTypeDef HAL_FLASH_ErasePages	(uint32_t startAddr, uint32_t dataLen);

#ifdef __cplusplus
}
#endif

#endif /* MFLASH_H_ */
