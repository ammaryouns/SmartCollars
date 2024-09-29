#ifndef _CRC32_H_
#define _CRC32_H_
#include "bsp.h"
#include "crc.h"


#define CRC_SEED 0x4C11DB7

#define HAL_CRC_RESET(__HANDLE__)  __HAL_CRC_DR_RESET(__HANDLE__)

uint32_t slow_crc32(uint32_t sum, uint8_t *p, uint32_t len);
uint32_t HAL_CRC_Calculate_ByteStream(CRC_HandleTypeDef *hcrc, uint8_t p[], uint32_t len, bool accumulate);

#endif
