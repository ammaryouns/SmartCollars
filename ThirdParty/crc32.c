#include "crc32.h"


#define POLY 0x4C11DB7

// note remember to feed size (4 for crc32) zeroes through the slow
// crc algorithm to make the fast and the slow compute the same checksum


uint32_t _slow_crc32(uint32_t sum, uint8_t *p, uint32_t len)
{

	while (len)
	{
		len--;
		int i;
		uint32_t byte = *(uint32_t*)p;
		p += 4;

		sum ^= byte;

		

		for (i = 0; i < 4 * 8; ++i)
		{
			uint32_t osum = sum;
			sum <<= 1;
			//      if (byte & 0x80)
			//        sum |= 1 ;
			//printf("%d, %08X\n", (osum & 0x80000000) == 0x80000000, osum);
			if (osum & 0x80000000)
				sum ^= POLY;
			//byte <<= 1;
		}
	}
	return sum;
}

uint32_t slow_crc32(uint32_t sum, uint8_t *p, uint32_t len)
{

	uint32_t pad = 0;
	uint8_t padCount = (len % 4);
	
	len /= 4;
	sum = _slow_crc32(sum, p, len);

    if (padCount > 0){

		memcpy((uint8_t*)&pad, &p[len*4], padCount);

		sum = _slow_crc32(sum, (uint8_t*)&pad, 1);
	}

	return sum;
}




uint32_t HAL_CRC_Calculate_ByteStream(CRC_HandleTypeDef *hcrc, uint8_t p[], uint32_t len, bool accumulate)
{

    uint32_t pad = 0;
    uint8_t padCount = len%4;
    
    uint32_t crc = 0;
    len /= 4;
    
    if (accumulate)
        crc = HAL_CRC_Accumulate(hcrc, (uint32_t*)p, len);
    else
        crc = HAL_CRC_Calculate(hcrc, (uint32_t*)p, len);
    
    if (padCount){

        memcpy((uint8_t*)&pad, &p[len*4], padCount);

        crc = HAL_CRC_Accumulate(hcrc, (uint32_t*)&pad, 1);

    }
    return crc;
}
