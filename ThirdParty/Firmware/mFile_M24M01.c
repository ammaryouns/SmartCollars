/**
 * @file   	mFile_M24M01.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Dec 29, 2016
 * 
 * @brief   
 */


#include "mFile_M24M01.h"
#include "M24M01.h"
#include "crc32.h"

/**
 * @defgroup
 * @{
 */


mFile_Return_t mFile_M24M01_write       (const mFile_t* file, uint32_t offset, uint8_t* srcAddr,  uint32_t length);
mFile_Return_t mFile_M24M01_read        (const mFile_t* file, uint32_t offset, uint8_t* destAddr, uint32_t length);
mFile_Return_t mFile_M24M01_erase       (const mFile_t* file);
uint32_t       mFile_M24M01_computeCRC  (const mFile_t* file, uint32_t offset, uint32_t length);

const mFile_Interface_t mFile_M24M01_ifc = {

		.mem_write  = mFile_M24M01_write,
		.mem_read   = mFile_M24M01_read,
        .erase      = mFile_M24M01_erase,
        .computeCRC = mFile_M24M01_computeCRC
};

/** @}*/


/**
 * @defgroup
 * @{
 */

/**
 * @brief			Wrapper function for writing file to M24M01 EEPROM.
 * @param destAddr
 * @param srcAddr
 * @param length
 * @return
 */
mFile_Return_t mFile_M24M01_write(const mFile_t* file, uint32_t offset, uint8_t* srcAddr, uint32_t length)
{
    uint32_t errorRetries = 3;
	///@todo Add preprocessing code for writing data to flash.
	/// 		like erasing the region, and checking input params
	while (errorRetries && HAL_OK != M24M01_Write((uint32_t)(file->memAddr + offset), srcAddr, length))
	{
        errorRetries--;
        HAL_Delay(1);
	}
    
    if(errorRetries == 0)
    {
        return mFile_ERROR;
    }
    
	return mFile_OK;
}


/**
 * @brief           Wrapper function to read file from M24M01 EEPROM
 * @param destAddr
 * @param srcAddr
 * @param length
 * @return
 */
mFile_Return_t mFile_M24M01_read(const mFile_t* file, uint32_t offset, uint8_t* destAddr, uint32_t length)
{
    uint32_t errorRetries = 3;
	///@todo Add preprocessing code for writing data to flash.
	/// 		like erasing the region, and checking input params
	while (errorRetries && HAL_OK != M24M01_Read((uint32_t)(file->memAddr + offset), destAddr, length))
	{
        errorRetries--;
        HAL_Delay(1);
	}
    
    if(errorRetries == 0)
    {
        return mFile_ERROR;
    }
    
	return mFile_OK;
}


/**
 *
 * @param file
 * @return
 */
mFile_Return_t mFile_M24M01_erase(const mFile_t* file)
{
    return mFile_OK;
}

uint32_t mFile_M24M01_computeCRC(const mFile_t* file, uint32_t offset, uint32_t length)
{
    static uint8_t buffer[256];
    uint32_t chunkLength = 0;
    uint32_t crc = CRC_SEED;
    
    
    if (offset > file->maxSize)
        return CRC_SEED;
    
    if (offset + length > file->maxSize)
        length = file->maxSize - offset;

    
    HAL_CRC_RESET(&hcrc);
    while (length > 0)
    {
        chunkLength = sizeof(buffer);
        if (chunkLength > length)
            chunkLength = length;
        
        if (mFile_OK != file->ifc->mem_read(file, offset, buffer, chunkLength))
            return crc;
        
        crc = HAL_CRC_Calculate_ByteStream(&hcrc, buffer, chunkLength, true);
        
        length -= chunkLength;
        offset += chunkLength;
    }    
    
    return crc;
}

/** @}*/

