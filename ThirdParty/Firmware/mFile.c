/**
 * @file   	mFile.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Nov 25, 2016
 * 
 * @brief   
 */


#include "mFile.h"
#include "mFlash.h"
#include "crc32.h"


mFile_Return_t mFile_Flash_write  (const mFile_t* file, uint32_t offset, uint8_t* srcAddr,  uint32_t length);
mFile_Return_t mFile_Flash_read   (const mFile_t* file, uint32_t offset, uint8_t* destAddr, uint32_t length);
mFile_Return_t mFile_Flash_erase  (const mFile_t* file);
uint32_t mFile_Flash_computeCRC   (const mFile_t* file, uint32_t offset, uint32_t length);


const mFile_Interface_t mFile_Flash_ifc = {

		.mem_write  = mFile_Flash_write,
		.mem_read   = mFile_Flash_read,
        .erase      = mFile_Flash_erase,
        .computeCRC = mFile_Flash_computeCRC
};

/**
 * @defgroup
 * @{
 */

/**
 * @brief			Wrapper function for writing file to flash.
 * @param destAddr
 * @param srcAddr
 * @param length
 * @return
 */
mFile_Return_t mFile_Flash_write(const mFile_t* file, uint32_t offset, uint8_t* srcAddr, uint32_t length)
{
    uint32_t errorRetries = 3;
	///@todo Add preprocessing code for writing data to flash.
	/// 		like erasing the region, and checking input params
	while (errorRetries && HAL_OK != HAL_FLASH_Write((uint32_t)(file->memAddr + offset), srcAddr, length))
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
 * @param destAddr
 * @param srcAddr
 * @param length
 * @return
 */
mFile_Return_t mFile_Flash_read(const mFile_t* file, uint32_t offset, uint8_t* destAddr, uint32_t length)
{
	memcpy(destAddr, (uint8_t*)(file->memAddr + offset), length);

	return mFile_OK;
}

/**
 * @brief           Erase flash memory to write new file data.
 * @param file
 * @return
 */
mFile_Return_t mFile_Flash_erase(const mFile_t* file)
{
    uint32_t errorRetries = 3;
	///@todo Add preprocessing code for writing data to flash.
	/// 		like erasing the region, and checking input params
	while (errorRetries && HAL_OK != HAL_FLASH_ErasePages(file->memAddr, file->maxSize))
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
 * @param offset    
 * @param length
 * @return
 */
uint32_t mFile_Flash_computeCRC(const mFile_t* file, uint32_t offset, uint32_t length)
{
    if (offset > file->maxSize)
        return CRC_SEED;
    
    if (offset + length > file->maxSize)
        length = file->maxSize - offset;
    
    return HAL_CRC_Calculate_ByteStream(&hcrc, (uint8_t*)(file->memAddr + offset), length, false);
}
/** @}*/

