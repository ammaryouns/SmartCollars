/**
 * @file   	FwFile.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Nov 25, 2016
 * 
 * @brief   
 */


#include "FwFile.h"
#include "Flash_Memory_Map.h"
#include "crc32.h"
#include "mFlash.h"
#include <string.h>
#include <stdbool.h>
#include "DEBUG_UART.h"
/**
 * @defgroup
 * @{
 */
 


/** @}*/



/**
 * @defgroup
 * @{
 */

/**
 *
 * @param fwFile
 * @param offset
 * @param srcAddr
 * @param length
 * @return
 */
__INLINE mFile_Return_t FwFile_write(const mFile_t* fwFile, uint32_t offset, uint8_t* srcAddr, uint32_t length)
{
    return fwFile->ifc->mem_write(fwFile, offset, srcAddr, length);
}


/**
 *
 * @param fwFile
 * @param offset
 * @param destAddr
 * @param length
 * @return
 */
__INLINE mFile_Return_t FwFile_read(const mFile_t* fwFile, uint32_t offset, uint8_t* destAddr, uint32_t length)
{
    return fwFile->ifc->mem_read(fwFile, offset, destAddr, length);
}

/**
 *
 * @param fwFile
 * @return
 */
__INLINE mFile_Return_t FwFile_erasep(const mFile_t* fwFile, uint32_t startAddr, uint32_t dataLen )
{
   // lemented yet;
    return HAL_FLASH_ErasePages(startAddr, dataLen);
}


/**
 *
 * @param fwFile
 * @return
 */
__INLINE mFile_Return_t FwFile_erase(const mFile_t* fwFile)
{
    return fwFile->ifc->erase(fwFile);
}


/**
 *
 * @param fwFile
 * @param fwHeader
 * @return
 */
__INLINE mFile_Return_t FwFile_getHeader(const mFile_t* fwFile, Firmware_Header_t* fwHeader)
{
    return FwFile_read(fwFile, 0, (uint8_t*)fwHeader, sizeof(Firmware_Header_t));
}


/**
 *
 * @param fwFile
 * @return
 */
__INLINE mFile_Return_t FwFile_delete(const mFile_t* fwFile)
{
    Firmware_Header_t fwHeader;
    memset(&fwHeader, 0, sizeof(Firmware_Header_t));
    
    FwFile_erase(fwFile);
    return FwFile_write(fwFile, 0, (uint8_t*)&fwHeader, sizeof(Firmware_Header_t) & 0xFFFFFFFC);
}

/**
 *
 * @param fwFile
 * @param offset
 * @param length
 * @return
 */
__INLINE uint32_t FwFile_computeCRC(const mFile_t* fwFile, uint32_t offset, uint32_t length)
{
    return fwFile->ifc->computeCRC(fwFile, offset, length);
}

/**
 *
 * @param fwFile
 * @return
 */
mFile_Return_t FwFile_verify(const mFile_t* fwFile)
{
    Firmware_Header_t fwHeader;
    uint32_t crc = 0;
    
    FwFile_getHeader(fwFile, &fwHeader);
    
    if(fwHeader.size == 0)
        return mFile_OK;
    
    if (fwHeader.size > fwFile->maxSize)
        goto error;
    
    crc = FwFile_computeCRC(fwFile, 4, fwHeader.size - 4);
    
    if (crc != fwHeader.crc)
        goto error;

    return mFile_OK;

    error:
    return FwFile_delete(fwFile);        
}


/**
 *
 * @param fwHeader
 * @return
 */
mFile_Return_t FwFile_printHeader(const Firmware_Header_t* fwHeader)
{
    printDEBUG("\nCRC        : %08X\n", fwHeader->crc);
    printDEBUG("Version      : %.2f\n", ((float)fwHeader->version)/100);
    printDEBUG("Size         : %u\n", fwHeader->size);
    printDEBUG("Device Type  : %u\n", fwHeader->deviceType);
    printDEBUG("App ID       : %08X\n", fwHeader->appID);
    
    return mFile_OK;
}
/** @}*/
