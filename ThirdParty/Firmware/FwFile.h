/**
 * @file   	FwFile.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Nov 25, 2016
 * 
 * @brief   
 */

#ifndef FWFILE_H_
#define FWFILE_H_

#include "mFile.h"

/**
 * @defgroup
 * @{
 */


#pragma anon_unions
typedef union{

	__packed struct{

		uint32_t crc;
		uint32_t version;
		uint32_t size;
		uint32_t deviceType;
		uint32_t appID;
		uint8_t compileTimeStamp[25];
	};

	uint8_t buffer[1];

}Firmware_Header_t;

/** @}*/


#ifdef __cplusplus
extern "C"{
#endif 

mFile_Return_t FwFile_write     (const mFile_t* fwFile, uint32_t offset, uint8_t* srcAddr, uint32_t length);
mFile_Return_t FwFile_read      (const mFile_t* fwFile, uint32_t offset, uint8_t* destAddr, uint32_t length);
mFile_Return_t FwFile_getHeader (const mFile_t* fwFile, Firmware_Header_t* fwHeader);   
mFile_Return_t FwFile_erase     (const mFile_t* fwFile);    
mFile_Return_t FwFile_delete    (const mFile_t* fwFile);
mFile_Return_t FwFile_verify    (const mFile_t* fwFile);

uint32_t FwFile_computeCRC      (const mFile_t* fwFile, uint32_t offset, uint32_t length);

    
mFile_Return_t FwFile_printHeader(const Firmware_Header_t* fwHeader);
    


#ifdef __cplusplus
}
#endif

#endif /* FWFILE_H_ */
