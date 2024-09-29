/**
 * @file   	mFile.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Nov 25, 2016
 * 
 * @brief   
 */

#ifndef MFILE_H_
#define MFILE_H_


#include <stdint.h>


#define mFile_Def( name, address, size, interface)        \
mFile_t name = {\
\
    .memAddr = address,\
    .maxSize = size,\
    .ifc     = interface,\
\
};\


/**
 * @defgroup
 * @{
 */

typedef enum
{
	mFile_OK 		= 0,
	mFile_ERROR,
	mFile_TIMEOUT

}mFile_Return_t;

/** @}*/


/**
 * @defgroup
 * @{
 */

typedef struct mFile_s mFile_t;

typedef struct{

	mFile_Return_t	(*mem_write)    (const mFile_t* file, uint32_t offset, uint8_t* srcAddr, uint32_t length);
	mFile_Return_t	(*mem_read)     (const mFile_t* file, uint32_t offset, uint8_t* destAddr, uint32_t length);
    mFile_Return_t  (*erase)        (const mFile_t* file);
    uint32_t        (*computeCRC)   (const mFile_t* file, uint32_t offset, uint32_t length);
    
}mFile_Interface_t;

typedef struct mFile_s{

    const uint32_t memAddr;
    const uint32_t maxSize;
    const mFile_Interface_t* const ifc;
    
}mFile_t;


/** @}*/

extern const mFile_Interface_t mFile_Flash_ifc;

#ifdef __cplusplus
extern "C"{
#endif 


#ifdef __cplusplus
}
#endif

#endif /* MFILE_H_ */
