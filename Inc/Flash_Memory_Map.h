/**
 * @file   	Flash_Memory_Map.h
 * @author 	Hassan Abid 
 * @version	v1.0.0
 * @date	Oct 25, 2016
 * 
 * @brief   This files defines the memory regions in the controller flash.
 */

#ifndef FLASH_MEMORY_MAP_H_
#define FLASH_MEMORY_MAP_H_

#include <stdint.h>

//                          |                   |
//                          |                   |
//                          |                   |
//               0x040000   +-------------------+
//                          |                   |        
//                          |                   |        
//               0x032000   +-------------------+------------------------ 256KB
//                          |                   |        
//                          |    Reserve        |        
//               0x039C00  +-------------------+
//                          |                   |        
//                          | ANN Parameters bkp|        
//                          |     20 KB         |        
//               0x034C00   +----      ---------+        
//                          |                   |
//                          |   Download        |
//                          |   Location        |
//                          |     For           |
//                          |  bkp Firmware     |
//                          |    (72 KB)        |
//                          |                   |
//               0x022000   +-------------------+
//                          |                   |
//                          |  Bootloader       |
//                          |     bkp           |
//                          |                   |
//               0x020000   +-------------------+------------------------ 128KB
//                          |                   |
//                          | CONSTANTS         |
//                          |  Parmeters        |
//               0x01EC00   +-------------------+
//                          |                   |
//                          |    Reserve        |        
//                          |                   |
//               0x019000   +-------------------+--------+
//                          |                   |        |
//                          |  ANN Parameters   |        |
//                          |     20 KB         |        |
//                  0x14C00 +----      ---------+        |
//                          |                   |        |
//                          |    Active         |        |
//                          |   Firmware        |
//                          |    (74.5KB)       |       95 KB 
//                          |                   |        |
//                          |                   |        |
//                  0x02200 +----      ---------+        |
//                          |   Header (0.5 KB) |        |
//               0x002000   +-------------------+--------+
//                          |                   |
//                          |  Bootloader (8 KB)|  
//                          |                   |
//               0x000000   +-------------------+------------------------   0KB



#define FLASH_FW_CRC_OFFSET                 ((uint32_t)0x00000000)
#define FLASH_FW_VER_OFFSET                 ((uint32_t)FLASH_FW_CRC_OFFSET    + 4 )
#define FLASH_FW_SIZE_OFFSET                ((uint32_t)FLASH_FW_VER_OFFSET    + 4 )
#define FLASH_FW_TYPE_OFFSET                ((uint32_t)FLASH_FW_SIZE_OFFSET   + 4 )
#define FLASH_FW_APP_ID_OFFSET              ((uint32_t)FLASH_FW_TYPE_OFFSET   + 4 )
#define FLASH_FW_CDT_OFFSET                 ((uint32_t)FLASH_FW_APP_ID_OFFSET + 4 )
#define FLASH_FW_VECT_OFFSET                ((uint32_t)0x200)       //VECT_TAB_OFFSET should be a multiple of 200.

#define FLASH_FW_HEADER_SIZE                ((uint32_t)FLASH_FW_VECT_OFFSET)

#define FLASH_FW_BaseAddr                   ((uint32_t)0x08002000)
#define FLASH_FW_CRC_ADDR                   ((uint32_t)FLASH_FW_BaseAddr + FLASH_FW_CRC_OFFSET    )
#define FLASH_FW_VER_ADDR                   ((uint32_t)FLASH_FW_BaseAddr + FLASH_FW_VER_OFFSET    )
#define FLASH_FW_SIZE_ADDR                  ((uint32_t)FLASH_FW_BaseAddr + FLASH_FW_SIZE_OFFSET   )
#define FLASH_FW_TYPE_ADDR                  ((uint32_t)FLASH_FW_BaseAddr + FLASH_FW_TYPE_OFFSET   )
#define FLASH_FW_APP_ID_ADDR                ((uint32_t)FLASH_FW_BaseAddr + FLASH_FW_APP_ID_OFFSET )
#define FLASH_FW_CDT_ADDR                   ((uint32_t)FLASH_FW_BaseAddr + FLASH_FW_CDT_OFFSET    )
#define FLASH_FW_VECT_ADDR                  ((uint32_t)FLASH_FW_BaseAddr + FLASH_FW_VECT_OFFSET   )
#define FLASH_FW_MAX_SIZE                   ((uint32_t)(128*1024))   

#define FLASH_ANN_PARAM_SIZE                ((uint32_t)(20*1024))
#define FLASH_ANN_PARAM_ADDR                ((uint32_t)(0x08014C00))


#define FLASH_FW_CONST_REGION_SIZE          ((uint32_t)(5*1024))
#define FLASH_FW_CONST_REGION_ADDR          ((uint32_t)(0x08030000))

#define FLASH_FW_PARAM_ADDR                 ((uint32_t)FLASH_FW_CONST_REGION_ADDR)
#define FLASH_FW_PARAM_SIZE                 ((uint32_t)2*1024) //Must be multiple of sector size

#define FLASH_FW_BKUP_PARAM_ADDR            ((uint32_t)(FLASH_FW_PARAM_ADDR + FLASH_FW_PARAM_SIZE))
#define FLASH_FW_BKUP_PARAM_SIZE            ((uint32_t)FLASH_FW_PARAM_SIZE)


//#define FLASH_FW2_SIZE                      (*(uint32_t*)FLASH_FW2_SIZE_ADDR)
//#define FLASH_FW2_CRC_OFFSET                ((uint32_t)(FLASH_FW2_SIZE - 4))

#define FLASH_FW2_BaseAddr                  ((uint32_t)0x08040000)
#define FLASH_FW2_CRC_ADDR                  ((uint32_t)FLASH_FW2_BaseAddr + FLASH_FW_CRC_OFFSET   )
#define FLASH_FW2_VER_ADDR                  ((uint32_t)FLASH_FW2_BaseAddr + FLASH_FW_VER_OFFSET   )
#define FLASH_FW2_SIZE_ADDR                 ((uint32_t)FLASH_FW2_BaseAddr + FLASH_FW_SIZE_OFFSET  )
#define FLASH_FW2_TYPE_ADDR                 ((uint32_t)FLASH_FW2_BaseAddr + FLASH_FW_TYPE_OFFSET  )
#define FLASH_FW2_APP_ID_ADDR               ((uint32_t)FLASH_FW2_BaseAddr + FLASH_FW2_APP_ID_ADDR )
#define FLASH_FW2_CDT_ADDR                  ((uint32_t)FLASH_FW2_BaseAddr + FLASH_FW_CDT_OFFSET   )
#define FLASH_FW2_VECT_ADDR                 ((uint32_t)FLASH_FW2_BaseAddr + FLASH_FW_VECT_OFFSET  )
#define FLASH_FW2_MAX_SIZE                  ((uint32_t)(FLASH_FW_MAX_SIZE))

#define FLASH_ANN_PARAM2_SIZE               FLASH_ANN_PARAM_SIZE
#define FLASH_ANN_PARAM2_ADDR               ((uint32_t)(0x08034C00))




extern const uint32_t NODE_FW_VERSION       __attribute__((at(FLASH_FW_VER_ADDR)));
extern const uint32_t NODE_TYPE             __attribute__((at(FLASH_FW_TYPE_ADDR)));
extern const uint32_t NODE_APP_ID           __attribute__((at(FLASH_FW_APP_ID_ADDR)));



#endif /* FLASH_MEMORY_MAP_H_ */
