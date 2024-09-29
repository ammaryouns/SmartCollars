/**
 * @file   	mBuffer.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Nov 28, 2016
 * 
 * @brief   
 */

#ifndef MBUFFER_H_
#define MBUFFER_H_

#include <stdint.h>
#include <string.h>

/**
 * @defgroup
 * @{
 */

typedef enum
{
    BUFFER_STATE_EMPTY = 0u,
    BUFFER_STATE_DATA,
    BUFFER_STATE_FULL,

}Buffer_State_t;

/** @}*/



/**
 * @defgroup
 * @{
 */

#define mBuffer_Def(varName, bufferSize )  	\
\
struct\
{\
    Buffer_State_t state;\
    uint16_t counter;\
    uint16_t index;\
    uint16_t size;\
    uint8_t buffer[bufferSize];\
\
} varName


#define mBuffer_ctor(varName)	\
	 { .counter = 0,  .index = 0, .size = sizeof(varName.buffer), .state = BUFFER_STATE_EMPTY }\


typedef mBuffer_Def( mBuffer_t, 1);

#define mBuffer_clear(buff)    memset(((mBuffer_t*)buff)->buffer, 0, ((mBuffer_t*)buff)->size)    
/** @}*/



#ifdef __cplusplus
extern "C"{
#endif 

void mBuffer_init(mBuffer_t* buffer);
void mBuffer_set_State(mBuffer_t* buffer);


#ifdef __cplusplus
}
#endif

#endif /* MBUFFER_H_ */
