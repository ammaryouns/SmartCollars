/**
 * @file   	mQueue.h
 * @author 	Hassan Abid
 * @version	1.0.0
 * @date	Mar 16, 2016
 *
 * @brief
 */

#ifndef MQUEUE_H_
#define MQUEUE_H_

#include <string.h>
#include <stdint.h>



/**
 * @defgroup mQueue_Public_Macros		mQueue Public Macro Definitions
 * @{
 */

#define mQueue_isFull(q)             (mQueue_get_Count(q) == mQueue_get_Size(q))
#define mQueue_isEmpty(q)            (mQueue_get_Count(q) == 0)


#define mQueue_ctor( elementType, queueSize, buffer, qIfc)  \
{\
\
    .count = 0,\
    .front = 0,\
    .back  = 0,\
    .size  = queueSize,\
    .elementSize = sizeof(elementType),\
    .buff  = (uint8_t*)buffer,\
    .ifc   = &qIfc,\
\
}\



#define mQueue_Def( name, elementType, queueSize, qIfc )     \
static elementType name##_buffer[queueSize];\
mQueueStruct_t name = mQueue_ctor(elementType, queueSize, name##_buffer, qIfc);\




/** @}*/

/**
 * @defgroup mQueue_Public_Enums		mQueue Public Enum Definitions
 * @{
 */

/**
 * @brief enum for return values from the mQueue functions
 *
 */
typedef enum{

	mQueue_OK = 0,	/*!< function executed successfully*/
	mQueue_ERROR	/*!< there was an error while executing the function*/

}mQueueReturn_t;

/** @}*/

/**
 * @defgroup mQueue_Public_Structs		mQueue Public Struct Definitions
 * @{
 */


struct mQueueStruct_t;
typedef  struct mQueueStruct_t mQueueStruct_t;

typedef struct{
    
        mQueueReturn_t (* const init)         (mQueueStruct_t* q);
        mQueueReturn_t (* const mem_read)     (mQueueStruct_t* q, uint8_t* destAddr, uint8_t* srcAddr, uint32_t len);
        mQueueReturn_t (* const mem_write)    (mQueueStruct_t* q, uint8_t* destAddr, uint8_t* srcAddr, uint32_t len);
        mQueueReturn_t (* const pullStruct)	  (mQueueStruct_t* q);
        mQueueReturn_t (* const commitStruct) (mQueueStruct_t* q);
    
}mQueue_Interface_t;

/**
 * @brief	structure to implement and manipulate
 * 			a character buffer in FIFO fashion (Queue).
 *
 * @note	The Queue is circular and oldest entries
 * 			are overwritten on overflow.
 *
 * @note   	The struct members {count, front, back, size} must be of the
 * 			same size.
 */

struct mQueueStruct_t{

	uint8_t elementSize;	/*!< size of the elements in queue in bytes.*/
    uint16_t count;			/*!< number of elements currently stored in queue*/
    uint16_t front;			/*!< index at which the next element will be pushed*/
	uint16_t back;			/*!< index from which the next element is to be popped */
	uint16_t size;			/*!< maximum number of elements queue can hold*/
	uint8_t* buff;			/*!< pointer to the character buffer for storing elements.*/
    
    const mQueue_Interface_t* const ifc;
    
};

/** @}*/


extern const mQueue_Interface_t mQueue_ifc;

#ifdef __cplusplus
extern "C" {
#endif


mQueueReturn_t mQueue_push		            (mQueueStruct_t* q, const void* element);
mQueueReturn_t mQueue_push_Front            (mQueueStruct_t* q, const void* element);
mQueueReturn_t mQueue_peekAt                (mQueueStruct_t* q, uint16_t index, void* const pRetMemAddr);
mQueueReturn_t mQueue_peek		            (mQueueStruct_t* q, void* const pRetMemAddr);
mQueueReturn_t mQueue_pop 		            (mQueueStruct_t* q, void* const pRetMemAddr);
mQueueReturn_t mQueue_popMultiple           (mQueueStruct_t* q, uint16_t count);
mQueueReturn_t mQueue_popMultiple_Back      (mQueueStruct_t* q, uint16_t count);
mQueueReturn_t mQueue_peekAt_Multiple       (mQueueStruct_t* q, uint16_t startIndex, uint16_t count, void* const pRetMemAddr);
mQueueReturn_t mQueue_peekAt_Multiple_Back  (mQueueStruct_t* q, uint16_t startIndex, uint16_t count, void* const pRetMemAddr);

mQueueReturn_t mQueue_push_Multiple         (mQueueStruct_t* q, uint16_t count, const void* elementArr);
mQueueReturn_t mQueue_push_Multiple_Front   (mQueueStruct_t* q, uint16_t count, const void* elementArr);


    
///default initializers for Queue interface functions.
mQueueReturn_t mQueue_init		 	(mQueueStruct_t* q);
uint16_t       mQueue_get_Count 	(mQueueStruct_t* q);
uint16_t       mQueue_get_Size   	(mQueueStruct_t* q);
    
    
#ifdef __cplusplus
}
#endif

#endif /* MQUEUE_H_ */
