/**
 * @file   	mQueue_List.h
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Jan 26, 2017
 * 
 * @brief   
 */

#ifndef MQUEUE_LIST_H_
#define MQUEUE_LIST_H_

#include "mQueue.h"


/**
 * @defgroup mQueue_List_Public_Macros				mQueue List Public Macros
 * @{
 */

#define mQueue_List_Element_Def( name, elementType, queueSize, buffer, qIfc )	\
\
mQueue_List_Element_t name =\
{\
		.q = mQueue_ctor(elementType, queueSize, buffer, qIfc),\
		.next = NULL,\
        .prev = NULL,\
\
};\

#define mQueue_List_Element_Def_AT( name, elementType, queueSize, buffer, qIfc, address )	\
\
mQueue_List_Element_t name __attribute__((section ("RW_IRAM2"))) =\
{\
		.q = mQueue_ctor(elementType, queueSize, buffer, qIfc),\
		.next = NULL,\
        .prev = NULL,\
\
};\


#define mQueue_List_Def( name ) \
\
mQueue_List_t name = {\
\
    .head = NULL,\
\
};\

/** @}*/

/**
 * @defgroup mQueue_List_Public_Structs				mQueue List Structures Definitions
 * @{
 */


typedef struct mQueue_List_Element_s mQueue_List_Element_t;

/**
 *
 */
typedef struct mQueue_List_Element_s{

	mQueueStruct_t q;
	mQueue_List_Element_t* next;
    mQueue_List_Element_t* prev;
    
}mQueue_List_Element_t;

/**
 *
 */
typedef struct{

	mQueue_List_Element_t* head;

}mQueue_List_t;

/** @}*/


#ifdef __cplusplus
extern "C"{
#endif 

mQueueReturn_t mQueue_List_AddQueue        (mQueue_List_t*, mQueue_List_Element_t*);
mQueueReturn_t mQueue_List_push            (mQueue_List_t*, const void*);
mQueueReturn_t mQueue_List_pull            (mQueue_List_t*, const void*);
mQueueReturn_t mQueue_List_peekAt_Back     (mQueue_List_t*, uint32_t, void* const);
mQueueReturn_t mQueue_List_PopMultiple_Back(mQueue_List_t* list, uint32_t count);

uint32_t mQueue_List_get_elementCount      (mQueue_List_t* list);


#ifdef __cplusplus
}
#endif

#endif /* MQUEUE_LIST_H_ */
