/**
 * @file   	mQueue.c
 * @author 	Hassan Abid
 * @version	1.0.0
 * @date	Mar 16, 2016
 *
 * @brief
 */



#include "mQueue.h"


mQueueReturn_t __mQueue_init      	(mQueueStruct_t* q);
mQueueReturn_t __mQueue_pullStruct 	(mQueueStruct_t* q);
mQueueReturn_t __mQueue_commitStruct(mQueueStruct_t* q);
mQueueReturn_t __mQueue_memcpy      (mQueueStruct_t* q, uint8_t* destAddr, uint8_t* srcAddr, uint32_t len);

/**
 * @defgroup mQueue_Exported_Members			mQueue Exported Members
 * @{
 */

/**
 *
 */
const mQueue_Interface_t mQueue_ifc = 
{

    .init           = __mQueue_init,
	.mem_read	    = __mQueue_memcpy,
	.mem_write      = __mQueue_memcpy,
	.pullStruct     = __mQueue_pullStruct,
	.commitStruct	= __mQueue_commitStruct,
};


/** @}*/

/**
 * @defgroup mQueue_private_Function mQueue Private Functions
 * @{
 */

/**
 * @brief calculate the corresponding index in the character buffer
 * 		  for the given element index in queue.
 * @param queueIndex 	index of element in queue.
 * @param elementSize   size of elements in queue.
 * @return the char buffer index for the given element index.
 */
static __inline uint32_t getCharBufferIndex(uint16_t queueIndex, uint16_t elementSize)
{
	return (queueIndex*elementSize);
}

/**
 * @brief			 	Increment and wrap the input queue index.
 * @param queueIndex	current value of index
 * @param queueSize		max number of elements in Queue.
 * @return				new queue index after increment.
 */
static __inline uint16_t incrementQueueIndex(uint16_t queueIndex, uint16_t queueSize)
{
	return ((queueIndex + 1) % queueSize);
}

/**
 * @brief			 	Decrement and wrap the input queue index.
 * @param queueIndex	current value of index
 * @param queueSize		max number of elements in Queue.
 * @return				new queue index after Decrement.
 */
static __inline uint16_t decrementQueueIndex(uint16_t queueIndex, uint16_t queueSize)
{
	return (((queueIndex + queueSize) - 1) % queueSize);
}

/**
 * @brief 			increment the count of number of elements currently in Queue
 * @param count		number of elements in queue.
 * @param queueSize max number of elements in queue.
 * @return			New count of elements. Same as before if queue full or
 * 					incremented by one.
 */
static __inline uint16_t incrementCount(uint16_t count, uint16_t queueSize){

	return (count >= queueSize ? count : count+1);
}

/**
 *
 * @param q
 * @param destAddr
 * @param srcAddr
 * @param len
 * @return
 */
__inline mQueueReturn_t __mQueue_memcpy(mQueueStruct_t* q, uint8_t* destAddr, uint8_t* srcAddr, uint32_t len)
{
    memcpy(destAddr, srcAddr, len);
    return mQueue_OK;
}

/**
 *
 * @param q
 * @return
 */
__inline mQueueReturn_t __mQueue_commitStruct(mQueueStruct_t* q)
{
    return mQueue_OK;
}

/**
 *
 * @param q
 * @return
 */
__inline mQueueReturn_t __mQueue_pullStruct(mQueueStruct_t* q)
{
    return mQueue_OK;
}

/**
 *
 * @param q
 * @return
 */
__inline mQueueReturn_t __mQueue_init(mQueueStruct_t* q)
{
    return mQueue_OK;
}


/** @}*/


/**
 * @defgroup mQueue_public_functions mQueue Public Functions
 * @{
 */

/**
 * @brief 		Initialize the input queue struct.
 * @param q		pointer to the Queue struct to be initialized.
 * @return 		mQueue_OK
 *
 * @note 		The struct members {buff, elementSize and size} must
 * 				be initialize at compile time.
 */
 mQueueReturn_t mQueue_init(mQueueStruct_t* q)
{
	return q->ifc->init(q);
}


/**
 * @brief 			Add a new element to the back of the Queue.
 * @param q			pointer to queue
 * @param element	pointer to element which is to be added to queue.
 * @return			mQueue_OK
 *
 * @note 			This function does not check for overflow condition
 * 					and the oldest entries are overwritten incase of overflow.
 */
__inline mQueueReturn_t mQueue_push(mQueueStruct_t* q, const void* element)
{
    return mQueue_push_Multiple(q, 1, element);
}





/**
 * @brief 			Add multiple elements to the back of the Queue.
 * @param q			pointer to queue
 * @param count     Number of elements to the added to the queue
 * @param element	pointer to element which is to be added to queue.
 * @return			mQueue_OK
 *
 * @note 			This function does not check for overflow condition
 * 					and the oldest entries are overwritten incase of overflow.
 */
mQueueReturn_t mQueue_push_Multiple(mQueueStruct_t* q, uint16_t count, const void* elementArr)
{
    uint32_t iter = 0;
    mQueueReturn_t retStatus = mQueue_ERROR;
    
	if (q->ifc->pullStruct(q) != mQueue_OK)
		return mQueue_ERROR;

    while(count)
    {
        //copy the element into the buffer at the back of queue.
        retStatus = q->ifc->mem_write( q, 
                            q->buff + getCharBufferIndex(q->back, q->elementSize),
                            (uint8_t*)elementArr + getCharBufferIndex(iter, q->elementSize),
                            q->elementSize);

        if (retStatus != mQueue_OK)
            return retStatus;
        
        //increment back index
        q->back = incrementQueueIndex(q->back, q->size);

        //if the queue has overflowed, update the front of the
        //queue as well.
        if (mQueue_isFull(q))
        {
           q->front = incrementQueueIndex(q->front, q->size);
        }

        //increment element count
        q->count = incrementCount(q->count, q->size);
        
        iter++;
        count--;
    }
    
	return q->ifc->commitStruct(q);
}






/**
 * @brief 			Add a new element to the back of the Queue.
 * @param q			pointer to queue
 * @param element	pointer to element which is to be added to queue.
 * @return			mQueue_OK
 *
 * @note 			This function does not check for overflow condition
 * 					and the oldest entries are overwritten incase of overflow.
 */
__inline mQueueReturn_t mQueue_push_Front(mQueueStruct_t* q, const void* element)
{
    return mQueue_push_Multiple_Front(q, 1, element);
}



/**
 * @brief 			Add multiple elements to the front of the Queue.
 * @param q			pointer to queue
 * @param count     Number of elements to the added to the queue
 * @param element	pointer to element which is to be added to queue.
 * @return			mQueue_OK
 *
 * @note 			This function does not check for overflow condition
 * 					and the oldest entries are overwritten incase of overflow.
 */
mQueueReturn_t mQueue_push_Multiple_Front(mQueueStruct_t* q, uint16_t count, const void* elementArr)
{
    uint32_t iter = 0;
    
	if (q->ifc->pullStruct(q) != mQueue_OK)
		return mQueue_ERROR;

    while(count)
    {
        //decrement front index
        q->front = decrementQueueIndex(q->front, q->size);
        
        //copy the element into the buffer at the back of queue.
        q->ifc->mem_write(  q, 
                            q->buff + getCharBufferIndex(q->front, q->elementSize),
                            (uint8_t*)elementArr + getCharBufferIndex(iter, q->elementSize),
                            q->elementSize);

        

        //if the queue has overflowed, update the back of the
        //queue as well.
        if (mQueue_isFull(q))
        {
           q->back = decrementQueueIndex(q->back, q->size); 
        }

        //increment element count
        q->count = incrementCount(q->count, q->size);
        
        iter++;
        count--;
    }
    
	return q->ifc->commitStruct(q);
}



/**
 * @brief 			  read the element at the specified index in the queue, with reference
 * 		  			  from the front of the queue.(i.e 0 would return front of queue)
 * @param q			  pointer to queue structure.
 * @param index		  index offset of the element to read from the front of the queue.
 * @param pRetMemAddr pointer to the element buffer for output to be stored.
 * @return
 * 		@arg mQueue_OK 		if the index is with in bounds or
 * 		@arg mQueue_ERROR	if the index is out of bounds
 */
__inline mQueueReturn_t mQueue_peekAt(mQueueStruct_t* q, uint16_t index, void* const pRetMemAddr)
{
    return mQueue_peekAt_Multiple(q, index, 1, pRetMemAddr);
}



/**
 * @brief 			  Read multiple elements starting from the specified index in the queue, with reference
 * 		  			  from the front of the queue.(i.e 0 would return front of queue)
 * @param q			  pointer to queue structure.
 * @param startIndex  index offset of the element to read from the front of the queue.
 * @param count       Number of elements to be read from the queue.  
 * @param pRetMemAddr pointer to the element buffer for output to be stored.
 * @return
 * 		@arg mQueue_OK 		if the index is with in bounds or
 * 		@arg mQueue_ERROR	if the index is out of bounds
 */
mQueueReturn_t mQueue_peekAt_Multiple(mQueueStruct_t* q, uint16_t startIndex, uint16_t count, void* const pRetMemAddr)
{
    uint32_t index = 0;
    uint32_t iter = 0;
    mQueueReturn_t retStatus = mQueue_ERROR;
    
	if (q->ifc->pullStruct(q) == mQueue_OK)
	{
		if (startIndex + count > q->count)
			return mQueue_ERROR;
        //translate offset into queue index.
        index = (q->front + startIndex)%q->size;
        
        while (count)
        {
            
            //set the pointer to absolute memory address of element in buffer.
            retStatus = q->ifc->mem_read( q,
                                (uint8_t*)pRetMemAddr + getCharBufferIndex(iter, q->elementSize),
                                q->buff + getCharBufferIndex(index, q->elementSize),
                                q->elementSize );

            if(retStatus != mQueue_OK)
                return retStatus;
            
            count--;
            iter++;
            index = incrementQueueIndex(index, q->size);
        }
    }
    
    return mQueue_OK;
}


/**
 * @brief 			  Read multiple elements starting from the specified index in the queue, with reference
 * 		  			  from the front of the queue.(i.e 0 would return front of queue)
 * @param q			  pointer to queue structure.
 * @param startIndex  index offset of the element to read from the front of the queue.
 * @param count       Number of elements to be read from the queue.  
 * @param pRetMemAddr pointer to the element buffer for output to be stored.
 * @return
 * 		@arg mQueue_OK 		if the index is with in bounds or
 * 		@arg mQueue_ERROR	if the index is out of bounds
 */
mQueueReturn_t mQueue_peekAt_Multiple_Back(mQueueStruct_t* q, uint16_t startIndex, uint16_t count, void* const pRetMemAddr)
{
    uint32_t index = 0;
    uint32_t iter = 0;
    mQueueReturn_t retStatus = mQueue_ERROR;
    
	if (q->ifc->pullStruct(q) == mQueue_OK)
	{
		if (index + count > q->count)
			return mQueue_ERROR;
        
        
        //translate offset into queue index.
        index = ((q->back + q->size) - startIndex)%q->size;
        
        while (count)
        {
            //The back index always points to the next empty slot.
            //So the index should be decremented before reading the element
            //from the back.
            index = decrementQueueIndex(index, q->size);
            //set the pointer to absolute memory address of element in buffer.
            retStatus = q->ifc->mem_read( q,
                                (uint8_t*)pRetMemAddr + getCharBufferIndex(iter, q->elementSize),
                                q->buff + getCharBufferIndex(index, q->elementSize),
                                q->elementSize );

            if(retStatus != mQueue_OK)
                return retStatus;
            
            count--;
            iter++;
            
        }
    }
    
    return mQueue_OK;
}

/**
 * @brief 		 	  read the element at the front of the queue.
 * @param q			  pointer to queue struct.
 * @param pRetMemAddr pointer to return value, the memory address of the element.
 * @return
 * 		@arg mQueue_OK 		if there is an element in the queue.
 * 		@arg mQueue_ERROR	if the queue is empty.
 */
__inline mQueueReturn_t mQueue_peek(mQueueStruct_t* q, void* const pRetMemAddr)
{
	return mQueue_peekAt(q, 0, pRetMemAddr);
}

/**
 * @brief 	Remove and return the oldest element from the queue.
 * @return
 * 		@arg mQueue_OK 		if there is an element in the queue.
 * 		@arg mQueue_ERROR	if the queue is empty.
*/
mQueueReturn_t mQueue_pop(mQueueStruct_t* q, void* const pRetMemAddr)
{
	mQueueReturn_t ret = mQueue_ERROR;

	if (pRetMemAddr == NULL)
		return mQueue_popMultiple(q, 1);

	ret = mQueue_peek(q, pRetMemAddr);
	if (ret != mQueue_OK)
		return ret;

	return mQueue_popMultiple(q, 1);
}


/**
 * @brief 		Remove multiple elements from the queue.
 * @param q		pointer to queue struct
 * @param count number of elements to be removed from the queue
 * @return
 * 		@arg mQueue_OK		if elements were successfully removed
 * 		@arg mQueue_ERROR	if the number of elements to be removed is greater than
 * 							number of elements in queue
 */
mQueueReturn_t mQueue_popMultiple(mQueueStruct_t* q, uint16_t count)
{
    
	if (q->ifc->pullStruct(q) != mQueue_OK)
		return mQueue_ERROR;

    if (count == 0)
        return mQueue_OK;
    
	if (count > q->count)
		return mQueue_ERROR;

	q->count = q->count - count;
	q->front = (q->front + count)%q->size;

	return q->ifc->commitStruct(q);
}

/**
 * @brief 		Remove multiple elements from the back of the queue.
 * @param q		pointer to queue struct
 * @param count number of elements to be removed from the queue
 * @return
 * 		@arg mQueue_OK		if elements were successfully removed
 * 		@arg mQueue_ERROR	if the number of elements to be removed is greater than
 * 							number of elements in queue
 */
mQueueReturn_t mQueue_popMultiple_Back(mQueueStruct_t* q, uint16_t count)
{
    
	if (q->ifc->pullStruct(q) != mQueue_OK)
		return mQueue_ERROR;

    if (count == 0)
        return mQueue_OK;
    
	if (count > q->count)
		return mQueue_ERROR;

	q->count = q->count - count;
	q->back = ((q->back + q->size) - count)%q->size;

	return q->ifc->commitStruct(q);
}

/**
 * @brief		Returns the number of elements currently in the Queue
 * @param q		Pointer to queue handler structure
 * @return		Number of elements in the queue.
 */
__inline uint16_t mQueue_get_Count(mQueueStruct_t* q)
{
    return q->count;
}


/**
 * @brief		Get the maximum number of elements the queue can hold
 * @param q		Pointer to queue handler structure.
 * @return		Max number of elements that can be stored in the queue.
 */
__inline uint16_t mQueue_get_Size(mQueueStruct_t* q)
{
   return q->size;
}
/** @}*/
