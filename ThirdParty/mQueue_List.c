/**
 * @file   	mQueue_List.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Jan 26, 2017
 * 
 * @brief   
 */

#include "mQueue_List.h"
#include <stdbool.h>

static uint8_t dataBuffer[256];

#define MAX_PACKETS_RW      5

/**
 * @defgroup mQueue_List_Private_Functions				mQueue List Private Functions
 * @{
 */

static uint32_t min_ui32(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}

static uint32_t max_ui32(uint32_t a, uint32_t b)
{
    return a > b ? a : b;
}

/**
 * @brief		Move data from the input queue in the list to the next queues.
 * @details		This function is used to move data from the queue if the queue overflows.
 * 				The function first determines the next queue in the list that can be successfully
 * 				accessed.
 * 				If the target queue is also full, a recursive call is placed to make space in the target
 * 				queue.
 * 				Then it reads at max 10 samples from the input queue and writes them to the target queue.
 * 				If the write to target queue fails, the function moves to the next queue in the list.
 *
 * @param q1	Pointer to queue in the list
 * @return		#mQueueReturn_t
 *
 * @note		This function has a recursive call.
 *
 * @todo		Find an implementation that does not rely on recursion.
 */
mQueueReturn_t move_Data_To_Next_Queue(mQueue_List_Element_t* q1)
{
    mQueue_List_Element_t* q2 = q1->next;
    uint32_t count = 0;
    
    while (q2 != NULL)
    {
        //Get next queue in list which can be read.
        while (q2 != NULL && mQueue_init(&q2->q) != mQueue_OK)
        {
            q2 = q2->next;
        }
        
        //if initialization failed for all queues in list, return error.
        if(q2 == NULL)
            return mQueue_ERROR;
        
        //if the second queue is full, move data from the second queue to the next queues
        //before writing the data from the first queue to the second.
        //The data from the first queue is to be written even if the data move operation from 
        //the second queue fails.
        if(mQueue_isFull(&q2->q))
            move_Data_To_Next_Queue(q2);
        
        count = min_ui32(mQueue_get_Size(&q2->q) - mQueue_get_Count(&q2->q), MAX_PACKETS_RW);
        count = min_ui32(count, mQueue_get_Count(&q1->q));
        count = max_ui32(count, 1);
        
        if (mQueue_peekAt_Multiple(&q1->q, 0, count, dataBuffer) != mQueue_OK)
            return mQueue_ERROR;
        
        if (mQueue_push_Multiple(&q2->q, count, dataBuffer) == mQueue_OK)
        {
            mQueue_popMultiple(&q1->q, count);
            return mQueue_OK;
        }
        
        q2 = q2->next;
    }

    return mQueue_ERROR;
}
    

/**
 * @brief		Get elements from the later queues
 * @param q1	Pointer to queue in the list
 * @return		#mQueueReturn_t
 */
mQueueReturn_t get_Data_From_Next_Queue(mQueue_List_Element_t* q1)
{
    mQueue_List_Element_t* q2 = q1->next;
    uint32_t count = 0;
    
    while (q2 != NULL)
    {
        //Get next queue in list which can be read.
        while (q2 != NULL && mQueue_init(&q2->q) != mQueue_OK)
        {
            q2 = q2->next;
        }
        
        //if initialization failed for all queues in list, return error.
        if(q2 == NULL)
            return mQueue_ERROR;

        count = min_ui32(mQueue_get_Count(&q2->q), MAX_PACKETS_RW);
        count = min_ui32(count, mQueue_get_Size(&q1->q) - mQueue_get_Count(&q1->q));
        
        //Read elements from the back of the next queue so that the first queue in the list
        //has the latest elements.
        if (mQueue_peekAt_Multiple_Back(&q2->q, 0, count, dataBuffer) == mQueue_OK)
        {
            //The elements are pushed to the front of the queue, because they are supposed to be older
            //than the front of the queue in order. 
            if (mQueue_push_Multiple_Front(&q1->q, count, dataBuffer) == mQueue_OK)
            {
                mQueue_popMultiple_Back(&q2->q, count);
                return mQueue_OK;
            }
        }
        
        q2 = q2->next;
    }

    return mQueue_ERROR;
}

/** @}*/


/**
 * @defgroup mQueue_List_Public_Functions				mQueue List Public Functions
 * @{
 */

/**
 * @brief			Add a new queue to the list of queues.
 * @details			This function adds a new queue to the start of the list. The new element is set to point to the
 * 					list and the head of the list is moved to the new element.
 * 					Therefore, the order of the elements in the list is determined by the order in which the elements are
 * 					added to the list.
 * @param list		Pointer to the list of the queues
 * @param element	Pointer to the new queue list element to be added to the queue.
 * @return			#mQueueReturn_t
 */
mQueueReturn_t mQueue_List_AddQueue(mQueue_List_t* list, mQueue_List_Element_t* element)
{
	element->next = list->head;
    element->prev = NULL;
    
    if(list->head != NULL)
        list->head->prev = element;
    
	list->head = element;

	return mQueue_OK;
}


/**
 * @brief			Push an element into the queues in the list
 * @details			The function attempts to the push an element into the queues in the list.
 * 					It attempts to push the element into the queue at the head of the list. If the
 * 					operation fails, a push operation into the next queue in the list is attempted.
 * 					The attempts continue until either the push is successful or there are no more queues left in the
 * 					list.
 * @param list		Pointer to the list of queues.
 * @param element	Element to the be pushed into the queue.
 * @return			#mQueueReturn_t
 */
mQueueReturn_t mQueue_List_push(mQueue_List_t* list, const void* element)
{
	mQueue_List_Element_t* iter = list->head;
    bool attempted_to_move_data = false;
	//A null value in the next pointer signals the end of the queue.
	while (iter != NULL)
	{
		//1. Update the queue structure.
		//2. Check if there is space in the queue or if there has been an attempt to move
        //   the data to the other queues in the list.
		//3. Attempt to push the element into the queue.
		//If the update or write operations fail or if the queue is already full,
		//the function moves on to the next queue in the list.

        ///@todo define threshold for moving data to the other queues.
        if( mQueue_init(&iter->q) == mQueue_OK &&
           (mQueue_isFull(&iter->q) == false || attempted_to_move_data == true) &&
		    mQueue_push(&iter->q, element) == mQueue_OK)
        {
        	return mQueue_OK;
        }
        
        if (mQueue_isFull(&iter->q) == true)
        {
            move_Data_To_Next_Queue(iter);
            attempted_to_move_data = true;
        }
        else
        {
            //The queue is not empty, which means that the read/write operation failed 
            //move to the next queue in the list.
            iter = iter->next;
            attempted_to_move_data = false;
        }
    }
	return mQueue_ERROR;
}

/**
 * @brief			Get the total number of elements stored in all queues in the list.
 * @param list		Pointer to the list of queues.
 * @return			#mQueueReturn_t
 */
uint32_t mQueue_List_get_elementCount(mQueue_List_t* list)
{
	mQueue_List_Element_t* iter = list->head;
	uint32_t count = 0;

	//A null value in the next pointer signals the end of the queue.
	while (iter != NULL)
	{
		//Update the queue structure before getting the count of elements in the queue.
		mQueue_init(&iter->q);
		count += mQueue_get_Count(&iter->q);
		iter = iter->next;
	}

	return count;
}


/**
 * @brief			Read element at the specified index from the front.
 * @details			This function outputs the queue element in the list at the
 * 					specified index. The queues are treated as contiguous memory,
 * 					with the head of the list being the back the queue.
 * 					The index is offset from the back of the queue, so in this case,
 * 					it will begin from the front of the first queue in the list.
 *
 * @param list			Pointer to list of queues
 * @param index			Offset of element from the front.
 * @param element[out]	Output buffer containing the element read from queue.
 * @return				#mQueueReturn_t
 */
mQueueReturn_t mQueue_List_peekAt_Back(mQueue_List_t* list, uint32_t index, void* const element)
{
	mQueue_List_Element_t* iter = list->head;
	uint32_t count = 0;

	count = mQueue_List_get_elementCount(list);
	if (count <= index)
		return mQueue_ERROR;
    
    while (iter != NULL)
    {
    	//if the index is not in the current queue,
    	//move to the next queue.
        if (index >= mQueue_get_Count(&iter->q))
        {
            index -= mQueue_get_Count(&iter->q);
            iter = iter->next;
            continue;
        }
        
        //Since we are traversing the list from the back, we need to invert the index,
        //as the mQueue_peekAt function takes the index with reference to the front of the queue.
        return mQueue_peekAt(&iter->q, mQueue_get_Count(&iter->q) - index - 1, element); 
    }

    return mQueue_ERROR;
}



/**
 * @brief			Remove multiple elements from the back of the queue.
 * @details			This function removes the latest elements added to the list.
 * 					Since the head of the list is the back of the list, the pop operation is
 * 					started off from there and then iterates through the list until the specified
 * 					number of elements have been removed.
 * 					Afterwards, it moves the elements from the end of the list to the beginning
 * 					because of the free space created by the pop operation.
 * @param list
 * @param count
 * @return
 */
mQueueReturn_t mQueue_List_PopMultiple_Back(mQueue_List_t* list, uint32_t count)
{
	mQueue_List_Element_t* iter = list->head;
	uint32_t totalCount = 0;
	uint32_t packetsToRemoveFromQueue = 0;

	totalCount = mQueue_List_get_elementCount(list);

	if (count > totalCount)
		return mQueue_ERROR;

    while (iter != NULL && count > 0)
    {
    	//Iterate the list and remove elements from the subsequent queues
    	//until either the end of list is reached or the specified number of elements
    	//have been removed from the queue.

    	//The count may exceed the number of elements in a single queue.
    	packetsToRemoveFromQueue = min_ui32(mQueue_get_Count(&iter->q), count);

    	if (mQueue_popMultiple_Back(&iter->q, packetsToRemoveFromQueue) != mQueue_OK)
    		return mQueue_ERROR;

    	count -= packetsToRemoveFromQueue;
        iter = iter->next;
    }

    //Iterate through the list and move data towards the head of the list
    //if there is space.
    iter = list->head;
    while (iter != NULL)
    {
        if (mQueue_isFull(&iter->q) == false)
        {
            get_Data_From_Next_Queue(iter);
        }
        
        iter = iter->next;
    }
    return mQueue_OK;
}

/** @}*/
