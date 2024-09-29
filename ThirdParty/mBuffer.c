/**
 * @file   	mBuffer.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Nov 28, 2016
 * 
 * @brief   
 */

#include "mBuffer.h"


/**
 * @defgroup
 * @{
 */

void mBuffer_set_State(mBuffer_t* buffer)
{
	if (buffer->counter == 0)
		buffer->state  = BUFFER_STATE_EMPTY;
	else if (buffer->counter == buffer->size)
		buffer->state = BUFFER_STATE_FULL;
	else
		buffer->state = BUFFER_STATE_DATA;
}


void mBuffer_init(mBuffer_t* buffer)
{
	buffer->counter = 0;
	buffer->index   = 0;
	buffer->state = BUFFER_STATE_EMPTY;

}
/** @}*/
