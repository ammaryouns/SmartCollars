/**
 * @file   	Flash_EEPROM_Queue.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Dec 5, 2016
 * 
 * @brief   
 */


#include "mQueue.h"
#include "Flash_EEPROM.h"
#include "crc32.h"






mQueueReturn_t Flash_EEPROM_Queue_clear       (mQueueStruct_t* q);
mQueueReturn_t Flash_EEPROM_Queue_pullStruct  (mQueueStruct_t* q);
mQueueReturn_t Flash_EEPROM_Queue_write       (mQueueStruct_t* q, uint8_t* destAddr, uint8_t* srcAddr, uint32_t len);
mQueueReturn_t Flash_EEPROM_Queue_read        (mQueueStruct_t* q, uint8_t* destAddr, uint8_t* srcAddr, uint32_t len);
mQueueReturn_t Flash_EEPROM_Queue_commitStruct(mQueueStruct_t* q);


const mQueue_Interface_t Flash_EEPROM_Queue_IFC =
{
    .init           = Flash_EEPROM_Queue_pullStruct,
    .mem_read       = Flash_EEPROM_Queue_read,
    .mem_write      = Flash_EEPROM_Queue_write,
    .pullStruct     = Flash_EEPROM_Queue_pullStruct,
    .commitStruct   = Flash_EEPROM_Queue_commitStruct,

};


#define EEPROM_QUEUE_STRUCT_OFFSET( q )                 ((uint32_t)(q->buff))
#define EEPROM_QUEUE_STRUCT_CRC_OFFSET( q )             ((uint32_t)(EEPROM_QUEUE_STRUCT_OFFSET(q) + offsetof(mQueueStruct_t, ifc)))


#define EEPROM_QUEUE_STRUCT_BKUP_OFFSET( q )            ((uint32_t)(EEPROM_QUEUE_STRUCT_OFFSET( q ) + EEPROM_QUEUE_OFFSET(q) + q->size*q->elementSize))
#define EEPROM_QUEUE_STRUCT_BKUP_CRC_OFFSET( q )        ((uint32_t)(EEPROM_QUEUE_STRUCT_BKUP_OFFSET(q) + offsetof(mQueueStruct_t, ifc)))

#define EEPROM_QUEUE_OFFSET( q )                        ((uint32_t)(offsetof(mQueueStruct_t, ifc) + 4))

/**
 * @defgroup
 * @{
 */


/**
 *
 * @param q
 * @return
 */
mQueueReturn_t Flash_EEPROM_Queue_clear(mQueueStruct_t* q)
{
    q->front = 0;
    q->back  = 0;
    q->count = 0;

    return q->ifc->commitStruct(q);
}


/**
 *
 * @param q
 * @return
 */
mQueueReturn_t Flash_EEPROM_Queue_pullStruct(mQueueStruct_t* q)
{
    uint32_t crc = 0;
    uint32_t bkup_crc = 0;

    static mQueueStruct_t tempQueue;
    static mQueueStruct_t tempQueue_bkup;

    if (HAL_OK != Flash_EEPROM_Read(EEPROM_QUEUE_STRUCT_OFFSET(q), (uint8_t*)&tempQueue, offsetof(mQueueStruct_t, ifc)))
        return mQueue_ERROR;

    if (HAL_OK != Flash_EEPROM_Read(EEPROM_QUEUE_STRUCT_CRC_OFFSET(q), (uint8_t*)&crc, 4))
        return mQueue_ERROR;

    if (HAL_OK != Flash_EEPROM_Read(EEPROM_QUEUE_STRUCT_BKUP_CRC_OFFSET(q), (uint8_t*)&bkup_crc, 4))
        return mQueue_ERROR;

    if (HAL_OK != Flash_EEPROM_Read(EEPROM_QUEUE_STRUCT_BKUP_OFFSET(q), (uint8_t*)&tempQueue_bkup, offsetof(mQueueStruct_t, ifc)))
        return mQueue_ERROR;

    
    if (crc == HAL_CRC_Calculate_ByteStream(&hcrc, (uint8_t*)&tempQueue, offsetof(mQueueStruct_t, ifc), false))
    {
        if (tempQueue.front < tempQueue.size &&
            tempQueue.back  < tempQueue.size &&
            tempQueue.count < tempQueue.size)
        {
            q->front = tempQueue.front;
            q->back  = tempQueue.back;
            q->count = tempQueue.count;
        }

        
        if (crc == bkup_crc &&
            bkup_crc == HAL_CRC_Calculate_ByteStream(&hcrc, (uint8_t*)&tempQueue_bkup, offsetof(mQueueStruct_t, ifc), false))
            return mQueue_OK;
        else
            return q->ifc->commitStruct(q);

    }



    if (bkup_crc == HAL_CRC_Calculate_ByteStream(&hcrc, (uint8_t*)&tempQueue_bkup, offsetof(mQueueStruct_t, ifc), false))
    {
        if (tempQueue_bkup.front < tempQueue_bkup.size &&
            tempQueue_bkup.back  < tempQueue_bkup.size &&
            tempQueue_bkup.count < tempQueue_bkup.size)
        {
            q->front = tempQueue_bkup.front;
            q->back  = tempQueue_bkup.back;
            q->count = tempQueue_bkup.count;
        }

        if (crc == bkup_crc &&
            crc == HAL_CRC_Calculate_ByteStream(&hcrc, (uint8_t*)&tempQueue, offsetof(mQueueStruct_t, ifc), false))
            return mQueue_OK;
        else
            return q->ifc->commitStruct(q);

    }

	return Flash_EEPROM_Queue_clear(q);
}

/**
 *
 * @param q
 * @param destAddr
 * @param srcAddr
 * @param len
 * @return
 */
mQueueReturn_t Flash_EEPROM_Queue_write(mQueueStruct_t* q, uint8_t* destAddr, uint8_t* srcAddr, uint32_t len)
{
	if (HAL_OK != Flash_EEPROM_Write(EEPROM_QUEUE_OFFSET(q) + (uint32_t)destAddr, srcAddr, len))
		return mQueue_ERROR;

	return mQueue_OK;
}


/**
 *
 * @param q
 * @param destAddr
 * @param srcAddr
 * @param len
 * @return
 */
mQueueReturn_t Flash_EEPROM_Queue_read(mQueueStruct_t* q, uint8_t* destAddr, uint8_t* srcAddr, uint32_t len)
{
	if(HAL_OK != Flash_EEPROM_Read(EEPROM_QUEUE_OFFSET(q) + (uint32_t)srcAddr, destAddr, len))
        return mQueue_ERROR;

    return mQueue_OK;
}


/**
 *
 * @param q
 * @return
 */
mQueueReturn_t Flash_EEPROM_Queue_commitStruct(mQueueStruct_t* q)
{
    uint32_t crc = HAL_CRC_Calculate_ByteStream(&hcrc, (uint8_t*)q, offsetof(mQueueStruct_t, ifc), false);

    if (HAL_OK != Flash_EEPROM_Write(EEPROM_QUEUE_STRUCT_OFFSET(q), (uint8_t*)q, offsetof(mQueueStruct_t, ifc)))
        return mQueue_ERROR;

    if (HAL_OK != Flash_EEPROM_Write(EEPROM_QUEUE_STRUCT_CRC_OFFSET(q), (uint8_t*)&crc, 4))
        return mQueue_ERROR;

    if (HAL_OK != Flash_EEPROM_Write(EEPROM_QUEUE_STRUCT_BKUP_OFFSET(q), (uint8_t*)q, offsetof(mQueueStruct_t, ifc)))
        return mQueue_ERROR;

    if (HAL_OK != Flash_EEPROM_Write(EEPROM_QUEUE_STRUCT_BKUP_CRC_OFFSET(q), (uint8_t*)&crc, 4))
        return mQueue_ERROR;

	return mQueue_OK;
}

/** @}*/
