/**
 * @file   	FSK_Application.c
 * @author 	Hassan Abid
 * @version	v1.0.0
 * @date	Jan 24, 2017
 * 
 * @brief   
 */

#include "FSK_Application.h"
#include "SX127x.h"


/**
 * @defgroup FSK_Application_Exported_Members				Radio FSK Application Exported Members
 * @{
 */

osThreadId	hthread_FSK_Node = NULL;
osThreadId	hthread_FSK_Gateway = NULL;

/** @}*/

extern osSemaphoreId hsmphr_RadioIRQ;
extern mQueueStruct_t mpuQueue;

struct MPU_DATA_s;
typedef struct MPU_DATA_s MPU_DATA_t;


uint32_t txFailures = 0;

/**
 * @defgroup FSK_Application_Private_Functions				Radio FSK Application Private Functions
 * @{
 */

/**
 * @brief				Wait for a packet with expected type.
 * @param expectedReply Expected frame type of packet. This can a combination of
 * 						@ref MAC_FrameType_t;
 * @param waitPeriod	Time to wait for response.
 * @param sleepOnExit	Set the radio in sleep mode at the end of operation.
 * @return
 * 		@arg MAC_OK			if a packet was received.
 * 		@arg MAC_ERROR  	if the payload had a CRC error.
 * 		@arg MAC_TIMEOUT	if the receive operation timed out.
 */
SX127xReturn_t FSK_WaitForPacket(uint32_t expectedReply, int32_t waitPeriod, bool sleepOnExit)
{

    uint32_t ticks;
    SX127xReturn_t ret = SX127x_TIMEOUT;

    //loop until timeout
    while (waitPeriod > 0){

    	//if radio is not in receive mode already, set it in receive mode.
    	if (SX127x_handle.mode != SX127x_RECEIVER)
    		SX127x_receiverMode();

        ticks = osKernelSysTick();

        //If DIO interrupt flag is set, process it, otherwise wait for semaphore
        //set in EXTI callback.
        if (SX127x_handle.nIRQ || osSemaphoreWait(hsmphr_RadioIRQ, waitPeriod) == osOK)
        {
            SX127x_nIRQHandler();
            SX127x_handle.nIRQ = 0;

            if (SX127x_handle.FSK.irqFlags.PayloadReady == true)
            {
                if (SX127x_handle.FSK.irqFlags.CrcOk == false)
                {
                    ret = SX127x_ERROR;
                }
                else 
                {
                    SX127x_handle.FSK.irqFlags.PayloadReady = false;
                    ret = SX127x_OK;
                }
                break;
            }
               
            waitPeriod -= osKernelSysTick() - ticks;
            ticks = osKernelSysTick();
            continue;
        }
        else		//semaphore timed out.
            break;
    }


    MAC_status.isWaitingForReply = false;
    if (sleepOnExit)
    	SX127x_sleepMode();
    return ret;


}



static void printPacket()
{
    printDEBUG("[%u][", SX127x_handle.rxBuffer[0]);
    for (int i = 1 ; i < SX127x_handle.rxBuffer[0] ; i++)
    {
        printDEBUG(" %02X ", SX127x_handle.rxBuffer[i]);
    }
    printDEBUG("]\r\n");
}
/** @}*/



/**
 * @defgroup FSK_Application_Public_Functions				Radio FSK Application Public Functions
 * @{
 */


/**
 *
 * @param param
 */
void thread_FSK_Node(const void* param)
{

    SX127xReturn_t ret = SX127x_ERROR;
    
    SX127x_Init();
    SX127x_config(SX127x_MODEM_FSK);
    uint8_t mpuData[3*mpuQueue.elementSize];
    
    for (;;)
    {
        if (mQueue_get_Count(&mpuQueue) >= 3)
        {
            mQueue_peek(&mpuQueue, &mpuData);
            mQueue_peekAt(&mpuQueue, 1, &mpuData[mpuQueue.elementSize]);
            mQueue_peekAt(&mpuQueue, 2, &mpuData[2*mpuQueue.elementSize]);
            
            ret = SX127x_FSK_TransmitPacket(mpuData, sizeof(mpuData));    
            
            if (ret == SX127x_OK)
            {
                mQueue_popMultiple(&mpuQueue, 3);
                continue;
            }
            else
                txFailures++;
            
        }
        
        osDelay(10);
    }

    osThreadTerminate(NULL);
}


/**
 *
 * @param param
 */
void thread_FSK_Gateway(const void* param)
{
    SX127xReturn_t retStatus = SX127x_ERROR;
    
    SX127x_Init();
    SX127x_config(SX127x_MODEM_FSK);
    
	for(;;)
	{
        retStatus = FSK_WaitForPacket(0xFFFF, 5*1000, true);
        if (retStatus == SX127x_OK)
        {
            printPacket();
        }
	}

	osThreadTerminate(NULL);
}

/** @}*/
