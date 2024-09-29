#include "mIWDG.h"
#include "DEBUG_UART.h"

#define IWDG_MAX_INSTANCES      15

//alternatively, we could create instances with in the threads and
//maintain their pointers here. Would be more efficient in terms of RAM space.
//however, that would also be more prone to errors in IWDG due to thread stack overflows.
//which is better??

mIWDG_HandleTypeDef  hmIWDG[ IWDG_MAX_INSTANCES ];

uint8_t mIWDG_count;
uint8_t mIWDG_overFlow;


void mIWDG_init(){

    for( int i = 0 ; i < IWDG_MAX_INSTANCES ; i++){
        hmIWDG[i].countDown = 0;
        hmIWDG[i].isRunning = false;
        hmIWDG[i].reload = 0;
    }
    mIWDG_count = 0;
    mIWDG_overFlow = false;
}


//reload value must be in ms.
mIWDG_HandleTypeDef* IWDG_initWDG(uint32_t reload){

    if (mIWDG_count >= IWDG_MAX_INSTANCES)
        for(;;);            //this should not happen, if it does wait for hardware
                            //watchdog to reset the system.
    
    hmIWDG[ mIWDG_count ].isRunning = false;
    hmIWDG[ mIWDG_count ].countDown = 0;
    hmIWDG[ mIWDG_count++ ].reload  = reload;
    //printTask("Watchdog number %d \n", mIWDG_count);
    return &hmIWDG[ mIWDG_count - 1];
    
}

void IWDG_startWDG(mIWDG_HandleTypeDef* hmIWDG){
    
    if (hmIWDG != NULL){
        hmIWDG->isRunning = true;
        hmIWDG->countDown = hmIWDG->reload + HAL_GetTick();
    }
}

void IWDG_refreshWDG  (mIWDG_HandleTypeDef* hmIWDG){

    //if the thread was late, do not update watchdog, so the watchdog thread can trip the timer.
//    printTask("IWDG_refreshWDG\n");

    if (hmIWDG != NULL)
    {
        if (hmIWDG->countDown > HAL_GetTick())
            hmIWDG->countDown = hmIWDG->reload + HAL_GetTick();
        else
        {
          //printTask("hmIWDG->countDown > HAL_GetTick() %d-%d\n", hmIWDG->countDown, HAL_GetTick());
        }

    }
}

void IWDG_stopWDG(mIWDG_HandleTypeDef* hmIWDG){

    if (hmIWDG != NULL){
        hmIWDG->isRunning = false;
    }
}

uint8_t IWDG_checkOverFlow(){

   
    for (int i = 0 ; i < mIWDG_count ; i++){
    
        if ( hmIWDG[i].isRunning ){
        
            if ( hmIWDG[i].countDown <= HAL_GetTick() ){
                mIWDG_overFlow = i + 1;          //redundant
                break;
            }
            
        }
        
    }
    
    return mIWDG_overFlow;  
    
}

//return true if any watchdog has overflowed.
//bool IWDG_updateCounters(uint32_t ticks){

//    for ( int i = 0 ; i < mIWDG_count ; i++ ){
//    
//        if ( hmIWDG[i].isRunning ){
//            
//            if ( ticks >= hmIWDG[i].countDown ){
//                hmIWDG[i].countDown = 0;
//                mIWDG_overFlow      = true;
//            }else
//                hmIWDG[i].countDown -= ticks;  
//        }
//    }
//    
//    return mIWDG_overFlow;
//    
//}
