
#ifndef __MIWDG_H_

#define __MIWDG_H_

#include "iwdg.h"
#include "stdbool.h"

typedef struct{
    
    uint32_t reload;
    uint32_t countDown;
    bool     isRunning;

}mIWDG_HandleTypeDef;


void mIWDG_init             (void);
void IWDG_startWDG          (mIWDG_HandleTypeDef* hmIWDG);
void IWDG_refreshWDG        (mIWDG_HandleTypeDef* hmIWDG);
void IWDG_stopWDG           (mIWDG_HandleTypeDef* hmIWDG);
bool IWDG_updateCounters    (uint32_t ticks);
uint8_t IWDG_checkOverFlow  (void);

mIWDG_HandleTypeDef* IWDG_initWDG(uint32_t reload);

#endif
