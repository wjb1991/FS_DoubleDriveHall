#ifndef __BSP_SYSTICK_H__
#define __BSP_SYSTICK_H__
#include "bsp.h"

extern volatile uint8_t ucSystem1msFlag;
extern volatile uint8_t ucSystem10msFlag;
extern volatile uint8_t ucSystem100msFlag;    

int32_t nBSP_SysTickConfig(int32_t nFrequency);

void vBSP_DelayUS(uint32_t us);

void vBSP_DelayMS(uint16_t ms);

#endif
