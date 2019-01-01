#ifndef __BSP_LED_H__
#define __BSP_LED_H__
#include "bsp.h"


void vBSP_LEDConfig(void);

void vBSP_SetKeyLed(int32_t nNewState);
void vBSP_CplKeyLed(void);

void vBSP_SetPowerCrtl(int32_t nNewState);

#endif
