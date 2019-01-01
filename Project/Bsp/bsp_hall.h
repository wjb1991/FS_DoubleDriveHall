#ifndef __BSP_HALL_H__
#define __BSP_HALL_H__

#include "bsp.h"
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
void vBSP_HALLInit(void);
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
uint8_t ucBSP_ReadHall(void);

#endif
