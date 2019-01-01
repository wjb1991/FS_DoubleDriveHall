
#ifndef __BSP_DAC_H__
#define __BSP_DAC_H__

#include "bsp.h"

void vBSP_DACConfig(void);

__INLINE void vBSP_SetDACValue(uint16_t val)
{
	DAC_SetChannel2Data(DAC_Align_12b_R, val);
}			

__INLINE uint16_t vBSP_GetDACValue()
{
	return(DAC_GetDataOutputValue(DAC_Channel_2)); 
}			


#endif /* __BSP_DAC_H__ */

