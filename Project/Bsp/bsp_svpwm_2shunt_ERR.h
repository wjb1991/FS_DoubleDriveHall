#ifndef __BSP_SVPWM_2SHUNT_H__
#define __BSP_SVPWM_2SHUNT_H__

#include "bsp.h"

extern uint16_t usMC_PhaseAOffset;
extern uint16_t usMC_PhaseBOffset;
void vBSP_SVPWM_2ShuntCurrentReadingCalibration(void);
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
__INLINE int16_t sBSP_GetCurrentPhaseU(void)
{
    return(ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1));
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
__INLINE int16_t sBSP_GetCurrentPhaseV(void)
{
    return(ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_1));
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
__INLINE  void vBSP_SetSVPWMDutyPhaseU_Pu(uint16_t usDuty)
{ 
    TIM_SetCompare1(TIM1, usDuty);
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
__INLINE  void vBSP_SetSVPWMDutyPhaseV_Pu(uint16_t usDuty)
{ 
    TIM_SetCompare2(TIM1, usDuty);
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
__INLINE  void vBSP_SetSVPWMDutyPhaseW_Pu(uint16_t usDuty)
{ 
    TIM_SetCompare3(TIM1, usDuty);
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
__INLINE void vBSP_EnableSVPWMOutput(void)
{
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
__INLINE void vBSP_DisableSVPWMOutput(void)
{
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
int32_t nBSP_SVPWM2ShuntInit(int nFrequency);

#endif
