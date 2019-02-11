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
__INLINE  void vBSP_SetSVPWMDutyPhaseU_Pu(TIM_TypeDef* TIMx,uint16_t usDuty)
{ 
    TIM_SetCompare1(TIMx, usDuty);
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
__INLINE  void vBSP_SetSVPWMDutyPhaseV_Pu(TIM_TypeDef* TIMx,uint16_t usDuty)
{ 
    TIM_SetCompare2(TIMx, usDuty);
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
__INLINE  void vBSP_SetSVPWMDutyPhaseW_Pu(TIM_TypeDef* TIMx,uint16_t usDuty)
{ 
    TIM_SetCompare3(TIMx, usDuty);
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
__INLINE void vBSP_EnableSVPWMOutput(void)
{
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
__INLINE void vBSP_DisableSVPWMOutput(void)
{
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    TIM_CtrlPWMOutputs(TIM8, DISABLE);
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
int32_t nBSP_SVPWM2ShuntInit(int nFrequency);

_iq vBSP_GetVBus_V(void);

_iq vBSP_GetVBusCurrent_A(void);

int32_t vBSP_BrakeEnable(void);

int32_t vBSP_BrakeDisable(void);

#endif
