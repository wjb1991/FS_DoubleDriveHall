#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__
#include "bsp.h"

/**
PA1     485_DIR     OUT                                                                                             
PE4     STATE       OUT 

PD9     AD_CS4      OUT             PB4     DAC_LDAC    OUT
PD11    AD_CS3      OUT             PB6     DAC_CS2     OUT
PD13    AD_CS2      OUT             PB7     DAC_CS1     OUT
PD15    AD_CS1      OUT

PA4     YWORK4      OUT                     
PA5     YWORK3      OUT                     
PA6     YWORK2      OUT                     
PA7     YWORK1      OUT 

PD0     YClose1     OUT             PC6     YClose2     OUT
PD1     YFailed1    OUT             PC7     YFailed2    OUT
PD2     YSuccess1   OUT             PC8     YSuccess2   OUT
PD4     YGasOUT1    OUT             PA11    YGasOUT2    OUT
PD5     YGasIN1     OUT             PA12    YGasIN2     OUT
            

PE12    YClose3     OUT             PB2     YClose4     OUT                   
PE13    YFailed3    OUT             PE7     YFailed4    OUT
PE14    YSuccess3   OUT             PE8     YSuccess4   OUT
PB10    YGasOUT3    OUT             PE10    YGasOUT4    OUT
PB11    YGasIN3     OUT             PE11    YGasIN4     OUT       
*/
__INLINE void vBSP_Set485DIR(uint8_t bitval){ GPIO_WriteBit(GPIOA,GPIO_Pin_1,bitval);}
__INLINE void vBSP_SetSTATELED(uint8_t bitval){ GPIO_WriteBit(GPIOE,GPIO_Pin_4,bitval);}

__INLINE void vBSP_SetADCS4(uint8_t bitval){ GPIO_WriteBit(GPIOD,GPIO_Pin_9,bitval);}
__INLINE void vBSP_SetADCS3(uint8_t bitval){ GPIO_WriteBit(GPIOD,GPIO_Pin_11,bitval);}
__INLINE void vBSP_SetADCS2(uint8_t bitval){ GPIO_WriteBit(GPIOD,GPIO_Pin_13,bitval);}
__INLINE void vBSP_SetADCS1(uint8_t bitval){ GPIO_WriteBit(GPIOD,GPIO_Pin_15,bitval);}

__INLINE void vBSP_SetDALDAC(uint8_t bitval){ GPIO_WriteBit(GPIOB,GPIO_Pin_4,bitval);}
__INLINE void vBSP_SetDACS2(uint8_t bitval){  GPIO_WriteBit(GPIOB,GPIO_Pin_6,bitval);}
__INLINE void vBSP_SetDACS1(uint8_t bitval){  GPIO_WriteBit(GPIOB,GPIO_Pin_7,bitval);}

__INLINE void vBSP_SetYWORK4(uint8_t bitval){ GPIO_WriteBit(GPIOA,GPIO_Pin_4,bitval);}
__INLINE void vBSP_SetYWORK3(uint8_t bitval){ GPIO_WriteBit(GPIOA,GPIO_Pin_5,bitval);}
__INLINE void vBSP_SetYWORK2(uint8_t bitval){ GPIO_WriteBit(GPIOA,GPIO_Pin_6,bitval);}
__INLINE void vBSP_SetYWORK1(uint8_t bitval){ GPIO_WriteBit(GPIOA,GPIO_Pin_7,bitval);}

__INLINE void vBSP_SetYClose1(uint8_t bitval)  { GPIO_WriteBit(GPIOD,GPIO_Pin_0,bitval);}
__INLINE void vBSP_SetYFailed1(uint8_t bitval) { GPIO_WriteBit(GPIOD,GPIO_Pin_1,bitval);}
__INLINE void vBSP_SetYSuccess1(uint8_t bitval){ GPIO_WriteBit(GPIOD,GPIO_Pin_2,bitval);}
__INLINE void vBSP_SetYGasOUT1(uint8_t bitval) { GPIO_WriteBit(GPIOD,GPIO_Pin_4,bitval);}
__INLINE void vBSP_SetYGasOUT1(uint8_t bitval) { GPIO_WriteBit(GPIOD,GPIO_Pin_5,bitval);}

__INLINE void vBSP_SetYClose2(uint8_t bitval)  { GPIO_WriteBit(GPIOC,GPIO_Pin_6,bitval);}
__INLINE void vBSP_SetYFailed2(uint8_t bitval) { GPIO_WriteBit(GPIOC,GPIO_Pin_7,bitval);}
__INLINE void vBSP_SetYSuccess2(uint8_t bitval){ GPIO_WriteBit(GPIOC,GPIO_Pin_8,bitval);}
__INLINE void vBSP_SetYGasOUT2(uint8_t bitval) { GPIO_WriteBit(GPIOA,GPIO_Pin_11,bitval);}
__INLINE void vBSP_SetYGasOUT2(uint8_t bitval) { GPIO_WriteBit(GPIOA,GPIO_Pin_12,bitval);}
                                                                           
__INLINE void vBSP_SetYClose3(uint8_t bitval)  { GPIO_WriteBit(GPIOE,GPIO_Pin_12,bitval);}
__INLINE void vBSP_SetYFailed3(uint8_t bitval) { GPIO_WriteBit(GPIOE,GPIO_Pin_13,bitval);}
__INLINE void vBSP_SetYSuccess3(uint8_t bitval){ GPIO_WriteBit(GPIOE,GPIO_Pin_14,bitval);}
__INLINE void vBSP_SetYGasOUT3(uint8_t bitval) { GPIO_WriteBit(GPIOB,GPIO_Pin_10,bitval);}
__INLINE void vBSP_SetYGasOUT3(uint8_t bitval) { GPIO_WriteBit(GPIOB,GPIO_Pin_11,bitval);}

__INLINE void vBSP_SetYClose4(uint8_t bitval)  { GPIO_WriteBit(GPIOB,GPIO_Pin_2,bitval);}
__INLINE void vBSP_SetYFailed4(uint8_t bitval) { GPIO_WriteBit(GPIOE,GPIO_Pin_7,bitval);}
__INLINE void vBSP_SetYSuccess4(uint8_t bitval){ GPIO_WriteBit(GPIOE,GPIO_Pin_8,bitval);}
__INLINE void vBSP_SetYGasOUT4(uint8_t bitval) { GPIO_WriteBit(GPIOB,GPIO_Pin_10,bitval);}
__INLINE void vBSP_SetYGasOUT4(uint8_t bitval) { GPIO_WriteBit(GPIOB,GPIO_Pin_11,bitval);}

/**
PC4     XStart4     IN
PC5     XStart3     IN
PB0     XStart2     IN
PB1     XStart1     IN

PE9     XStartup4   IN
PE15    XStartup3   IN
PC9     XStartup2   IN
PD3     XStartup1   IN 
 
PB12    XEms        IN
 
PD8     AD_DRDY4    IN
PD10    AD_DRDY3    IN
PD12    AD_DRDY2    IN
PD14    AD_DRDY1    IN
*/
__INLINE uint8_t ucBSP_GetXStart4(void){ return (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4));}
__INLINE uint8_t ucBSP_GetXStart3(void){ return (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5));}
__INLINE uint8_t ucBSP_GetXStart2(void){ return (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0));}
__INLINE uint8_t ucBSP_GetXStart1(void){ return (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1));}

__INLINE uint8_t ucBSP_GetXStartup4(void){ return (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_9));}
__INLINE uint8_t ucBSP_GetXStartup3(void){ return (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15));}
__INLINE uint8_t ucBSP_GetXStartup2(void){ return (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9));}
__INLINE uint8_t ucBSP_GetXStartup1(void){ return (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3));}

__INLINE uint8_t ucBSP_GetXEms(void){ return (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12));}

__INLINE uint8_t ucBSP_GetADDRDY4(void){ return (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8));}
__INLINE uint8_t ucBSP_GetADDRDY3(void){ return (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_10));}
__INLINE uint8_t ucBSP_GetADDRDY2(void){ return (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12));}
__INLINE uint8_t ucBSP_GetADDRDY1(void){ return (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14));}

void vBSP_GPIOConfig(void);


#endif
