#ifndef  __IODEFINE_H__
#define __IODEFINE_H__
//#include "stm8s.h"
#include "bsp.h"
#include <stdint.h>

/**

    CS      PB7
    CE      PB6
    MOSI    PB5
    MISO    PB4
    SCLK    PB3
    IRQ     PA15

*/

  #define IRQPIN    GPIO_Pin_15  //PA15
  #define IRQHI()   GPIO_SetBits(GPIOA,IRQPIN)                          //PC_ODR |= IRQPIN
  #define IRQLOW()  GPIO_ResetBits(GPIOA,IRQPIN) 
  #define IRQVal()  GPIO_ReadInputDataBit(GPIOA,IRQPIN)                //(PC_IDR & IRQPIN)
  
  
  #define CEPIN     GPIO_Pin_6   //PB6
  #define CEHI()    GPIO_SetBits(GPIOB,CEPIN);vBSP_DelayUS(10)          //PB_ODR |= CEPIN
  #define CELOW()   GPIO_ResetBits(GPIOB,CEPIN);vBSP_DelayUS(10)        //PB_ODR &= ~CEPIN
  
  #define CSNPIN    GPIO_Pin_7   //PB7
  #define CSNHI()   GPIO_SetBits(GPIOB,CSNPIN);vBSP_DelayUS(10)         //PB_ODR |= CSNPIN
  #define CSNLOW()  GPIO_ResetBits(GPIOB,CSNPIN);vBSP_DelayUS(10)       //PB_ODR &= ~CSNPIN

  #define MISOPIN   GPIO_Pin_4   //PB4
  #define MISOHI()  GPIO_SetBits(GPIOB,MISOPIN)                         //PC_ODR |= MISOPIN
  #define MISOLOW() GPIO_ResetBits(GPIOB,MISOPIN)                       //PC_ODR &= ~MISOPIN 
  #define MISOVAL() GPIO_ReadInputDataBit(GPIOB,MISOPIN)               //(PC_IDR & MISOPIN)
  
  #define MOSIPIN   GPIO_Pin_5   //PB5
  #define MOSIHI()  GPIO_SetBits(GPIOB,MOSIPIN)                         //PC_ODR |= MOSIPIN
  #define MOSILOW() GPIO_ResetBits(GPIOB,MOSIPIN)                       //PC_ODR &= ~MOSIPIN 

  #define SCKPIN    GPIO_Pin_3   //PB3
  #define SCKHI()   GPIO_SetBits(GPIOB,SCKPIN)                          //PC_ODR |= SCKPIN
  #define SCKLOW()  GPIO_ResetBits(GPIOB,SCKPIN)                        //PC_ODR &= ~SCKPIN 

  
#endif
