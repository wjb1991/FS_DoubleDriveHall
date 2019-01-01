/*
 * FreeModbus Libary: Atmel AT91SAM3S Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: porttimer.c,v 1.2 2010/06/06 13:46:42 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/

/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/


#define MB_TIMER_DEBUG                      ( 0 )
#define MB_TIMER_PRESCALER                  ( 128UL )
#define MB_TIMER_TICKS                      ( BOARD_MCK / MB_TIMER_PRESCALER )
#define MB_50US_TICKS                       ( 20000UL )
#if 0
#define TCX                                 ( TC0 )
#define TCXIRQ                              ( TC0_IRQn )
#define TCCHANNEL                           ( 0 )
#define TCX_IRQHANDLER                      TC0_IrqHandler

#define TC_CMRX_WAVE                        ( 0x1 << 15 )
#define TC_CMRX_TCCLKS_TIMER_DIV4_CLOCK     ( 0x3 << 0 )
#define TC_CMRX_CPCSTOP                     ( 0x1 << 6 )
#define TC_CMRX_WAVESEL_UP_RC               ( 0x2 << 13 )

#define TC_IERX_CPCS                        ( 0x1 << 4 )
#define TC_IERX_CPAS                        ( 0x1 << 2 )
#define TC_SRX_CPAS                         ( 0x1 << 2 )
#endif
#if MB_TIMER_DEBUG == 1
#define TIMER_PIN { 1 << 6, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT }
#endif

#ifndef SYSTICK_COUNTFLAG
/* missing in CMSIS */
#define SYSTICK_COUNTFLAG                   ( 16 )             
#endif
/* ----------------------- Static variables ---------------------------------*/
#if MB_TIMER_DEBUG == 1
const static Pin xTimerDebugPins[] = { TIMER_PIN };
#endif

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */ 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;   
    
    /* System Clocks Configuration */
    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* NVIC Configuration */
    /* Enable the TIM2 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    // 115200 303.819 psc = 7199 per = 4999 是 500ms 取反  10K?  500取反 1000ms = 1k
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 305-1;
    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000UL) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* TIM IT enable */
    //TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
    
    return TRUE;
}

void  vMBPortTimerClose( void )
{
    NVIC_InitTypeDef NVIC_InitStructure;   
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_DeInit(TIM2);
}

void vMBPortTimersEnable(  )
{
    //TIM_Cmd(TIM2, ENABLE);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    TIM_SetCounter(TIM2, 0);

    TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);
}

void vMBPortTimersDisable(  )
{
    //TIM_Cmd(TIM2, DISABLE);
  TIM_SetCounter(TIM2,0);
  TIM_ITConfig(TIM2,TIM_IT_Update, DISABLE);
}

void vMBPortTimersDelay( USHORT usTimeOutMS )
{

}

void TIM2_IRQHandler( void )
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update))
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        (void)pxMBPortCBTimerExpired();
    }   
}


