#ifndef __BSP_CPU_H__
#define __BSP_CPU_H__

#include "stm32f10x.h"
#include <stdio.h>
#include <stdint.h>   //包含了
#include <string.h>

/**

static __INLINE void __enable_irq()               { __ASM volatile ("cpsie i"); }
static __INLINE void __disable_irq()              { __ASM volatile ("cpsid i"); }

static __INLINE void __enable_fault_irq()         { __ASM volatile ("cpsie f"); }
static __INLINE void __disable_fault_irq()        { __ASM volatile ("cpsid f"); }

static __INLINE void __NOP()                      { __ASM volatile ("nop"); }
static __INLINE void __WFI()                      { __ASM volatile ("wfi"); }
static __INLINE void __WFE()                      { __ASM volatile ("wfe"); }
static __INLINE void __SEV()                      { __ASM volatile ("sev"); }
static __INLINE void __ISB()                      { __ASM volatile ("isb"); }
static __INLINE void __DSB()                      { __ASM volatile ("dsb"); }
static __INLINE void __DMB()                      { __ASM volatile ("dmb"); }
static __INLINE void __CLREX()                    { __ASM volatile ("clrex"); }

*/

static volatile uint32_t unNesting = 0;

__INLINE void vBSP_DisableIRQ(void) 
{  
    __disable_irq(); 
};

__INLINE void vBSP_EnableIRQ(void) 
{  
    __enable_irq(); 
}; 


__INLINE void vBSP_EnterCritical(void)
{
    unNesting++;
    vBSP_DisableIRQ();
}

__INLINE void vBSP_ExitCritical(void)
{ 
    unNesting--;
    if( 0 == unNesting )
        vBSP_EnableIRQ();
}

#endif
