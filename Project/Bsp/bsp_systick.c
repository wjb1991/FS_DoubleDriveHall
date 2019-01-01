#include "bsp_systick.h"

static uint32_t unSystickFrequency = 0;

volatile uint32_t unSystemRunTiming = 0;

volatile uint8_t ucSystem1msFlag = 0;
volatile uint8_t ucSystem10msFlag = 0;
volatile uint8_t ucSystem100msFlag = 0;    

int32_t nBSP_SysTickConfig(int32_t nFrequency)
{

    if (SysTick_Config(SystemCoreClock / nFrequency))   //默认开启SysTick中断且设置优先级为最低
    { 
    /* Capture error */ 
        return -1;
    }  
    else
    {
        unSystickFrequency = nFrequency;
        return 0;
    }
}

/*******************************************************************************
* 函数名称       : vBSP_DelayUS
* 功能描述       : 延时函数，72M条件下ms<=1864000
* 入口参数       : us 延时大小
* 出口参数       : 无
*******************************************************************************/
void vBSP_DelayUS(uint32_t us)
{
    uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0;
    uint32_t reload;
       
    reload = SysTick->LOAD;                
    ticks = us * (SystemCoreClock / 1000000);	 

    tcnt = 0;
    told = SysTick->VAL;             

    while (tcnt < ticks)             
    {
        tnow = SysTick->VAL;    
        if (tnow != told)
        {
              if (tnow < told)
              {
                  tcnt += told - tnow;    
              }
              else
              {
                  tcnt += reload - tnow + told;    
              }        
              told = tnow;
        }  
    }
} 

/*******************************************************************************
* 函数名称       : vBSP_DelayMS
* 功能描述       : 延时函数，72M条件下ms<=1864
* 入口参数       : ms 延时大小
* 出口参数       : 无
*******************************************************************************/
void vBSP_DelayMS(u16 ms)
{	 		  	  
    while(ms--)
    {
        vBSP_DelayUS(998);    
    }   
} 

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    if( unSystemRunTiming < unSystickFrequency * 60 * 60 * 24)     //一天的计数总数 unSystickFrequency最大49710
        unSystemRunTiming++;
    else
        unSystemRunTiming = 0;
    
    if( (unSystemRunTiming%(unSystickFrequency / 1000)) == 0)
        ucSystem1msFlag = 1;
    if( (unSystemRunTiming%(unSystickFrequency / 100)) == 0)
        ucSystem10msFlag = 1;
    if( (unSystemRunTiming%(unSystickFrequency / 10)) == 0)
        ucSystem100msFlag = 1;   
}

