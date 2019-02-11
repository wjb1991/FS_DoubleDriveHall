#include "bsp_key.h"

bitfilter_t keyFilter = DEF_DEFAULT_BITFILTER;

void vBSP_KeyConfig(void)
{
    int i = 0;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       //下拉输入

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  

    for( i = 0; i< 20;i++)
    {
        vBSP_BitFilter(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4),&keyFilter);
    }  
}

uint8_t ucBSP_GetPowButtonState(void)
{
    vBSP_BitFilter(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4),&keyFilter);
    return keyFilter.nState;
}




