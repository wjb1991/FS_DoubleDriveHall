#include "bsp_key.h"

bitfilter_t keyFilter = DEF_DEFAULT_BITFILTER;

void vBSP_KeyConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       //下拉输入

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
    
    {
        int i = 0;
        for( i = 0; i< 20;i++)
        {
            vBSP_BitFilter(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4),&keyFilter);
        }  
    } 
}

uint8_t ucBSP_GetPowButtonState(void)
{
    vBSP_BitFilter(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4),&keyFilter);
    return keyFilter.nState;
}




