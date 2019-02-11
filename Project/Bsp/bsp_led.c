#include "bsp_led.h"

void vBSP_LEDConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB , ENABLE);
    
    vBSP_SetKeyLed(0);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;               //sk-1s pb2 pbc pb9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);   

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;               //sk-1s pb2 pbc pb9
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
}

void vBSP_SetKeyLed(int32_t nNewState)
{
    if( nNewState )
        GPIO_SetBits(GPIOB, GPIO_Pin_2);
    else
        GPIO_ResetBits(GPIOB, GPIO_Pin_2);
}

void vBSP_CplKeyLed(void)
{
    if( GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_2) )
        GPIO_ResetBits(GPIOB, GPIO_Pin_2);
    else
        GPIO_SetBits(GPIOB, GPIO_Pin_2);
}

void vBSP_SetPowerCrtl(int32_t nNewState)
{
    if( nNewState )
        GPIO_SetBits(GPIOA, GPIO_Pin_5);
    else
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}