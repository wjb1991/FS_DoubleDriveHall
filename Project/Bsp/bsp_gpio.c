#include "bsp_gpio.h"

/**
PA0 
PA1     485_DIR     OUT
PA2     485_TX      USART2
PA3     485_RX      USART2
PA4     YWORK4      OUT
PA5     YWORK3      OUT
PA6     YWORK2      OUT
PA7     YWORK1      OUT

PA8     
PA9     232_TX      USART1
PA10    232_RX      USART1
PA11    YGasOUT2    OUT
PA12    YGasIN2     OUT
PA13    SWDIO
PA14    SWCLK
PA15    YWORK1      OUT
*/

/**
PC0 
PC1     
PC2     
PC3          
PC4     XStart4     IN
PC5     XStart3     IN
PC6     YClose2     OUT
PC7     YFailed2    OUT

PC8     YSuccess2   OUT
PC9     XStartup2   IN
PC10    
PC11    
PC12    
PC13    
PC14    
PC15    
*/

/**
PE0 
PE1     
PE2     
PE3          
PE4     STATE       OUT
PE5     
PE6     
PE7     YFailed4    OUT

PE8     YSuccess4   OUT
PE9     XStartup4   IN
PE10    YGasOUT4    OUT
PE11    YGasIN4     OUT
PE12    YClose3     OUT
PE13    YFailed3    OUT
PE14    YSuccess3   OUT
PE15    XStarup3    IN
*/

/**
PB0     XStart2     IN
PB1     XStart1     IN
PB2     YClose4     OUT
PB3     DAC_SCK          
PB4     DAC_LDAC    OUT
PB5     DAC_MOSI    
PB6     DAC_CS2     OUT
PB7     DAC_CS1     OUT

PB8     
PB9     
PB10    YGasOUT3    OUT
PB11    YGasIN3     OUT
PB12    XEms        IN
PB13    AD_SCLK     OUT
PB14    AD_DOUT     OUT
PB15    AD_DIN      IN
*/

/**
PD0     YClose1     OUT
PD1     YFailed1    OUT
PD2     YSuccess1   IN
PD3     XStartup1   IN     
PD4     YGasOUT1    OUT
PD5     YGasIN1     OUT
PD6     
PD7     

PD8     AD_DRDY4    IN
PD9     AD_CS4      OUT
PD10    AD_DRDY3    IN
PD11    AD_CS3      OUT
PD12    AD_DRDY2    IN
PD13    AD_CS2      OUT
PD14    AD_DRDY1    IN
PD15    AD_CS1      OUT
*/

/**

PA1     485_DIR     OUT                     PB2     YClose4     OUT 
PA4     YWORK4      OUT                     PB4     DAC_LDAC    OUT
PA5     YWORK3      OUT                     PB6     DAC_CS2     OUT
PA6     YWORK2      OUT                     PB7     DAC_CS1     OUT
PA7     YWORK1      OUT                     PB10    YGasOUT3    OUT 
PA11    YGasOUT2    OUT                     PB11    YGasIN3     OUT
PA12    YGasIN2     OUT                     
PA15    YWORK1      OUT                     

                                            
PC6     YClose2     OUT                     
PC7     YFailed2    OUT                     PD0     YClose1     OUT
PC8     YSuccess2   OUT                     PD1     YFailed1    OUT
                                            PD2     YSuccess1   OUT
                                            PD4     YGasOUT1    OUT
PE4     STATE       OUT                     PD5     YGasIN1     OUT
PE7     YFailed4    OUT                       
PE8     YSuccess4   OUT                     PD9     AD_CS4      OUT
PE10    YGasOUT4    OUT                     PD11    AD_CS3      OUT
PE11    YGasIN4     OUT                     PD13    AD_CS2      OUT
PE12    YClose3     OUT                     PD15    AD_CS1      OUT
PE13    YFailed3    OUT
PE14    YSuccess3   OUT

*/

/**
PB0     XStart2     IN
PB1     XStart1     IN
PB12    XEms        IN

PC4     XStart4     IN
PC5     XStart3     IN
PC9     XStartup2   IN


PD3     XStartup1   IN   
PD8     AD_DRDY4    IN
PD10    AD_DRDY3    IN
PD12    AD_DRDY2    IN
PD14    AD_DRDY1    IN

PE9     XStartup4   IN
PE15    XStarup3    IN
*/

void vBSP_GPIOConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 |
                                  GPIO_Pin_7 | GPIO_Pin_11 | GPIO_Pin_12;     
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7 |
                                  GPIO_Pin_10 | GPIO_Pin_11;  
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;    
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_15;   
	GPIO_Init(GPIOD, &GPIO_InitStructure);  
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_10 |
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;   
	GPIO_Init(GPIOE, &GPIO_InitStructure);

    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       //上拉输入

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_12;  
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_9;    
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_10 |
                                  GPIO_Pin_12 | GPIO_Pin_14;   
	GPIO_Init(GPIOD, &GPIO_InitStructure);  
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_15;   
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
}




