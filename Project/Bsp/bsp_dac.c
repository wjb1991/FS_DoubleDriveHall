
#include "bsp_dac.h"

static void RCC_Configuration(void);
static void GPIO_Configuration(void);

void vBSP_DACConfig(void)
{

	DAC_InitTypeDef DAC_InitStruct;
	
    RCC_Configuration();
    GPIO_Configuration();
    
	DAC_DeInit();
	
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	
	DAC_Init(DAC_Channel_2, &DAC_InitStruct);
	
	DAC_Cmd(DAC_Channel_2, ENABLE);
	
	DAC_SetChannel2Data(DAC_Align_12b_R, 0);

}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
static void RCC_Configuration(void)
{   
  /* Enable peripheral clocks ------------------------------------------------*/
  /* GPIOA Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  /* DAC Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
static void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

