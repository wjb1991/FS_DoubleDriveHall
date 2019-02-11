#include "bsp_svpwm_2shunt.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void vBSP_HALLInit(void);
uint8_t ucBSP_ReadHall(void);
static void vGPIO_Configuration(void);
  
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
void vBSP_HALLInit(void)
{
    vGPIO_Configuration();
}  
 
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
uint8_t ucBSP_ReadHall(void)
{
    return (GPIO_ReadInputData(GPIOC)>>13&0x07);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
static void vGPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 
    
    //配置时钟定时器时钟 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);        //使能GPIO和服用功能时钟

    //PC10 - PC12
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;               //开启内部上拉和外部上拉一起
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    //PB5 - PB7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;               //开启内部上拉和外部上拉一起
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

