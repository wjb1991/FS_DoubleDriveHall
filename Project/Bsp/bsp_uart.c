#include "bsp_uart.h"

#if 1
#define USARTx                           USART3
#define USARTx_CLK                       RCC_APB1Periph_USART3
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART3_IRQn
#define USARTx_IRQHandler                USART3_IRQHandler

#define USARTx_GPIOCLK_INIT              RCC_APB2PeriphClockCmd //F1用APB2   F4用AHB1
#define USARTx_OPEN_AFCLK                RCC_APB2Periph_AFIO    //打开复用时钟    F1用F4不用
                                         
#define USARTx_TX_PIN                    GPIO_Pin_10                
#define USARTx_TX_GPIO_PORT              GPIOB                       
#define USARTx_TX_GPIO_CLK               RCC_APB2Periph_GPIOB   //F1用APB2   F4用AHB1
#define USARTx_TX_SOURCE                 GPIO_PinSource10        //F1用不到 F4才用的到
#define USARTx_TX_AF                     GPIO_AF_USART3         //F1用不到 F4才用的到

#define USARTx_RX_PIN                    GPIO_Pin_11               
#define USARTx_RX_GPIO_PORT              GPIOB                    
#define USARTx_RX_GPIO_CLK               RCC_APB2Periph_GPIOB   //F1用APB2   F4用AHB1
#define USARTx_RX_SOURCE                 GPIO_PinSource11       //F1用不到 F4才用的到
#define USARTx_RX_AF                     GPIO_AF_USART3         //F1用不到 F4才用的到
#endif


#if 0   //使用USART2时的配置
#define USARTx                           USART2
#define USARTx_CLK                       RCC_APB1Periph_USART2
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define USARTx_GPIOCLK_INIT              RCC_APB2PeriphClockCmd //F1用APB2   F4用AHB1
#define USARTx_OPEN_AFCLK                RCC_APB2Periph_AFIO    //打开复用时钟    F1用F4不用

#define USARTx_TX_PIN                    GPIO_Pin_2                
#define USARTx_TX_GPIO_PORT              GPIOA                       
#define USARTx_TX_GPIO_CLK               RCC_APB2Periph_GPIOA   //F1用APB2   F4用AHB1
#define USARTx_TX_SOURCE                 GPIO_PinSource2        //F1用不到 F4才用的到
#define USARTx_TX_AF                     GPIO_AF_USART2         //F1用不到 F4才用的到

#define USARTx_RX_PIN                    GPIO_Pin_3                
#define USARTx_RX_GPIO_PORT              GPIOA                    
#define USARTx_RX_GPIO_CLK               RCC_APB2Periph_GPIOA   //F1用APB2   F4用AHB1
#define USARTx_RX_SOURCE                 GPIO_PinSource3       //F1用不到 F4才用的到
#define USARTx_RX_AF                     GPIO_AF_USART2         //F1用不到 F4才用的到
#endif


#if 0   //使用USART1时的配置
#define USARTx                           USART1
#define USARTx_CLK                       RCC_APB2Periph_USART1
#define USARTx_CLK_INIT                  RCC_APB2PeriphClockCmd
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

#define USARTx_GPIOCLK_INIT              RCC_APB2PeriphClockCmd //F1用APB2   F4用AHB1
#define USARTx_OPEN_AFCLK                RCC_APB2Periph_AFIO    //打开复用时钟    F1用F4不用

#define USARTx_TX_PIN                    GPIO_Pin_9                
#define USARTx_TX_GPIO_PORT              GPIOA                       
#define USARTx_TX_GPIO_CLK               RCC_APB2Periph_GPIOA   //F1用APB2   F4用AHB1
#define USARTx_TX_SOURCE                 GPIO_PinSource9        //F1用不到 F4才用的到
#define USARTx_TX_AF                     GPIO_AF_USART1         //F1用不到 F4才用的到

#define USARTx_RX_PIN                    GPIO_Pin_10                
#define USARTx_RX_GPIO_PORT              GPIOA                    
#define USARTx_RX_GPIO_CLK               RCC_APB2Periph_GPIOA   //F1用APB2   F4用AHB1
#define USARTx_RX_SOURCE                 GPIO_PinSource10       //F1用不到 F4才用的到
#define USARTx_RX_AF                     GPIO_AF_USART1         //F1用不到 F4才用的到
#endif


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
int nBSP_USARTConfig(int nBaudRate)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

	/* 串口1 TX = PA9   RX = PA10 */
	/* 第1步： 配置GPIO */
	/* 打开 GPIO 时钟 */
    USARTx_GPIOCLK_INIT(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

	/* 打开复用功能时钟 */
	USARTx_GPIOCLK_INIT(USARTx_OPEN_AFCLK,ENABLE);    //F1用 F4不用
    
	/* 打开 UART 时钟 */
    USARTx_CLK_INIT(USARTx_CLK, ENABLE);
    
    //GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

	/* 将 PA9 映射为 USART1_TX */
	/* 将 PA10 映射为 USART1_RX */
	/* 配置 USART Tx 为复用功能 */	
    GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;              //TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
    
	GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;             //RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);  

    //GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

    /* Enable the USART OverSampling by 8 */
    //USART_OverSampling8Cmd(USARTx, ENABLE);  

	/* 第2步： 配置串口硬件参数 */
    USART_InitStructure.USART_BaudRate = nBaudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx, &USART_InitStructure);

    /* 开启串口中断 */
    NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能串口 */
    USART_Cmd(USARTx, ENABLE);
    
	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USARTx, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */  
    
    return 0;
}    

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(USARTx, (uint8_t) ch);

    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
    {}

    return ch;
}

//void USARTx_IRQHandler(void)
//{    
//}
