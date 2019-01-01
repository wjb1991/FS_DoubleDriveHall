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
 * File: $Id: portserial.c,v 1.2 2010/06/06 13:46:42 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include <stdlib.h>
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#if 0   //使用USART3时的配置
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

#if 1   //使用USART1时的配置
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


/* ----------------------- Static variables ---------------------------------*/

/**
* @brief 控制接收和发送状态
* @param xRxEnable 接收使能、
*        xTxEnable 发送使能
* @retval None
*/
void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{

    if( xRxEnable )
    {
        //使能接收和接收中断
        USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
        //MAX485操作 低电平为接收模式
        //GPIO_ResetBits(GPIOD,GPIO_Pin_8);
    }
    else
    {
        USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE); 
        //MAX485操作 高电平为发送模式
        //GPIO_SetBits(GPIOD,GPIO_Pin_8);
    }

    if( xTxEnable )
    {
        //使能发送完成中断
        USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
    }
    else
    {
        //禁止发送完成中断
        USART_ITConfig(USARTx, USART_IT_TXE, DISABLE);
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    
    
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    (void)ucPORT;       //不修改串口
    (void)ucDataBits;   //不修改数据位长度
    (void)eParity;      //不修改校验格式

	/* 串口1 TX = PA9   RX = PA10 */
	/* 第1步： 配置GPIO */
	/* 打开 GPIO 时钟 */
    USARTx_GPIOCLK_INIT(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

	/* 打开复用功能时钟 */
	USARTx_GPIOCLK_INIT(USARTx_OPEN_AFCLK,ENABLE);    //F1用 F4不用
    
	/* 打开 UART 时钟 */
    USARTx_CLK_INIT(USARTx_CLK, ENABLE);

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

    /* Enable the USART OverSampling by 8 */
    //USART_OverSampling8Cmd(USARTx, ENABLE);  

	/* 第2步： 配置串口硬件参数 */
    USART_InitStructure.USART_BaudRate = 115200;
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
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能串口 */
    USART_Cmd(USARTx, ENABLE);
    
	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USARTx, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */  
    
    return TRUE;
}

void
vMBPortSerialClose( void )
{
     USART_Cmd(USARTx, DISABLE);
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    USART_SendData(USARTx, ucByte);
    //while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
    //{}
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    *pucByte = USART_ReceiveData(USARTx);
    return TRUE;
}

void USARTx_IRQHandler(void)
{
    //发生接收中断
    if(USART_GetITStatus(USARTx, USART_IT_RXNE) == SET)
    {
        //清除中断标志位 
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
        pxMBFrameCBByteReceived(  );
    }

    //发生完成中断
    if(USART_GetITStatus(USARTx, USART_IT_TXE) == SET)
    {
        //清除中断标志
        USART_ClearITPendingBit(USART1, USART_IT_TXE);
        pxMBFrameCBTransmitterEmpty(  );
    }
}
