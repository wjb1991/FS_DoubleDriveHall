/*--------------------------------------------------------------------------------------
*This file is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
*****************
*MCU: STC89LE58RD+
Compiler:uVision2 2.40
*****************
* website: http://www.bekencorp.com
---------------------------------------------------------------------------------------*/
#ifndef __BK2425_H__
#define __BK2425_H__

//#include <intrins.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
//#include <REG51F.H>
#include "iodefine.h"

#define TRUE 1
#define FALSE 0


#define UINT8 unsigned char
#define INT8 char
#define INT16 int
#define UINT8 unsigned char
#define UINT16 unsigned int
#define UINT32 unsigned long

/*
sbit CE =  P2^3;
sbit CSN=  P2^4;
sbit SCK=  P2^5;
sbit MOSI= P2^2;
sbit MISO= P2^1;
sbit IRQ = P0^4;

#define sBitCE   (1<<3)
#define BitCE_bit   (3)	
*/
#define MAX_PACKET_LEN  32// max value is 32


//************************FSK COMMAND and REGISTER****************************************//
// SPI(BK2425) commands
#define READ_REG        0x00  // Define read command to register
#define WRITE_REG       0x20  // Define write command to register
#define RD_RX_PLOAD     0x61  // Define RX payload register address
#define WR_TX_PLOAD     0xA0  // Define TX payload register address
#define FLUSH_TX        0xE1  // Define flush TX register command
#define FLUSH_RX        0xE2  // Define flush RX register command
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command
#define W_TX_PAYLOAD_NOACK_CMD	0xb0
#define W_ACK_PAYLOAD_CMD	0xa8
#define ACTIVATE_CMD		0x50
#define R_RX_PL_WID_CMD		0x60
#define NOP             0xFF  // Define No Operation, might be used to read status register

// SPI(BK2425) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS          0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address
#define PAYLOAD_WIDTH   0x1f  // 'payload length of 256 bytes modes register address

//interrupt status
#define STATUS_RX_DR 0x40
#define STATUS_TX_DS 0x20
#define STATUS_MAX_RT 0x10

#define STATUS_TX_FULL 0x01

//FIFO_STATUS
#define FIFO_STATUS_TX_REUSE 0x40
#define FIFO_STATUS_TX_FULL 0x20
#define FIFO_STATUS_TX_EMPTY 0x10

#define FIFO_STATUS_RX_FULL 0x02
#define FIFO_STATUS_RX_EMPTY 0x01


UINT8 SPI_Read_Reg(UINT8 reg);
void SPI_Read_Buf(UINT8 reg, UINT8 *pBuf, UINT8 bytes);

void SPI_Write_Reg(UINT8 reg, UINT8 value);
void SPI_Write_Buf(UINT8 reg, UINT8 *pBuf, UINT8 bytes);


void SwitchToTxMode(void);
void SwitchToRxMode(void);

void SPI_Bank1_Read_Reg(UINT8 reg, UINT8 *pBuf);
void SPI_Bank1_Write_Reg(UINT8 reg, UINT8 *pBuf);
void SwitchCFG(char _cfg);

void DelayMs(UINT16 ms);

void Send_Packet(UINT8 type,UINT8* pbuf,UINT8 len);

//void Receive_Packet(void);
UINT8 Receive_Packet(void);

UINT8 Get_Chip_ID(void);

void BK2425_Initialize(void* ptrfun,UINT8* RX0_Addr,UINT8* RX1_Addr,UINT8* TX_Addr);

void  SetTXAddr(UINT8* TX_Addr);

void  SetRX0Addr(UINT8* RX0_Addr);

void  BK2425_SetToPD(void);

#endif