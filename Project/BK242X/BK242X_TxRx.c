#include "bk242X.h"

/**************************************************
Function: Send_Packet
Description:
	fill FIFO to send a packet
Parameter:
	type: WR_TX_PLOAD or  W_TX_PAYLOAD_NOACK_CMD
	pbuf: a buffer pointer
	len: packet length
Return:
	None
**************************************************/
void Send_Packet(uint8_t type,uint8_t* pbuf,uint8_t len)
{
	uint8_t fifo_sta;

	fifo_sta=SPI_Read_Reg(FIFO_STATUS);	// read register FIFO_STATUS's value

	if((fifo_sta&FIFO_STATUS_TX_FULL)==0)//if not full, send data  
	  	SPI_Write_Buf(type, pbuf, len); // Writes data to buffer  A0,B0,A8
	  	 	
}
int8_t Wait_Sender(void)
{
    
    __IO uint8_t i = 0;
    __IO uint8_t time_out = 0;
    while(1)
    {
        i = SPI_Read_Reg(STATUS);
        if(i & STATUS_MAX_RT )
        {
            SPI_Write_Reg(WRITE_REG_CMD|STATUS,STATUS_MAX_RT);
            return -1;
        }
          
        if(i & STATUS_TX_DS )
        {     
            SPI_Write_Reg(WRITE_REG_CMD|STATUS,STATUS_TX_DS);
            return 0;
        }
        
        DelayMs(1);
        if( ++time_out >= 5)
        {
            return -2;
        }
    }
#if 0    
    do
    {
        i = SPI_Read_Reg(STATUS);
    }
    while(!(STATUS_TX_DS & i));
    SPI_Write_Reg(WRITE_REG|STATUS,STATUS_TX_DS);
#endif
}

/**************************************************
Function: Receive_Packet
Description:
	read FIFO to read a packet
Parameter:
	None
Return:
	None
**************************************************/
void Receive_Packet(void* hook)
{
	uint8_t len,i,sta,fifo_sta;
	uint8_t rx_buf[MAX_PACKET_LEN];

	sta=SPI_Read_Reg(STATUS);	// read register STATUS's value
	if( sta & STATUS_RX_DR)				// if receive data ready (RX_DR) interrupt  <<这里有问题
	{

		do
		{
			len=SPI_Read_Reg(R_RX_PL_WID_CMD);	// read len

			if(len<=MAX_PACKET_LEN)
			{
				SPI_Read_Buf(RD_RX_PLOAD,rx_buf,len);// read receive payload from RX_FIFO buffer       
                
                if ( len != 0 && hook != NULL )
                {
                    //recvCallBark  callbark = （recvCallBark）hook；
                    //(*callbark)(len , rx_buf);
                    (*((recvCallBark)hook))(len , rx_buf);
                }
                
#ifdef DEBUG
				printf("\nPacket:");
				for(i=0;i<len;i++)
				{
					printf("%x;",(int)rx_buf[i]);
				}
#endif
			}
			else
			{
				SPI_Write_Reg(FLUSH_RX,0);//flush Rx
			}

			fifo_sta=SPI_Read_Reg(FIFO_STATUS);	// read register FIFO_STATUS's value
						
		}while((fifo_sta&FIFO_STATUS_RX_EMPTY)==0); //while not empty

	}

	SPI_Write_Reg(WRITE_REG_CMD|STATUS,sta);// clear RX_DR or TX_DS or MAX_RT interrupt flag
	
}


