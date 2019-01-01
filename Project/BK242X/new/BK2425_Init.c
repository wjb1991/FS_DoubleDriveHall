#include "bk2425.h"

UINT8 op_status;

static RFProc_CB callbark = NULL;

/* RF channel table,updated on May 30,2012 */
const UINT8 RF_TABLE[16]={
2402,2405,2413,
2415,2417,2420,
2423,2428,2432,
2435,2438,2440,
2442,2446,2457,2459,//for 16 channels,use above frequency 
};

/* Bank1 register initialization value,updated on May 17,2012 */

//In the array Bank1_Reg0_13[],all the register values are the byte reversed!!!!!!!!!!!!!!!!!!!!!
const unsigned long Bank1_Reg0_13[]={
0x1B8296f9,
/*REG4,
----------------------------------------------------------------
|			               1Mbps      | 2Mbps	          |	250Kbps		|
|     Normal Mode		 0x1B8296f9	|	0xdB8296f9      | 0xdB8a96f9
|     CW Normal Mode 0x218296f9	
|     外部PA         0xDB8A96C1 | 0x1B8296C1      | 0xDB8296C1
----------------------------------------------------------------
*/
0xA60F0624,
/*REG5,
----------------------------------------------------------------
|			1Mbps:0xA60F0624      disable rssi
|     2Mbps:0xB60F0624			disable rssi
|     259kbps:0xB60F0624		disable rssi
----------------------------------------------------------------
*/
0x00127300,
/*REG12,120517
0x00127300:PLL locking time 120us 
0x00127305(chip default):PLL locking time 130us compatible with nRF24L01;
*/
0x36B48000,//REG13
};

const UINT8 Bank1_Reg14[]=
{
0x41,0x20,0x08,0x04,0x81,0x20,0xcf,0xF7,0xfe,0xff,0xff
};

//Bank0 register initialization value
const UINT8 Bank0_Reg[][2]={
{0,0x0F},   //开启RX_DR   TX_DR   MAX_RT中断
{1,0x3F},   //开启6个数据通道
{2,0x3F},   //开启6个数据通道
{3,0x03},   //5个byte地址
{4,0xff},   //15次通讯失败 wait400us
{5,0x17},   //设置频段
{6,0x07},	//20170619 改为0x07 设置为1Mbps REG6,120517,0x0F or 0x2F:2Mbps; 0x07:1Mbps ; 0x27:250Kbps
{7,0x07},   //状态寄存器
{8,0x00},   
{9,0x00},
{12,0xc3},  //数据流2地址
{13,0xc4},  //数据流3地址
{14,0xc5},  //数据流4地址
{15,0xc6},  //数据流5地址
{17,0x20},  //数据流0长度 32byte
{18,0x20},  //数据流1长度 32byte
{19,0x20},  //数据流2长度 32byte
{20,0x20},  //数据流3长度 32byte
{21,0x20},  //数据流4长度 32byte
{22,0x20},  //数据流5长度 32byte
{23,0x00},  //FIFO_STATUS
{28,0x3F},  //6个通道数据流全部开启 开启数据动态长度 
{29,0x07}   //开启数据动态长度  开启应答  开启W_TX_PAYLOAD_NOACK
};


const UINT8 RX0_Address[]={0x15,0x59,0x23,0xC6,0x29};
const UINT8 RX1_Address[]={0x10,0x56,0x24,0xCD,0x78};

///////////////////////////////////////////////////////////////////////////////
//                  SPI access                                               //
///////////////////////////////////////////////////////////////////////////////

/**************************************************         
Function: SPI_RW();                                         
                                                            
Description:                                                
	Writes one UINT8 to BK2425, and return the UINT8 read 
/**************************************************/        
UINT8 SPI_RW(UINT8 value)                                    
{                                                           
	UINT8 bit_ctr;
	UINT8 i;
	for(bit_ctr=0;bit_ctr<8;bit_ctr++)   // output 8-bit
	{
		if(value & 0x80)
		{
			MOSIHI();
		}
		else
		{
			MOSILOW();		
		}
		for(i = 4;i > 0;i --);						// Delay

		value = (value << 1);             // shift next bit into MSB..
		SCKHI();                          // Set SCK high..
		for(i = 2;i > 0;i --);						// Delay
		
		//value |= MISO;       		        // capture current MISO bit
    if( MISOVal() != 0)
      value |= 1;
			
		for(i = 2;i > 0;i --);						// Delay
		SCKLOW();            		          // ..then set SCK low again
	}
	return(value);           		        // return read UINT8
}                                                           
/**************************************************/        
                                                            
/**************************************************         
Function: SPI_Write_Reg();                                  
                                                            
Description:                                                
	Writes value 'value' to register 'reg'              
/**************************************************/        
void SPI_Write_Reg(UINT8 reg, UINT8 value)                 
{
	CSNLOW();                   // CSN low, init SPI transaction
	op_status = SPI_RW(reg);      // select register
	SPI_RW(value);             // ..and write value to it..
	CSNHI();                   // CSN high again
}                                                           
/**************************************************/        
                                                            
/**************************************************         
Function: SPI_Read_Reg();                                   
                                                            
Description:                                                
	Read one UINT8 from BK2425 register, 'reg'           
/**************************************************/        
UINT8 SPI_Read_Reg(UINT8 reg)                               
{                                                           
	UINT8 value;
	CSNLOW();                // CSN low, initialize SPI communication...
	op_status=SPI_RW(reg);            // Select register to read from..
	value = SPI_RW(0);    // ..then read register value
	CSNHI();                // CSN high, terminate SPI communication

	return(value);        // return register value
}                                                           
/**************************************************/        
                                                            
/**************************************************         
Function: SPI_Read_Buf();                                   
                                                            
Description:                                                
	Reads 'length' #of length from register 'reg'         
/**************************************************/        
void SPI_Read_Buf(UINT8 reg, UINT8 *pBuf, UINT8 length)     
{                                                           
	UINT8 status,byte_ctr;                              
                                                            
	CSNLOW();                    		// Set CSN l
	status = SPI_RW(reg);       		// Select register to write, and read status UINT8
                                                            
	for(byte_ctr=0;byte_ctr<length;byte_ctr++)           
		pBuf[byte_ctr] = SPI_RW(0);    // Perform SPI_RW to read UINT8 from BK2425
                                                            
	CSNHI();                           // Set CSN high again
               
}                                                           
/**************************************************/        
                                                            
/**************************************************         
Function: SPI_Write_Buf();                                  
                                                            
Description:                                                
	Writes contents of buffer '*pBuf' to BK2425         
/**************************************************/        
void SPI_Write_Buf(UINT8 reg, UINT8 *pBuf, UINT8 length)    
{                                                           
	UINT8 byte_ctr;                              
                                                            
	CSNLOW();                   // Set CSN low, init SPI tranaction
	op_status = SPI_RW(reg);    // Select register to write to and read status UINT8
	for(byte_ctr=0; byte_ctr<length; byte_ctr++) // then write all UINT8 in buffer(*pBuf) 
		SPI_RW(*pBuf++);                                    
	CSNHI();                 // Set CSN high again      

}                                                           
/**************************************************/        


/**************************************************
Function: SwitchToRxMode();
Description:
	switch to Rx mode
/**************************************************/
void SwitchToRxMode(void)
{
  UINT8 value;

	SPI_Write_Reg(FLUSH_RX,0);//flush Rx

	value=SPI_Read_Reg(STATUS);	// read register STATUS's value
	SPI_Write_Reg(WRITE_REG|STATUS,value);// clear RX_DR or TX_DS or MAX_RT interrupt flag

	CELOW();

	value=SPI_Read_Reg(CONFIG);	// read register CONFIG's value
//PRX
	value=value|0x01;//set bit 1
  	SPI_Write_Reg(WRITE_REG | CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..

	CEHI();
}

/**************************************************
Function: SwitchToTxMode();
Description:
	switch to Tx mode
**************************************************/
void SwitchToTxMode(void)
{
	UINT8 value;
	SPI_Write_Reg(FLUSH_TX,0);//flush Tx

	CELOW();
	value=SPI_Read_Reg(CONFIG);	// read register CONFIG's value
//PTX
	value=value&0xfe;//set bit 1
  	SPI_Write_Reg(WRITE_REG | CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled.

	CEHI();
}

/**************************************************
Function: SwitchCFG();
                                                            
Description:
	 access switch between Bank1 and Bank0 

Parameter:
	_cfg      1:register bank1
	          0:register bank0
Return:
     None
**************************************************/
void SwitchCFG(char _cfg)//1:Bank1 0:Bank0
{
	UINT8 Tmp;

	Tmp=SPI_Read_Reg(7);
	Tmp=Tmp&0x80;

	if( ( (Tmp)&&(_cfg==0) )
	||( ((Tmp)==0)&&(_cfg) ) )
	{
		SPI_Write_Reg(ACTIVATE_CMD,0x53);
	}
}

/**************************************************
Function: SetChannelNum();
Description:
	set channel number

/**************************************************/
void SetChannelNum(UINT8 ch)
{
	SPI_Write_Reg((UINT8)(WRITE_REG|5),(UINT8)(ch));
}



///////////////////////////////////////////////////////////////////////////////
//                  BK2425 initialization                                    //
///////////////////////////////////////////////////////////////////////////////
/**************************************************         
Function: BK2425_Initialize();                                  

Description:                                                
	register initialization
**************************************************/   
void BK2425_Initialize(void* ptrfun,UINT8* RX0_Addr,UINT8* RX1_Addr,UINT8* TX_Addr)
{
	INT8 i,j;
 	UINT8 WriteArr[4];

	DelayMs(100);//delay more than 50ms.
	
	SwitchCFG(0);

    callbark = (RFProc_CB)ptrfun;
  
//********************Write Bank0 register******************
	for(i=20;i!=0xff;i--)
		SPI_Write_Reg((WRITE_REG|Bank0_Reg[i][0]),Bank0_Reg[i][1]);

//reg 10 - Rx0 addr 数据流0地址
	SPI_Write_Buf((WRITE_REG|10),RX0_Addr,5);
	
//REG 11 - Rx1 addr 数据流1地址 数据流2-5的地址前4个byte和数据流1的地址是一样的仅最后一个字节不一样
	SPI_Write_Buf((WRITE_REG|11),RX1_Addr,5);

//REG 16 - TX addr  发送的地址
	SPI_Write_Buf((WRITE_REG|16),TX_Addr,5);
//	printf("\nEnd Load Reg");

	i=SPI_Read_Reg(29);

	if(i==0) // i!=0 showed that chip has been actived.so do not active again.
		SPI_Write_Reg(ACTIVATE_CMD,0x73);// Active

	for(i=22;i>=21;i--)
		SPI_Write_Reg((WRITE_REG|Bank0_Reg[i][0]),Bank0_Reg[i][1]);

//********************Write Bank1 register******************
	SwitchCFG(1);

	for(i=0;i<=1;i++)//reverse
	{
		for(j=0;j<4;j++)
			WriteArr[j]=(Bank1_Reg0_13[i]>>(8*(j) ) )&0xff;

		SPI_Write_Buf((WRITE_REG|(i+4)),&(WriteArr[0]),4);
	}

	for(i=2;i<=3;i++)
	{
		for(j=0;j<4;j++)
			WriteArr[j]=(Bank1_Reg0_13[i]>>(8*(3-j) ) )&0xff;

		SPI_Write_Buf((WRITE_REG|(i+10)),&(WriteArr[0]),4);
	}

	SPI_Write_Buf((WRITE_REG|14),(uint8_t*)(&(Bank1_Reg14[0])),11);

//toggle REG4<25,26>
	for(j=0;j<4;j++)
		WriteArr[j]=(Bank1_Reg0_13[0]>>(8*(j) ) )&0xff;

	WriteArr[0]=WriteArr[0]|0x06;
	SPI_Write_Buf((WRITE_REG|4),&(WriteArr[0]),4);

	WriteArr[0]=WriteArr[0]&0xf9;
	SPI_Write_Buf((WRITE_REG|4),&(WriteArr[0]),4);

    Set_Sen_Mode(1);    //20170618设置高灵敏度
	DelayMs(10);
	
//********************switch back to Bank0 register access******************
	SwitchCFG(0);

	SwitchToRxMode();//switch to RX mode

	do
	{
		WriteArr[0] = Get_Chip_ID();
	}
	while(WriteArr[0] != 0x63);

}

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
void Send_Packet(UINT8 type,UINT8* pbuf,UINT8 len)
{
	UINT8 fifo_sta;

	fifo_sta=SPI_Read_Reg(FIFO_STATUS);	// read register FIFO_STATUS's value
/*  //20170218删，解决遥控器和主控失联的问题
	if((fifo_sta&FIFO_STATUS_TX_FULL)==0)//if not full, send data  
	  	SPI_Write_Buf(type, pbuf, len); // Writes data to buffer  A0,B0,A8
	 */  //20170218删，解决遥控器和主控失联的问题
while((fifo_sta&FIFO_STATUS_TX_FULL)!=0); //20170218增，解决遥控器和主控失联的问题
    SPI_Write_Buf(type, pbuf, len);       //20170218增，解决遥控器和主控失联的问题
do{fifo_sta=SPI_Read_Reg(FIFO_STATUS);}   //20170218增，解决遥控器和主控失联的问题
while((  fifo_sta&FIFO_STATUS_TX_EMPTY)==0);//20170218增，解决遥控器和主控失联的问题

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
UINT8 rx_buf[MAX_PACKET_LEN];
UINT8 Receive_Packet(void)
{
	UINT8 len,i,sta,fifo_sta;

	sta=SPI_Read_Reg(STATUS);	// read register STATUS's value
//	if(STATUS_RX_DR)   // if receive data ready (RX_DR) interrupt   //20170218删，解决遥控器和主控失联的问题
	if(sta&STATUS_RX_DR==0)				//20170218增，解决遥控器和主控失联的问题
	{                             //20170218增，解决遥控器和主控失联的问题
		return 0;                   //20170218增，解决遥控器和主控失联的问题
  }                             //20170218增，解决遥控器和主控失联的问题
	else                          //20170218增，解决遥控器和主控失联的问题
	{                             
		do
		{
			len=SPI_Read_Reg(R_RX_PL_WID_CMD);	// read len

			if(len<=MAX_PACKET_LEN)
			{
				SPI_Read_Buf(RD_RX_PLOAD,rx_buf,len);// read receive payload from RX_FIFO buffer
				//while(1);
				//add user app
				//if( len != 0 )
				//{
				//	rx_buf[MAX_PACKET_LEN] = 0;
				//}
				
                if ( len != 0 && callbark != NULL )
                {
                  //(void)
                  (*callbark)(len , rx_buf);   //执行RF_Dataprocess()
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
						
		//}while((fifo_sta&FIFO_STATUS_RX_EMPTY)==0); //while not empty //20170220 删除，试为了解决遥控器失联的问题
		}while(0); //20170220增加，试为了解决遥控器失联的问题
	}

	SPI_Write_Reg(WRITE_REG|STATUS,sta);// clear RX_DR or TX_DS or MAX_RT interrupt flag
	return len;                         //20170221
}


/**************************************************         
Function: DelayMs();                                  

Description:                                                
	delay ms,please implement this function according to your MCU.
**************************************************/  
void DelayMs(UINT16 ms)
{
  Delay200us();
  Delay200us();
  Delay200us();
  Delay200us();
  Delay200us();
}

void  SetTXAddr(UINT8* TX_Addr)
{
//REG 16 - TX addr  发送的地址
	SPI_Write_Buf((WRITE_REG|16),TX_Addr,5);
}

void  SetRX0Addr(UINT8* RX0_Addr)
{
  //reg 10 - Rx0 addr 数据流0地址
	SPI_Write_Buf((WRITE_REG|10),RX0_Addr,5);
}

void  BK2425_SetToPD(void)
{
  //reg 10 - Rx0 addr 数据流0地址
	SPI_Write_Reg((WRITE_REG|0),0x0d);
}
