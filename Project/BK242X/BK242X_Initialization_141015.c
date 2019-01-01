#include "bk242X.h"


uint8_t op_status;

/* RF channel table,updated on May 30,2012 */
const  uint16_t RF_TABLE[16]={
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

const uint8_t Bank1_Reg14[]=
{
0x41,0x20,0x08,0x04,0x81,0x20,0xcf,0xF7,0xfe,0xff,0xff
};

//Bank0 register initialization value
const uint8_t Bank0_Reg[][2]={
    {0,0x0F},  //关闭RX_DR   TX_DR   MAX_RT中断
    {1,0x3F},  //开启6个数据通道  自动确认功能
    {2,0x3F},  //开启6个数据通道
    {3,0x03},  //5个byte地址
    {4,0xff},  //15次通讯失败 wait400us
    {5,0x39},  //设置频段
    {6,0x07},  //REG6,120517,0x0F or 0x2F:2Mbps; 0x07:1Mbps ; 0x27:250Kbps
    {7,0x07},  //状态寄存器
    {8,0x00},  
    {9,0x00},  
    {12,0xc3}, //数据流2地址
    {13,0xc4}, //数据流3地址
    {14,0xc5}, //数据流4地址
    {15,0xc6}, //数据流5地址
    {17,0x20}, //数据流0长度 32byte
    {18,0x20}, //数据流1长度 32byte
    {19,0x20}, //数据流2长度 32byte
    {20,0x20}, //数据流3长度 32byte
    {21,0x20}, //数据流4长度 32byte
    {22,0x20}, //数据流5长度 32byte
    {23,0x00}, //FIFO_STATUS
    {28,0x3F}, //6个通道数据流全部开启 开启数据动态长度 
    {29,0x07}  //开启数据动态长度  开启应答  开启W_TX_PAYLOAD_NOACK
};

uint8_t RX0_Address[]={0x80,0x1c,0x00,0x38,0x06};     //通讯通道 0x80,0x1c,0x00,0x2f,0x06
const uint8_t RX1_Address[]={0x1,0x59,0x23,0xC6,0x29};      //配对通道

//const uint8_t RX0_Address[]={0x15,0x59,0x23,0xC6,0x29};
//const uint8_t RX1_Address[]={0x10,0x56,0x24,0xCD,0x78};
//{0x1,0x59,0x23,0xC6,0x29}通讯公共地址
///////////////////////////////////////////////////////////////////////////////
//                  SPI access                                               //
///////////////////////////////////////////////////////////////////////////////

/**************************************************         
Function: SPI_RW();                                         
                                                            
Description:                                                
	Writes one uint8_t to BK2425, and return the uint8_t read 
**************************************************/        
uint8_t SPI_RW(uint8_t value)                                    
{                                                           
	uint8_t bit_ctr;
	for(bit_ctr=0;bit_ctr<8;bit_ctr++)   // output 8-bit
	{
        //int j = 0;
		if(value & 0x80)
		{
			MOSIHI();
		}
		else
		{
			MOSILOW();		
		}
        //for(j  = 0 ; j< 200; j ++);
		value = (value << 1);           // shift next bit into MSB..
		SCKHI();                      // Set SCK high..
        //for(j  = 0 ; j< 200; j ++);
		//value |= MISOVAL();       		  // capture current MISO bit  
        if(MISOVAL()!=0)
          value |= 1;
		SCKLOW();            		  // ..then set SCK low again
	}
	return(value);           		  // return read uint8_t
}                                                           
/**************************************************/        
                                                            
/**************************************************         
Function: SPI_Write_Reg();                                  
                                                            
Description:                                                
	Writes value 'value' to register 'reg'              
**************************************************/        
void SPI_Write_Reg(uint8_t reg, uint8_t value)                 
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
	Read one uint8_t from BK2425 register, 'reg'           
**************************************************/        
uint8_t SPI_Read_Reg(uint8_t reg)                               
{                                                           
	uint8_t value;
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
**************************************************/        
void SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t length)     
{                                                           
	uint8_t status,byte_ctr;                              
                                              
	CSNLOW();                    		// Set CSN l
	status = SPI_RW(reg);       		// Select register to write, and read status uint8_t
                                                            
	for(byte_ctr=0;byte_ctr<length;byte_ctr++)           
		pBuf[byte_ctr] = SPI_RW(0);    // Perform SPI_RW to read uint8_t from BK2425
                                                            
	CSNHI();                           // Set CSN high again
               
}                                                           
/**************************************************/        
                                                            
/**************************************************         
Function: SPI_Write_Buf();                                  
                                                            
Description:                                                
	Writes contents of buffer '*pBuf' to BK2425         
**************************************************/        
void SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t length)    
{                                                           
	uint8_t byte_ctr;                              
                                                            
	CSNLOW();                   // Set CSN low, init SPI tranaction
	op_status = SPI_RW(reg);    // Select register to write to and read status uint8_t
	for(byte_ctr=0; byte_ctr<length; byte_ctr++) // then write all uint8_t in buffer(*pBuf) 
		SPI_RW(*pBuf++);                                    
	CSNHI();                 // Set CSN high again      

}                                                           
/**************************************************/        


/**************************************************
Function: SwitchToRxMode();
Description:
	switch to Rx mode
**************************************************/
void SwitchToRxMode(void)
{
	uint8_t value;

	SPI_Write_Reg(FLUSH_RX,0);//flush Rx

	value=SPI_Read_Reg(STATUS);	// read register STATUS's value
	SPI_Write_Reg(WRITE_REG_CMD|STATUS,value);// clear RX_DR or TX_DS or MAX_RT interrupt flag

	CELOW();

	value=SPI_Read_Reg(CONFIG);	// read register CONFIG's value
//PRX
	value=value|0x01;//set bit 1
  	SPI_Write_Reg(WRITE_REG_CMD | CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..

	CEHI();
}

/**************************************************
Function: SwitchToTxMode();
Description:
	switch to Tx mode
**************************************************/
void SwitchToTxMode(void)
{
	uint8_t value;
	SPI_Write_Reg(FLUSH_TX,0);//flush Tx

	CELOW();
	value=SPI_Read_Reg(CONFIG);	// read register CONFIG's value
//PTX
	value=value&0xfe;//set bit 1
  	SPI_Write_Reg(WRITE_REG_CMD | CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled.

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
	uint8_t Tmp;

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

**************************************************/
void SetChannelNum(uint8_t ch)
{
	SPI_Write_Reg((uint8_t)(WRITE_REG_CMD|5),(uint8_t)(ch));
}



///////////////////////////////////////////////////////////////////////////////
//                  BK2425 initialization                                    //
///////////////////////////////////////////////////////////////////////////////
/**************************************************         
Function: BK2425_Initialize();                                  

Description:                                                
	register initialization
**************************************************/   
void BK2425_Initialize(void)
{
	int8_t i,j;
 	uint8_t WriteArr[4];

	DelayMs(100);//delay more than 50ms.

//	do
//	{
//		WriteArr[0] = Get_Chip_ID();
//	}
//	while(WriteArr[0] != 0x63);    
    
	SwitchCFG(0);

//********************Write Bank0 register******************
	for(i=20;i>=0;i--)
    //for(i=20;i!=0xff;i--)
		SPI_Write_Reg((WRITE_REG_CMD|Bank0_Reg[i][0]),Bank0_Reg[i][1]);

//reg 10 - Rx0 addr
	SPI_Write_Buf((WRITE_REG_CMD|10),(uint8_t*)RX0_Address,5);
	
//REG 11 - Rx1 addr
	SPI_Write_Buf((WRITE_REG_CMD|11),(uint8_t*)RX1_Address,5);

//REG 16 - TX addr
	SPI_Write_Buf((WRITE_REG_CMD|16),(uint8_t*)RX0_Address,5);
//	printf("\nEnd Load Reg");

	i=SPI_Read_Reg(29);

	if(i==0) // i!=0 showed that chip has been actived.so do not active again.
		SPI_Write_Reg(ACTIVATE_CMD,0x73);// Active

	for(i=22;i>=21;i--)
		SPI_Write_Reg((WRITE_REG_CMD|Bank0_Reg[i][0]),Bank0_Reg[i][1]);

//********************Write Bank1 register******************
	SwitchCFG(1);

	for(i=0;i<=1;i++)//reverse
	{
		for(j=0;j<4;j++)
			WriteArr[j]=(Bank1_Reg0_13[i]>>(8*(j) ) )&0xff;

		SPI_Write_Buf((WRITE_REG_CMD|(i+4)),&(WriteArr[0]),4);
	}

	for(i=2;i<=3;i++)
	{
		for(j=0;j<4;j++)
			WriteArr[j]=(Bank1_Reg0_13[i]>>(8*(3-j) ) )&0xff;

		SPI_Write_Buf((WRITE_REG_CMD|(i+10)),&(WriteArr[0]),4);
	}

	SPI_Write_Buf((WRITE_REG_CMD|14),(uint8_t*)&(Bank1_Reg14[0]),11);

//toggle REG4<25,26>
	for(j=0;j<4;j++)
		WriteArr[j]=(Bank1_Reg0_13[0]>>(8*(j) ) )&0xff;

	WriteArr[0]=WriteArr[0]|0x06;
	SPI_Write_Buf((WRITE_REG_CMD|4),&(WriteArr[0]),4);

	WriteArr[0]=WriteArr[0]&0xf9;
	SPI_Write_Buf((WRITE_REG_CMD|4),&(WriteArr[0]),4);

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

void Set_Pipe0Address(uint8_t* address)
{
    memcpy(RX0_Address,address,5);
//reg 10 - Rx0 addr
	SPI_Write_Buf((WRITE_REG_CMD|10),(uint8_t*)address,5);
//REG 16 - TX addr
	SPI_Write_Buf((WRITE_REG_CMD|16),(uint8_t*)address,5);
}

void Get_Pipe0Address(uint8_t* address)
{
    memcpy(address,RX0_Address,5);
}


/**************************************************         
Function: DelayMs();                                  

Description:                                                
	delay ms,please implement this function according to your MCU.
**************************************************/  
//extern void Delay(__IO uint32_t nTime);
void DelayMs(uint16_t ms)
{
    vBSP_DelayMS(ms);
}


