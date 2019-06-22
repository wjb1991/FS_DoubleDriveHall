#include "rf2_4g.h"
#include "bk242X.h"

rfconntstate_t ucRfConntState = eRFDisconnt;
rfworkstate_t ucRfWorkState = eRFInit;

uint8_t ucResetCnt = 0;

uint8_t ucSystemCalibrationFinishEvent = 0;

uint8_t ucaRxdBuff[10] = {0};
uint8_t ucRxdLen = 0;

uint8_t ucaTxdBuff[10] = {0};
uint8_t ucTxdLen = 0;
uint32_t unRfDisconntCnt = 0;

uint8_t ucSpeedMode = 0;            //0 低速模式    1 高速模式
uint8_t ucDirMode = 0;
int16_t sADValue = 0;

uint8_t ucVdcBusState = 0;
uint8_t ucComErrState = 0;

uint8_t RF2_4G_OnReadyRead(uint8_t len , uint8_t* pbuff);
void RF2_4G_Hanlde(uint8_t len , uint8_t* pbuff);

void RF2_4G_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
    
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
/*

//CS      PB7
//CE      PB6
//MOSI    PB5
//MISO    PB4
//SCLK    PB3
//IRQ     PA15
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;               //sk-1s pb2 pbc pb9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);    
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;               //开启内部上拉和外部上拉一起
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;               //开启内部上拉和外部上拉一起
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  */      
    
    //MOSI 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;              
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);    
    
    //SCLK
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;            
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);   
    
    //CS
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;            
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
    
    //CE
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;            
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
    
    //MISO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;               //开启内部上拉和外部上拉一起
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
    
    //IRQ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;               //开启内部上拉和外部上拉一起
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);   
    
    
    BK2425_Initialize();
    
    {
        uint8_t Rf_Address[10];
        //vBSP_EnterCritical();
        bsp_ReadCpuFlash(ADDR_FLASH_SECTOR_3,Rf_Address,6);
        Set_Pipe0Address(Rf_Address); //配置通道0的地址
        //vBSP_ExitCritical(); 
    }
}

void RF2_4G_Poll(void)
{
    if ( IRQVal() == 0)
    {
        Receive_Packet((void *)RF2_4G_OnReadyRead);
        if( ucRxdLen != 0 )
        {
            RF2_4G_Hanlde(ucRxdLen,ucaRxdBuff);
            ucRxdLen = 0;
        }
    }  
}

void RF2_4G_Poll_10ms(void)
{
    if( ucRfConntState == eRFConnt )
    {
        if(unRfDisconntCnt > 100)
           sADValue = 0;               //断线情况下重置遥控器位置
        if(unRfDisconntCnt > 250)
        {
            ucRfConntState = eRFDisconnt;
        }
        else
        {
            unRfDisconntCnt++;  
            ucResetCnt = 0;            
        }
    }
    else 
    {
        if(++ucResetCnt >= 100)     //断线情况下每1s重置一次遥控器
        {
            ucResetCnt = 0;
            //RF2_4G_Init();
            SwitchToTxMode();
            vBSP_DelayUS(200);
            Send_Packet(WR_TX_PLOAD,ucaTxdBuff, 7);
            Wait_Sender();
            ucTxdLen = 0;
            SwitchToRxMode();
        }
        unRfDisconntCnt = 0;
    }
}

//处理并回发
void RF2_4G_Hanlde(uint8_t len , uint8_t* pbuff)
{
    static uint8_t SuccseeCnt = 0;
    uint8_t checkSum = 0;
    uint8_t i;
    
    //准备应答的数据
    for(i = 0; i<(len-1);i++)
        checkSum += pbuff[i]; 

    
    //接受到地址数据并转发
    if (pbuff[0] == 0xcc && len == 7 && checkSum == pbuff[6] && ucRfWorkState == eRFCalibration)
    {
        ucaTxdBuff[0] = 0xcc;
        
        ucRfConntState = eRFConnt;
        
        if(memcmp(&ucaTxdBuff[1],&pbuff[1],5) == 0)
        {
            //和上一次的结果相同
            if( SuccseeCnt > 5)
                Set_Pipe0Address(&ucaTxdBuff[1]); //配置通道0的地址
            else
                SuccseeCnt++;
        }
        else
        {
            SuccseeCnt = 0;
            memcpy(&ucaTxdBuff[1],&pbuff[1],5);                
        }
        ucaTxdBuff[6] = 0;
        for( i = 0; i < 6; i++)
        {
            ucaTxdBuff[6] += ucaTxdBuff[i];
        } 
        ucTxdLen = 7;
    }
    
    //接受到工作数据 配对完成
    if (pbuff[0] == 0xa5  && len == 5 && checkSum == pbuff[4])
    {
        if( ucRfWorkState == eRFCalibration )
        {
            uint8_t Rf_Address[10];
            ucSystemCalibrationFinishEvent = 1; //发送配对完成事件
            Get_Pipe0Address(Rf_Address);
            vBSP_EnterCritical();
            bsp_WriteCpuFlash(ADDR_FLASH_SECTOR_3,Rf_Address,6);
            vBSP_ExitCritical();
            ucRfWorkState = eRFWork;
            //vBSP_RF2_4Init();
            //遥控器配对完成
        }
        

        ucRfConntState = eRFConnt;

        
        ucSpeedMode = (pbuff[3]&0x01);
        ucDirMode = ((pbuff[3]&0x02)>>1);
        sADValue = ((pbuff[1]&0xff)<<8)|(pbuff[2]&0xff);
        
        //准备应答的数据
            
        ucaTxdBuff[0] = 0xaa;
        ucaTxdBuff[1] = ucVdcBusState;
        ucaTxdBuff[2] = ucComErrState; //0x01控制出错R轴 0x02:温度65(主机没有)  0x04:温度75(主机没有) 0x08:参数错误R轴 
                                       //0x10控制出错L轴 0x20:温度65(从机才有)  0x40:温度75(从机才有) 0x80:参数错误L轴
        
        ucaTxdBuff[3] = 0xcc;
        ucaTxdBuff[4] = 0;
        for( i = 0; i < 4; i++)
        {
            ucaTxdBuff[4] += ucaTxdBuff[i];
        } 
        ucTxdLen = 5;
    }

    if( ucTxdLen != 0)
    {
        SwitchToTxMode();
        vBSP_DelayUS(200);
        Send_Packet(WR_TX_PLOAD,ucaTxdBuff, ucTxdLen);
        Wait_Sender();
        ucTxdLen = 0;
        SwitchToRxMode();
    }
}

uint8_t RF2_4G_OnReadyRead(uint8_t len , uint8_t* pbuff)
{

    unRfDisconntCnt = 0;
    memcpy(ucaRxdBuff,pbuff,len);
    ucRxdLen = len;
    return 0;
}

//开启配对
void RF2_4G_StartCalibration(void)
{
    uint8_t ucaNullBuff[5] = {0};            
    ucRfWorkState = eRFCalibration;
    Set_Pipe0Address(ucaNullBuff);      //地址清0           
}

