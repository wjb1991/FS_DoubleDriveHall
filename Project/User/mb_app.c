#include "mb_app.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START                 ( 1000 )
#define REG_INPUT_NREGS                 ( 64 )

#define REG_HOLDING_START               ( 1 )
#define REG_HOLDING_NREGS               ( 400 )

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];
static USHORT   usRegHoldingStart = REG_HOLDING_START;
USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
const  UCHAR    ucSlaveID[] = { 0xAA, 0xBB, 0xCC }; //从机信息？
/* ----------------------- Start implementation -----------------------------*/
adc_pack_t* pAdcPack[4] = { (void*)&usRegHoldingBuf[0], (void*)&usRegHoldingBuf[100],
                            (void*)&usRegHoldingBuf[200],(void*)&usRegHoldingBuf[200]};
io_park_t* pIoPack[4]  =  { (void*)&usRegHoldingBuf[0+50], (void*)&usRegHoldingBuf[100+50],
                            (void*)&usRegHoldingBuf[200+50],(void*)&usRegHoldingBuf[200+50]};

int32_t nMB_APP_Init(void);
void vMB_APP_Poll(void);
                            
int32_t nMB_APP_Init(void)
{
    memset(usRegHoldingBuf,0,REG_HOLDING_NREGS);
    
    if( MB_ENOERR != eMBInit( MB_RTU, 0x01, 1, 115200, MB_PAR_EVEN ) )
    {
        (void)eMBDisable();
        (void)eMBClose();  
        return -1UL;        
    }

    if( MB_ENOERR != eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3 ) )
    {
        (void)eMBDisable();
        (void)eMBClose();  
        return -2UL;        
    }
    
    if( MB_ENOERR != eMBEnable())
    {
        (void)eMBDisable();
        (void)eMBClose();  
        return -3UL;        
    }
    
    return 0;
}
                            
                            
void vMB_APP_Poll(void)
{
    (void)eMBPoll();   
}

eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}
