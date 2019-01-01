#ifndef __MB_APP_H__
#define __MB_APP_H__

#include "bsp.h"

typedef struct {
    uint32_t    unStartTiming;     //开始转换的时间
    uint32_t    unEndTiming;       //结束转换的时间
    uint32_t    unConversionCont;  //转换次数计数 每次从平衡切换到检测时清零?
    uint32_t    unConversionResul; //ADC转换结果
}adc_pack_t;

//后面版start(i) work(o) 
//控制盒startup(i) success(o) failed(o) close(o)
typedef struct {
    uint16_t    usBoxInReg;            //b0 startup                         
    uint16_t    usBoxOutReg;           //b0 success b1 failed  b2 close  
    uint16_t    usPanelInReg;          //b0 start                         
    uint16_t    usPanelOutReg;         //b0 work
}io_park_t;


extern adc_pack_t* pAdcPack[4];
extern io_park_t* pIoPack[4];

int32_t nMB_APP_Init(void);
void vMB_APP_Poll(void);

__INLINE uint32_t unMB_Write32BIT(uint32_t unData)
{

    uint16_t tmp[2];
    tmp[0] = unData >> 16;
    tmp[1] = unData & 0x0000ffff;
    return (*((uint32_t *)tmp));

/** return (__REV(unData));*/
}


#include "mb.h"
#include "mbport.h"
extern USHORT   usRegHoldingBuf[400];


#endif
