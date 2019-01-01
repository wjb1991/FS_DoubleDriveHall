#include "bsp_bitfilter.h"

void vBSP_BitFilter(uint8_t ucBitVal, bitfilter_t * pBitFilter)
{
    if(ucBitVal)
        pBitFilter->nCnt++;
    else
        pBitFilter->nCnt--;
    
    if( pBitFilter->nCnt > pBitFilter->nHigtLimit )
    {
        pBitFilter->nCnt = pBitFilter->nHigtLimit;
        pBitFilter->nState = 1;
    }
        
    if( pBitFilter->nCnt < pBitFilter->nLowLimit )
    {
        pBitFilter->nCnt = pBitFilter->nLowLimit;
        pBitFilter->nState = 0;
    }   
    
    pBitFilter->nEdgeEvent =  pBitFilter->nState ^ pBitFilter->nLastState;  //异或 相同出0 不同出1 发生反转时为 1
    pBitFilter->nLastState = pBitFilter->nState;
}
