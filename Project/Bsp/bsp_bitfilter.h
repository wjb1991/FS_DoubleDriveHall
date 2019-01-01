#ifndef __BSP_BITFILER_H__
#define __BSP_BITFILER_H__
#include "bsp.h"

//20170902      V1.0 新添加1位滤波模块 主要用于IO输入的滤波 支持边沿时间捕捉

typedef struct {
    int nCnt;
    int nHigtLimit;
    int nLowLimit;
    int nLastState;
    int nState;
    int nEdgeEvent;
}bitfilter_t;

#define DEF_DEFAULT_BITFILTER   {0,16,-16,0,0,0}

void vBSP_BitFilter(uint8_t ucBitVal, bitfilter_t * pBitFilter);


#endif
