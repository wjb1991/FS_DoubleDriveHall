#ifndef __MC_EST_HALLL_H__
#define __MC_EST_HALLL_H__
#include "bsp.h"

extern uint16_t usMC_AngleFbL;
extern uint8_t ucMC_HallEva_LowSpeedorEdgedL;//高低速模式标记
extern int16_t usMC_HallEva_IntervalTimeL;

extern int16_t sMC_HallEstSpeed_RPML;
extern uint8_t ucMC_HallSectorErrL;

//获取角度估算角度模块
extern void vMC_EST_HallAngleL(void);

#endif
