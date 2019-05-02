#ifndef __MC_EST_HALLR_H__
#define __MC_EST_HALLR_H__
#include "bsp.h"

extern uint16_t usMC_AngleFbR;
extern uint8_t ucMC_HallEva_LowSpeedorEdgedR;//高低速模式标记
extern int16_t usMC_HallEva_IntervalTimeR;

extern int16_t sMC_HallEstSpeed_RPMR;
extern uint8_t ucMC_HallSectorErrR;

//获取角度估算角度模块
extern void vMC_EST_HallAngleR(void);

#endif
