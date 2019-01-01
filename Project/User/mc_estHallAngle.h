#ifndef __MC_EST_HALL_H__
#define __MC_EST_HALL_H__
#include "bsp.h"

extern uint16_t usMC_AngleFb;
extern uint8_t ucMC_HallEva_LowSpeedorEdged;//高低速模式标记
extern int16_t usMC_HallEva_IntervalTime;

extern int16_t sMC_HallEstSpeed_RPM;
extern uint8_t ucMC_HallSectorErr;

//获取角度估算角度模块
extern void vMC_EST_HallAngle(void);

#endif
