#ifndef __FOC_H__
#define __FOC_H__
#include "bsp.h"

extern int16_t	usFOC_AlphaOut;
extern int16_t	usFOC_BetaOut;
extern int16_t	sFOC_QOut;
extern int16_t	usFOC_DOut;

extern int16_t usFOC_CurrU;
extern int16_t usFOC_CurrV;
extern int16_t	usFOC_CurrAlpha;
extern int16_t	usFOC_CurrBeta;
extern int16_t	usFOC_CurrD;
extern int16_t	usFOC_CurrQ;
extern int16_t	usFOC_DCurrPIDInteger;

void vFOC_Clark(void);
void vFOC_Park(void);
void vFOC_IPark(void); 
void vFOC_SVPWM(void);
void vFOC_MotorLock(void);
void vFOC_DCurrentControl(void);

#endif
