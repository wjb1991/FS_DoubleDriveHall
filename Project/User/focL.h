#ifndef __FOC_H__
#define __FOC_H__
#include "bsp.h"

/**/
#define DEF_MOTOR_L  TIM1
#define DEF_MOTOR_R  TIM8

//extern int16_t	usFOC_AlphaOut;
//extern int16_t	usFOC_BetaOut;
extern int16_t	sFOC_QOutL;
extern int16_t	sFOC_DOutL;

extern int16_t sFOC_CurrUL;
extern int16_t sFOC_CurrVL;
//extern int16_t	usFOC_CurrAlpha;
//extern int16_t	usFOC_CurrBeta;
//extern int16_t	usFOC_CurrD;
//extern int16_t	usFOC_CurrQ;
//extern int16_t	usFOC_DCurrPIDInteger;

void vFOC_Clark(void);
void vFOC_Park(void);
void vFOC_IPark(void); 
void vFOC_SVPWM(void);
void vFOC_MotorLock(void);
void vFOC_DCurrentControl(void);

#endif
