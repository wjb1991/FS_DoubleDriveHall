#ifndef __FOC_H__
#define __FOC_H__
#include "bsp.h"

/**/
#define DEF_MOTOR_L  TIM1
#define DEF_MOTOR_R  TIM8


extern int16_t	sFOC_QOutL;
extern int16_t	sFOC_DOutL;

extern int16_t sFOC_CurrUL;
extern int16_t sFOC_CurrVL;


extern int16_t	sFOC_QOutR;
extern int16_t	sFOC_DOutR;

extern int16_t sFOC_CurrUR;
extern int16_t sFOC_CurrVR;

void vFOC_ClarkL(void);
void vFOC_ParkL(void);
void vFOC_IParkL(void); 
void vFOC_SVPWML(void);
void vFOC_MotorLockL(void);
void vFOC_DCurrentControlL(void);


void vFOC_ClarkR(void);
void vFOC_ParkR(void);
void vFOC_IParkR(void); 
void vFOC_SVPWMR(void);
void vFOC_MotorLockR(void);
void vFOC_DCurrentControlR(void);


#endif
