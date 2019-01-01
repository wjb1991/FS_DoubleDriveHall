#ifndef __RF_2_4G_H__
#define __RF_2_4G_H__
#include "bsp.h"

typedef enum { eRFDisconnt,eRFConnt} rfconntstate_t;
typedef enum { eRFInit,eRFWork,eRFCalibration } rfworkstate_t;


extern uint8_t ucSystemCalibrationFinishEvent;
extern rfworkstate_t ucRfWorkState;
extern rfconntstate_t ucRfConntState;

extern uint8_t ucSpeedMode;
extern uint8_t ucDirMode;
extern int16_t sADValue;

extern uint8_t ucVdcBusState;
extern uint8_t ucComErrState;

void RF2_4G_Init(void);
void RF2_4G_Poll(void);
void RF2_4G_Poll_10ms(void);

void RF2_4G_StartCalibration(void);

#endif
