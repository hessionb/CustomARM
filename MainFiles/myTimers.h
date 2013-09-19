#ifndef _MY_TIMERS_H
#define _MY_TIMERS_H
#include "lcdTask.h"
#include "i2cVolt.h"
void startTimerForLCD(vtLCDStruct *vtLCDdata);
void startTimerForVoltage(vtVoltStruct *vtVoltdata);
#endif