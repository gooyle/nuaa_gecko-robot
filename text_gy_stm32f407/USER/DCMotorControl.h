#ifndef _DCMOTORCONTROL_H
#define _DCMOTORCONTROL_H
#include "sys.h"
#define PDIR 1
#define NDIR 0

void DCMotorControl(uint16_t Mpwm,uint8_t Dir,uint8_t Enable);
#endif
