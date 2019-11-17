/*
 * MotorDriver.h
 *
 *  Created on: Nov 11, 2019
 *      Author: Kaan
 */

#ifndef SRC_MOTORDRIVER_H_
#define SRC_MOTORDRIVER_H_

#include "MotorDriver_Defs.h"

void MotorDriver_Init(void);
void MotorDriver_CloseRainBox(void);
void MotorDriver_CloseDustBox(void);
void MotorDriver_onDustBoxClosed(void);
void MotorDriver_onRainBoxClosed(void);
void MotorDriver_Stop(void);
MotorDriver_Status_t MotorDriver_GetStatus(void);


#endif /* SRC_MOTORDRIVER_H_ */
