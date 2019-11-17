/*
 * MotorDriver_Defs.h
 *
 *  Created on: Nov 11, 2019
 *      Author: Kaan
 */

#ifndef INC_MOTORDRIVER_DEFS_H_
#define INC_MOTORDRIVER_DEFS_H_

#include <stdint.h>
#include <stddef.h>

#define MOTOR_DRIVER_DIR_PIN_PORT			GPIOB
#define MOTOR_DRIVER_DIR_PIN_NO				GPIO_PIN_7

#define MOTOR_DRIVER_SPEED_PIN_PORT			GPIOB
#define MOTOR_DRIVER_SPEED_PIN_NO			GPIO_PIN_14


typedef enum MotorDriver_Status_t{
	MOTOR_CLOSING_RAIN_BOX = 0,
	MOTOR_COVER_ON_RAIN_BOX = 1,
	MOTOR_CLOSING_DUST_BOX = 2,
	MOTOR_COVER_ON_DUST_BOX = 3,
	MOTOR_STOPPED = 4
}MotorDriver_Status_t;

typedef struct MotorCommands{

}MotorCommands;

#endif /* INC_MOTORDRIVER_DEFS_H_ */
