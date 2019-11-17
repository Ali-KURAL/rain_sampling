/*
 * ValveDriver.c
 *
 *  Created on: Nov 15, 2019
 *      Author: Kaan
 */


#include "ValveDriver.h"
#include "stm32f7xx_hal.h"

void ValveDriver_OpenDischarcingValve(){
	HAL_GPIO_WritePin( VALVE_DISCHARGING_PIN_PORT, VALVE_DISCHARGING_PIN_NO, GPIO_PIN_SET );
}

void ValveDriver_CloseDischarcingValve(){
	HAL_GPIO_WritePin( VALVE_DISCHARGING_PIN_PORT, VALVE_DISCHARGING_PIN_NO, GPIO_PIN_RESET );
}

void ValveDriver_OpenSamplingBoxValve(){
	HAL_GPIO_WritePin( VALVE_SAMPLING_BOX_PIN_PORT, VALVE_SAMPLING_BOX_PIN_NO, GPIO_PIN_SET );
}

void ValveDriver_CloseSamplingBoxValve(){
	HAL_GPIO_WritePin( VALVE_SAMPLING_BOX_PIN_PORT, VALVE_SAMPLING_BOX_PIN_NO, GPIO_PIN_RESET );
}
