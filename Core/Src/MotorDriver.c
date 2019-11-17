/*
 * MotorDriver.c
 *
 *  Created on: Nov 11, 2019
 *      Author: Kaan
 */

#include "MotorDriver.h"
#include "stm32f7xx_hal.h"

MotorDriver_Status_t _motorStatus;

// public
void MotorDriver_Init(void){
	_motorStatus = MOTOR_STOPPED;
	HAL_GPIO_WritePin(MOTOR_DRIVER_SPEED_PIN_PORT, MOTOR_DRIVER_SPEED_PIN_NO, GPIO_PIN_RESET );
}


void MotorDriver_CloseRainBox(void){
	_motorStatus = MOTOR_CLOSING_RAIN_BOX;
	HAL_GPIO_WritePin( MOTOR_DRIVER_DIR_PIN_PORT, MOTOR_DRIVER_DIR_PIN_NO, GPIO_PIN_SET );
	HAL_GPIO_WritePin( MOTOR_DRIVER_SPEED_PIN_PORT, MOTOR_DRIVER_SPEED_PIN_NO, GPIO_PIN_SET );
}

void MotorDriver_CloseDustBox(void){
	_motorStatus = MOTOR_CLOSING_DUST_BOX;
	HAL_GPIO_WritePin( MOTOR_DRIVER_DIR_PIN_PORT, MOTOR_DRIVER_DIR_PIN_NO, GPIO_PIN_RESET );
	HAL_GPIO_WritePin( MOTOR_DRIVER_SPEED_PIN_PORT, MOTOR_DRIVER_SPEED_PIN_NO, GPIO_PIN_SET );
}

void MotorDriver_onRainBoxClosed(void){
	if ( _motorStatus == MOTOR_CLOSING_RAIN_BOX ){
		_motorStatus = MOTOR_COVER_ON_DUST_BOX;
		HAL_GPIO_WritePin( MOTOR_DRIVER_SPEED_PIN_PORT, MOTOR_DRIVER_SPEED_PIN_NO, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( MOTOR_DRIVER_DIR_PIN_PORT, MOTOR_DRIVER_DIR_PIN_NO, GPIO_PIN_RESET );
	}
}

void MotorDriver_onDustBoxClosed(void){
	if( _motorStatus == MOTOR_CLOSING_DUST_BOX ){
		_motorStatus = MOTOR_COVER_ON_DUST_BOX;
		HAL_GPIO_WritePin( MOTOR_DRIVER_SPEED_PIN_PORT, MOTOR_DRIVER_SPEED_PIN_NO, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( MOTOR_DRIVER_DIR_PIN_PORT, MOTOR_DRIVER_DIR_PIN_NO, GPIO_PIN_RESET );
	}
}

void MotorDriver_Stop(void){
	_motorStatus = MOTOR_STOPPED;
	HAL_GPIO_WritePin(MOTOR_DRIVER_SPEED_PIN_PORT, MOTOR_DRIVER_SPEED_PIN_NO, GPIO_PIN_RESET );
	HAL_GPIO_WritePin( MOTOR_DRIVER_DIR_PIN_PORT, MOTOR_DRIVER_DIR_PIN_NO, GPIO_PIN_RESET );
}

MotorDriver_Status_t MotorDriver_GetStatus(void){
	return _motorStatus;
}

// private



