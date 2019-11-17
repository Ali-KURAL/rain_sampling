/*
 * StateMachine_Defs.h
 *
 *  Created on: Nov 6, 2019
 *      Author: Kaan
 */

#ifndef INC_STATEMACHINE_DEFS_H_
#define INC_STATEMACHINE_DEFS_H_
#include <stdint.h>
#include <stdbool.h>

typedef enum SystemAction {
	// Async system inputs
	RAIN_STARTED,
	RAIN_FINISHED,
	RAIN_BOX_COVER_CLOSED,
	RAIN_BOX_COVER_OPENED,
	DUST_BOX_COVER_CLOSED,
	DUST_BOX_COVER_OPENED,
	SAMPLING_BOX_FILLED,
	SAMPLING_BOX_STARTED_DRAINING,
	RAIN_BOX_FILLED,
	RAIN_BOX_STARTED_DRAINING,
	RAIN_BOX_STARTED_FILLING,
	RAIN_BOX_EMPTIED
}SystemAction;

typedef enum SystemCommand {
	// Commands for valves
	OPEN_SAMPLING_BOX_VALVE,
	CLOSE_SAMPLING_BOX_VALVE,
	OPEN_DISCHARGING_VALVE,
	CLOSE_DISCHARGING_VALVE,

	// Commands for cover motor
	TURN_COVER_MOTOR_TO_DUST_BOX,
	TURN_COVER_MOTOR_TO_RAIN_BOX,
	STOP_COVER_MOTOR
}SystemCommand;

typedef enum ValveState {
	VALVE_OPEN,
	VALVE_CLOSED
}ValveState;

typedef enum MotorState {
	TURNING_TO_DUST_BOX,
	TURNING_TO_RAIN_BOX,
	STOPPED
}MotorState;

typedef enum CoverState{
	ON_RAIN_BOX,
	ON_DUST_BOX,
	ON_MOVEMENT
}CoverState;



typedef struct SystemState {
	bool isRaining;
	ValveState samplingBoxValve;
	ValveState discharcingValve;
	MotorState coverMotor;
	CoverState cover;
	bool isRainBoxFull;
	bool isRainBoxEmpty;
	bool isSamplingBoxFull;
}SystemState;

typedef void(*StateDispatch)(SystemCommand action,const SystemState* state);

#endif /* INC_STATEMACHINE_DEFS_H_ */
