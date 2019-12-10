/*
 * StateMachine.cpp
 *
 *  Created on: Nov 6, 2019
 *      Author: Kaan
 */


#include "StateMachine.h"
#include "StateMachine_Config.h"
#include "main.h"

#include <stddef.h>
#include <stdbool.h>

SystemState _state;
StateDispatch _dispatch;
int userBtnStateGlobal = 0;

void StateMachine_Init(){
	_state.discharcingValve = VALVE_CLOSED;
	_state.samplingBoxValve = VALVE_CLOSED;
	_state.isRaining = false;
	_state.isRainBoxEmpty = true;
	_state.isRainBoxFull = false;
	_state.isSamplingBoxFull = false;
	_state.coverMotor = STOPPED;
	_state.cover = ON_RAIN_BOX;
}

/* Element Mapping
 * Vana1 : Discharging Valve
 * Vana2 : Sampling Box Valve
 * Şamandıra 1 : Rain Box Fulled
 * Şamandıra 2 : Rain Box Emptied
 * Şamandıra 3 : Sampling Box Fulled
 */

void StateMachine_Act( SystemAction action, StateDispatch dispatch ){
	if( dispatch == NULL ){
		return;
	}
	switch(action){
		case RAIN_STARTED:
			if( !_state.isRaining ){
				_state.isRaining = true;
				/* Info 1 : Yağmur sensörü ilk yağmuru algıladığı durumda motor çalışacak
				 * ve kapak toz toplama haznesini kapatacaktır.
				 */
				// Eger kapak toz kutusu uzerinde degilse
				// Turn motor to Dust box
				if( _state.cover != ON_DUST_BOX ){
					_state.coverMotor = TURNING_TO_DUST_BOX;
					dispatch( TURN_COVER_MOTOR_TO_DUST_BOX, &_state );
				}
				 /*
				 * Info 2 : Yagis basladigi anda vana1 ve vana 2 kapali durumda olacak ve Yagmur suyu
				 *  toplama kabina yagmur toplanmaya baslayacaktir.
				 */
				// Close Sampling Valve
				_state.samplingBoxValve = VALVE_CLOSED;
				dispatch( CLOSE_SAMPLING_BOX_VALVE, &_state );
				if( _state.isSamplingBoxFull == false ){
					// Close Discharcing Valve
					_state.discharcingValve = VALVE_CLOSED;
					dispatch( CLOSE_DISCHARGING_VALVE, &_state );
				}else{
					// INFO3: Eger ornek alma haznesi dolu ise, direkt tahliye valfi acilacak.
					// Bu durumda zaten rain box da su dolmasi soz konusu degil...
					// Open Discharcing Valve
					_state.discharcingValve = VALVE_OPEN;
					dispatch( OPEN_DISCHARGING_VALVE, &_state );
				}
			}
			break;
		case RAIN_FINISHED:
			if( _state.isRaining ){
				_state.isRaining = false;

				// yagmur bittiyse ve kapak yagmur kutusu uzerinde degilse
				// kapagi yagmur kutusuna tasiyabiliriz
				if( _state.cover != ON_RAIN_BOX ){
					_state.coverMotor = TURNING_TO_RAIN_BOX;
					dispatch( TURN_COVER_MOTOR_TO_RAIN_BOX, &_state );
				}

				// Ornek alma islemi tamamlandi mi tamamlanmadi mi bakalim..
				if( _state.isSamplingBoxFull ){
					/* Info : Ornek alma islemi tamamlandiktan sonra Yağmur durana kadar vana 1 açık kalacaktır.
					*/  // Ornek alma islemi tamamlanmis, yagmur bitmis. kapatabiliriz
					_state.discharcingValve = VALVE_CLOSED;
					dispatch( CLOSE_DISCHARGING_VALVE, &_state );
				}else{
					// Ornek alma islemi tamamlanmadi ama yagmur bittigi icin haznede dolan suyu, orneklemeye atiyoruz
					_state.samplingBoxValve = VALVE_OPEN;
					dispatch( OPEN_SAMPLING_BOX_VALVE, &_state );
				}
			}
			break;
		case RAIN_BOX_FILLED:
			_state.isRainBoxFull = true;
			/* Info : Kaptaki şamandıra1 aktif olduğunda(kapta 5 litre numune olduğunda)
			 *	Vana 2 açılacak ve kaptaki yağmur suyu altta bulunan polietilen kaba boşalacaktır.
			 */
			_state.samplingBoxValve = VALVE_OPEN;
			dispatch( OPEN_SAMPLING_BOX_VALVE, &_state );
			break;
		case RAIN_BOX_STARTED_DRAINING:
			_state.isRainBoxFull = false;
			break;
		case RAIN_BOX_STARTED_FILLING:
			_state.isRainBoxEmpty = false;
			break;
		case RAIN_BOX_EMPTIED:
			_state.isRainBoxEmpty = true;
			if( !_state.isRaining ){
				_state.samplingBoxValve = VALVE_CLOSED;
				dispatch( CLOSE_SAMPLING_BOX_VALVE, &_state );

				_state.discharcingValve = VALVE_CLOSED;
				dispatch( CLOSE_DISCHARGING_VALVE, &_state );
			}
			break;
		case SAMPLING_BOX_FILLED:
			// Ornek alma islemi tamamlanmis demektir
			_state.isSamplingBoxFull = true;
			// Guvence olsun diye tekrardan ornekleme valfini kapatip bosaltma valfini aciyoruz
			_state.samplingBoxValve = VALVE_CLOSED;
			dispatch( CLOSE_SAMPLING_BOX_VALVE, &_state );

			_state.discharcingValve = VALVE_OPEN;
			dispatch( OPEN_DISCHARGING_VALVE, &_state );
			break;
		case SAMPLING_BOX_STARTED_DRAINING:
			_state.isSamplingBoxFull = false;
			break;
		case RAIN_BOX_COVER_CLOSED:
			_state.coverMotor = STOPPED;
			_state.cover = ON_RAIN_BOX;
			dispatch( STOP_COVER_MOTOR, &_state );
			break;
		case DUST_BOX_COVER_CLOSED:
			_state.coverMotor = STOPPED;
			_state.cover = ON_DUST_BOX;
			dispatch( STOP_COVER_MOTOR, &_state );
			break;
		case DUST_BOX_COVER_OPENED:
		case RAIN_BOX_COVER_OPENED:
			_state.cover = ON_MOVEMENT;
			break;
		default:
			return;
	}
}


