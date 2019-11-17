#include "EventGenerator.h"
#include "StateMachine.h"

#define DIGITAL_INPUTS_SIZE 		16

DigitalInputState _inputs[DIGITAL_INPUTS_SIZE];
uint8_t _inputsCount;

EventSystemHandler_t EventGenerator_AddInput(
	uint16_t period,
	uint8_t threshold, 
	uint8_t initialState,
	GPIO_TypeDef *port,
	uint16_t pin,
	TransitionCallback transitionCallback )
{
	if( _inputsCount == DIGITAL_INPUTS_SIZE ){
		return -1;
	}
	EventSystemHandler_t returnValue = _inputsCount;
	DigitalInputState* _newInput = &(_inputs[_inputsCount]);
	_newInput->Period = period;
	_newInput->LastState = initialState;
	_newInput->Threshold = threshold;
	_newInput->Port = port;
	_newInput->Pin = pin;
	_newInput->Counter = 0;
	_newInput->PeriodCounter = period;
	_newInput->Callback = transitionCallback;
	_newInput->Changed = 0;
	_inputsCount++;
	return returnValue;
}	

void EventGenerator_ReadInputs( uint8_t readPeriod ){
	DigitalInputState* _currentInput = NULL;
	for(int k = 0; k < _inputsCount; k++ ){
		_currentInput = &(_inputs[k]);
		if( _currentInput == NULL ){
			continue; // assert here
		}
		_currentInput->PeriodCounter -= readPeriod;
		if( _currentInput->PeriodCounter <= 0 )
		{
			_currentInput->PeriodCounter = _currentInput->Period;
			GPIO_PinState _state = HAL_GPIO_ReadPin(_currentInput->Port, _currentInput->Pin );
			if( _state != _currentInput->LastState ){
				_currentInput->Counter = 0;
				if( _currentInput->Changed ){
					_currentInput->Changed = 0;
				}else{
					_currentInput->Changed = 1;
				}
				_currentInput->LastState = _state;
			}
			else{
				if( _currentInput->Changed )
					_currentInput->Counter++;
			}
			if( _currentInput->Counter >= _currentInput->Threshold ){
				_currentInput->Changed = 0;
				_currentInput->Counter = 0;
				if ( _currentInput->Callback != NULL ){
					_currentInput->Callback(_state);
				}
			}
		}
	}
}
