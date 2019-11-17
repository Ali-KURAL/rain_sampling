#ifndef EVENT_GENERATOR_H
#define EVENT_GENERATOR_H

#include <stdint.h>
#include <stddef.h>
#include "stm32f7xx_hal.h"

typedef void(*TransitionCallback)( uint8_t newState);

typedef int16_t EventSystemHandler_t;


typedef struct DigitalInputState{
	uint16_t Period;
	uint8_t LastState;
	uint8_t Counter;
	uint8_t Threshold;
	uint16_t PeriodCounter;
	GPIO_TypeDef *Port;
	uint16_t Pin;
	TransitionCallback Callback;
	uint8_t Changed;
}DigitalInputState;

EventSystemHandler_t EventGenerator_AddInput(
	uint16_t period,
	uint8_t threshold,
	uint8_t initialState,
	GPIO_TypeDef *port,
	uint16_t pin,
	TransitionCallback transitionCallback
);

void EventGenerator_ReadInputs( uint8_t readPeriod );




#endif // EVENT_GENERATOR_H
