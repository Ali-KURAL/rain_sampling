/*
 * StateMachine.hpp
 *
 *  Created on: Nov 6, 2019
 *      Author: Kaan
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_

#include "StateMachine_Defs.h"

void StateMachine_Init();
void StateMachine_Act(SystemAction action, StateDispatch dispatch);
void StateMachine_ReadInputs(void);



#endif /* INC_STATEMACHINE_H_ */
