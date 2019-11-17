/*
 * FutureContracts.h
 *
 *  Created on: Nov 10, 2019
 *      Author: Kaan
 */

#ifndef INC_FUTURECONTRACTS_H_
#define INC_FUTURECONTRACTS_H_

#include "FutureContracts_Defs.h"

void FutureContracts_Init(void);
FutureContract_Handle_t FutureContracts_Register( uint16_t timeout, uint16_t recurrence, FutureCallback callback );
void FutureContracts_Unregister( FutureContract_Handle_t* handle );
void FutureContracts_Work( uint16_t period );

#endif /* INC_FUTURECONTRACTS_H_ */
