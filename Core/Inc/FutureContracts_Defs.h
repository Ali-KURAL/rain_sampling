/*
 * FutureContract_Defs.h
 *
 *  Created on: Nov 10, 2019
 *      Author: Kaan
 */

#ifndef INC_FUTURECONTRACTS_DEFS_H_
#define INC_FUTURECONTRACTS_DEFS_H_

#include <stdint.h>
#include <stddef.h>

#define FUTURE_CONTRACTS_LENGTH 	16
#define FUTURE_CONTRACT_INFINITE_RECURRENCE  9999

typedef int16_t FutureContract_Handle_t;
typedef void(*FutureCallback)(void);

typedef struct FutureContract{
	uint16_t Timeout;
	uint16_t RemainingTimeout;
	uint16_t Recurrence;
	FutureCallback Callback;
}FutureContract;

#endif /* INC_FUTURECONTRACTS_DEFS_H_ */
