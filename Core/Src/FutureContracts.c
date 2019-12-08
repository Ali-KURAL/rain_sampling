/*
 * FutureContracts.c
 *
 *  Created on: Nov 10, 2019
 *      Author: Kaan
 */

#include "FutureContracts.h"

FutureContract _contracts[FUTURE_CONTRACTS_LENGTH];
uint8_t _contractUsage[FUTURE_CONTRACTS_LENGTH];

uint16_t _lastIndex;
uint16_t _contractCount;

void FutureContracts_Init(void){
	_lastIndex = 0;
	_contractCount = 0;
	for( int k = 0; k < FUTURE_CONTRACTS_LENGTH ; k++ ){
		_contractUsage[k] = 0;
	}
	for( int k = 0; k < FUTURE_CONTRACTS_LENGTH ; k++ ){
		_contracts[k].Timeout = 0;
	}
}
FutureContract_Handle_t FutureContracts_Register( uint16_t timeout, uint16_t recurrence, FutureCallback callback ){
	FutureContract_Handle_t emptyIndex = 0;
	uint8_t slotFound = 0;
	// look for empty slot
	for( int k = 0; k < FUTURE_CONTRACTS_LENGTH; k++ ){
		if( _contractUsage[k] == 0 ){
			emptyIndex = k;
			slotFound = 1;
			break;
		}
	}
	// fail safe
	if( slotFound != 1 ){
		return -1;
	}
	FutureContract* _newContract = &(_contracts[emptyIndex]);
	_newContract->Timeout = timeout;
	_newContract->Recurrence = recurrence;
	_newContract->RemainingTimeout = timeout;
	_newContract->Callback = callback;
	_contractUsage[emptyIndex] = 1;
	return emptyIndex;
}

void FutureContracts_Unregister( FutureContract_Handle_t* handle ){
	if( *handle != -1 && *handle < FUTURE_CONTRACTS_LENGTH ){
		_contractUsage[*handle] = 0;
		_contracts[*handle].Timeout = 0;
		_contracts[*handle].Callback = NULL;
		*handle = -1;
	}
}

void FutureContracts_Work( uint16_t period ){
	for( int k = 0; k < FUTURE_CONTRACTS_LENGTH; k++ ){
		if( _contractUsage[k] ){
			FutureContract *_contract = &(_contracts[k]);
			// Consume on Timeout
			_contract->RemainingTimeout -= period;
			if( _contract->RemainingTimeout <= 0 ){
				// Consume on recurrence
				if( _contract->Recurrence != FUTURE_CONTRACT_INFINITE_RECURRENCE ){
					_contract->Recurrence--;
				}
				// Fire Callback
				if( _contract->Callback != NULL ){
					_contract->Callback();
				}
				// Delete if it is done or refresh RemainingTimeout
				if( _contract->Recurrence == 0 ){
					FutureContracts_Unregister(&k);
				}else{
					_contract->RemainingTimeout = _contract->Timeout;
				}
			}
		}
	}
}
