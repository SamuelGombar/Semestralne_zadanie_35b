/*
 * decision.c
 *
 *  Created on: Dec 18, 2024
 *      Author: user
 */
#include "stdint.h"
#include "math.h"

uint8_t decision=0;
//Forward 		- 0
//Strong Left 	- 1
//Left 			- 2
//Backward 		- 3
//Right 		- 4
//Strong Right 	- 5

uint8_t decisionLogic(){
	LL_mDelay(1000);

	decision=decision+1;
	if(decision>5){
		decision=0;
	}

	return decision;
}


