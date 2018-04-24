/*
 * globals.h
 *
 *  Created on: Apr 21, 2018
 *      Author: gab
 */

#include <stdio.h>
#include "msp.h"

#ifndef GLOBALS_H_
#define GLOBALS_H_

/*
 * Preprocesator parameters
 */

#define __initialUmbral      600
#define __dayIntensityUmbral 900
#define __darkIntensityUmbral 300
#define __waitTimeforToggl 10


/*
 * Global variables
 *
 */

_Bool startState;
float lux;               //intensity of ambient light
uint16_t soundIntensity; //intensity of ambient sound
_Bool outState;
_Bool onCondition;
_Bool offCondition;
_Bool state2;
_Bool state3;
int initialUmbral;

/*
 * Startup function
 *
 */

void initialSeptUpParameters();


#endif /* GLOBALS_H_ */
