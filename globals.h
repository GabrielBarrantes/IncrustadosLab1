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

#define __initialUmbral       600
#define __dayIntensityUmbral  900
#define __darkIntensityUmbral 300
#define __waitTimeforToggl    10
#define __SAMPLE_LENGTH       16000 //guardamos muestras de los ultimos dos segundos
#define __Last_Length         4000 //64 funciona bien, 4000 toma el promedio final de medio segundo
#define __frecuencyMultiplier 16

#define __Multiplier_On_Time  4
#define __Number_Of_Lights    1


/*
 * Global variables
 *
 */

_Bool g_bOutState;
_Bool g_bOnCondition;
_Bool g_bOffCondition;
int   g_iInitialUmbral;
int   g_iCounter;
float g_fLastMeanSound;
float g_fMeanSound;
float g_fdataArray[__SAMPLE_LENGTH];
int   g_iNumberOfLights;

/*
 * general functions
 *
 */

void InitialSeptUpParameters();
void TurnOnLight();
void TurnOffLight();


#endif /* GLOBALS_H_ */
