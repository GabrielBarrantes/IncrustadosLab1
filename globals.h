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
#define __SAMPLE_LENGTH       16000
#define __Last_Length         64
//#define multi                 16
#define __frecuencyMultiplier 16

#define __Multiplier_On_Time  2


/*
 * Global variables
 *
 */

//_Bool startState;
//float g_fLux;            //intensity of ambient light
//uint16_t soundIntensity; //intensity of ambient sound
_Bool g_bOutState;
_Bool g_bOnCondition;
_Bool g_bOffCondition;
//_Bool state2;
//_Bool state3;
int g_iInitialUmbral;
//_Bool sampleArray;
int g_iCounter;
float g_fLastMeanSound;
float g_fMeanSound;
float g_fdataArray[__SAMPLE_LENGTH];
//int16_t data_array1[__SAMPLE_LENGTH];
//int16_t data_array2[__SAMPLE_LENGTH];

/*
 * Startup function
 *
 */

void InitialSeptUpParameters();


#endif /* GLOBALS_H_ */
