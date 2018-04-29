/*
 * globals.c
 *
 *  Created on: Apr 23, 2018
 *      Author: gab
 */

#include <stdio.h>
#include "msp.h"
#include "globals.h"

void InitialSeptUpParameters()
{
    g_iInitialUmbral = __initialUmbral;
    g_bOnCondition = 0;
    g_bOffCondition = 0;
    //sampleArray =0;
    g_iCounter = 0;
    g_fMeanSound = 0;
    g_fLastMeanSound=0;
    //int i;
    //for( i=0 ; i < __SAMPLE_LENGTH; i++){ data_array[i]=0; }
}

