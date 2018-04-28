/*
 * globals.c
 *
 *  Created on: Apr 23, 2018
 *      Author: gab
 */

#include <stdio.h>
#include "msp.h"
#include "globals.h"

void initialSeptUpParameters()
{
    initialUmbral = __initialUmbral;
    onCondition = 0;
    offCondition = 0;
    sampleArray =0;
    counter = 0;
    meanSound = 0;

    int i;
    //for( i=0 ; i < __SAMPLE_LENGTH; i++){ data_array[i]=0; }
}

