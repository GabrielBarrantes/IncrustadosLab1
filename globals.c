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
    g_iCounter = 0;
    g_fMeanSound = 0;
    g_fLastMeanSound=0;
    g_iNumberOfLights=__Number_Of_Lights;
}

void TurnOffLight()
{
    P1->OUT &= ~BIT0;
    P2->OUT &= ~BIT0;
    P5->OUT &= ~BIT6;
}

void TurnOnLight()
{
    P1->OUT |= BIT0;
    if (g_iNumberOfLights>=2){ P2->OUT |= BIT0; }
    if (g_iNumberOfLights==3){ P5->OUT |= BIT6; }

}
