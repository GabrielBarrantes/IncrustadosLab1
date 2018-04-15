/*
 * interrups.cpp
 * contains the implementations of the interrutions
 *
 *  Created on: Apr 9, 2018
 *      Author: gab
 */

#include "msp.h"
#include <msp432p401r.h>

#ifdef __cplusplus
extern "C"
{
#endif

void PORT1_IRQHandler( void )                                  // Interrupt handler for port 1
{
    if(P1->IFG & BIT1)
    {
    P1->IE &= ~BIT1;                                           // Disable interrupt for S1 (P1.1) for debouncing
    P1->OUT ^= BIT0;                                           // Toggle LED1 (P1.0)
    __delay_cycles(600000);                                    // Delay of 200ms
    P1->IFG &= ~BIT1;                                          // Clear pending interrupt flag for S1 (P1.1)
    P1->IE |= BIT1;                                            // Enable interrupt for S1 (P1.1)
    }
}

#ifdef __cplusplus
}
#endif
