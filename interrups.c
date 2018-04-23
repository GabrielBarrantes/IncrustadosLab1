/*
 * interrups.c
 * contains the implementations of the interrutions
 *
 *  Created on: Apr 9, 2018
 *      Author: gab
 */

#include "msp.h"
#include <msp432p401r.h>
#include "HAL_OPT3001.h"
#include "globals.h"

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

void T32_INT1_IRQHandler(void)
{
    __disable_irq();
    TIMER32_1->INTCLR = 0U;
    ///////////////////////
    P1->OUT ^= BIT0;        //toggl indicator led
    //if( start ){ P2->OUT ^= BIT0; }
    lux = OPT3001_getLux(); //read light sensor value
    //P2->OUT ^= BIT0;

    ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_SC; // read adc
    state2 =1;
    state3=1;
    ///////////////////////
    __enable_irq();
    return;
}

void T32_INT2_IRQHandler(void)
{
    __disable_irq();
    TIMER32_2->INTCLR = 0U;
    ///////////////////////
    P2->OUT ^= BIT4;        //toggl indicator led
    state3=0;
    ///////////////////////
    __enable_irq();
    return;
}

void TA0_0_IRQHandler(void)
{
    __disable_irq();
    //TIMER32_2->INTCLR = 0U;
    ///////////////////////
    //if(state3){ P2->OUT ^= BIT0;}        //toggl indicator led
    //P2->OUT &= !BIT0;
    ///////////////////////
    __enable_irq();
    return;
}



void ADC14_IRQHandler(void)
{
    __disable_irq();
    soundIntensity = ADC14->MEM[0];
    ADC14->CLRIFGR0 = ADC14_CLRIFGR0_CLRIFG0;
    __enable_irq();
    return;
}

#ifdef __cplusplus
}
#endif
