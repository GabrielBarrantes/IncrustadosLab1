#include "msp.h"
#include <msp432p401r.h>
#include <stdint.h>

// LED1      @ P1.0
// Button S1 @ P1.1

void main( void )
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;                // stop watchdog timer

    P2->DIR = BIT0;                                            // on the red led for indicator
    P2->OUT = BIT0 | BIT1 | BIT2;

    //P1->SEL0 = 0x00;                                         // Clear selection register 0 for port 1
    //P1->SEL1 = 0x00;                                         // Clear selection register 1 for port 1

    P1->REN  = BIT1;                                           // Enable resistor for S1 (P1.1)
    P1->OUT  = BIT1;                                           // S1 (P1.1) high -> pull-up resistor, others low
    P1->DIR  = BIT0;                                           // LED1 (P1.0) to output direction, others input

    P1->IES  = BIT1;                                           // Interrupt on high->low transition for S1 (P1.1)
    P1->IFG  = 0x00;                                           // Clear any pending flags
    P1->IE   = BIT1;                                           // Enable interrupt for S1 (P1.1)

    NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);
    __enable_interrupt();                                      // Enable global interrupts


    P2->DIR = BIT0;
    P2->OUT = BIT0 | BIT1 | BIT2;

    for (;;)                                                   //Infinite Loop
    {

    }
}
/*
 * Implementation of Interruption functions
 */
void PORT1_IRQHandler( void )                                // Interrupt handler for port 1
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

