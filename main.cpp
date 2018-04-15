#include "msp.h"
#include <msp432p401r.h>
#include <stdint.h>
#include <stdio.h>
#include "HAL_I2C.h"
#include "HAL_OPT3001.h"
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "HAL_I2C.h"
#include "HAL_OPT3001.h"

// LED1      @ P1.0
// Button S1 @ P1.1


void PORT1_IRQHandler();

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

    //Setup for optical sensor
    //OPT3001_init();
    float lux;
    bool state;


    for (;;)                                                   //Infinite Loop
    {
        //lux = OPT3001_getLux();
        //if (lux>300 && state==0){ PORT1_IRQHandler(); state=1; }
        //else if (lux<200 && state==1){ PORT1_IRQHandler(); state=0; }

    }
}
