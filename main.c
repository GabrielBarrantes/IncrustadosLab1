#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "HAL_I2C.h"
#include "HAL_OPT3001.h"
//#include "LcdDriver/Crystalfontz128x128_ST7735.h"
//#include "LcdDriver/HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include "globals.h"

void main( void )
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;                // stop watchdog timer

    P2->REN |= BIT0;   //Red led on RGB led, launchpad
    P2->DIR |= BIT0;
    P2->OUT &= BIT0;

    P5->REN |= BIT6;
    P5->DIR |= BIT6;   //BlueLed on RGB led, BoosterPack
    P5->OUT |= BIT6;

    P2->REN |= BIT4;
    P2->DIR |= BIT4;   //GreenLed on RGB led, BoosterPack
    P2->OUT |= BIT4;

    //P1->SEL0 = 0x00;                                         // Clear selection register 0 for port 1
    //P1->SEL1 = 0x00;                                         // Clear selection register 1 for port 1

    P1->REN  = BIT1;                                           // Enable resistor for S1 (P1.1)
    P1->OUT  = BIT1;                                           // S1 (P1.1) high -> pull-up resistor, others low
    P1->DIR  = BIT0;                                           // LED1 (P1.0) to output direction, others input
    ///////////////////////////////
    P1->IES  = BIT1;                                           // Interrupt on high->low transition for S1 (P1.1)
    P1->IFG  = 0x00;                                           // Clear any pending flags
    P1->IE   = BIT1;                                           // Enable interrupt for S1 (P1.1)
    ///////////////////////////////
    NVIC_SetPriority(PORT1_IRQn,1);                            //Set interrup of boton priority
    NVIC_EnableIRQ(PORT1_IRQn);                                //Enable the buton interrup
///////////////////////////////////////////////////////////////////////////
    // Set P4.3 for Analog input, disabling the I/O circuit.
    P4->SEL0 = BIT3;
    P4->SEL1 = BIT3;
    P4->DIR &= ~BIT3;

    ADC14->CTL0 = ADC14_CTL0_PDIV_0 | ADC14_CTL0_SHS_0 | ADC14_CTL0_DIV_7 |
                  ADC14_CTL0_SSEL__MCLK | ADC14_CTL0_SHT0_1 | ADC14_CTL0_ON
                  | ADC14_CTL0_SHP;
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_10 | ADC14_MCTLN_VRSEL_0;
    ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_ENC;
    ADC14->IER0 = ADC14_IER0_IE0;
    NVIC_SetPriority(ADC14_IRQn,1);
    NVIC_EnableIRQ(ADC14_IRQn);
//////////////////////////////////////////////////////////////////////////
    // Configuracion del timer de muestreo de luz (tambien se puede usar con el sonido)
    // Cada periodo de tiempo se llama a la interrupcion T32_INT1_IRQHandler(void)
    TIMER32_1->LOAD =  0x0002DC6C0; //~1s ---> a 3Mhz
    TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
    NVIC_SetPriority(T32_INT1_IRQn,1);
    NVIC_EnableIRQ(T32_INT1_IRQn);
    // Configuracion del timer de muestreo de luz (tambien se puede usar con el sonido)
    // Cada periodo de tiempo se llama a la interrupcion T32_INT1_IRQHandler(void)
    TIMER32_2->LOAD = 2* 0x0002DC6C0; //~1s ---> a 3Mhz
    TIMER32_2->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
    //TIMER32_2->
    NVIC_SetPriority(T32_INT2_IRQn,1);
    NVIC_EnableIRQ(T32_INT2_IRQn);
    //
    NVIC_SetPriority(TA0_0_IRQn,1);
    NVIC_EnableIRQ(TA0_0_IRQn);
    //
    //TIMER_A0->R =  0x0000FFFF;
    //uint16_t valores[5]; valores[0]=0xFFFF; valores[1]=0xFFFF;
    TIMER_A0->R = 0x0;
    //TIMER_A0->CTL =
    TIMER_A0->CCR[0] =0xFFFF;

    TA0CTL = TASSEL_2 | TAIE | MC_1;  // enable timerA0 interrupt, select MC up mode, Select SMCLK as timerclock

    TA0CCTL0 = CCIE | CCIS_0 | OUTMOD_3;

    TA0CCTL0 &= ~CAP;

    TA0CCR1 = 0xFFFF;

    TA0CCR0 = 0xFFFF;

    TA0R = 0;

    TA0IV = 0x02;



    NVIC_SetPriority(TA0_0_IRQn,1);
    NVIC_EnableIRQ(TA0_0_IRQn);


    ////
    //Setup for optical sensor
    Init_I2C_GPIO();
    I2C_init();
    OPT3001_init();
    bool state = 1;

    ////////////////////
    // Secuence start //
    ////////////////////
    P2->OUT |= BIT0;   //on the light
    __delay_cycles(3000000);
    P2->OUT = 0x0;  //off the light
    __delay_cycles(3000000);
    P2->OUT |= BIT0;   //on the light
    __delay_cycles(3000000);
    P2->OUT &= !BIT0;   //off the light
    __delay_cycles(3000000);
    P2->OUT |= BIT0;   //on the light
    __delay_cycles(3000000);
    P2->OUT &= !BIT0;   //off the light




    __delay_cycles(600000);

    for (;;)                                                   //Infinite Loop
    {
        if (lux>600 && state){ P2->OUT ^= BIT0; state =0; }
        else if (lux<200 && !state){ P2->OUT ^= BIT0; state=1; }

        if( soundIntensity>10000U && state2 )
        {
            //P2->OUT ^= BIT0;
            P5->OUT ^= BIT6;
            state2 = 0;
        }
    }
}


/*

#include "msp.h"

uint16_t ADC14Result = 0U;
_Bool state = 1;
int main(void)
{
    //uint32_t csregister = 0x0;

    WDTCTL = WDTPW | WDTHOLD;                    // Stop watchdog timer //

    P1->REN |= BIT0;
    P1->DIR |= BIT0;
    P1->OUT |= BIT0;

    //
    //
    P2->REN |= BIT0;   //Red led, placa abajo
    P2->DIR |= BIT0;
    P2->OUT &= !BIT0;

    P5->REN |= BIT6;
    P5->DIR |= BIT6; //BlueLed
    P5->OUT |= BIT6;
    //P2->REN |= BIT4;
    //P2->DIR |= BIT4; //GreenLed
    //P2->OUT |= BIT4;
    //P2->REN |= BIT6;
    //P2->DIR |= BIT6; //RedLed *** no funciona ***
    //P2->OUT |= BIT6;


    //
    //

    // Set P4.3 for Analog input, disabling the I/O circuit.
    P4->SEL0 = BIT3;
    P4->SEL1 = BIT3;
    P4->DIR &= ~BIT3;

    //TIMER32_1->LOAD = 0x00B71B00; //~0.5s ---> a 48Mhz
    TIMER32_1->LOAD =  0x0002DC6C0; //~1s ---> a 3Mhz
    TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
    NVIC_SetPriority(T32_INT1_IRQn,1);
    NVIC_EnableIRQ(T32_INT1_IRQn);

    ADC14->CTL0 = ADC14_CTL0_PDIV_0 | ADC14_CTL0_SHS_0 | ADC14_CTL0_DIV_7 |
                  ADC14_CTL0_SSEL__MCLK | ADC14_CTL0_SHT0_1 | ADC14_CTL0_ON
                  | ADC14_CTL0_SHP;
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_10 | ADC14_MCTLN_VRSEL_0;
    ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_ENC;
    ADC14->IER0 = ADC14_IER0_IE0;
    NVIC_SetPriority(ADC14_IRQn,1);
    NVIC_EnableIRQ(ADC14_IRQn);
    while (1)
    {
        //16383
        if( ADC14Result>10000 && state )
        {
            P2->OUT ^= BIT0;
            P5->OUT ^= BIT6;
            state = 0;
        }
    }

    return 0;
}

void T32_INT1_IRQHandler(void)
{
    __disable_irq();

    state =1;


    TIMER32_1->INTCLR = 0U;
    P1->OUT ^= BIT0;
    ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_SC; // Start
    __enable_irq();
    return;
}

void ADC14_IRQHandler(void)
{
    __disable_irq();
    ADC14Result = ADC14->MEM[0];
    ADC14->CLRIFGR0 = ADC14_CLRIFGR0_CLRIFG0;
    __enable_irq();
    return;
}

*/
