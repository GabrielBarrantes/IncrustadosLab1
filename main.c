#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "HAL_I2C.h"
#include "HAL_OPT3001.h"
#include <stdio.h>
#include "globals.h"




void main( void )
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;                // stop watchdog timer

    P2->REN |= BIT0;   //Red led on RGB led, launchpad
    P2->DIR |= BIT0;//P2->OUT |= BIT0;
    P2->OUT &= ~BIT0;

    P5->REN |= BIT6;
    P5->DIR |= BIT6;   //BlueLed on RGB led, BoosterPack
    //P5->OUT |= BIT6;
    P5->OUT &= ~BIT6;

    P2->REN |= BIT4;
    P2->DIR |= BIT4;   //GreenLed on RGB led, BoosterPack
    P2->OUT |= BIT4;
    P2->OUT &= ~BIT4;

    //P1->SEL0 = 0x00;                                         // Clear selection register 0 for port 1
    //P1->SEL1 = 0x00;                                         // Clear selection register 1 for port 1
    //////////////////Buton interrup
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
    //////////////////Buton interrup
    P3->REN  |= BIT5;                                           // Enable resistor for S1 (P1.1)
    P3->OUT  |= BIT5;                                           // S1 (P1.1) high -> pull-up resistor, others low
    P3->DIR  &= ~BIT5;                                           // LED1 (P1.0) to output direction, others input
    ///////////////////////////////
    P3->IES  |= BIT5;                                           // Interrupt on high->low transition for S1 (P1.1)
    P3->IFG  = 0x00;                                           // Clear any pending flags
    P3->IE   |= BIT5;                                           // Enable interrupt for S1 (P1.1)
    ///////////////////////////////
    NVIC_SetPriority(PORT3_IRQn,1);                            //Set interrup of boton priority
    NVIC_EnableIRQ(PORT3_IRQn);                                //Enable the buton interrup
    //////////////////Buton interrup
    P5->REN  |= BIT1;                                           // Enable resistor for S1 (P1.1)
    P5->OUT  |= BIT1;                                           // S1 (P1.1) high -> pull-up resistor, others low
    P5->DIR  &= ~BIT1;                                           // LED1 (P1.0) to output direction, others input
    ///////////////////////////////
    P5->IES  |= BIT1;                                           // Interrupt on high->low transition for S1 (P1.1)
    P5->IFG  = 0x00;                                           // Clear any pending flags
    P5->IE   |= BIT1;                                           // Enable interrupt for S1 (P1.1)
    ///////////////////////////////
    NVIC_SetPriority(PORT5_IRQn,1);                            //Set interrup of boton priority
    NVIC_EnableIRQ(PORT5_IRQn);                                //Enable the buton interrup

    ///////////////////////////////////////////////////////////////////////////
    // Set P4.3 for Analog input, disabling the I/O circuit.
    P4->SEL0 = BIT3;
    P4->SEL1 = BIT3;
    P4->DIR &= ~BIT3;

    //////////////////////////
    // Setup for adc        //
    //////////////////////////
    ADC14->CTL0 = ADC14_CTL0_PDIV_0 | ADC14_CTL0_SHS_0 | ADC14_CTL0_DIV_7 |
                  ADC14_CTL0_SSEL__MCLK | ADC14_CTL0_SHT0_1 | ADC14_CTL0_ON
                  | ADC14_CTL0_SHP;
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_10 | ADC14_MCTLN_VRSEL_0;
    ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_ENC;
    ADC14->IER0 = ADC14_IER0_IE0;
    NVIC_SetPriority(ADC14_IRQn,1);
    NVIC_EnableIRQ(ADC14_IRQn);
    //////////////////////////////////////////////////////////////////////
    // Configuracion del timer32_1 para el tiempo de encendido de la luz
    TIMER32_1->LOAD = __frecuencyMultiplier * __Multiplier_On_Time * 0x0002DC6C0; //~1s ---> a 48Mhz
    TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
    NVIC_SetPriority(T32_INT1_IRQn,1);
    NVIC_EnableIRQ(T32_INT1_IRQn);
    // Configuracion del timer de muestreo del sonido
    // Cada periodo de tiempo se llama a la interrupcion T32_INT1_IRQHandler(void)
    TIMER32_2->LOAD =  6000; //48000; //~Sampling 8kHz ---> a 48Mhz
    TIMER32_2->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
    NVIC_SetPriority(T32_INT2_IRQn,1);
    NVIC_EnableIRQ(T32_INT2_IRQn);
/*
    TA0CTL = TASSEL_2 | TAIE | MC_1;  // enable timerA0 interrupt, select MC up mode, Select SMCLK as timerclock
    TA0CCR0 = 0x0006; //use MC_1 for a count to this value
    NVIC_SetPriority(TA0_N_IRQn,1);
    NVIC_EnableIRQ(TA0_N_IRQn);

    TA1CTL = TASSEL_2 | TAIE | MC_2;  // enable timerA0 interrupt, select MC up mode, Select SMCLK as timerclock
    //TA1CCR0 = 0x0006; //use MC_1 for a count to this value
    NVIC_SetPriority(TA1_N_IRQn,1);
    NVIC_EnableIRQ(TA1_N_IRQn);
*/
    ////////////////////////////
    //Setup for optical sensor//
    ////////////////////////////
    Init_I2C_GPIO();
    I2C_init();
    OPT3001_init();
    ////////////////////
    // Secuence start //
    ////////////////////
    InitialSeptUpParameters();
    ///////////////////////////
    TurnOnLight();//P2->OUT |= BIT0;
    __delay_cycles(__frecuencyMultiplier * 1500000);
    TurnOffLight();//P2->OUT &= ~BIT0;
    __delay_cycles(__frecuencyMultiplier * 1500000);
    TurnOnLight();//P2->OUT |= BIT0;
    __delay_cycles(__frecuencyMultiplier * 1500000);
    TurnOffLight();//P2->OUT &= ~BIT0;
    __delay_cycles(__frecuencyMultiplier * 1500000);
    TurnOnLight();//P2->OUT |= BIT0;
    __delay_cycles(__frecuencyMultiplier * 1500000);
    TurnOffLight();//P2->OUT &= ~BIT0;
    __delay_cycles(__frecuencyMultiplier * 1500000);
    ///////////////////////////
    //    Get initial lux    //
    ///////////////////////////
    if (OPT3001_getLux() < g_iInitialUmbral)
    {
        TIMER32_1->LOAD = __frecuencyMultiplier * __Multiplier_On_Time * 0x002DC6C0;
        TurnOnLight();
        g_bOutState = 1;
        g_bOffCondition = 0;

    }
    else { TurnOffLight(); g_bOutState = 0; }

    for (;;)                                                   //Infinite Loop
    {
        if(g_bOnCondition)
        {
            TIMER32_1->LOAD = __frecuencyMultiplier * __Multiplier_On_Time * 0x002DC6C0;//Carga tiempo de espera minimo de luz encendida
            g_bOnCondition=0;
            g_bOutState =1;
            TurnOnLight();
        }
        else if(g_bOffCondition)
        {
            g_bOffCondition=0;
            g_bOutState =0;
            g_iNumberOfLights=1;
            TurnOffLight();
        }
    }
}
