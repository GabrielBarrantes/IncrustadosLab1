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
    //P1->OUT ^= BIT0;                                           // Toggle LED1 (P1.0)
    if(!g_bOutState){ g_bOnCondition=1; }
    else { g_bOffCondition=1; }
    __delay_cycles(__frecuencyMultiplier*600000);                                    // Delay of 200ms
    P1->IFG &= ~BIT1;                                          // Clear pending interrupt flag for S1 (P1.1)
    P1->IE |= BIT1;                                            // Enable interrupt for S1 (P1.1)
    }
}

void PORT3_IRQHandler( void )                                  // Interrupt handler for port 1
{
    if(P3->IFG & BIT5)
    {
    P3->IE &= ~BIT5;                                           // Disable interrupt for S1 (P1.1) for debouncing
    //P2->OUT ^= BIT0;                                           // Toggle LED1 (P1.0)
    if(!g_bOutState){ g_bOnCondition=1; }
    else { g_bOffCondition=1; }
    __delay_cycles(__frecuencyMultiplier*600000);                              // Delay of 200ms
    P3->IFG &= ~BIT5;                                          // Clear pending interrupt flag for S1 (P1.1)
    P3->IE |= BIT5;                                            // Enable interrupt for S1 (P1.1)
    }
}

void PORT5_IRQHandler( void )                                  // Interrupt handler for port 1
{
    if(P5->IFG & BIT1)
    {
    P5->IE &= ~BIT1;                                           // Disable interrupt for S1 (P1.1) for debouncing
    //P2->OUT ^= BIT0;                                         // Toggle LED1 (P1.0)
    if(!g_bOutState){ g_bOnCondition=1; }
    else { g_bOffCondition=1; }
    __delay_cycles(__frecuencyMultiplier*600000);                                    // Delay of 200ms
    P5->IFG &= ~BIT1;                                          // Clear pending interrupt flag for S1 (P1.1)
    P5->IE |= BIT1;                                            // Enable interrupt for S1 (P1.1)
    }
}

void T32_INT1_IRQHandler(void)
{
    __disable_irq();
    TIMER32_1->INTCLR = 0U;
    ///////////////////////
    P1->OUT ^= BIT0;        //toggl indicator led
    //if( start ){ P2->OUT ^= BIT0; }
    //lux = OPT3001_getLux(); //read light sensor value
    //P2->OUT ^= BIT0;
    if(g_bOutState){ g_bOffCondition = 1; }
    //ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_SC; // read adc
    //state2 =1;
    //state3=1;
    ///////////////////////
    __enable_irq();
    return;
}

void T32_INT2_IRQHandler(void)
{
    __disable_irq();
    TIMER32_2->INTCLR = 0U;
    ///////////////////////
    //P2->OUT ^= BIT4;        //toggl indicator led
    //state3=0;
    ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_SC; // esta linea lee el valor del adc y recalcula los promedios
    ///////////////////////
    __enable_irq();
    return;
}

void TA0_0_IRQHandler(void)
{
    __disable_irq();
    //TIMER__Last_Length_2->INTCLR = 0U;
    ///////////////////////
    //if(state3){ P2->OUT ^= BIT0;}        //toggl indicator led
    //P2->OUT &= ~BIT0;
    ///////////////////////
    __enable_irq();
    return;
}



void ADC14_IRQHandler(void)
{
    __disable_irq();

    g_fdataArray[g_iCounter]= ADC14->MEM[0];

    int j=g_iCounter-__Last_Length+1;
    if(g_iCounter-__Last_Length+1<0){ j = __SAMPLE_LENGTH + g_iCounter-__Last_Length+1 ;}



    g_fLastMeanSound=g_fLastMeanSound+(g_fdataArray[g_iCounter]-g_fdataArray[j])/__Last_Length;

    int k = g_iCounter-__Last_Length; int l = g_iCounter+1;
    if(g_iCounter==__SAMPLE_LENGTH){ g_iCounter=0; }
    if(g_iCounter-__Last_Length<0){ k = __SAMPLE_LENGTH + g_iCounter-__Last_Length ;}

    g_fMeanSound = g_fMeanSound+(-g_fdataArray[l]+g_fdataArray[k])/(__SAMPLE_LENGTH-__Last_Length) ;

    if(g_fLastMeanSound>1.005*g_fMeanSound && OPT3001_getLux() < g_iInitialUmbral){ g_bOnCondition=1; }


    g_iCounter++;
    if(g_iCounter==__SAMPLE_LENGTH){g_iCounter = 0;}



    ADC14->CLRIFGR0 = ADC14_CLRIFGR0_CLRIFG0;
    __enable_irq();
    return;
}

#ifdef __cplusplus
}
#endif


