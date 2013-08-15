#include "xhw_types.h"
#include "xhw_ints.h"
#include "xcore.h"
#include "xhw_memmap.h"
#include "xhw_nvic.h"
#include "xhw_sysctl.h"
#include "xdebug.h"
#include "xsysctl.h"
#include "xhw_gpio.h"
#include "xgpio.h"
#include "xhw_adc.h"
#include "xadc.h"  


// ulRate <= 200KHz
void ADCInit(unsigned long ulBase, unsigned long ulRate)
{

    unsigned long temp    = 0;
    unsigned long ulClk   = 0;

    // Note: ADC Module Maximum frequecy is 13MHz
    /*ulClk = SysCtlPeripheralClockGet(SYSCTL_PERIPH_ADC);*/
    /*ulClk /= 13000000;*/

    //xHWREG(ulBase + AD_CR) = (ulClk<<CR_CLKDIV_S) | BIT_32_0;
    xHWREG(ulBase + AD_CR) = 0;

    // Set clock frequency
    ulClk = SysCtlPeripheralClockGet(SYSCTL_PERIPH_ADC);

    // The APB clock (PCLK_ADC0) is divided by (CLKDIV+1) to produce the clock for
    // A/D converter, which should be less than or equal to 13MHz.
    // A fully conversion requires 65 of these clocks.
    // ADC clock = PCLK_ADC0 / (CLKDIV + 1);
    // ADC rate = ADC clock / 65;
    temp = ulRate * 65;

    // Get the round value by fomular: (2*A + B)/(2*B)
    temp = (ulClk * 2 + temp)/(2 * temp) - 1; 

    xHWREG(ulBase + AD_CR) = (temp<<CR_CLKDIV_S) | BIT_32_0;
}

/*
//! Start burst mode
#define ADC_START_MODE_BURST        CR_BURST

//! Start conversion now
#define ADC_START_MODE_NOW          CR_START_NOW

//! Start conversion when the edge selected by bit 27 occurs on P2.10/EINT0
#define ADC_START_MODE_EINT0        CR_START_EINT0             

//! Start conversion when the edge selected by bit 27 occurs on P1.27/CAP0.1
#define ADC_START_MODE_CAP01        CR_START_CAP01             

//! Start conversion when the edge selected by bit 27 occurs on MAT0.1
#define ADC_START_MODE_MAT01        CR_START_MAT01             

//! Start conversion when the edge selected by bit 27 occurs on MAT0.3
#define ADC_START_MODE_MAT03        CR_START_MAT03             

//! Start conversion when the edge selected by bit 27 occurs on MAT1.0
#define ADC_START_MODE_MAT10        CR_START_MAT10             

//! Start conversion when the edge selected by bit 27 occurs on MAT1.1
#define ADC_START_MODE_MAT11        CR_START_MAT11             
*/

void ADCStart(unsigned long ulBase, unsigned long ulChs, unsigned long ulMode)
{
    unsigned long ulTmpReg = 0;

    // Check the parameters.
    xASSERT(ulBase == ADC_BASE);
    xASSERT( (ulMode == ADC_START_MODE_BURST)  ||
             (ulMode == ADC_START_MODE_NOW  )  ||
             (ulMode == ADC_START_MODE_EINT0)  ||
             (ulMode == ADC_START_MODE_CAP01)  ||
             (ulMode == ADC_START_MODE_MAT01)  ||
             (ulMode == ADC_START_MODE_MAT03)  ||
             (ulMode == ADC_START_MODE_MAT10)  ||
             (ulMode == ADC_START_MODE_MAT11)  );

    if(ulMode != ADC_START_MODE_BURST)        // Normal Mode
    {
        // Configure ADC Channel
        ulTmpReg =  xHWREG(ulBase + AD_CR);
        ulTmpReg &= ~(CR_START_M | CR_BURST);
        ulTmpReg |= (CR_PDN | ulMode);
        xHWREG(ulBase + AD_CR) = ulTmpReg;    

        // Configure ADC into normal mode, and select
        ulTmpReg =  xHWREG(ulBase + AD_CR);
        ulTmpReg &= ~(CR_START_M | CR_BURST | CR_SEL_M);
        ulTmpReg |= (CR_PDN | ulMode | ulChs);
        xHWREG(ulBase + AD_CR) = ulTmpReg;
    }
    else                                      // Burst Mode
    {
        ulTmpReg =  xHWREG(ulBase + AD_CR);
        ulTmpReg &= ~(CR_START_M | CR_SEL_M);
        ulTmpReg |= (CR_PDN | CR_BURST | ulChs);
        xHWREG(ulBase + AD_CR) = ulTmpReg;
    }
}

void ADCStop(unsigned long ulBase)
{
    unsigned long ulTmpReg = 0;

    // Check the parameters.
    xASSERT(ulBase == ADC_BASE);

    // Stop ADC by set ADC into power dowm mode then configure stop bit
    ulTmpReg = xHWREG(ulBase + AD_CR);
    ulTmpReg &= ~(CR_PDN | CR_START_M);
    xHWREG(ulBase + AD_CR) = ulTmpReg;
}


/*
#define ADC_CH_M                BIT_MASK(32, 6, 0)

#define ADC_CH_0                BIT_32_0
#define ADC_CH_1                BIT_32_1
#define ADC_CH_2                BIT_32_2
#define ADC_CH_3                BIT_32_3
#define ADC_CH_4                BIT_32_4
#define ADC_CH_5                BIT_32_5
#define ADC_CH_6                BIT_32_6
*/


void ADCIntEnable(unsigned long ulBase, unsigned long ulChs)
{
    unsigned long ulTmpReg = 0;

    // Check the parameters.
    xASSERT(ulBase == ADC_BASE);
    xASSERT( (ulChs & ~ADC_CH_M) == 0 );

    // Enable Selected ADC Channel
    ulTmpReg = xHWREG(ulBase + AD_INTEN);
    ulTmpReg &= ~INTEN_GEN;
    ulTmpReg |= ulChs;
    xHWREG(ulBase + AD_INTEN) = ulTmpReg;
}

void ADCIntDisable(unsigned long ulBase, unsigned long ulChs)
{
    unsigned long ulTmpReg = 0;

    // Check the parameters.
    xASSERT(ulBase == ADC_BASE);
    xASSERT( (ulChs & ~ADC_CH_M) == 0 );

    // Disable Selected ADC Channel
    ulTmpReg = xHWREG(ulBase + AD_INTEN);
    ulTmpReg &= ~INTEN_GEN;
    ulTmpReg &= ~ulChs;
    xHWREG(ulBase + AD_INTEN) = ulTmpReg;
} 


unsigned long ADCStatusGet(unsigned long ulBase, unsigned long ulChs)
{
    ulChs |= (ulChs << 8);

    return (xHWREG(ulBase + AD_INTEN) &  ulChs);
}

/*
#define ADC_DONE                BIT_32_0
#define ADC_OVERRUN             BIT_32_1
*/

xtBoolean ADCStatusCheck(unsigned long ulBase, unsigned long ulChs, unsigned long ulFlags)
{

    switch(ulFlags)
    {
        case ADC_DONE:
            {
                ulChs = ulChs;
                break;
            }

        case ADC_OVERRUN:
            {
                ulChs <<= 8;
                break;                
            }

        case ADC_DONE | ADC_OVERRUN:
            {
                ulChs |= (ulChs << 8);
                break;
            }
        default:
            {
                while(1);
            }
    }

    if( xHWREG(ulBase + AD_STAT) &  ulChs )
    {
        return (xtrue);
    }
    else
    {
        return (xfalse);
    }
}

unsigned long ADCDataRead(unsigned long ulBase, unsigned long ulCh)
{
    unsigned long ulTmpReg = 0;

    switch(ulCh)
    {
        case ADC_CH_0:
            {
                ulTmpReg =  xHWREG(ulBase + AD_DR0);
                break;
            }

        case ADC_CH_1:
            {
                ulTmpReg =  xHWREG(ulBase + AD_DR1);
                break;
            }

        case ADC_CH_2:
            {
                ulTmpReg =  xHWREG(ulBase + AD_DR2);
                break;
            }

        case ADC_CH_3:
            {
                ulTmpReg =  xHWREG(ulBase + AD_DR3);
                break;
            }

        case ADC_CH_4:
            {
                ulTmpReg =  xHWREG(ulBase + AD_DR4);
                break;
            }

        case ADC_CH_5:
            {
                ulTmpReg =  xHWREG(ulBase + AD_DR5);
                break;
            }

        case ADC_CH_6:
            {
                ulTmpReg =  xHWREG(ulBase + AD_DR6);
                break;
            }

        default:
            {
                while(1);
            }
    }

    ulTmpReg = (ulTmpReg & BIT_MASK(32, 15, 4)) >> 4;
    return (ulTmpReg);

}


