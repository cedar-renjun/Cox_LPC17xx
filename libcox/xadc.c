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

//*****************************************************************************
//
//! \brief  Init ADC module
//!         This function can be used to configure ADC peripherals clock frequecy.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be - \ref xADC0_BASE.
//!
//! \param  [in] ulRate is the ADC convert frequecy, it must be lower than 200KHz.
//!
//! \return None.
//
//*****************************************************************************
void ADCInit(unsigned long ulBase, unsigned long ulRate)
{

    unsigned long temp    = 0;
    unsigned long ulClk   = 0;

    // Note: ADC Module Maximum frequecy is 13MHz
    // ulRate <= 200KHz
    // ulClk = SysCtlPeripheralClockGet(SYSCTL_PERIPH_ADC);
    // ulClk /= 13000000;

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

//*****************************************************************************
//
//! \brief  Start special ADC channel.
//!         This function configure ADC convert mode, triggle mode.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be - \ref xADC0_BASE.
//!
//! \param  [in] ulChs is ADC channel.
//!              This value can be OR of the following value:
//!              - \ref ADC_CH_0
//!              - \ref ADC_CH_1
//!              - \ref ADC_CH_2
//!              - \ref ADC_CH_3
//!              - \ref ADC_CH_4
//!              - \ref ADC_CH_5
//!              - \ref ADC_CH_6
//!
//! \param  [in] ulMode is adc convert mode, consist of burst and external
//!              triggle mode. this value can be one of the following value:
//!              - \ref ADC_START_MODE_BURST
//!              - \ref ADC_START_MODE_NOW  
//!              - \ref ADC_START_MODE_EINT0
//!              - \ref ADC_START_MODE_CAP01
//!              - \ref ADC_START_MODE_MAT01
//!              - \ref ADC_START_MODE_MAT03
//!              - \ref ADC_START_MODE_MAT10
//!              - \ref ADC_START_MODE_MAT11
//!
//! \return None.
//
//*****************************************************************************
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

//*****************************************************************************
//
//! \brief  Stop ADC convert procedure.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be - \ref xADC0_BASE.
//!
//! \return None.
//
//*****************************************************************************
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

//*****************************************************************************
//
//! \brief  Enable speical ADC interrupt.
//!         This function can be used to enable ADC convert done interrupt.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be - \ref xADC0_BASE.
//!
//! \param  [in] ulChs is ADC channel.
//!              This value can be OR of the following value:
//!              - \ref ADC_CH_0
//!              - \ref ADC_CH_1
//!              - \ref ADC_CH_2
//!              - \ref ADC_CH_3
//!              - \ref ADC_CH_4
//!              - \ref ADC_CH_5
//!              - \ref ADC_CH_6
//!
//! \return None.
//
//*****************************************************************************
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

//*****************************************************************************
//
//! \brief  Disable speical ADC interrupt.
//!         This function can be used to disable ADC convert done interrupt.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be - \ref xADC0_BASE.
//!
//! \param  [in] ulChs is ADC channel.
//!              This value can be OR of the following value:
//!              - \ref ADC_CH_0
//!              - \ref ADC_CH_1
//!              - \ref ADC_CH_2
//!              - \ref ADC_CH_3
//!              - \ref ADC_CH_4
//!              - \ref ADC_CH_5
//!              - \ref ADC_CH_6
//!
//! \return None.
//
//*****************************************************************************
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


//*****************************************************************************
//
//! \brief  Check ADC status flag.
//!         This function can be use to check ADC special channel DONE and
//!         OVERRUN flag.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be - \ref xADC0_BASE.
//!
//! \param  [in] ulChs is ADC channel.
//!              This value can be OR of the following value:
//!              - \ref ADC_CH_0
//!              - \ref ADC_CH_1
//!              - \ref ADC_CH_2
//!              - \ref ADC_CH_3
//!              - \ref ADC_CH_4
//!              - \ref ADC_CH_5
//!              - \ref ADC_CH_6
//!
//! \param  [in] ulFlags is used to check done or overrun status bit.
//!              This flag is OR of the following value:
//!              - \ref ADC_DONE
//!              - \ref ADC_OVERRUN
//!
//! \return The ADC channel status flag.
//!              - \ref xtrue when flag has been set.
//!              - \ref xflase when flag has not been set.
//
//*****************************************************************************
xtBoolean ADCStatusCheck(unsigned long ulBase, unsigned long ulChs, unsigned long ulFlags)
{

    switch(ulFlags)
    {
        case ADC_DONE:                 // Check ADC convert done flag. 
            {
                ulChs = ulChs;
                break;
            }

        case ADC_OVERRUN:              // Check ADC overrun flag. 
            {
                ulChs <<= 8;
                break;                
            }

        case ADC_DONE | ADC_OVERRUN:   // Check ADC convert done and overrun flag. 
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

//*****************************************************************************
//
//! \brief  Read ADC channel converted data.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be - \ref xADC0_BASE.
//!
//! \param  [in] ulChs is ADC channel.
//!              This value can be OR of the following value:
//!              - \ref ADC_CH_0
//!              - \ref ADC_CH_1
//!              - \ref ADC_CH_2
//!              - \ref ADC_CH_3
//!              - \ref ADC_CH_4
//!              - \ref ADC_CH_5
//!              - \ref ADC_CH_6
//!
//! \return The ADC channel data.
//!
//! \note   The ADC convert data is 12-bit length.
//
//*****************************************************************************
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
                while(1);              // Error.
            }
    }

    // Return ADC channel value.
    ulTmpReg = (ulTmpReg & BIT_MASK(32, 15, 4)) >> 4;
    return (ulTmpReg);
}

