//*****************************************************************************
//
//! \file xsysctl.c
//! \brief Driver for the System Controller.
//! \version V2.1.1.0
//! \date 11/14/2011
//! \author CooCox
//! \copy
//!
//! Copyright (c)  2011, CooCox
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!     * Redistributions of source code must retain the above copyright
//! notice, this list of conditions and the following disclaimer.
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution.
//!     * Neither the name of the <ORGANIZATION> nor the names of its
//! contributors may be used to endorse or promote products derived
//! from this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#include "xhw_types.h"
#include "xhw_ints.h"
#include "xhw_memmap.h"
#include "xhw_nvic.h"
#include "xdebug.h"
#include "xcore.h"
#include "xhw_sysctl.h"
#include "xsysctl.h"

static unsigned long s_ulExtClockMHz = 12;

//*****************************************************************************
//
//! Peripheral Base and ID Table structure type
//
//*****************************************************************************
typedef struct
{
    unsigned long ulPeripheralBase;
    unsigned long ulPeripheralID;
    unsigned long ulPeripheralIntNum;
}
tPeripheralTable;

//*****************************************************************************
//
// An array that maps the peripheral base and peripheral ID and interrupt number
// together to enablea peripheral or peripheral interrupt by a peripheral base.
//
//*****************************************************************************
static const tPeripheralTable g_pPeripherals[] =
{
    { xGPIO_PORTA_BASE, xSYSCTL_PERIPH_GPIOA,    INT_GPIO   },
    { xGPIO_PORTB_BASE, xSYSCTL_PERIPH_GPIOB,    INT_GPIO   },
    { xGPIO_PORTC_BASE, xSYSCTL_PERIPH_GPIOC,    INT_GPIO   },
    { xGPIO_PORTD_BASE, xSYSCTL_PERIPH_GPIOD,    INT_GPIO   },
    { xGPIO_PORTE_BASE, xSYSCTL_PERIPH_GPIOE,    INT_GPIO   },
    { xGPIO_PORTF_BASE, xSYSCTL_PERIPH_GPIOF,    INT_GPIO   },
    { xUART0_BASE,      xSYSCTL_PERIPH_UART0,    INT_UART0  },
    { xUART1_BASE,      xSYSCTL_PERIPH_UART1,    INT_UART1  },
    { xUART2_BASE,      xSYSCTL_PERIPH_UART2,    INT_UART2  },
    { xSPI0_BASE,       xSYSCTL_PERIPH_SPI0,     INT_SSP0  },
    {0, 0, 0},
};

//*****************************************************************************
//
//! \internal
//! Checks a peripheral identifier.
//!
//! \param ulPeripheral is the peripheral identifier.
//!
//! This function determines if a peripheral identifier is valid.
//!
//! \return Returns \b true if the peripheral identifier is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef xDEBUG
static xtBoolean
SysCtlPeripheralValid(unsigned long ulPeripheral)
{
    return((ulPeripheral == SYSCTL_PERIPH_ADC)  ||
           (ulPeripheral == SYSCTL_PERIPH_CAN0) ||
           (ulPeripheral == SYSCTL_PERIPH_CAN1) ||
           (ulPeripheral == SYSCTL_PERIPH_CAN2) ||
           (ulPeripheral == SYSCTL_PERIPH_RIT)  ||
           (ulPeripheral == SYSCTL_PERIPH_ETH)  ||
           (ulPeripheral == SYSCTL_PERIPH_GPIO) ||
           (ulPeripheral == SYSCTL_PERIPH_RTC)  ||
           (ulPeripheral == SYSCTL_PERIPH_I2C0) ||
           (ulPeripheral == SYSCTL_PERIPH_I2C1) ||
           (ulPeripheral == SYSCTL_PERIPH_I2C2) ||
           (ulPeripheral == SYSCTL_PERIPH_I2S)  ||
           (ulPeripheral == SYSCTL_PERIPH_PWM)  ||
           (ulPeripheral == SYSCTL_PERIPH_QEI)  ||
           (ulPeripheral == SYSCTL_PERIPH_SSP0) ||
           (ulPeripheral == SYSCTL_PERIPH_SSP1) ||
           (ulPeripheral == SYSCTL_PERIPH_TIMER0) ||
           (ulPeripheral == SYSCTL_PERIPH_TIMER1) ||
           (ulPeripheral == SYSCTL_PERIPH_TIMER2) ||
           (ulPeripheral == SYSCTL_PERIPH_TIMER3) ||
           (ulPeripheral == SYSCTL_PERIPH_UART0) ||
           (ulPeripheral == SYSCTL_PERIPH_UART1) ||
           (ulPeripheral == SYSCTL_PERIPH_UART2) ||
           (ulPeripheral == SYSCTL_PERIPH_UART3) ||
           (ulPeripheral == SYSCTL_PERIPH_DMA)   ||
           (ulPeripheral == SYSCTL_PERIPH_USB));
}
#endif

//*****************************************************************************
//
//! \brief Sets the clocking of the device.
//!
//! \param ulSysClk is the target system HCLK.
//! \param ulConfig is the required configuration of the device clocking.
//!
//! This function configures the clocking of the device.  The input crystal
//! frequency, oscillator to be used, use of the PLL, and the system clock
//! divider are all configured with this function.
//!
//! The \e ulConfig parameter is the logical OR of several different values,
//! many of which are grouped into sets where only one can be chosen.
//!
//! The external crystal frequency is chosen with one of the following values:
//! \b SYSCTL_XTAL_1MHZ, \b SYSCTL_XTAL_2MHZ, \b SYSCTL_XTAL_4MHZ,
//! \b SYSCTL_XTAL_5MHZ, \b SYSCTL_XTAL_6MHZ, \b SYSCTL_XTAL_8MHZ,
//! \b SYSCTL_XTAL_10MHZ, \b SYSCTL_XTAL_12MHZ, \b SYSCTL_XTAL_16MHZ,
//!
//! The internal crystal frequency is chosen with one of the following values:
//! \b xSYSCTL_INT_16MHZ.
//!
//! The external slow clk frequency is chosen with one of the following values:
//! \b xSYSCTL_XTALSL_32768HZ.
//!
//! The internal slow clk frequency is chosen with one of the following values:
//! \b xSYSCTL_INTSL_30KHZ.
//!
//! The oscillator source is chosen with one of the following values:
//! \b SYSCTL_OSC_MAIN, \b SYSCTL_OSC_INT, \b SYSCTL_OSC_INTSL,
//! \b SYSCTL_OSC_EXTSL,.
//!
//! The internal and main oscillators are disabled with the
//! \b SYSCTL_INT_OSC_DIS and \b SYSCTL_MAIN_OSC_DIS flags, respectively.
//! The external oscillator must be enabled in order to use an external clock
//! source.  Note that attempts to disable the oscillator used to clock the
//! device will be prevented by the hardware.
//!
//! The PLL is disabled with the \b xSYSCTL_PLL_PWRDN.
//!
//! More info please refrence \ref Stellaris_SysCtl_Clock_Config.
//!
//! \return None.
//
//*****************************************************************************
void
xSysCtlClockSet(unsigned long ulSysClk, unsigned long ulConfig)
{
    unsigned long ulDelay, ulRCC, ulRCC2;
    unsigned long ulOscFreq, ulSysDiv;

    xASSERT(ulSysClk > 0 && ulSysClk <= 100000000);

    //
    // Calc oscillator freq
    //
    switch(ulConfig & SYSCTL_CLKSRCSEL_M)
    {
        case xSYSCTL_OSC_MAIN:
        {
            xASSERT(!(ulConfig & xSYSCTL_MAIN_OSC_DIS));
            xHWREG(SYSCTL_CLKSRCSEL) = SYSCTL_CLKSRCSEL_MAINOSC;
            switch(ulConfig & SYSCTL_RCC_XTAL_M)
            {
                case xSYSCTL_XTAL_1MHZ:
                {
                    s_ulExtClockMHz = 1;
                    break;
                }
                case xSYSCTL_XTAL_2MHZ:
                {
                    s_ulExtClockMHz = 2;
                    break;
                }
                case xSYSCTL_XTAL_4MHZ:
                {
                    s_ulExtClockMHz = 4;
                    break;
                }
                case xSYSCTL_XTAL_5MHZ:
                {
                    s_ulExtClockMHz = 5;
                    break;
                }
                case xSYSCTL_XTAL_6MHZ:
                {
                    s_ulExtClockMHz = 6;
                    break;
                }
                case xSYSCTL_XTAL_8MHZ:
                {
                    s_ulExtClockMHz = 8;
                    break;
                }
                case xSYSCTL_XTAL_10MHZ:
                {
                    s_ulExtClockMHz = 10;
                    break;
                }
                case xSYSCTL_XTAL_12MHZ:
                {
                    s_ulExtClockMHz = 12;
                    break;
                }
                case xSYSCTL_XTAL_16MHZ:
                {
                    s_ulExtClockMHz = 16;
                    break;
                }
                case xSYSCTL_XTAL_24MHZ:
                {
                    s_ulExtClockMHz = 24;
                    break;
                }
                case xSYSCTL_XTAL_25MHZ:
                {
                    s_ulExtClockMHz = 25;
                    break;
                }
                default:
                {
                    xASSERT(0);
                    break;
                }
            }
            ulOscFreq = s_ulExtClockMHz * 1000000;
            break;
        }

        case xSYSCTL_OSC_INT:
        {
            xASSERT(!(ulConfig & xSYSCTL_INT_OSC_DIS));
            xHWREG(SYSCTL_CLKSRCSEL) = SYSCTL_CLKSRCSEL_INTRC;
            ulOscFreq = 4000000;
            break;
        }
        case xSYSCTL_OSC_EXTSL:
        {
            xHWREG(SYSCTL_CLKSRCSEL) = SYSCTL_CLKSRCSEL_RTCOSC;
            ulOscFreq = 32768;
            break;
        }
        default:
        {
            xASSERT(0);
            break;
        }

    }

    if(ulSysClk == ulOscFreq)
    {
        xHWREG(SYSCTL_PLL0CON) &= ~(SYSCTL_PLL0CON_PLLE0 | SYSCTL_PLL0CON_PLLC0);
        xHWREG(SYSCTL_CLKCFG) = 0;
    }
    else if (ulSysClk <= ulOscFreq)
    {
        //
        // Calc the SysDiv
        //
        xASSERT(ulSysClk <= ulOscFreq);

        for(ulSysDiv = 1; ulSysDiv < 256; ulSysDiv++)
        {
            if((ulOscFreq / (ulSysDiv + 1)) <= ulSysClk)
            {
                break;
            }
        }
        xASSERT(ulSysDiv < 256);

        xHWREG(SYSCTL_CLKCFG) = ulSysDiv;
    }
    else
    {
        xHWREG(SYSCTL_PLL0CON) |= (SYSCTL_PLL0CON_PLLE0 | SYSCTL_PLL0CON_PLLC0);
        xASSERT(!(ulConfig & xSYSCTL_PLL_PWRDN));
        xASSERT((ulConfig & SYSCTL_RCC_OSCSRC_M == xSYSCTL_OSC_MAIN) ||
                (ulConfig & SYSCTL_RCC_OSCSRC_M == xSYSCTL_OSC_INT));

        for(ulSysDiv = 1; ulSysDiv < 256; ulSysDiv++)
        {
            if((ulSysClk / (ulSysDiv + 1)) <= ulOscFreq)
            {
                break;
            }
        }

        for(ulSysDiv = 2; ulSysDiv <= 255; ulSysDiv++)
        {
            if((400000000 / ulSysDiv) <= ulSysClk)
            {
                break;
            }
        }

        xASSERT(ulSysDiv <= 255);

        if(ulSysDiv % 2)
        {
            //
            // RCC2.SYSDIV2 : SYSDIV2LSB(28:23)=(ulSysDiv - 1)
            //
            ulRCC2 |= SYSCTL_RCC2_USERCC2 | (ulSysDiv - 1) << 22 |
                SYSCTL_RCC2_DIV400;

            ulRCC |= SYSCTL_RCC_USESYSDIV;

        }
        else
        {
            //
            // RCC2.SYSDIV2(28:23)= (ulSysDiv / 2 - 1)
            // RCC.SYSDIV = (ulSysDiv / 2 - 1)
            //

            ulRCC2 &= ~(SYSCTL_RCC2_DIV400);
            ulRCC2 |= ((ulSysDiv / 2) - 1) << 23;
            ulRCC |= ((ulSysDiv / 2) - 1) << 23 | SYSCTL_RCC_USESYSDIV;

        }
    }


    //
    // See if the PLL output is being used to clock the system.
    //
    if(!(ulConfig & SYSCTL_RCC_BYPASS))
    {
        //
        // Wait until the PLL has locked.
        //
        for(ulDelay = 32768; ulDelay > 0; ulDelay--)
        {
            if(xHWREG(SYSCTL_RIS) & SYSCTL_INT_PLL_LOCK)
            {
                break;
            }
        }

        //
        // Enable use of the PLL.
        //
        ulRCC &= ~(SYSCTL_RCC_BYPASS);
        ulRCC2 &= ~(SYSCTL_RCC2_BYPASS2);
    }

    //
    // Write the final RCC value.
    //
    xHWREG(SYSCTL_RCC) = ulRCC;
    xHWREG(SYSCTL_RCC2) = ulRCC2;

    //
    // Delay for a little bit so that the system divider takes effect.
    //
    SysCtlDelay(16);

}

//*****************************************************************************
//
//! \brief Enables a peripheral.
//!
//! \param ulPeripheralBase a Peripheral base indicate which peripheral to be
//! enabled.Details please refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! Peripherals are enabled with this function.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! The \e ulPeripheral parameter must be only one of the following values:
//! Details please refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! \return None.
//
//*****************************************************************************
void
xSysCtlPeripheralEnable2(unsigned long ulPeripheralBase)
{
    unsigned long i;
    for(i=0; g_pPeripherals[i].ulPeripheralBase != 0; i++)
    {
        if(ulPeripheralBase == g_pPeripherals[i].ulPeripheralBase)
        {
            SysCtlPeripheralEnable(g_pPeripherals[i].ulPeripheralID);
            break;
        }
    }
}

//*****************************************************************************
//
//! \brief Disables a peripheral.
//!
//! \param ulPeripheralBase a Peripheral base indicate which peripheral to be
//! enabled.Details please refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! Peripherals are disabled with this function.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! The \e ulPeripheral parameter must be only one of the following values:
//! Details please refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! \return None.
//
//*****************************************************************************
void
xSysCtlPeripheralDisable2(unsigned long ulPeripheralBase)
{
    unsigned long i;

    for(i = 0; g_pPeripherals[i].ulPeripheralBase != 0; i++)
    {
        if(ulPeripheralBase == g_pPeripherals[i].ulPeripheralBase)
        {
            SysCtlPeripheralDisable(g_pPeripherals[i].ulPeripheralID);
            break;
        }
    }
}

//*****************************************************************************
//
//! \brief Reset a peripheral.
//!
//! \param ulPeripheralBase a Peripheral base indicate which peripheral to be
//! Reset.Details please refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! Peripherals are Reset with this function.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! The \e ulPeripheral parameter must be only one of the following values:
//! Details please refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! \return None.
//
//*****************************************************************************
void
xSysCtlPeripheralReset2(unsigned long ulPeripheralBase)
{
    unsigned long i;

    for(i = 0; g_pPeripherals[i].ulPeripheralBase != 0; i++)
    {
        if(ulPeripheralBase == g_pPeripherals[i].ulPeripheralBase)
        {
            SysCtlPeripheralReset(g_pPeripherals[i].ulPeripheralID);
            break;
        }
    }
}

//*****************************************************************************
//
//! \brief Get the peripheral interrupt number through peripheral base.
//!
//! \param ulPeripheral The peripheral's base
//!
//! \note It's especially useful to enable the short pin's corresponding
//! peripheral interrupt: Use the short pin to Get the GPIO base through
//! \ref xGPIOSPinToPort function, and then use this function to enable the GPIO
//! interrupt.
//!
//! \return None.
//
//*****************************************************************************
unsigned long
xSysCtlPeripheralIntNumGet(unsigned long ulPeripheralBase)
{
    unsigned long i;

    for(i = 0; g_pPeripherals[i].ulPeripheralBase != 0; i++)
    {
        if(ulPeripheralBase == g_pPeripherals[i].ulPeripheralBase)
        {
            return g_pPeripherals[i].ulPeripheralIntNum;
        }
    }

    return 0;
}

//*****************************************************************************
//
//! \brief Get the peripheral ID through peripheral base.
//!
//! \param ulPeripheralBase The peripheral's base
//!
//!
//! \return None.
//
//*****************************************************************************
unsigned long
xSysCtlPeripheralIdGet(unsigned long ulPeripheralBase)
{
    unsigned long i;

    for(i = 0; g_pPeripherals[i].ulPeripheralBase != 0; i++)
    {
        if(ulPeripheralBase == g_pPeripherals[i].ulPeripheralBase)
        {
            return g_pPeripherals[i].ulPeripheralID;
        }
    }

    return 0;
}

//*****************************************************************************
//
//! \brief Enables a peripheral.
//!
//! \param ulPeripheral is the peripheral to enable.
//!
//! Peripherals are enabled with this function.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! The \e ulPeripheral parameter must be only one of the following values:
//! \b SYSCTL_PERIPH_ADC, \b SYSCTL_PERIPH_CAN1, \b SYSCTL_PERIPH_CAN2,
//! \b SYSCTL_PERIPH_RIT, \b SYSCTL_PERIPH_ETH, \b SYSCTL_PERIPH_GPIO,
//! \b SYSCTL_PERIPH_RTC, \b SYSCTL_PERIPH_I2C0, \b SYSCTL_PERIPH_I2C1,
//! \b SYSCTL_PERIPH_I2C2, \b SYSCTL_PERIPH_I2S, \b SYSCTL_PERIPH_PWM,
//! \b SYSCTL_PERIPH_QEI,  \b SYSCTL_PERIPH_SSP0, \b SYSCTL_PERIPH_SSP1,
//! \b SYSCTL_PERIPH_TIMER0, \b SYSCTL_PERIPH_TIMER1, \b SYSCTL_PERIPH_TIMER2,
//! \b SYSCTL_PERIPH_TIMER3, \b SYSCTL_PERIPH_UART0, \b SYSCTL_PERIPH_UART1,
//! \b SYSCTL_PERIPH_UART2, \b SYSCTL_PERIPH_UART3, \b SYSCTL_PERIPH_PCMCPWM
//! \b SYSCTL_PERIPH_DMA, \b SYSCTL_PERIPH_USB,  or \b SYSCTL_PERIPH_WDOG.
//!
//! \note It takes five clock cycles after the write to enable a peripheral
//! before the the peripheral is actually enabled.  During this time, attempts
//! to access the peripheral will result in a bus fault.  Care should be taken
//! to ensure that the peripheral is not accessed during this brief time
//! period.
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlPeripheralEnable(unsigned long ulPeripheral)
{
    //
    // Check the arguments.
    //
    xASSERT(SysCtlPeripheralValid(ulPeripheral));

    //
    // Enable this peripheral.
    //
    xHWREG(SYSCTL_PCONP) |= ulPeripheral;
}

//*****************************************************************************
//
//! \brief Disables a peripheral.
//!
//! \param ulPeripheral is the peripheral to disable.
//!
//! Peripherals are disabled with this function.  Once disabled, they will not
//! operate or respond to register reads/writes.
//!
//! The \e ulPeripheral parameter must be only one of the following values:
//! \b SYSCTL_PERIPH_ADC, \b SYSCTL_PERIPH_CAN1, \b SYSCTL_PERIPH_CAN2,
//! \b SYSCTL_PERIPH_RIT, \b SYSCTL_PERIPH_ETH, \b SYSCTL_PERIPH_GPIO,
//! \b SYSCTL_PERIPH_RTC, \b SYSCTL_PERIPH_I2C0, \b SYSCTL_PERIPH_I2C1,
//! \b SYSCTL_PERIPH_I2C2, \b SYSCTL_PERIPH_I2S, \b SYSCTL_PERIPH_PWM,
//! \b SYSCTL_PERIPH_QEI,  \b SYSCTL_PERIPH_SSP0, \b SYSCTL_PERIPH_SSP1,
//! \b SYSCTL_PERIPH_TIMER0, \b SYSCTL_PERIPH_TIMER1, \b SYSCTL_PERIPH_TIMER2,
//! \b SYSCTL_PERIPH_TIMER3, \b SYSCTL_PERIPH_UART0, \b SYSCTL_PERIPH_UART1,
//! \b SYSCTL_PERIPH_UART2, \b SYSCTL_PERIPH_UART3, \b SYSCTL_PERIPH_PCMCPWM
//! \b SYSCTL_PERIPH_DMA, \b SYSCTL_PERIPH_USB,  or \b SYSCTL_PERIPH_WDOG.
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlPeripheralDisable(unsigned long ulPeripheral)
{
    //
    // Check the arguments.
    //
    xASSERT(SysCtlPeripheralValid(ulPeripheral));

    //
    // Disable this peripheral.
    //
    xHWREG(SYSCTL_PCONP) &= ulPeripheral;
}

//*****************************************************************************
//
//! \brief Enables individual system control interrupt sources.
//!
//! \param ulInts is a bit mask of the interrupt sources to be enabled.  Must
//! be a logical OR of \b SYSCTL_INT_PLL_LOCK, \b SYSCTL_INT_CUR_LIMIT,
//! \b SYSCTL_INT_IOSC_FAIL, \b SYSCTL_INT_MOSC_FAIL, \b SYSCTL_INT_POR,
//! \b SYSCTL_INT_BOR, and/or \b SYSCTL_INT_PLL_FAIL.
//!
//! Enables the indicated system control interrupt sources.  Only the sources
//! that are enabled can be reflected to the processor interrupt; disabled
//! sources have no effect on the processor.
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlIntEnable(unsigned long ulInts)
{
    //
    // Enable the specified interrupts.
    //
    xHWREG(SYSCTL_IMC) |= ulInts;
}

//*****************************************************************************
//
//! \brief Disables individual system control interrupt sources.
//!
//! \param ulInts is a bit mask of the interrupt sources to be disabled.  Must
//! be a logical OR of \b SYSCTL_INT_PLL_LOCK, \b SYSCTL_INT_CUR_LIMIT,
//! \b SYSCTL_INT_IOSC_FAIL, \b SYSCTL_INT_MOSC_FAIL, \b SYSCTL_INT_POR,
//! \b SYSCTL_INT_BOR, and/or \b SYSCTL_INT_PLL_FAIL.
//!
//! Disables the indicated system control interrupt sources.  Only the sources
//! that are enabled can be reflected to the processor interrupt; disabled
//! sources have no effect on the processor.
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlIntDisable(unsigned long ulInts)
{
    //
    // Disable the specified interrupts.
    //
    xHWREG(SYSCTL_IMC) &= ~(ulInts);
}

//*****************************************************************************
//
//! \brief Clears system control interrupt sources.
//!
//! \param ulInts is a bit mask of the interrupt sources to be cleared.  Must
//! be a logical OR of \b SYSCTL_INT_PLL_LOCK, \b SYSCTL_INT_CUR_LIMIT,
//! \b SYSCTL_INT_IOSC_FAIL, \b SYSCTL_INT_MOSC_FAIL, \b SYSCTL_INT_POR,
//! \b SYSCTL_INT_BOR, and/or \b SYSCTL_INT_PLL_FAIL.
//!
//! The specified system control interrupt sources are cleared, so that they no
//! longer assert.  This must be done in the interrupt handler to keep it from
//! being called again immediately upon exit.
//!
//! \note Because there is a write buffer in the Cortex-M3 processor, it may
//! take several clock cycles before the interrupt source is actually cleared.
//! Therefore, it is recommended that the interrupt source be cleared early in
//! the interrupt handler (as opposed to the very last action) to avoid
//! returning from the interrupt handler before the interrupt source is
//! actually cleared.  Failure to do so may result in the interrupt handler
//! being immediately reentered (because the interrupt controller still sees
//! the interrupt source asserted).
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlIntClear(unsigned long ulInts)
{
    //
    // Clear the requested interrupt sources.
    //
    xHWREG(SYSCTL_MISC) = ulInts;
}

//*****************************************************************************
//
//! \brief Gets the current interrupt status.
//!
//! \param bMasked is false if the raw interrupt status is required and true if
//! the masked interrupt status is required.
//!
//! This returns the interrupt status for the system controller.  Either the
//! raw interrupt status or the status of interrupts that are allowed to
//! reflect to the processor can be returned.
//!
//! \return The current interrupt status, enumerated as a bit field of
//! \b SYSCTL_INT_PLL_LOCK, \b SYSCTL_INT_CUR_LIMIT, \b SYSCTL_INT_IOSC_FAIL,
//! \b SYSCTL_INT_MOSC_FAIL, \b SYSCTL_INT_POR, \b SYSCTL_INT_BOR, and
//! \b SYSCTL_INT_PLL_FAIL.
//
//*****************************************************************************
unsigned long
SysCtlIntStatus(xtBoolean bMasked)
{
    //
    // Return either the interrupt status or the raw interrupt status as
    // requested.
    //
    if(bMasked)
    {
        return(xHWREG(SYSCTL_MISC));
    }
    else
    {
        return(xHWREG(SYSCTL_RIS));
    }
}

//*****************************************************************************
//
//! \brief Resets the device.
//!
//! This function will perform a software reset of the entire device.  The
//! processor and all peripherals will be reset and all device registers will
//! return to their default values (with the exception of the reset cause
//! register, which will maintain its current value but have the software reset
//! bit set as well).
//!
//! \return This function does not return.
//
//*****************************************************************************
void
SysCtlReset(void)
{
    //
    // Perform a software reset request.  This will cause the device to reset,
    // no further code will be executed.
    //
    //xHWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;

    //
    // The device should have reset, so this should never be reached.  Just in
    // case, loop forever.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
//! \brief Puts the processor into sleep mode.
//!
//! This function places the processor into sleep mode; it will not return
//! until the processor returns to run mode.  The peripherals that are enabled
//! via SysCtlPeripheralSleepEnable() continue to operate and can wake up the
//! processor (if automatic clock gating is enabled with
//! SysCtlPeripheralClockGating(), otherwise all peripherals continue to
//! operate).
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlSleep(void)
{
    xHWREG(SYSCTL_PCON) = 0;
    //
    // Wait for an interrupt.
    //
    xCPUwfi();
}

//*****************************************************************************
//
//! \brief Puts the processor into deep-sleep mode.
//!
//! This function places the processor into deep-sleep mode; it will not return
//! until the processor returns to run mode.  The peripherals that are enabled
//! via SysCtlPeripheralDeepSleepEnable() continue to operate and can wake up
//! the processor (if automatic clock gating is enabled with
//! SysCtlPeripheralClockGating(), otherwise all peripherals continue to
//! operate).
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlDeepSleep(void)
{
    //
    // Enable deep-sleep.
    //
    xHWREG(SYSCTL_PCON) = 0x08;

    //
    // Wait for an interrupt.
    //
    xCPUwfi();

    //
    // Disable deep-sleep so that a future sleep will work correctly.
    //
    xHWREG(NVIC_SCR) |= NVIC_SCR_SLEEPDEEP;
}

//*****************************************************************************
//
//! \brief Gets the reason for a reset.
//!
//! This function will return the reason(s) for a reset.  Since the reset
//! reasons are sticky until either cleared by software or an external reset,
//! multiple reset reasons may be returned if multiple resets have occurred.
//! The reset reason will be a logical OR of \b SYSCTL_CAUSE_WDOG,
//! \b SYSCTL_CAUSE_BOR, \b SYSCTL_CAUSE_POR, and/or \b SYSCTL_CAUSE_EXT.
//!
//! \return Returns the reason(s) for a reset.
//
//*****************************************************************************
unsigned long
SysCtlResetCauseGet(void)
{
    //
    // Return the reset reasons.
    //
    return(xHWREG(SYSCTL_RSID));
}

//*****************************************************************************
//
//! \brief Clears reset reasons.
//!
//! \param ulCauses are the reset causes to be cleared; must be a logical OR of
//! \b SYSCTL_CAUSE_WDOG, \b SYSCTL_CAUSE_BOR, \b SYSCTL_CAUSE_POR, \b SYSCTL_CAUSE_EXT.
//!
//! This function clears the specified sticky reset reasons.  Once cleared,
//! another reset for the same reason can be detected, and a reset for a
//! different reason can be distinguished (instead of having two reset causes
//! set).  If the reset reason is used by an application, all reset causes
//! should be cleared after they are retrieved with SysCtlResetCauseGet().
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlResetCauseClear(unsigned long ulCauses)
{
    //
    // Clear the given reset reasons.
    //
    xHWREG(SYSCTL_RSID) &= ~(ulCauses);
}

//*****************************************************************************
//
//! \brief Configures the brown-out control.
//!
//! \param ulConfig is the desired configuration of the brown-out control.
//! Must be the logical OR of \b SYSCTL_BOR_RESET and/or
//! \b SYSCTL_BOR_RESAMPLE.
//! \param ulDelay is the number of internal oscillator cycles to wait before
//! resampling an asserted brown-out signal.  This value only has meaning when
//! \b SYSCTL_BOR_RESAMPLE is set and must be less than 8192.
//!
//! This function configures how the brown-out control operates.  It can detect
//! a brown-out by looking at only the brown-out output, or it can wait for it
//! to be active for two consecutive samples separated by a configurable time.
//! When it detects a brown-out condition, it can either reset the device or
//! generate a processor interrupt.
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlBrownOutConfigSet(unsigned long ulConfig, unsigned long ulDelay)
{
    //
    // Check the arguments.
    //
    xASSERT(!(ulConfig & ~(SYSCTL_BOR_RESET | SYSCTL_BOR_RESAMPLE)));
    xASSERT(ulDelay < 8192);

    //
    // Configure the brown-out reset control.
    //
    xHWREG(SYSCTL_PBORCTL) = (ulDelay << SYSCTL_PBORCTL_BORTIM_S) | ulConfig;
}

//*****************************************************************************
//
//! \brief Provides a small delay.
//!
//! \param ulCount is the number of delay loop iterations to perform.
//!
//! This function provides a means of generating a constant length delay.  It
//! is written in assembly to keep the delay consistent across tool chains,
//! avoiding the need to tune the delay based on the tool chain in use.
//!
//! The loop takes 3 cycles/loop.
//!
//! \return None.
//
//*****************************************************************************
#if defined(ewarm) || defined(__ICCARM__) || defined(DOXYGEN)
void
SysCtlDelay(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(gcc) || defined(__GNUC__)
void __attribute__((naked))
SysCtlDelay(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne     SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(rvmdk) || defined(__CC_ARM)
__asm void
SysCtlDelay(unsigned long ulCount)
{
    subs    r0, #1;
    bne     SysCtlDelay;
    bx      lr;
}
#endif

//*****************************************************************************
//
//! \brief Sets the clocking of the device.
//!
//! \param ulConfig is the required configuration of the device clocking.
//!
//! This function configures the clocking of the device.  The input crystal
//! frequency, oscillator to be used, use of the PLL, and the system clock
//! divider are all configured with this function.
//!
//! The \e ulConfig parameter is the logical OR of several different values,
//! many of which are grouped into sets where only one can be chosen.
//!
//! The system clock divider is chosen with one of the following values:
//! \b SYSCTL_SYSDIV_1, \b SYSCTL_SYSDIV_2, \b SYSCTL_SYSDIV_3, ...
//! \b SYSCTL_SYSDIV_64.  Only \b SYSCTL_SYSDIV_1 through \b SYSCTL_SYSDIV_16
//! are valid on Sandstorm-class devices.
//!
//! The use of the PLL is chosen with either \b SYSCTL_USE_PLL or
//! \b SYSCTL_USE_OSC.
//!
//! The external crystal frequency is chosen with one of the following values:
//! \b SYSCTL_XTAL_1MHZ, \b SYSCTL_XTAL_1_84MHZ, \b SYSCTL_XTAL_2MHZ,
//! \b SYSCTL_XTAL_2_45MHZ, \b SYSCTL_XTAL_3_57MHZ, \b SYSCTL_XTAL_3_68MHZ,
//! \b SYSCTL_XTAL_4MHZ, \b SYSCTL_XTAL_4_09MHZ, \b SYSCTL_XTAL_4_91MHZ,
//! \b SYSCTL_XTAL_5MHZ, \b SYSCTL_XTAL_5_12MHZ, \b SYSCTL_XTAL_6MHZ,
//! \b SYSCTL_XTAL_6_14MHZ, \b SYSCTL_XTAL_7_37MHZ, \b SYSCTL_XTAL_8MHZ,
//! \b SYSCTL_XTAL_8_19MHZ, \b SYSCTL_XTAL_10MHZ, \b SYSCTL_XTAL_12MHZ,
//! \b SYSCTL_XTAL_12_2MHZ, \b SYSCTL_XTAL_13_5MHZ, \b SYSCTL_XTAL_14_3MHZ,
//! \b SYSCTL_XTAL_16MHZ, or \b SYSCTL_XTAL_16_3MHZ.  Values below
//! \b SYSCTL_XTAL_3_57MHZ are not valid when the PLL is in operation.  On
//! Sandstorm- and Fury-class devices, values above \b SYSCTL_XTAL_8_19MHZ are
//! not valid.
//!
//! The oscillator source is chosen with one of the following values:
//! \b SYSCTL_OSC_MAIN, \b SYSCTL_OSC_INT, \b SYSCTL_OSC_INT4,
//! \b SYSCTL_OSC_INT30, or \b SYSCTL_OSC_EXT32.  On Sandstorm-class devices,
//! \b SYSCTL_OSC_INT30 and \b SYSCTL_OSC_EXT32 are not valid.
//! \b SYSCTL_OSC_EXT32 is only available on devices with the hibernate module,
//! and then only when the hibernate module has been enabled.
//!
//! The internal and main oscillators are disabled with the
//! \b SYSCTL_INT_OSC_DIS and \b SYSCTL_MAIN_OSC_DIS flags, respectively.
//! The external oscillator must be enabled in order to use an external clock
//! source.  Note that attempts to disable the oscillator used to clock the
//! device will be prevented by the hardware.
//!
//! To clock the system from an external source (such as an external crystal
//! oscillator), use \b SYSCTL_USE_OSC \b | \b SYSCTL_OSC_MAIN.  To clock the
//! system from the main oscillator, use \b SYSCTL_USE_OSC \b |
//! \b SYSCTL_OSC_MAIN.  To clock the system from the PLL, use
//! \b SYSCTL_USE_PLL \b | \b SYSCTL_OSC_MAIN, and select the appropriate
//! crystal with one of the \b SYSCTL_XTAL_xxx values.
//!
//! \note If selecting the PLL as the system clock source (that is, via
//! \b SYSCTL_USE_PLL), this function will poll the PLL lock interrupt to
//! determine when the PLL has locked.  If an interrupt handler for the
//! system control interrupt is in place, and it responds to and clears the
//! PLL lock interrupt, this function will delay until its timeout has occurred
//! instead of completing as soon as PLL lock is achieved.
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlClockSet(unsigned long ulConfig)
{
    unsigned long ulDelay, ulRCC, ulRCC2;

    //
    // See if this is a Sandstorm-class device and clocking features from newer
    // devices were requested.
    //
    if(CLASS_IS_SANDSTORM && (ulConfig & SYSCTL_RCC2_USERCC2))
    {
        //
        // Return without changing the clocking since the requested
        // configuration can not be achieved.
        //
        return;
    }

    //
    // Get the current value of the RCC and RCC2 registers.  If using a
    // Sandstorm-class device, the RCC2 register will read back as zero and the
    // writes to it from within this function will be ignored.
    //
    ulRCC = xHWREG(SYSCTL_RCC);
    ulRCC2 = xHWREG(SYSCTL_RCC2);

    //
    // Bypass the PLL and system clock dividers for now.
    //
    ulRCC |= SYSCTL_RCC_BYPASS;
    ulRCC &= ~(SYSCTL_RCC_USESYSDIV);
    ulRCC2 |= SYSCTL_RCC2_BYPASS2;

    //
    // Write the new RCC value.
    //
    xHWREG(SYSCTL_RCC) = ulRCC;
    xHWREG(SYSCTL_RCC2) = ulRCC2;

    //
    // See if either oscillator needs to be enabled.
    //
    if(((ulRCC & SYSCTL_RCC_IOSCDIS) && !(ulConfig & SYSCTL_RCC_IOSCDIS)) ||
       ((ulRCC & SYSCTL_RCC_MOSCDIS) && !(ulConfig & SYSCTL_RCC_MOSCDIS)))
    {
        //
        // Make sure that the required oscillators are enabled.  For now, the
        // previously enabled oscillators must be enabled along with the newly
        // requested oscillators.
        //
        ulRCC &= (~(SYSCTL_RCC_IOSCDIS | SYSCTL_RCC_MOSCDIS) |
                  (ulConfig & (SYSCTL_RCC_IOSCDIS | SYSCTL_RCC_MOSCDIS)));

        //
        // Write the new RCC value.
        //
        xHWREG(SYSCTL_RCC) = ulRCC;

        //
        // Wait for a bit, giving the oscillator time to stabilize.  The number
        // of iterations is adjusted based on the current clock source; a
        // smaller number of iterations is required for slower clock rates.
        //
        if(((ulRCC2 & SYSCTL_RCC2_USERCC2) &&
            (((ulRCC2 & SYSCTL_RCC2_OSCSRC2_M) == SYSCTL_RCC2_OSCSRC2_30) ||
             ((ulRCC2 & SYSCTL_RCC2_OSCSRC2_M) == SYSCTL_RCC2_OSCSRC2_32))) ||
           (!(ulRCC2 & SYSCTL_RCC2_USERCC2) &&
            ((ulRCC & SYSCTL_RCC_OSCSRC_M) == SYSCTL_RCC_OSCSRC_30)))
        {
            //
            // Delay for 4096 iterations.
            //
            SysCtlDelay(4096);
        }
        else
        {
            //
            // Delay for 524,288 iterations.
            //
            SysCtlDelay(524288);
        }
    }

    //
    // Set the new crystal value, oscillator source, and PLL configuration.
    // Since the OSCSRC2 field in RCC2 overlaps the XTAL field in RCC, the
    // OSCSRC field has a special encoding within ulConfig to avoid the
    // overlap.
    //
    ulRCC &= ~(SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M |
               SYSCTL_RCC_PWRDN | SYSCTL_RCC_OEN);
    ulRCC |= ulConfig & (SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M |
                         SYSCTL_RCC_PWRDN | SYSCTL_RCC_OEN);
    ulRCC2 &= ~(SYSCTL_RCC2_USERCC2 | SYSCTL_RCC2_OSCSRC2_M |
                SYSCTL_RCC2_PWRDN2);
    ulRCC2 |= ulConfig & (SYSCTL_RCC2_USERCC2 | SYSCTL_RCC_OSCSRC_M |
                          SYSCTL_RCC2_PWRDN2);
    ulRCC2 |= (ulConfig & 0x00000008) << 3;

    //
    // Clear the PLL lock interrupt.
    //
    xHWREG(SYSCTL_MISC) = SYSCTL_INT_PLL_LOCK;

    //
    // Write the new RCC value.
    //
    if(ulRCC2 & SYSCTL_RCC2_USERCC2)
    {
        xHWREG(SYSCTL_RCC2) = ulRCC2;
        xHWREG(SYSCTL_RCC) = ulRCC;
    }
    else
    {
        xHWREG(SYSCTL_RCC) = ulRCC;
        xHWREG(SYSCTL_RCC2) = ulRCC2;
    }

    //
    // Wait for a bit so that new crystal value and oscillator source can take
    // effect.
    //
    SysCtlDelay(16);

    //
    // Set the requested system divider and disable the appropriate
    // oscillators.  This will not get written immediately.
    //
    ulRCC &= ~(SYSCTL_RCC_SYSDIV_M | SYSCTL_RCC_USESYSDIV |
               SYSCTL_RCC_IOSCDIS | SYSCTL_RCC_MOSCDIS);
    ulRCC |= ulConfig & (SYSCTL_RCC_SYSDIV_M | SYSCTL_RCC_USESYSDIV |
                         SYSCTL_RCC_IOSCDIS | SYSCTL_RCC_MOSCDIS);
    ulRCC2 &= ~(SYSCTL_RCC2_SYSDIV2_M);
    ulRCC2 |= ulConfig & SYSCTL_RCC2_SYSDIV2_M;
    if(ulConfig & SYSCTL_RCC2_DIV400)
    {
        ulRCC |= SYSCTL_RCC_USESYSDIV;
        ulRCC2 &= ~(SYSCTL_RCC_USESYSDIV);
        ulRCC2 |= ulConfig & (SYSCTL_RCC2_DIV400 | SYSCTL_RCC2_SYSDIV2LSB);
    }
    else
    {
        ulRCC2 &= ~(SYSCTL_RCC2_DIV400);
    }

    //
    // See if the PLL output is being used to clock the system.
    //
    if(!(ulConfig & SYSCTL_RCC_BYPASS))
    {
        //
        // Wait until the PLL has locked.
        //
        for(ulDelay = 32768; ulDelay > 0; ulDelay--)
        {
            if(xHWREG(SYSCTL_RIS) & SYSCTL_INT_PLL_LOCK)
            {
                break;
            }
        }

        //
        // Enable use of the PLL.
        //
        ulRCC &= ~(SYSCTL_RCC_BYPASS);
        ulRCC2 &= ~(SYSCTL_RCC2_BYPASS2);
    }

    //
    // Write the final RCC value.
    //
    xHWREG(SYSCTL_RCC) = ulRCC;
    xHWREG(SYSCTL_RCC2) = ulRCC2;

    //
    // Delay for a little bit so that the system divider takes effect.
    //
    SysCtlDelay(16);
}

//*****************************************************************************
//
//! \brief Gets the processor clock rate.
//!
//! This function determines the clock rate of the processor clock.  This is
//! also the clock rate of all the peripheral modules (with the exception of
//! PWM, which has its own clock divider).
//!
//! \note This will not return accurate results if SysCtlClockSet() has not
//! been called to configure the clocking of the device, or if the device is
//! directly clocked from a crystal (or a clock source) that is not one of the
//! supported crystal frequencies.  In the later case, this function should be
//! modified to directly return the correct system clock rate.
//!
//! \return The processor clock rate.
//
//*****************************************************************************
unsigned long
SysCtlClockGet(void)
{
    unsigned long ulRCC, ulMSEL, ulNSEL, ulDiv, ulClk;

    //
    // Get the base clock rate.
    //
    ulRCC = xHWREG(SYSCTL_CLKSRCSEL) & SYSCTL_CLKSRCSEL_M;

    switch(ulRCC & SYSCTL_CLKSRCSEL_CLKSRC_M)
    {
        //
        // The main oscillator is the clock source.  Determine its rate from
        // the crystal setting field.
        //
        case SYSCTL_CLKSRCSEL_MAINOSC:
        {
            ulClk = s_ulExtClockMHz * 1000000;
            break;
        }

        //
        // The internal oscillator is the source clock.
        //
        case SYSCTL_CLKSRCSEL_INTRC:
        {

            ulClk = 4000000;
            break;
        }

        //
        // The 32 KHz clock from the hibernate module is the source clock.
        //
        case SYSCTL_CLKSRCSEL_OSC_32:
        {
            ulClk = 32768;
            break;
        }

        //
        // An unknown setting, so return a zero clock (that is, an unknown
        // clock rate).
        //
        default:
        {
            return(0);
        }
    }

    //
    // See if the PLL is being used.
    //
    if(xHWREG(SYSCTL_PLL0CON) & SYSCTL_PLL0CON_PLLE0)
    {
        //
        // Get the PLL configuration.
        //
        ulMSEL = (xHWREG(SYSCTL_PLL0STAT) & SYSCTL_PLL0STAT_MSEL0_M) >> SYSCTL_PLL0STAT_MSEL0_S;
        ulNSEL = (xHWREG(SYSCTL_PLL0STAT) & SYSCTL_PLL0STAT_NSEL0_M) >> SYSCTL_PLL0STAT_NSEL0_S;

        ulClk = (2 * (ulMSEL + 1) * ulClk) / (ulNSEL + 1);
    }

    ulDiv = xHWREG(SYSCTL_CLKCFG);

    ulClk = ulClk / (ulDiv + 1);

    //
    // Return the computed clock rate.
    //
    return(ulClk);
}

//*****************************************************************************
//
//! \brief Set clock source of a peripheral and peripheral divide.
//!
//! \param ulPeripheralSrc is the peripheral clock source to set.
//! \param ulDivide is the peripheral clock divide to set.
//!
//! Peripherals clock source are seted with this function.
//!
//! The \e ulPeripheralSrc parameter must be only one of the following values:
//! \ref xSysCtl_Peripheral_Src_Clk.
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlPeripheralClockSourceSet(unsigned long ulPeripheralSrc,
                                unsigned long ulDivide)
{
    //
    // Check the arguments.
    //
    xASSERT(SysCtlPeripheralValid(ulPeripheral));
    xASSERT((ulDivide == SYSCTL_SYSDIV_1) ||
            (ulDivide == SYSCTL_SYSDIV_2) ||
            (ulDivide == SYSCTL_SYSDIV_4) ||
            (ulDivide == SYSCTL_SYSDIV_6) ||
            (ulDivide == SYSCTL_SYSDIV_8));
    xASSERT(((ulPeripheralSrc ==SYSCTL_PERIPH_CAN1) ||
             (ulPeripheralSrc == SYSCTL_PERIPH_CAN2)) &&
            (ulDivide != SYSCTL_SYSDIV_8));

    //
    // Set the peripheral clock source
    //
    xHWREG(SYSCTL_RCC) = ((xHWREG(SYSCTL_RCC) &
                          ~(SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_M)) |
                         ulConfig);
}

//*****************************************************************************
//
//! \brief Gets the current PWM clock configuration.
//!
//! This function returns the current PWM clock configuration.
//!
//! \return Returns the current PWM clock configuration; will be one of
//! \b SYSCTL_PWMDIV_1, \b SYSCTL_PWMDIV_2, \b SYSCTL_PWMDIV_4,
//! \b SYSCTL_PWMDIV_8, \b SYSCTL_PWMDIV_16, \b SYSCTL_PWMDIV_32, or
//! \b SYSCTL_PWMDIV_64.
//
//*****************************************************************************
unsigned long
SysCtlPWMClockGet(void)
{
    //
    // Check that there is a PWM block on this part.
    //
    xASSERT(xHWREG(SYSCTL_DC1) & SYSCTL_DC1_PWM);

    //
    // Return the current PWM clock configuration.  Make sure that
    // SYSCTL_PWMDIV_1 is returned in all cases where the divider is disabled.
    //
    if(!(xHWREG(SYSCTL_RCC) & SYSCTL_RCC_USEPWMDIV))
    {
        //
        // The divider is not active so reflect this in the value we return.
        //
        return(SYSCTL_PWMDIV_1);
    }
    else
    {
        //
        // The divider is active so directly return the masked register value.
        //
        return(xHWREG(SYSCTL_RCC) &
               (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_M));
    }
}

//*****************************************************************************
//
//! \brief Enables a GPIO peripheral for access from the AHB.
//!
//! \param ulGPIOPeripheral is the GPIO peripheral to enable.
//!
//! This function is used to enable the specified GPIO peripheral to be
//! accessed from the Advanced Host Bus (AHB) instead of the legacy Advanced
//! Peripheral Bus (APB).  When a GPIO peripheral is enabled for AHB access,
//! the \b _AHB_BASE form of the base address should be used for GPIO
//! functions.  For example, instead of using \b GPIO_PORTA_BASE as the base
//! address for GPIO functions, use \b GPIO_PORTA_AHB_BASE instead.
//!
//! The \e ulGPIOPeripheral argument must be only one of the following values:
//! \b SYSCTL_PERIPH_GPIOA, \b SYSCTL_PERIPH_GPIOB, \b SYSCTL_PERIPH_GPIOC,
//! \b SYSCTL_PERIPH_GPIOD, \b SYSCTL_PERIPH_GPIOE, \b SYSCTL_PERIPH_GPIOF,
//! \b SYSCTL_PERIPH_GPIOG, or \b SYSCTL_PERIPH_GPIOH.
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlGPIOAHBEnable(unsigned long ulGPIOPeripheral)
{
    //
    // Check the arguments.
    //
    xASSERT((ulGPIOPeripheral == SYSCTL_PERIPH_GPIOA) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOB) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOC) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOD) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOE) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOF) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOG) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOH) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOJ));

    //
    // Enable this GPIO for AHB access.
    //
    xHWREG(SYSCTL_GPIOHBCTL) |= ulGPIOPeripheral & 0xFFFF;
}

//*****************************************************************************
//
//! \brief Disables a GPIO peripheral for access from the AHB.
//!
//! \param ulGPIOPeripheral is the GPIO peripheral to disable.
//!
//! This function disables the specified GPIO peripheral for access from the
//! Advanced Host Bus (AHB).  Once disabled, the GPIO peripheral is accessed
//! from the legacy Advanced Peripheral Bus (AHB).
//!
//! The \b ulGPIOPeripheral argument must be only one of the following values:
//! \b SYSCTL_PERIPH_GPIOA, \b SYSCTL_PERIPH_GPIOB, \b SYSCTL_PERIPH_GPIOC,
//! \b SYSCTL_PERIPH_GPIOD, \b SYSCTL_PERIPH_GPIOE, \b SYSCTL_PERIPH_GPIOF,
//! \b SYSCTL_PERIPH_GPIOG, or \b SYSCTL_PERIPH_GPIOH.
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlGPIOAHBDisable(unsigned long ulGPIOPeripheral)
{
    //
    // Check the arguments.
    //
    xASSERT((ulGPIOPeripheral == SYSCTL_PERIPH_GPIOA) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOB) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOC) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOD) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOE) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOF) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOG) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOH) ||
           (ulGPIOPeripheral == SYSCTL_PERIPH_GPIOJ));

    //
    // Disable this GPIO for AHB access.
    //
    xHWREG(SYSCTL_GPIOHBCTL) &= ~(ulGPIOPeripheral & 0xFFFF);
}

//*****************************************************************************
//
//! \brief Powers up the USB PLL.
//!
//! This function will enable the USB controller's PLL which is used by it's
//! physical layer.  This call is necessary before connecting to any external
//! devices.
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlUSBPLLEnable(void)
{
    //
    // Turn on the USB PLL.
    //
    xHWREG(SYSCTL_RCC2) &= ~SYSCTL_RCC2_USBPWRDN;
}

//*****************************************************************************
//
//! \brief Powers down the USB PLL.
//!
//! This function will disable the USB controller's PLL which is used by it's
//! physical layer.  The USB registers are still accessible, but the physical
//! layer will no longer function.
//!
//! \return None.
//
//*****************************************************************************
void
SysCtlUSBPLLDisable(void)
{
    //
    // Turn of USB PLL.
    //
    xHWREG(SYSCTL_RCC2) |= SYSCTL_RCC2_USBPWRDN;
}

//*****************************************************************************
//
//! \brief Sets the MCLK frequency provided to the I2S module.
//!
//! \param ulInputClock is the input clock to the MCLK divider.  If this is
//! zero, the value is computed from the current PLL configuration.
//! \param ulMClk is the desired MCLK frequency.  If this is zero, MCLK output
//! is disabled.
//!
//! This function sets the dividers to provide MCLK to the I2S module.  A MCLK
//! divider will be chosen that produces the MCLK frequency that is the closest
//! possible to the requested frequency, which may be above or below the
//! requested frequency.
//!
//! The actual MCLK frequency will be returned.  It is the responsibility of
//! the application to determine if the selected MCLK is acceptable; in general
//! the human ear can not discern the frequency difference if it is within 0.3%
//! of the desired frequency (though there is a very small percentage of the
//! population that can discern lower frequency deviations).
//!
//! \return Returns the actual MCLK frequency.
//
//*****************************************************************************
unsigned long
SysCtlI2SMClkSet(unsigned long ulInputClock, unsigned long ulMClk)
{
    unsigned long ulDivInt, ulDivFrac, ulPLL;

    //
    // See if the I2S MCLK should be disabled.
    //
    if(ulMClk == 0)
    {
        //
        // Disable the I2S MCLK and return.
        //
        xHWREG(SYSCTL_I2SMCLKCFG) = 0;
        return(0);
    }

    //
    // See if the input clock was specified.
    //
    if(ulInputClock == 0)
    {
        //
        // The input clock was not specified, so compute the output frequency
        // of the PLL.  Get the current PLL configuration.
        //
        ulPLL = xHWREG(SYSCTL_PLLCFG);

        //
        // Get the frequency of the crystal in use.
        //
        ulInputClock = g_pulXtals[(xHWREG(SYSCTL_RCC) & SYSCTL_RCC_XTAL_M) >>
                                  SYSCTL_RCC_XTAL_S];

        //
        // Calculate the PLL output frequency.
        //
        ulInputClock = ((ulInputClock * ((ulPLL & SYSCTL_PLLCFG_F_M) >>
                                         SYSCTL_PLLCFG_F_S)) /
                        ((((ulPLL & SYSCTL_PLLCFG_R_M) >>
                           SYSCTL_PLLCFG_R_S) + 1)));

        //
        // See if the optional output divide by 2 is being used.
        //
        if(ulPLL & SYSCTL_PLLCFG_OD_2)
        {
            ulInputClock /= 2;
        }

        //
        // See if the optional output divide by 4 is being used.
        //
        if(ulPLL & SYSCTL_PLLCFG_OD_4)
        {
            ulInputClock /= 4;
        }
    }

    //
    // Verify that the requested MCLK frequency is attainable.
    //
    xASSERT(ulMClk < ulInputClock);

    //
    // Add a rounding factor to the input clock, so that the MCLK frequency
    // that is closest to the desire value is selected.
    //
    ulInputClock += (ulMClk / 32) - 1;

    //
    // Compute the integer portion of the MCLK divider.
    //
    ulDivInt = ulInputClock / ulMClk;

    //
    // If the divisor is too large, then simply use the maximum divisor.
    //
    if(CLASS_IS_TEMPEST && REVISION_IS_B1 && (ulDivInt > 255))
    {
        ulDivInt = 255;
        ulDivFrac = 15;
    }
    else if(ulDivInt > 1023)
    {
        ulDivInt = 1023;
        ulDivFrac = 15;
    }
    else
    {
        //
        // Compute the fractional portion of the MCLK divider.
        //
        ulDivFrac = ((ulInputClock - (ulDivInt * ulMClk)) * 16) / ulMClk;
    }

    //
    // Set the divisor for the Tx and Rx MCLK generators and enable the clocks.
    //
    xHWREG(SYSCTL_I2SMCLKCFG) = (SYSCTL_I2SMCLKCFG_RXEN |
                                (ulDivInt << SYSCTL_I2SMCLKCFG_RXI_S) |
                                (ulDivFrac << SYSCTL_I2SMCLKCFG_RXF_S) |
                                SYSCTL_I2SMCLKCFG_TXEN |
                                (ulDivInt << SYSCTL_I2SMCLKCFG_TXI_S) |
                                (ulDivFrac << SYSCTL_I2SMCLKCFG_TXF_S));

    //
    // Return the actual MCLK frequency.
    //
    ulInputClock -= (ulMClk / 32) - 1;
    ulDivInt = (ulDivInt * 16) + ulDivFrac;
    ulMClk = (ulInputClock / ulDivInt) * 16;
    ulMClk += ((ulInputClock - ((ulMClk / 16) * ulDivInt)) * 16) / ulDivInt;
    return(ulMClk);
}

