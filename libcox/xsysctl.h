//*****************************************************************************
//
//! \file xsysctl.h
//! \brief Prototypes for the system control driver.
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

#ifndef __xSYSCTL_H__
#define __xSYSCTL_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup CoX_Peripheral_Lib
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup SysCtl
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSysCtl
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSysCtl_Peripheral_ID xSysCtl Peripheral ID
//! \brief Values that show xSysCtl Peripheral ID
//!
//! \section xSysCtl_Peripheral_SID 1. Where to use this group
//! Values that can be passed to the
//! xSysCtlPeripheralPresent(), xSysCtlPeripheralEnable(),
//! xSysCtlPeripheralDisable(), and xSysCtlPeripheralReset() APIs as the
//! ulPeripheral parameter.
//!
//! \section xSysCtl_Peripheral_ID_CoX 2.CoX Port Details
//! \verbatim
//! +--------------------------+----------------+--------------------------+
//! |SysCtl Peripheral ID      |       CoX      |        Stellaris         |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_ACMPn      |  Non-Mandatory |   xSYSCTL_PERIPH_ACMP0   |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_ADCn       |    Mandatory   |    xSYSCTL_PERIPH_ADC0   |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_DMA        |  Non-Mandatory |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_GPIOn      |    Mandatory   |   xSYSCTL_PERIPH_GPIOA   |
//! |                          |                |--------------------------|
//! |                          |                |   xSYSCTL_PERIPH_GPIOB   |
//! |                          |                |--------------------------|
//! |                          |                |   xSYSCTL_PERIPH_GPIOC   |
//! |                          |                |--------------------------|
//! |                          |                |   xSYSCTL_PERIPH_GPIOD   |
//! |                          |                |--------------------------|
//! |                          |                |   xSYSCTL_PERIPH_GPIOE   |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_I2Cn       |  Non-Mandatory |    xSYSCTL_PERIPH_I2C0   |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_PERIPH_I2C1   |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_PWMn       |  Non-Mandatory |    xSYSCTL_PERIPH_PWM0   |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_PERIPH_PWM1   |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_PERIPH_PWM2   |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_PERIPH_PWM3   |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_RTC        |    Mandatory   |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_SPIn       |    Mandatory   |    xSYSCTL_PERIPH_SSI0   |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_PERIPH_SPI1   |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_PERIPH_SPI2   |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_PERIPH_SPI3   |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_TIMERn     |    Mandatory   |   xSYSCTL_PERIPH_TIMER0  |
//! |                          |                |--------------------------|
//! |                          |                |   xSYSCTL_PERIPH_TIMER1  |
//! |                          |                |--------------------------|
//! |                          |                |   xSYSCTL_PERIPH_TIMER2  |
//! |                          |                |--------------------------|
//! |                          |                |   xSYSCTL_PERIPH_TIMER3  |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_UARTn      |    Mandatory   |   xSYSCTL_PERIPH_UART0   |
//! |                          |                |--------------------------|
//! |                          |                |   xSYSCTL_PERIPH_UART1   |
//! |                          |                |--------------------------|
//! |                          |                |   xSYSCTL_PERIPH_UART2   |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_WDOG       |    Mandatory   |             Y            |
//! +--------------------------+----------------+--------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

#define xSYSCTL_PERIPH_ADC0     SYSCTL_PERIPH_ADC
#define xSYSCTL_PERIPH_DMA      SYSCTL_PERIPH_DMA
#define xSYSCTL_PERIPH_GPIOA    SYSCTL_PERIPH_GPIO
#define xSYSCTL_PERIPH_GPIOB    SYSCTL_PERIPH_GPIO
#define xSYSCTL_PERIPH_GPIOC    SYSCTL_PERIPH_GPIO
#define xSYSCTL_PERIPH_GPIOD    SYSCTL_PERIPH_GPIO
#define xSYSCTL_PERIPH_GPIOE    SYSCTL_PERIPH_GPIO
#define xSYSCTL_PERIPH_GPIOF    SYSCTL_PERIPH_GPIO
#define xSYSCTL_PERIPH_I2C0     SYSCTL_PERIPH_I2C0
#define xSYSCTL_PERIPH_I2C1     SYSCTL_PERIPH_I2C1
#define xSYSCTL_PERIPH_I2C2     SYSCTL_PERIPH_I2C2
#define xSYSCTL_PERIPH_I2S      SYSCTL_PERIPH_I2S
// PWM add in future
#define xSYSCTL_PERIPH_RTC      SYSCTL_PERIPH_RTC
#define xSYSCTL_PERIPH_SPI0     SYSCTL_PERIPH_SSI0
#define xSYSCTL_PERIPH_SPI1     SYSCTL_PERIPH_SSI1
#define xSYSCTL_PERIPH_TIMER0   SYSCTL_PERIPH_TIMER0
#define xSYSCTL_PERIPH_TIMER1   SYSCTL_PERIPH_TIMER1
#define xSYSCTL_PERIPH_TIMER2   SYSCTL_PERIPH_TIMER2
#define xSYSCTL_PERIPH_TIMER3   SYSCTL_PERIPH_TIMER3
#define xSYSCTL_PERIPH_UART0    SYSCTL_PERIPH_UART0
#define xSYSCTL_PERIPH_UART1    SYSCTL_PERIPH_UART1
#define xSYSCTL_PERIPH_UART2    SYSCTL_PERIPH_UART2
#define xSYSCTL_PERIPH_UART3    SYSCTL_PERIPH_UART3
#define xSYSCTL_PERIPH_PWM      SYSCTL_PERIPH_PWM
#define xSYSCTL_PERIPH_MPWM     SYSCTL_PERIPH_MPWM

//*****************************************************************************
//
//! @}
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup xSysCtl_Clock_Set_Config xSysCtl Clock Set Configuration
//! \brief Values that show xSysCtl Clock Set Configuration
//!
//! \section xSysCtl_Clock_Set_SConfig 1. Where to use this group
//! Values that can be passed to the xSysCtlClockSet() API as the
//! \b ulConfig parameter.
//!
//! \section xSysCtl_Clock_Set_SConfig 2. ulConfig parameter description
//! The ulConfig parameter is the logical OR of several different values,
//! many of which are grouped into sets where only one can be chosen.
//! ulConfig contains the external and internal crystal, main oscillators
//! and PLL disabled options.
//!
//! \section xSysCtl_Clock_Set_Config_CoX 3.CoX Port Details
//! \verbatim
//! +--------------------------+----------------+--------------------------+
//! |SysCtl Clock Set Configs  |       CoX      |         Stellaris        |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_OSC_MAIN          |    Mandatory   |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_OSC_INT           |    Mandatory   |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_OSC_INTSL         |  Non-Mandatory |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_OSC_EXTSL         |  Non-Mandatory |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_XTAL_nMHZ         |  Non-Mandatory |     xSYSCTL_XTAL_1MHZ    |
//! |                          |                |--------------------------|
//! |                          |                |     xSYSCTL_XTAL_2MHZ    |
//! |                          |                |--------------------------|
//! |                          |                |     xSYSCTL_XTAL_4MHZ    |
//! |                          |                |--------------------------|
//! |                          |                |     xSYSCTL_XTAL_5MHZ    |
//! |                          |                |--------------------------|
//! |                          |                |     xSYSCTL_XTAL_8MHZ    |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_XTAL_10MHZ    |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_XTAL_12MHZ    |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_XTAL_16MHZ    |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_INT_nMHZ          |  Non-Mandatory |     xSYSCTL_INT_16MHZ    |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_INTSL_nKHZ/HZ     |  Non-Mandatory |    xSYSCTL_INTSL_30KHZ   |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_XTALSL_nHZ        |  Non-Mandatory |  xSYSCTL_XTALSL_32768HZ  |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_INT_OSC_DIS       |  Non-Mandatory |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_MAIN_OSC_DIS      |  Non-Mandatory |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PLL_PWRDN         |  Non-Mandatory |             Y            |
//! +--------------------------+----------------+--------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

#define xSYSCTL_OSC_MAIN        0x00000000  // Osc source is main osc
#define xSYSCTL_OSC_INT         0x00000010  // Osc source is int. osc
#define xSYSCTL_OSC_INTSL       0x00000030  // Osc source is int. 30 KHz
#define xSYSCTL_OSC_EXTSL       0x80000038  // Osc source is ext. 32 KHz

//
//! \brief XTAL Select
//
#define xSYSCTL_XTAL_1MHZ       0x00000000  // External main clock crystal is 1MHz.
#define xSYSCTL_XTAL_2MHZ       0x00000080  // External main clock crystal is 2MHz.
#define xSYSCTL_XTAL_4MHZ       0x00000180  // External main clock crystal is 4MHz.
#define xSYSCTL_XTAL_5MHZ       0x00000240  // External main clock crystal is 5MHz.
#define xSYSCTL_XTAL_6MHZ       0x000002C0  // External main clock crystal is 6MHz.
#define xSYSCTL_XTAL_8MHZ       0x00000380  // External main clock crystal is 8MHz.
#define xSYSCTL_XTAL_10MHZ      0x00000400  // External main clock crystal is 10MHz.
#define xSYSCTL_XTAL_12MHZ      0x00000440  // External main clock crystal is 12MHz.
#define xSYSCTL_XTAL_16MHZ      0x00000540  // External main clock crystal is 16MHz.

#define xSYSCTL_INT_16MHZ       0x00000000  // Internal main clock is 16MHz

#define xSYSCTL_INTSL_30KHZ     0x00000000  // Internal slow clock  is 30KHz.
#define xSYSCTL_XTALSL_32768HZ  0x00000000  // External slow clock crystal is 32KHz.

#define xSYSCTL_INT_OSC_DIS     0x00000002  // Disable internal oscillator
#define xSYSCTL_MAIN_OSC_DIS    0x00000001  // Disable main oscillator
#define xSYSCTL_PLL_PWRDN       0x00002000  // Disable main oscillator

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSysCtl_Exported_APIs xSysCtl API
//! \brief xSysCtl API Reference
//!
//! \section xSysCtl_Exported_APIs_Port CoX Port Details
//! \verbatim
//! +--------------------------------+----------------+-----------+
//! |xSysCtl API                     |       CoX      | Stellaris |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlPeripheralReset          |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlPeripheralEnable         |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlPeripheralDisable        |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlPeripheralReset2         |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlPeripheralEnable2        |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlPeripheralDisable2       |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlPeripheraIntNumGet       |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlClockSet                 |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlClockGet                 |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlDelay                    |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlReset                    |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlSleep                    |    Mandatory   |     Y     |
//! +--------------------------------+----------------+-----------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief Performs a software reset of a peripheral.
//!
//! \param ulPeripheral is the peripheral to reset.
//!
//! This function performs a software reset of the specified peripheral.  An
//! individual peripheral reset signal is asserted for a brief period and then
//! deasserted, returning the internal state of the peripheral to its reset
//! condition.
//!
//! The \e ulPeripheral parameter must be only one of the following values:
//! such as \b xSYSCTL_PERIPH_ADC0, \b xSYSCTL_PERIPH_UART0,
//! Details please reference to \ref xSysCtl_Peripheral_ID_CoX.
//!
//! \return None.
//
//*****************************************************************************
#define xSysCtlPeripheralReset(ulPeripheral)                                  \
        SysCtlPeripheralReset(ulPeripheral)

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
//! such as \b xSYSCTL_PERIPH_ADC0, \b xSYSCTL_PERIPH_UART0,
//! Details please reference to \ref xSysCtl_Peripheral_ID_CoX.
//!
//! \return None.
//
//*****************************************************************************
#define xSysCtlPeripheralEnable(ulPeripheral)                                 \
        SysCtlPeripheralEnable(ulPeripheral)

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
//! such as \b xSYSCTL_PERIPH_ADC0, \b xSYSCTL_PERIPH_UART0,
//! Details please reference to \ref xSysCtl_Peripheral_ID_CoX.
//!
//! \return None.
//
//*****************************************************************************

#define xSysCtlPeripheralDisable(ulPeripheral)                                \
        SysCtlPeripheralDisable(ulPeripheral)


extern void xSysCtlPeripheralEnable2(unsigned long ulPeripheralBase);
extern void xSysCtlPeripheralDisable2(unsigned long ulPeripheralBase);
extern void xSysCtlPeripheralReset2(unsigned long ulPeripheralBase);

extern unsigned long xSysCtlPeripheralIntNumGet(unsigned long ulPeripheralBase);
extern unsigned long xSysCtlPeripheralIdGet(unsigned long ulPeripheralBase);

extern void xSysCtlClockSet(unsigned long ulSysClk, unsigned long ulConfig);

//*****************************************************************************
//
//! \brief Gets the processor clock rate.
//!
//! This function determines the clock rate of the processor clock.  This is
//! also the clock rate of all the peripheral modules (with the exception of
//! PWM, which has its own clock divider).
//!
//! \return The processor clock rate.
//
//*****************************************************************************
#define xSysCtlClockGet()                                                     \
        SysCtlClockGet()

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
#define xSysCtlDelay(ulCount)                                                 \
        SysCtlDelay(ulCount)

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
#define xSysCtlReset()                                                        \
        SysCtlReset()

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
#define xSysCtlSleep()                                                        \
        SysCtlSleep()

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Stellaris_SysCtl
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Stellaris_SysCtl_Peripheral_ID
//! Values that can be passed to the
//! SysCtlPeripheralPresent(), SysCtlPeripheralEnable(),
//! SysCtlPeripheralDisable(), and SysCtlPeripheralReset() APIs as the
//! ulPeripheral parameter.  The peripherals in the fourth group (upper nibble
//! is 3) can only be used with the SysCtlPeripheralPresent() API.
//! @{
//
//*****************************************************************************

#define SYSCTL_PERIPH_TIMER0    0x00000002  // Timer 0
#define SYSCTL_PERIPH_TIMER1    0x00000004  // Timer 1
#define SYSCTL_PERIPH_UART0     0x00000008  // UART 0
#define SYSCTL_PERIPH_UART1     0x00000010  // UART 0
#define SYSCTL_PERIPH_PWM       0x00000040  // PWM
#define SYSCTL_PERIPH_I2C0      0x00000080  // I2C 0
#define SYSCTL_PERIPH_SPI       0x00000100  // SPI
#define SYSCTL_PERIPH_RTC       0x00000200  // RTC
#define SYSCTL_PERIPH_SSP1      0x00000400  // SSP 1
#define SYSCTL_PERIPH_ADC       0x00001000  // ADC
#define SYSCTL_PERIPH_CAN1      0x00002000  // CAN 1
#define SYSCTL_PERIPH_CAN2      0x00004000  // CAN 2
#define SYSCTL_PERIPH_GPIO      0x00008000  // GPIO
#define SYSCTL_PERIPH_RIT       0x00010000  // RIT
#define SYSCTL_PERIPH_PCMCPWM   0x00020000  // Motor Control PWM
#define SYSCTL_PERIPH_QEI       0x00040000  // QEI
#define SYSCTL_PERIPH_I2C1      0x00080000  // I2C 1
#define SYSCTL_PERIPH_SSP0      0x00200000  // SSP 0
#define SYSCTL_PERIPH_TIMER2    0x00400000  // Timer 2
#define SYSCTL_PERIPH_TIMER3    0x00800000  // Timer 3
#define SYSCTL_PERIPH_UART2     0x01000000  // UART 2
#define SYSCTL_PERIPH_UART3     0x02000000  // UART 3
#define SYSCTL_PERIPH_I2C2      0x04000000  // I2C 2
#define SYSCTL_PERIPH_I2S       0x08000000  // I2S
#define SYSCTL_PERIPH_DMA       0x20000000  // DMA
#define SYSCTL_PERIPH_ETH       0x40000000  // ETH
#define SYSCTL_PERIPH_USB       0x80000000  // USB

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Stellaris_SysCtl_Present_PIN_ID
//! The following are values that can be passed to the SysCtlPinPresent() API
//! as the ulPin parameter.
//! @{
//
//*****************************************************************************

#define SYSCTL_PIN_PWM0         0x00000001  // PWM0 pin
#define SYSCTL_PIN_PWM1         0x00000002  // PWM1 pin
#define SYSCTL_PIN_PWM2         0x00000004  // PWM2 pin
#define SYSCTL_PIN_PWM3         0x00000008  // PWM3 pin
#define SYSCTL_PIN_PWM4         0x00000010  // PWM4 pin
#define SYSCTL_PIN_PWM5         0x00000020  // PWM5 pin
#define SYSCTL_PIN_PWM6         0x00000040  // PWM6 pin
#define SYSCTL_PIN_PWM7         0x00000080  // PWM7 pin
#define SYSCTL_PIN_C0MINUS      0x00000040  // C0- pin
#define SYSCTL_PIN_C0PLUS       0x00000080  // C0+ pin
#define SYSCTL_PIN_C0O          0x00000100  // C0o pin
#define SYSCTL_PIN_C1MINUS      0x00000200  // C1- pin
#define SYSCTL_PIN_C1PLUS       0x00000400  // C1+ pin
#define SYSCTL_PIN_C1O          0x00000800  // C1o pin
#define SYSCTL_PIN_C2MINUS      0x00001000  // C2- pin
#define SYSCTL_PIN_C2PLUS       0x00002000  // C2+ pin
#define SYSCTL_PIN_C2O          0x00004000  // C2o pin
#define SYSCTL_PIN_MC_FAULT0    0x00008000  // MC0 Fault pin
#define SYSCTL_PIN_ADC0         0x00010000  // ADC0 pin
#define SYSCTL_PIN_ADC1         0x00020000  // ADC1 pin
#define SYSCTL_PIN_ADC2         0x00040000  // ADC2 pin
#define SYSCTL_PIN_ADC3         0x00080000  // ADC3 pin
#define SYSCTL_PIN_ADC4         0x00100000  // ADC4 pin
#define SYSCTL_PIN_ADC5         0x00200000  // ADC5 pin
#define SYSCTL_PIN_ADC6         0x00400000  // ADC6 pin
#define SYSCTL_PIN_ADC7         0x00800000  // ADC7 pin
#define SYSCTL_PIN_CCP0         0x01000000  // CCP0 pin
#define SYSCTL_PIN_CCP1         0x02000000  // CCP1 pin
#define SYSCTL_PIN_CCP2         0x04000000  // CCP2 pin
#define SYSCTL_PIN_CCP3         0x08000000  // CCP3 pin
#define SYSCTL_PIN_CCP4         0x10000000  // CCP4 pin
#define SYSCTL_PIN_CCP5         0x20000000  // CCP5 pin
#define SYSCTL_PIN_32KHZ        0x80000000  // 32kHz pin

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Stellaris_SysCtl_Int_Sources
//! The following are values that can be passed to the SysCtlIntEnable(),
//! SysCtlIntDisable(), and SysCtlIntClear() APIs, or returned in the bit mask
//! by the SysCtlIntStatus() API.
//! @{
//
//*****************************************************************************

#define SYSCTL_INT_MOSC_PUP     0x00000100  // MOSC power-up interrupt
#define SYSCTL_INT_USBPLL_LOCK  0x00000080  // USB PLL lock interrupt
#define SYSCTL_INT_PLL_LOCK     0x00000040  // PLL lock interrupt
#define SYSCTL_INT_CUR_LIMIT    0x00000020  // Current limit interrupt
#define SYSCTL_INT_IOSC_FAIL    0x00000010  // Internal oscillator failure int
#define SYSCTL_INT_MOSC_FAIL    0x00000008  // Main oscillator failure int
#define SYSCTL_INT_POR          0x00000004  // Power on reset interrupt
#define SYSCTL_INT_BOR          0x00000002  // Brown out interrupt
#define SYSCTL_INT_PLL_FAIL     0x00000001  // PLL failure interrupt

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Stellaris_SysCtl_Reset_Sources
//! The following are values that can be passed to the SysCtlResetCauseClear()
//! API or returned by the SysCtlResetCauseGet() API.
//! @{
//
//*****************************************************************************

#define SYSCTL_CAUSE_BOR        0x00000008  // Brown-out reset
#define SYSCTL_CAUSE_WDOG       0x00000004  // Watchdog reset
#define SYSCTL_CAUSE_EXT        0x00000002  // External reset
#define SYSCTL_CAUSE_POR        0x00000001  // Power on reset

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Stellaris_SysCtl_BrownOut_Config
//! The following are values that can be passed to the SysCtlBrownOutConfigSet()
//! API as the ulConfig parameter.
//! @{
//
//*****************************************************************************

#define SYSCTL_BOR_RESET        0x00000002  // Reset instead of interrupting
#define SYSCTL_BOR_RESAMPLE     0x00000001  // Resample BOR before asserting

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Stellaris_SysCtl_PWM_Clock_Config
//! The following are values that can be passed to the SysCtlPWMClockSet() API
//! as the ulConfig parameter, and can be returned by the SysCtlPWMClockGet()
// API.
//! @{
//
//*****************************************************************************

#define SYSCTL_PWMDIV_1         0x00000000  // PWM clock is processor clock /1
#define SYSCTL_PWMDIV_2         0x00100000  // PWM clock is processor clock /2
#define SYSCTL_PWMDIV_4         0x00120000  // PWM clock is processor clock /4
#define SYSCTL_PWMDIV_8         0x00140000  // PWM clock is processor clock /8
#define SYSCTL_PWMDIV_16        0x00160000  // PWM clock is processor clock /16
#define SYSCTL_PWMDIV_32        0x00180000  // PWM clock is processor clock /32
#define SYSCTL_PWMDIV_64        0x001A0000  // PWM clock is processor clock /64

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Stellaris_SysCtl_ADC_Speed
//! The following are values that can be passed to the SysCtlADCSpeedSet() API
//! as the ulSpeed parameter, and can be returned by the SyCtlADCSpeedGet()
//! API.
//! @{
//
//*****************************************************************************

#define SYSCTL_ADCSPEED_1MSPS   0x00000F00  // 1,000,000 samples per second
#define SYSCTL_ADCSPEED_500KSPS 0x00000A00  // 500,000 samples per second
#define SYSCTL_ADCSPEED_250KSPS 0x00000500  // 250,000 samples per second
#define SYSCTL_ADCSPEED_125KSPS 0x00000000  // 125,000 samples per second

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Stellaris_SysCtl_Clock_Config
//! The following are values that can be passed to the SysCtlClockSet() API as
//! the ulConfig parameter.
//! @{
//
//*****************************************************************************

#define SYSCTL_SYSDIV_1         0x07800000  // Processor clock is osc/pll /1
#define SYSCTL_SYSDIV_2         0x00C00000  // Processor clock is osc/pll /2
#define SYSCTL_SYSDIV_3         0x01400000  // Processor clock is osc/pll /3
#define SYSCTL_SYSDIV_4         0x01C00000  // Processor clock is osc/pll /4
#define SYSCTL_SYSDIV_5         0x02400000  // Processor clock is osc/pll /5
#define SYSCTL_SYSDIV_6         0x02C00000  // Processor clock is osc/pll /6
#define SYSCTL_SYSDIV_7         0x03400000  // Processor clock is osc/pll /7
#define SYSCTL_SYSDIV_8         0x03C00000  // Processor clock is osc/pll /8
#define SYSCTL_SYSDIV_9         0x04400000  // Processor clock is osc/pll /9
#define SYSCTL_SYSDIV_10        0x04C00000  // Processor clock is osc/pll /10
#define SYSCTL_SYSDIV_11        0x05400000  // Processor clock is osc/pll /11
#define SYSCTL_SYSDIV_12        0x05C00000  // Processor clock is osc/pll /12
#define SYSCTL_SYSDIV_13        0x06400000  // Processor clock is osc/pll /13
#define SYSCTL_SYSDIV_14        0x06C00000  // Processor clock is osc/pll /14
#define SYSCTL_SYSDIV_15        0x07400000  // Processor clock is osc/pll /15
#define SYSCTL_SYSDIV_16        0x07C00000  // Processor clock is osc/pll /16
#define SYSCTL_SYSDIV_17        0x88400000  // Processor clock is osc/pll /17
#define SYSCTL_SYSDIV_18        0x88C00000  // Processor clock is osc/pll /18
#define SYSCTL_SYSDIV_19        0x89400000  // Processor clock is osc/pll /19
#define SYSCTL_SYSDIV_20        0x89C00000  // Processor clock is osc/pll /20
#define SYSCTL_SYSDIV_21        0x8A400000  // Processor clock is osc/pll /21
#define SYSCTL_SYSDIV_22        0x8AC00000  // Processor clock is osc/pll /22
#define SYSCTL_SYSDIV_23        0x8B400000  // Processor clock is osc/pll /23
#define SYSCTL_SYSDIV_24        0x8BC00000  // Processor clock is osc/pll /24
#define SYSCTL_SYSDIV_25        0x8C400000  // Processor clock is osc/pll /25
#define SYSCTL_SYSDIV_26        0x8CC00000  // Processor clock is osc/pll /26
#define SYSCTL_SYSDIV_27        0x8D400000  // Processor clock is osc/pll /27
#define SYSCTL_SYSDIV_28        0x8DC00000  // Processor clock is osc/pll /28
#define SYSCTL_SYSDIV_29        0x8E400000  // Processor clock is osc/pll /29
#define SYSCTL_SYSDIV_30        0x8EC00000  // Processor clock is osc/pll /30
#define SYSCTL_SYSDIV_31        0x8F400000  // Processor clock is osc/pll /31
#define SYSCTL_SYSDIV_32        0x8FC00000  // Processor clock is osc/pll /32
#define SYSCTL_SYSDIV_33        0x90400000  // Processor clock is osc/pll /33
#define SYSCTL_SYSDIV_34        0x90C00000  // Processor clock is osc/pll /34
#define SYSCTL_SYSDIV_35        0x91400000  // Processor clock is osc/pll /35
#define SYSCTL_SYSDIV_36        0x91C00000  // Processor clock is osc/pll /36
#define SYSCTL_SYSDIV_37        0x92400000  // Processor clock is osc/pll /37
#define SYSCTL_SYSDIV_38        0x92C00000  // Processor clock is osc/pll /38
#define SYSCTL_SYSDIV_39        0x93400000  // Processor clock is osc/pll /39
#define SYSCTL_SYSDIV_40        0x93C00000  // Processor clock is osc/pll /40
#define SYSCTL_SYSDIV_41        0x94400000  // Processor clock is osc/pll /41
#define SYSCTL_SYSDIV_42        0x94C00000  // Processor clock is osc/pll /42
#define SYSCTL_SYSDIV_43        0x95400000  // Processor clock is osc/pll /43
#define SYSCTL_SYSDIV_44        0x95C00000  // Processor clock is osc/pll /44
#define SYSCTL_SYSDIV_45        0x96400000  // Processor clock is osc/pll /45
#define SYSCTL_SYSDIV_46        0x96C00000  // Processor clock is osc/pll /46
#define SYSCTL_SYSDIV_47        0x97400000  // Processor clock is osc/pll /47
#define SYSCTL_SYSDIV_48        0x97C00000  // Processor clock is osc/pll /48
#define SYSCTL_SYSDIV_49        0x98400000  // Processor clock is osc/pll /49
#define SYSCTL_SYSDIV_50        0x98C00000  // Processor clock is osc/pll /50
#define SYSCTL_SYSDIV_51        0x99400000  // Processor clock is osc/pll /51
#define SYSCTL_SYSDIV_52        0x99C00000  // Processor clock is osc/pll /52
#define SYSCTL_SYSDIV_53        0x9A400000  // Processor clock is osc/pll /53
#define SYSCTL_SYSDIV_54        0x9AC00000  // Processor clock is osc/pll /54
#define SYSCTL_SYSDIV_55        0x9B400000  // Processor clock is osc/pll /55
#define SYSCTL_SYSDIV_56        0x9BC00000  // Processor clock is osc/pll /56
#define SYSCTL_SYSDIV_57        0x9C400000  // Processor clock is osc/pll /57
#define SYSCTL_SYSDIV_58        0x9CC00000  // Processor clock is osc/pll /58
#define SYSCTL_SYSDIV_59        0x9D400000  // Processor clock is osc/pll /59
#define SYSCTL_SYSDIV_60        0x9DC00000  // Processor clock is osc/pll /60
#define SYSCTL_SYSDIV_61        0x9E400000  // Processor clock is osc/pll /61
#define SYSCTL_SYSDIV_62        0x9EC00000  // Processor clock is osc/pll /62
#define SYSCTL_SYSDIV_63        0x9F400000  // Processor clock is osc/pll /63
#define SYSCTL_SYSDIV_64        0x9FC00000  // Processor clock is osc/pll /64
#define SYSCTL_SYSDIV_2_5       0xC1000000  // Processor clock is pll / 2.5
#define SYSCTL_SYSDIV_3_5       0xC1800000  // Processor clock is pll / 3.5
#define SYSCTL_SYSDIV_4_5       0xC2000000  // Processor clock is pll / 4.5
#define SYSCTL_SYSDIV_5_5       0xC2800000  // Processor clock is pll / 5.5
#define SYSCTL_SYSDIV_6_5       0xC3000000  // Processor clock is pll / 6.5
#define SYSCTL_SYSDIV_7_5       0xC3800000  // Processor clock is pll / 7.5
#define SYSCTL_SYSDIV_8_5       0xC4000000  // Processor clock is pll / 8.5
#define SYSCTL_SYSDIV_9_5       0xC4800000  // Processor clock is pll / 9.5
#define SYSCTL_SYSDIV_10_5      0xC5000000  // Processor clock is pll / 10.5
#define SYSCTL_SYSDIV_11_5      0xC5800000  // Processor clock is pll / 11.5
#define SYSCTL_SYSDIV_12_5      0xC6000000  // Processor clock is pll / 12.5
#define SYSCTL_SYSDIV_13_5      0xC6800000  // Processor clock is pll / 13.5
#define SYSCTL_SYSDIV_14_5      0xC7000000  // Processor clock is pll / 14.5
#define SYSCTL_SYSDIV_15_5      0xC7800000  // Processor clock is pll / 15.5
#define SYSCTL_SYSDIV_16_5      0xC8000000  // Processor clock is pll / 16.5
#define SYSCTL_SYSDIV_17_5      0xC8800000  // Processor clock is pll / 17.5
#define SYSCTL_SYSDIV_18_5      0xC9000000  // Processor clock is pll / 18.5
#define SYSCTL_SYSDIV_19_5      0xC9800000  // Processor clock is pll / 19.5
#define SYSCTL_SYSDIV_20_5      0xCA000000  // Processor clock is pll / 20.5
#define SYSCTL_SYSDIV_21_5      0xCA800000  // Processor clock is pll / 21.5
#define SYSCTL_SYSDIV_22_5      0xCB000000  // Processor clock is pll / 22.5
#define SYSCTL_SYSDIV_23_5      0xCB800000  // Processor clock is pll / 23.5
#define SYSCTL_SYSDIV_24_5      0xCC000000  // Processor clock is pll / 24.5
#define SYSCTL_SYSDIV_25_5      0xCC800000  // Processor clock is pll / 25.5
#define SYSCTL_SYSDIV_26_5      0xCD000000  // Processor clock is pll / 26.5
#define SYSCTL_SYSDIV_27_5      0xCD800000  // Processor clock is pll / 27.5
#define SYSCTL_SYSDIV_28_5      0xCE000000  // Processor clock is pll / 28.5
#define SYSCTL_SYSDIV_29_5      0xCE800000  // Processor clock is pll / 29.5
#define SYSCTL_SYSDIV_30_5      0xCF000000  // Processor clock is pll / 30.5
#define SYSCTL_SYSDIV_31_5      0xCF800000  // Processor clock is pll / 31.5
#define SYSCTL_SYSDIV_32_5      0xD0000000  // Processor clock is pll / 32.5
#define SYSCTL_SYSDIV_33_5      0xD0800000  // Processor clock is pll / 33.5
#define SYSCTL_SYSDIV_34_5      0xD1000000  // Processor clock is pll / 34.5
#define SYSCTL_SYSDIV_35_5      0xD1800000  // Processor clock is pll / 35.5
#define SYSCTL_SYSDIV_36_5      0xD2000000  // Processor clock is pll / 36.5
#define SYSCTL_SYSDIV_37_5      0xD2800000  // Processor clock is pll / 37.5
#define SYSCTL_SYSDIV_38_5      0xD3000000  // Processor clock is pll / 38.5
#define SYSCTL_SYSDIV_39_5      0xD3800000  // Processor clock is pll / 39.5
#define SYSCTL_SYSDIV_40_5      0xD4000000  // Processor clock is pll / 40.5
#define SYSCTL_SYSDIV_41_5      0xD4800000  // Processor clock is pll / 41.5
#define SYSCTL_SYSDIV_42_5      0xD5000000  // Processor clock is pll / 42.5
#define SYSCTL_SYSDIV_43_5      0xD5800000  // Processor clock is pll / 43.5
#define SYSCTL_SYSDIV_44_5      0xD6000000  // Processor clock is pll / 44.5
#define SYSCTL_SYSDIV_45_5      0xD6800000  // Processor clock is pll / 45.5
#define SYSCTL_SYSDIV_46_5      0xD7000000  // Processor clock is pll / 46.5
#define SYSCTL_SYSDIV_47_5      0xD7800000  // Processor clock is pll / 47.5
#define SYSCTL_SYSDIV_48_5      0xD8000000  // Processor clock is pll / 48.5
#define SYSCTL_SYSDIV_49_5      0xD8800000  // Processor clock is pll / 49.5
#define SYSCTL_SYSDIV_50_5      0xD9000000  // Processor clock is pll / 50.5
#define SYSCTL_SYSDIV_51_5      0xD9800000  // Processor clock is pll / 51.5
#define SYSCTL_SYSDIV_52_5      0xDA000000  // Processor clock is pll / 52.5
#define SYSCTL_SYSDIV_53_5      0xDA800000  // Processor clock is pll / 53.5
#define SYSCTL_SYSDIV_54_5      0xDB000000  // Processor clock is pll / 54.5
#define SYSCTL_SYSDIV_55_5      0xDB800000  // Processor clock is pll / 55.5
#define SYSCTL_SYSDIV_56_5      0xDC000000  // Processor clock is pll / 56.5
#define SYSCTL_SYSDIV_57_5      0xDC800000  // Processor clock is pll / 57.5
#define SYSCTL_SYSDIV_58_5      0xDD000000  // Processor clock is pll / 58.5
#define SYSCTL_SYSDIV_59_5      0xDD800000  // Processor clock is pll / 59.5
#define SYSCTL_SYSDIV_60_5      0xDE000000  // Processor clock is pll / 60.5
#define SYSCTL_SYSDIV_61_5      0xDE800000  // Processor clock is pll / 61.5
#define SYSCTL_SYSDIV_62_5      0xDF000000  // Processor clock is pll / 62.5
#define SYSCTL_SYSDIV_63_5      0xDF800000  // Processor clock is pll / 63.5
#define SYSCTL_USE_PLL          0x00000000  // System clock is the PLL clock
#define SYSCTL_USE_OSC          0x00003800  // System clock is the osc clock
#define SYSCTL_XTAL_1MHZ        0x00000000  // External crystal is 1MHz
#define SYSCTL_XTAL_1_84MHZ     0x00000040  // External crystal is 1.8432MHz
#define SYSCTL_XTAL_2MHZ        0x00000080  // External crystal is 2MHz
#define SYSCTL_XTAL_2_45MHZ     0x000000C0  // External crystal is 2.4576MHz
#define SYSCTL_XTAL_3_57MHZ     0x00000100  // External crystal is 3.579545MHz
#define SYSCTL_XTAL_3_68MHZ     0x00000140  // External crystal is 3.6864MHz
#define SYSCTL_XTAL_4MHZ        0x00000180  // External crystal is 4MHz
#define SYSCTL_XTAL_4_09MHZ     0x000001C0  // External crystal is 4.096MHz
#define SYSCTL_XTAL_4_91MHZ     0x00000200  // External crystal is 4.9152MHz
#define SYSCTL_XTAL_5MHZ        0x00000240  // External crystal is 5MHz
#define SYSCTL_XTAL_5_12MHZ     0x00000280  // External crystal is 5.12MHz
#define SYSCTL_XTAL_6MHZ        0x000002C0  // External crystal is 6MHz
#define SYSCTL_XTAL_6_14MHZ     0x00000300  // External crystal is 6.144MHz
#define SYSCTL_XTAL_7_37MHZ     0x00000340  // External crystal is 7.3728MHz
#define SYSCTL_XTAL_8MHZ        0x00000380  // External crystal is 8MHz
#define SYSCTL_XTAL_8_19MHZ     0x000003C0  // External crystal is 8.192MHz
#define SYSCTL_XTAL_10MHZ       0x00000400  // External crystal is 10 MHz
#define SYSCTL_XTAL_12MHZ       0x00000440  // External crystal is 12 MHz
#define SYSCTL_XTAL_12_2MHZ     0x00000480  // External crystal is 12.288 MHz
#define SYSCTL_XTAL_13_5MHZ     0x000004C0  // External crystal is 13.56 MHz
#define SYSCTL_XTAL_14_3MHZ     0x00000500  // External crystal is 14.31818 MHz
#define SYSCTL_XTAL_16MHZ       0x00000540  // External crystal is 16 MHz
#define SYSCTL_XTAL_16_3MHZ     0x00000580  // External crystal is 16.384 MHz
#define SYSCTL_OSC_MAIN         0x00000000  // Osc source is main osc
#define SYSCTL_OSC_INT          0x00000010  // Osc source is int. osc
#define SYSCTL_OSC_INT4         0x00000020  // Osc source is int. osc /4
#define SYSCTL_OSC_INT30        0x00000030  // Osc source is int. 30 KHz
#define SYSCTL_OSC_EXT4_19      0x80000028  // Osc source is ext. 4.19 MHz
#define SYSCTL_OSC_EXT32        0x80000038  // Osc source is ext. 32 KHz
#define SYSCTL_INT_PIOSC_DIS    0x00000004  // Disable interal precision osc.
#define SYSCTL_INT_OSC_DIS      0x00000002  // Disable internal oscillator
#define SYSCTL_MAIN_OSC_DIS     0x00000001  // Disable main oscillator

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Stellaris_SysCtl_Exported_APIs
//! @{
//
//*****************************************************************************

extern unsigned long SysCtlSRAMSizeGet(void);
extern unsigned long SysCtlFlashSizeGet(void);
extern xtBoolean SysCtlPinPresent(unsigned long ulPin);
extern xtBoolean SysCtlPeripheralPresent(unsigned long ulPeripheral);
extern void SysCtlPeripheralReset(unsigned long ulPeripheral);
extern void SysCtlPeripheralEnable(unsigned long ulPeripheral);
extern void SysCtlPeripheralDisable(unsigned long ulPeripheral);
extern void SysCtlPeripheralSleepEnable(unsigned long ulPeripheral);
extern void SysCtlPeripheralSleepDisable(unsigned long ulPeripheral);
extern void SysCtlPeripheralDeepSleepEnable(unsigned long ulPeripheral);
extern void SysCtlPeripheralDeepSleepDisable(unsigned long ulPeripheral);
extern void SysCtlPeripheralClockGating(xtBoolean bEnable);
extern void SysCtlIntEnable(unsigned long ulInts);
extern void SysCtlIntDisable(unsigned long ulInts);
extern void SysCtlIntClear(unsigned long ulInts);
extern unsigned long SysCtlIntStatus(xtBoolean bMasked);
extern void SysCtlLDOSet(unsigned long ulVoltage);
extern unsigned long SysCtlLDOGet(void);
extern void SysCtlLDOConfigSet(unsigned long ulConfig);
extern void SysCtlReset(void);
extern void SysCtlSleep(void);
extern void SysCtlDeepSleep(void);
extern unsigned long SysCtlResetCauseGet(void);
extern void SysCtlResetCauseClear(unsigned long ulCauses);
extern void SysCtlBrownOutConfigSet(unsigned long ulConfig,
                                    unsigned long ulDelay);
extern void SysCtlDelay(unsigned long ulCount);
extern void SysCtlClockSet(unsigned long ulConfig);
extern unsigned long SysCtlClockGet(void);
extern void SysCtlPWMClockSet(unsigned long ulConfig);
extern unsigned long SysCtlPWMClockGet(void);
extern void SysCtlADCSpeedSet(unsigned long ulSpeed);
extern unsigned long SysCtlADCSpeedGet(void);
extern void SysCtlIOSCVerificationSet(xtBoolean bEnable);
extern void SysCtlMOSCVerificationSet(xtBoolean bEnable);
extern void SysCtlPLLVerificationSet(xtBoolean bEnable);
extern void SysCtlClkVerificationClear(void);
extern void SysCtlGPIOAHBEnable(unsigned long ulGPIOPeripheral);
extern void SysCtlGPIOAHBDisable(unsigned long ulGPIOPeripheral);
extern void SysCtlUSBPLLEnable(void);
extern void SysCtlUSBPLLDisable(void);
extern unsigned long SysCtlI2SMClkSet(unsigned long ulInputClock,
                                      unsigned long ulMClk);

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __xSYSCTL_H__



