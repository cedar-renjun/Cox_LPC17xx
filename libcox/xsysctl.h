//*****************************************************************************
//
//! \file xsysctl.h
//! \brief Prototypes for the System Manager Driver.
//! \version V2.2.1.0
//! \todo Update this time information.
//! \date 11/20/2011
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
//! \section xSysCtl_Peripheral_ID 1. Where to use this group
//! Values that can be passed to the
//! xSysCtlPeripheralPresent(), xSysCtlPeripheralEnable(),
//! xSysCtlPeripheralDisable(), and xSysCtlPeripheralReset() APIs as the
//! ulPeripheral parameter.  
//! 
//! \section xSysCtl_Peripheral_ID_CoX 2.CoX Port Details
//! \verbatim
//! +--------------------------+----------------+--------------------------+
//! |SysCtl Peripheral ID      |       CoX      |       LPC17xx            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_ACMPn      |  Non-Mandatory |                          |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_ADCn       |    Mandatory   |                          |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_DMA        |  Non-Mandatory |                          |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_GPIOn      |    Mandatory   |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_I2Cn       |  Non-Mandatory |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_PWMn       |  Non-Mandatory |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_RTC        |    Mandatory   |                          |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_SPIn       |    Mandatory   |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_TIMERn     |    Mandatory   |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_UARTn      |    Mandatory   |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |                          |                |--------------------------|
//! |                          |                |                          |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_PERIPH_WDOG       |    Mandatory   |                          |
//! +--------------------------+----------------+--------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

#define xSYSCTL_PERIPH_ACMP0    0
#define xSYSCTL_PERIPH_ADC1     0
#define xSYSCTL_PERIPH_ADC2     0
#define xSYSCTL_PERIPH_DMA1     0
#define xSYSCTL_PERIPH_DMA2     0
#define xSYSCTL_PERIPH_GPIOA    0
#define xSYSCTL_PERIPH_GPIOB    0
#define xSYSCTL_PERIPH_GPIOC    0
#define xSYSCTL_PERIPH_GPIOD    0
#define xSYSCTL_PERIPH_GPIOE    0
#define xSYSCTL_PERIPH_I2C1     0
#define xSYSCTL_PERIPH_I2C2     0
#define xSYSCTL_PERIPH_PWMA     0
#define xSYSCTL_PERIPH_PWMB     0                      
#define xSYSCTL_PERIPH_PWMC     0                      
#define xSYSCTL_PERIPH_PWMD     0                      
#define xSYSCTL_PERIPH_PWME     0                      
#define xSYSCTL_PERIPH_PWMF     0                      
#define xSYSCTL_PERIPH_PWMG     0                      
#define xSYSCTL_PERIPH_PWMH     0                      
#define xSYSCTL_PERIPH_PWMI     0                      
#define xSYSCTL_PERIPH_PWMJ     0                      
#define xSYSCTL_PERIPH_PWMK     0                      
#define xSYSCTL_PERIPH_PWML     0                      
#define xSYSCTL_PERIPH_RTC      0                      
#define xSYSCTL_PERIPH_SPI1     0                      
#define xSYSCTL_PERIPH_SPI2     0
#define xSYSCTL_PERIPH_SPI3     0
#define xSYSCTL_PERIPH_TIMER1   0                 
#define xSYSCTL_PERIPH_TIMER2   0                 
#define xSYSCTL_PERIPH_TIMER3   0                 
#define xSYSCTL_PERIPH_TIMER4   0                 
#define xSYSCTL_PERIPH_UART1    0                  
#define xSYSCTL_PERIPH_UART2    0                  
#define xSYSCTL_PERIPH_UART3    0                  
#define xSYSCTL_PERIPH_UART4    0                  
#define xSYSCTL_PERIPH_UART5    0                  
#define xSYSCTL_PERIPH_WDOG     0

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
//! |SysCtl Clock Set Configs  |       CoX      |       LPC17xx          |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_OSC_MAIN          |    Mandatory   |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_OSC_INT           |    Mandatory   |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_OSC_INTSL         |  Non-Mandatory |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_OSC_EXTSL         |  Non-Mandatory |             Y            |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_XTAL_nMHZ         |  Non-Mandatory |     xSYSCTL_XTAL_4MHZ    |
//! |                          |                |--------------------------|
//! |                          |                |            ...           |
//! |                          |                |--------------------------|
//! |                          |                |     xSYSCTL_XTAL_8MHZ    |
//! |                          |                |--------------------------|
//! |                          |                |            ...           |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_XTAL_12MHZ    |
//! |                          |                |--------------------------|
//! |                          |                |            ...           |
//! |                          |                |--------------------------|
//! |                          |                |    xSYSCTL_XTAL_25MHZ    |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_INT_nMHZ          |  Non-Mandatory |     xSYSCTL_INT_22MHZ    |
//! |--------------------------|----------------|--------------------------|
//! |xSYSCTL_INTSL_nKHZ/HZ     |  Non-Mandatory |    xSYSCTL_INTSL_10KHZ   |
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

#define xSYSCTL_OSC_MAIN        SYSCTL_OSC_MAIN | SYSCTL_PLL_MAIN
#define xSYSCTL_OSC_INT         SYSCTL_OSC_INT | SYSCTL_PLL_INT

//
//! \brief XTAL Select
//
#define xSYSCTL_XTAL_4MHZ       SYSCTL_XTAL_4MHZ
#define xSYSCTL_XTAL_5MHZ       SYSCTL_XTAL_5MHZ
#define xSYSCTL_XTAL_6MHZ       SYSCTL_XTAL_6MHZ
#define xSYSCTL_XTAL_7MHZ       SYSCTL_XTAL_7MHZ
#define xSYSCTL_XTAL_8MHZ       SYSCTL_XTAL_8MHZ
#define xSYSCTL_XTAL_9MHZ       SYSCTL_XTAL_9MHZ
#define xSYSCTL_XTAL_10MHZ      SYSCTL_XTAL_10MHZ
#define xSYSCTL_XTAL_11MHZ      SYSCTL_XTAL_11MHZ
#define xSYSCTL_XTAL_12MHZ      SYSCTL_XTAL_12MHZ
#define xSYSCTL_XTAL_13MHZ      SYSCTL_XTAL_13MHZ
#define xSYSCTL_XTAL_14MHZ      SYSCTL_XTAL_14MHZ
#define xSYSCTL_XTAL_15MHZ      SYSCTL_XTAL_15MHZ
#define xSYSCTL_XTAL_16MHZ      SYSCTL_XTAL_16MHZ
#define xSYSCTL_XTAL_17MHZ      SYSCTL_XTAL_17MHZ
#define xSYSCTL_XTAL_18MHZ      SYSCTL_XTAL_18MHZ
#define xSYSCTL_XTAL_19MHZ      SYSCTL_XTAL_19MHZ
#define xSYSCTL_XTAL_20MHZ      SYSCTL_XTAL_20MHZ
#define xSYSCTL_XTAL_21MHZ      SYSCTL_XTAL_21MHZ
#define xSYSCTL_XTAL_22MHZ      SYSCTL_XTAL_22MHZ
#define xSYSCTL_XTAL_23MHZ      SYSCTL_XTAL_23MHZ
#define xSYSCTL_XTAL_24MHZ      SYSCTL_XTAL_24MHZ
#define xSYSCTL_XTAL_25MHZ      SYSCTL_XTAL_25MHZ

//
//! Internal main clock is 8MHz
//
#define xSYSCTL_INT_8MHZ        0  

//
//! Internal slow clock  is 40KHz.
//
#define xSYSCTL_INTSL_40KHZ     0  

//
//! External slow clock crystal is 32.768KHz.
//
#define xSYSCTL_XTALSL_32768HZ  0  

//
//! Disable internal oscillator
//
#define xSYSCTL_INT_OSC_DIS     SYSCTL_INT_OSC_DIS  

//
//! Disable main oscillator
//
#define xSYSCTL_MAIN_OSC_DIS    SYSCTL_MAIN_OSC_DIS  

//
//! Disable PLL
//
#define xSYSCTL_PLL_PWRDN       SYSCTL_PLL_PWRDN  

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSysCtl_Peripheral_Src_Clk SysCtl Peripheral Source Clock
//! \brief Values that show SysCtl Peripheral Source Clock
//! The following are values that can be passed to the
//! xSysCtlPeripheralClockSourceSet()  API as the
//! ulPeripheralsrc parameter.  
//!
//! \section xSysCtl_Peripheral_Src_Clk_Def SysCtl Peripheral Short Name define
//! The macros of General Peripheral Source Clock always like:
//! <b> ModuleName + n + SourceClock</b>, such as xSYSCTL_WDT_EXTSL, 
//! xSYSCTL_ADC0_MAIN.
//!
//! \section xSysCtl_Peripheral_Src_Clk_CoX CoX Port Details
//! \verbatim
//! +-------------------------- +----------------+--------------------------+
//! |Peripheral Source Clock Set|       CoX      |       LPC17xx          |
//! |---------------------------|----------------|--------------------------|
//! |Those are all Non-Mandatory|  Non-Mandatory |             Y            |
//! | parameter,the Mandatory   |                |                          |
//! | is variable naming        |                |                          |
//! |ModuleName+n+SourceClock   |                |                          |
//! |---------------------------|----------------|--------------------------|
//! |xSYSCTL_WDT_HCLK_n         |  Non-Mandatory |             N            |
//! |-------------------------- |----------------|--------------------------|
//! |xSYSCTL_WDT_HCLK_2048      |  Non-Mandatory |             N            |
//! |-------------------------- |----------------|--------------------------|
//! |xSYSCTL_WDT_INTSL          |  Non-Mandatory |             N            |
//! |-------------------------- |----------------|--------------------------|
//! |xSYSCTL_ADC0_MAIN          |  Non-Mandatory |             Y            |
//! |-------------------------- |----------------|--------------------------|
//! |xSYSCTL_ADC0_PLL           |  Non-Mandatory |             N            |
//! |-------------------------- |----------------|--------------------------|
//! |......                     |  Non-Mandatory |             Y            |
//! |-------------------------- |----------------|--------------------------|
//! |xSYSCTL_PWMB_INT           |  Non-Mandatory |             Y            |
//! |-------------------------- |----------------|--------------------------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//!  HCLK used as xWDT clock
//
#define xSYSCTL_WDT_HCLK        0x00000000
//
//!  HCLK used as ADC clock
//
#define xSYSCTL_ADC0_MAIN       SYSCTL_ADC_HCLK

//
//! LSE oscillator clock used as RTC clock
//
#define xSYSCTL_RTC_LSE         SYSCTL_RTC_LSE

//
//! LSI oscillator clock used as RTC clock
//
#define xSYSCTL_RTC_LSI         SYSCTL_RTC_LSI

//
//! HSE oscillator clock divided by 128 used as RTC clock
//
#define xSYSCTL_RTC_LSE_128     SYSCTL_RTC_LSE_128

//
//! Microcontroller clock output System clock (SYSCLK) selected
//
#define xSYSCTL_MCO_SYSCLK      SYSCTL_MCO_SYSCLK

//
//! Microcontroller clock output HSI clock selected
//
#define xSYSCTL_MCO_HSI         SYSCTL_MCO_HSI

//
//! Microcontroller clock output HSE clock selected
//
#define xSYSCTL_MCO_HSE         SYSCTL_MCO_HSE

//
//! Microcontroller clock output PLL clock divided by 2 selected
//
#define xSYSCTL_MCO_PLL_2       SYSCTL_MCO_PLL_2

//
//! Microcontroller clock output PLL2 clock selected
//
#define xSYSCTL_MCO_PLL2        SYSCTL_MCO_PLL2

//
//! Microcontroller clock output PLL3 clock selected
//
#define xSYSCTL_MCO_PLL3_2      SYSCTL_MCO_PLL3_2

//
//! XT1 external 3-25 MHz oscillator clock selected (for Ethernet)
//
#define xSYSCTL_MCO_XT1         SYSCTL_MCO_XT1

//
//! Microcontroller clock output PLL3 clock selected (for Ethernet)
//
#define xSYSCTL_MCO_PLL3        SYSCTL_MCO_PLL3

//
//! System clock (SYSCLK) selected as I2S3 clock entry
//
#define xSYSCTL_I2S3_SYSCLK     SYSCTL_I2S3_SYSCLK

//
//! PLL3 VCO clock selected as I2S3 clock entry
//
#define xSYSCTL_I2S3_PLL3       SYSCTL_I2S3_PLL3

//
//! System clock (SYSCLK) selected as I2S2 clock entry
//
#define xSYSCTL_I2S2_SYSCLK     SYSCTL_I2S2_SYSCLK

//
//! PLL3 VCO clock selected as I2S2 clock entry
//
#define xSYSCTL_I2S2_PLL3       SYSCTL_I2S2_PLL3

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSysCtl_Peripheral_Short SysCtl Peripheral Short Name
//! \brief Values that show SysCtl Peripheral Source Clock
//! The following are values that can be passed to the
//! xSysCtlPeripheralClockSourceSet2()  API as the
//! ulPeripheral parameter.
//!
//! \section xSysCtl_Peripheral_Short_Def SysCtl Peripheral Short Name define
//! The macros of General Peripheral Short Name always like:
//! <b> ModuleName + n </b>, such as CAN0, ADC0.
//!
//! \section xSysCtl_Peripheral_Short_CoX CoX Port Details
//! \verbatim
//! +-------------------------- +----------------+--------------------------+
//! |Peripheral Short name      |       CoX      |       LPC17xx          |
//! |---------------------------|----------------|--------------------------|
//! |ADCn                       |  Non-Mandatory |           ADC0           |
//! |---------------------------|----------------|--------------------------|
//! |PWMn                       |  Non-Mandatory |           PWMA           |
//! |                           |                |           PWMB           |
//! |-------------------------- |----------------|--------------------------|
//! |CANn                       |  Non-Mandatory |           CAN0           |
//! |-------------------------- |----------------|--------------------------|
//! |UARTn                      |  Non-Mandatory |           UART0          |
//! |                           |                |           UART1          |
//! |                           |                |           UART2          |
//! |-------------------------- |----------------|--------------------------|
//! |I2Sn                       |  Non-Mandatory |           I2S0           |
//! |-------------------------- |----------------|--------------------------|
//! |WDTn                       |  Non-Mandatory |           WDT0           |
//! |-------------------------- |----------------|--------------------------|
//! |TIMERn                     |  Non-Mandatory |          TIMER0          |
//! |                           |                |          TIMER1          |
//! |                           |                |          TIMER2          |
//! |                           |                |          TIMER3          |
//! |-------------------------- |----------------|--------------------------|
//! |I2Cn                       |  Non-Mandatory |             N            |
//! |-------------------------- |----------------|--------------------------|
//! |SPIn                       |  Non-Mandatory |             N            |
//! |-------------------------- |----------------|--------------------------|
//! |SPIn                       |  Non-Mandatory |             N            |
//! |-------------------------- |----------------|--------------------------|
//! |ACMPn                      |  Non-Mandatory |             N            |
//! |-------------------------- |----------------|--------------------------|
//! |RTC                        |  Non-Mandatory |             N            |
//! |-------------------------- |----------------|--------------------------|
//! |GPIO                       |  Non-Mandatory |             N            |
//! |-------------------------- |----------------|--------------------------|
//!
//! +-------------------------- +----------------+--------------------------+
//! |Peripheral Clock source    |       CoX      |       LPC17xx          |
//! |---------------------------|----------------|--------------------------|
//! |INT                        |    Mandatory   |             Y            |
//! |---------------------------|----------------|--------------------------|
//! |HCLK                       |    Mandatory   |             Y            |
//! |---------------------------|----------------|--------------------------|
//! |HCLK_n                     |  Non-Mandatory |         HCLK_2048        |
//! |---------------------------|----------------|--------------------------|
//! |EXTSL                      |    Mandatory   |             Y            |
//! |---------------------------|----------------|--------------------------|
//! |INTSL                      |    Mandatory   |             Y            |
//! |---------------------------|----------------|--------------------------|
//! |MAIN                       |    Mandatory   |             Y            |
//! |---------------------------|----------------|--------------------------|
//! |PLL                        |    Mandatory   |             Y            |
//! |---------------------------|----------------|--------------------------|
//! |PLL_n                      |  Non-Mandatory |           PLL_2          |
//! |---------------------------|----------------|--------------------------|
//! |EXTTRG                     |  Non-Mandatory |             Y            |
//! |---------------------------|----------------|--------------------------|
//! \endverbatim
//! @{
//
//*****************************************************************************

#define MCO                     MCO
#define I2S2                    I2S2
#define I2S3                    I2S3
#define RTC                     RTC


//
//! internal high speed oscillator
//
#define INT                     INT

//
//! The system clock
//
#define HCLK                    HCLK

//
//! The system clock divide by 2048
//
#define HCLK_2048               HCLK_2048

//
//! external low speed crystal
//
#define EXTSL                   EXTSL

//
//! internal low speed oscillator
//
#define INTSL                   INTSL

//
//! external high speed oscillator
//
#define MAIN                    MAIN

//
//! PLL output
//
#define PLL                     PLL

//
//! PLL output divide by 2
//
#define PLL_2                   PLL_2

//
//! external clock input
//
#define EXTTRG                  EXTTRG
 
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
//! |xSysCtl API                     |       CoX      | LPC17xx |
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
//! |--------------------------------|----------------|-----------|
//! |xSysCtlPeripheralClockSourceSet |    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! |xSysCtlPeripheralClockSourceSet2|    Mandatory   |     Y     |
//! |--------------------------------|----------------|-----------|
//! \endverbatim
//!
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief Performs a software reset of a peripheral.
//!
//! \param ulPeripheralID is the peripheral to reset. 
//! Details please refer to \ref xSysCtl_Peripheral_ID.
//!
//! This function performs a software reset of the specified peripheral.  An
//! individual peripheral reset signal is asserted for a brief period and then
//! deasserted, returning the internal state of the peripheral to its reset
//! condition.
//!
//! The \e ulPeripheralID parameter must be only one of the following values:
//! Details please refer to \ref xSysCtl_Peripheral_ID_CoX.
//!
//! \return None.
//
//*****************************************************************************       
#define xSysCtlPeripheralReset(ulPeripheralID)                                \
        SysCtlPeripheralReset(ulPeripheralID)
        
//*****************************************************************************
//
//! \brief Enables a peripheral.
//!
//! \param ulPeripheralID is the peripheral to enable.
//! Details please refer to \ref xSysCtl_Peripheral_ID.
//!
//! Peripherals are enabled with this function.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! The \e ulPeripheralID parameter must be only one of the following values:
//! Details please refer to \ref xSysCtl_Peripheral_ID_CoX.
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
#define xSysCtlPeripheralEnable(ulPeripheralID)                               \
        SysCtlPeripheralEnable(ulPeripheralID)

//*****************************************************************************
//
//! \brief Disable a peripheral.
//!
//! \param ulPeripheralID is the peripheral to disable.
//! Details please Refer to \ref xSysCtl_Peripheral_ID.
//!
//! Peripherals are disabled with this function.  Once disabled, they will not
//! operate or respond to register reads/writes.
//!
//! The \e ulPeripheralID parameter must be only one of the following values:
//! Details please Refer to \ref xSysCtl_Peripheral_ID_CoX.
//!
//! \return None.
//
//*****************************************************************************
#define xSysCtlPeripheralDisable(ulPeripheralID)                              \
        SysCtlPeripheralDisable(ulPeripheralID)
        
//*****************************************************************************
//
//! \brief Enable a peripheral.
//!
//! \param ulPeripheralBase a Peripheral base indicate which peripheral to be 
//! enabled.Details please Refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! Peripherals are enabled with this function.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! The \e ulPeripheral parameter must be only one of the following values:
//! Details please Refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! \note None.
//!
//! \return None.
//
//*****************************************************************************
extern void xSysCtlPeripheralEnable2(unsigned long ulPeripheralBase);
        
//*****************************************************************************
//
//! \brief Disable a peripheral.
//!
//! \param ulPeripheralBase a Peripheral base indicate which peripheral to be 
//! enabled.Details please Refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! Peripherals are disabled with this function.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! The \e ulPeripheral parameter must be only one of the following values:
//! Details please Refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! \note None.
//!
//! \return None.
//
//*****************************************************************************
extern void xSysCtlPeripheralDisable2(unsigned long ulPeripheralBase);
        
//*****************************************************************************
//
//! \brief Reset a peripheral.
//!
//! \param ulPeripheralBase a Peripheral base indicate which peripheral to be 
//! Reset.Details please Refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! Peripherals are Reset with this function.  At power-up, all peripherals
//! are disabled; they must be enabled in order to operate or respond to
//! register reads/writes.
//!
//! The \e ulPeripheral parameter must be only one of the following values:
//! Details please Refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! \note None.
//!
//! \return None.
//
//*****************************************************************************
extern void xSysCtlPeripheralReset2(unsigned long ulPeripheralBase);

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
extern unsigned long xSysCtlPeripheraIntNumGet(unsigned long ulPeripheralBase);

//*****************************************************************************
//
//! \brief Set the clocking of the device.
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
//! The external crystal frequency is chosen with one of the following values:
//! \ref xSYSCTL_XTAL_4MHZ, \ref xSYSCTL_XTAL_5MHZ, \ref xSYSCTL_XTAL_6MHZ,
//! \ref xSYSCTL_XTAL_24MHZ.
//!
//! The oscillator source is chosen with one of the following values:
//! \ref xSYSCTL_OSC_MAIN, \ref xSYSCTL_OSC_INT, \ref xSYSCTL_OSC_INTSL,
//! \ref xSYSCTL_OSC_EXTSL.
//!
//! The internal and main oscillators and PLL are disabled with the
//! \ref xSYSCTL_INT_OSC_DIS and \ref xSYSCTL_MAIN_OSC_DIS flags, 
//! \ref xSYSCTL_PLL_PWRDN respectively.
//! The external oscillator must be enabled in order to use an external clock
//! source.  Note that attempts to disable the oscillator used to clock the
//! device will be prevented by the hardware.
//! <br />
//! Details please Refer to \ref XSysCtl_Clock_Set_Config_CoX.
//! 
//!
//! \return None.
//
//*****************************************************************************
#define xSysCtlClockSet(ulSysClk, ulConfig)                                   \
        SysCtlClockSet(ulSysClk, ulConfig)

//*****************************************************************************
//
//! \brief Get the processor clock rate.
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
#define xSysCtlClockGet()                                                     \
        SysCtlHClockGet()

//*****************************************************************************
//
//! \brief Provide a small delay.
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
//! \brief Reset the device.
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
//! \brief Put the processor into sleep mode.
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
//! \brief Set a peripheral clock source and peripheral divide.
//!
//! \param ulPeripheralSrc is the peripheral clock source to set.
//! \param ulDivide is the peripheral clock divide to set.
//!
//! Peripherals clock source are seted with this function.  At power-up, all 
//! Peripherals clock source are Peripherals clock source; they must be set in 
//! order to operate or respond to register reads/writes.
//!
//! The \e ulPeripheralSrc parameter must be only one of the following values:
//! \ref xSysCtl_Peripheral_Src_Clk.
//!
//! \return None.
//
//*****************************************************************************
#define xSysCtlPeripheralClockSourceSet(ulPeripheralSrc, ulDivide)            \
        SysCtlPeripheralClockSourceSet(ulPeripheralSrc, ulDivide)

//*****************************************************************************
//
//! \brief Set a peripheral clock source and peripheral divide.
//!
//! \param ePeripheral is the peripheral which's clock source will be set.
//! \param eSrc is the clock source will be set.
//! \param ulDivide is the peripheral clock divide to set.
//!
//! Peripherals clock source are seted with this function.  At power-up, all 
//! Peripherals clock source are Peripherals clock source; they must be set in 
//! order to operate or respond to register reads/writes.
//!
//! The \e ulPeripheralSrc parameter must be only one of the following values:
//! \ref xSysCtl_Peripheral_Src_Clk_CoX.
//! \verbatim
//! +--------------------+------------------------+---------------------------+
//! |    manufacturer    |ePeripheral             |eSrc                       |
//! |--------------------|------------------------|---------------------------|
//! |    CoX Common &    |This parameter is a     |This parameter is a        |
//! |      Mandatory     |mandatory.Mandatory     |mandatory. So it           |
//! |                    |is the format of        |should be: INT             |
//! |                    |Variable naming.So it   |HCLK  HCLK_n EXTSL         |
//! |                    |should be: ADCn,        |INTSL  MAIN  PLL           |
//! |                    |TIMERn or UARTn         |PLL_n  EXTTRG              |
//! |                    |n indicate the pin      |n indicate the pin         |
//! |                    |number such as          |number such as             |
//! |                    |0 1 2 3 ....            |0 1 2 3 ....               |
//! |--------------------|------------------------|---------------------------|
//! |       LPC17xx      |    ADC0                |MAIN PLL INT               |
//! |                    |    PWMB                |INT HCLK EXTSL MAIN        |
//! |                    |    PWMA                |INT HCLK EXTSL MAIN        |
//! |                    |    FRQDIV              |INT HCLK EXTSL MAIN        |
//! |                    |    I2S0                |INT HCLK PLL MAIN          |
//! |                    |    TIMER0              |INT EXTTRG HCLK EXTSL MAIN |
//! |                    |    TIMER1              |INT EXTTRG HCLK EXTSL MAIN |
//! |                    |    TIMER2              |INT EXTTRG HCLK EXTSL MAIN |
//! |                    |    TIMER3              |INT EXTTRG HCLK EXTSL MAIN |
//! |                    |    UART0               |INT PLL MAIN               |
//! |                    |    UART1               |INT PLL MAIN               |
//! |                    |    UART2               |INT PLL MAIN               |
//! |                    |    CAN0                |INT PLL MAIN               |
//! |                    |    WDT0                |INTSL HCLK_2048 EXTSL      |
//! |--------------------|------------------------|---------------------------|
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************                                          unsigned long ulDivide);
#define xSysCtlPeripheralClockSourceSet2(ePeripheral, eSrc, ulDivide)         \
        SysCtlIPClockSourceSetConvert(ePeripheral, eSrc, ulDivide)


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
//! \addtogroup LPC17xx_SysCtl
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_SysCtl_Clock_Config LPC17xx SysCtl Clock Configuration
//! \brief LPC17xx SysCtl Clock Configuration
//! The following are values that can be passed to the \ref SysCtlClockSet()
//! API as the ulConfig parameter.
//! @{
//
//*****************************************************************************

// Clock Frequency bit[4:0] mask.
// \note This macro is used for \ref SysCtlClockSet function.
#define SYSCTL_XTAL_nMHZ_MASK          BIT_MASK(32, 4, 0)

//! Main Oscillator 1 Mhz.
#define SYSCTL_XTAL_1_MHZ              1

//! Main Oscillator 2 Mhz
#define SYSCTL_XTAL_2_MHZ              2

//! Main Oscillator 3 Mhz
#define SYSCTL_XTAL_3_MHZ              3

//! Main Oscillator 4 Mhz
#define SYSCTL_XTAL_4_MHZ              4

//! Main Oscillator 5 Mhz
#define SYSCTL_XTAL_5_MHZ              5

//! Main Oscillator 6 Mhz
#define SYSCTL_XTAL_6_MHZ              6

//! Main Oscillator 7 Mhz
#define SYSCTL_XTAL_7_MHZ              7

//! Main Oscillator 8 Mhz
#define SYSCTL_XTAL_8_MHZ              8

//! Main Oscillator 9 Mhz
#define SYSCTL_XTAL_9_MHZ              9

//! Main Oscillator 10 Mhz
#define SYSCTL_XTAL_10_MHZ             10

//! Main Oscillator 11 Mhz
#define SYSCTL_XTAL_11_MHZ             11

//! Main Oscillator 12 Mhz
#define SYSCTL_XTAL_12_MHZ             12

//! Main Oscillator 13 Mhz
#define SYSCTL_XTAL_13_MHZ             13

//! Main Oscillator 14 Mhz
#define SYSCTL_XTAL_14_MHZ             14

//! Main Oscillator 15 Mhz
#define SYSCTL_XTAL_15_MHZ             15

//! Main Oscillator 16 Mhz
#define SYSCTL_XTAL_16_MHZ             16

//! Main Oscillator 17 Mhz
#define SYSCTL_XTAL_17_MHZ             17

//! Main Oscillator 18 Mhz
#define SYSCTL_XTAL_18_MHZ             18

//! Main Oscillator 19 Mhz
#define SYSCTL_XTAL_19_MHZ             19

//! Main Oscillator 20 Mhz
#define SYSCTL_XTAL_20_MHZ             20

//! Main Oscillator 21 Mhz
#define SYSCTL_XTAL_21_MHZ             21

//! Main Oscillator 22 Mhz
#define SYSCTL_XTAL_22_MHZ             22

//! Main Oscillator 23 Mhz
#define SYSCTL_XTAL_23_MHZ             23

//! Main Oscillator 24 Mhz
#define SYSCTL_XTAL_24_MHZ             24

//! Main Oscillator 25Mhz
#define SYSCTL_XTAL_25_MHZ             25

//! Device Maximum Clock Speed,120Mhz for LPC1759/LPC1759, 100MHz for others Device.
#define SYSTEM_CLOCK_MAX               ((unsigned long)120000000)

//! Use Main Oscillator as input clock source.
#define SYSCTL_OSC_MAIN                BIT_32_5

//! Use Internal Oscillator as input clock source.
#define SYSCTL_OSC_INT                 BIT_32_6

//! Disable Internal Oscillator.
#define SYSCTL_INT_OSC_DIS             BIT_32_7

//! Disable Main Oscillator.
#define SYSCTL_MAIN_OSC_DIS            BIT_32_8

//! Power Down PLL Module.
#define SYSCTL_PLL_PWRDN               BIT_32_9

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_SysCtl_ExtInt_Config LPC17xx SysCtl External Interrupt
//!             Configure.
//! \brief LPC17xx SysCtl External Interrupt Configuration.
//!
//! The following are values that can be passed to the following function:
//!     - \ref SysCtlExtIntCfg
//!     - \ref SysCtlExtIntFlagGet
//!     - \ref SysCtlExtIntFlagCheck
//!     - \ref SysCtlExtIntFlagClear
//! As the input/output parameter.
//! @{
//
//*****************************************************************************

//! External Interrupt channel 0
#define EXT_INT_0                      BIT_32_0

//! External Interrupt channel 0
#define EXT_INT_1                      BIT_32_1

//! External Interrupt channel 0
#define EXT_INT_2                      BIT_32_2

//! External Interrupt channel 0
#define EXT_INT_3                      BIT_32_3

//! \internal
//! External Interrupt channel mask
#define EXT_INT_MASK                   BIT_MASK(32, 3, 0)

//! External Interrupt type: High level trigger
#define EXT_INT_LV_H                   BIT_32_0

//! External Interrupt type: Low level trigger
#define EXT_INT_LV_L                   BIT_32_1

//! External Interrupt type: Rising edge trigger
#define EXT_INT_EG_R                   BIT_32_2

//! External Interrupt type: Falling edge trigger
#define EXT_INT_EG_F                   BIT_32_3

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_SysCtl_Reset_Config LPC17xx Sysctl Reset Configure.
//! \brief      LPC17xx SysCtl Reset source configuration.
//!
//! Those macro can be used in the following functions:
//!     - \ref SysCtlResetFlagGet
//!     - \ref SysCtlResetFlagCheck
//! @{
//
//*****************************************************************************

//! Power on reset
#define RESET_FLAG_POR                 RSID_POR

//! External reset signal
#define RESET_FLAG_EXTR                RSID_EXTR

//! Watchdog Timer reset
#define RESET_FLAG_WDTR                RSID_WDTR

//! Brown-out reset
#define RESET_FLAG_BODR                RSID_BODR

//! System reset requet reset
#define RESET_FLAG_SYSRESET            RSID_SYSRESET

//! Lockup reset
#define RESET_FLAG_LOCKUP              RSID_LOCKUP

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_SysCtl_PeripheralClock_Config LPC17xx Sysctl Peripheral
//!             Clock Configure.
//! \brief      LPC17xx SysCtl Peripheral Clock Configure configuration.
//!
//! Those macro can be used in the following functions:
//!     - \ref SysCtlPeripheralClockCfg
//! @{
//
//***************************************************************************** 

//! Peripheral Clock for WatchDog.
#define PCLKSEL_WDT                    PCLKSEL0_WDT_S

//! Peripheral Clock for Timer0.
#define PCLKSEL_TIMER0                 PCLKSEL0_TIMER0_S

//! Peripheral Clock for Timer1.
#define PCLKSEL_TIMER1                 PCLKSEL0_TIMER1_S

//! Peripheral Clock for UART0.
#define PCLKSEL_UART0                  PCLKSEL0_UART0_S

//! Peripheral Clock for UART1.
#define PCLKSEL_UART1                  PCLKSEL0_UART1_S

//! Peripheral Clock for PWM1.
#define PCLKSEL_PWM1                   PCLKSEL0_PWM1_S

//! Peripheral Clock for I2C0.
#define PCLKSEL_I2C0                   PCLKSEL0_I2C0_S

//! Peripheral Clock for SPI.
#define PCLKSEL_SPI                    PCLKSEL0_SPI_S

//! Peripheral Clock for SSP1.
#define PCLKSEL_SSP1                   PCLKSEL0_SSP1_S

//! Peripheral Clock for DAC.
#define PCLKSEL_DAC                    PCLKSEL0_DAC_S

//! Peripheral Clock for ADC.
#define PCLKSEL_ADC                    PCLKSEL0_ADC_S

//! Peripheral Clock for CAN1.
#define PCLKSEL_CAN1                   PCLKSEL0_CAN1_S

//! Peripheral Clock for CAN2.
#define PCLKSEL_CAN2                   PCLKSEL0_CAN2_S

//! Peripheral Clock for ACF.
#define PCLKSEL_ACF                    PCLKSEL0_ACF_S

//! Peripheral Clock for QEI.
#define PCLKSEL_QEI                    (PCLKSEL1_QEI_S     + 32)

//! Peripheral Clock for GPIOINT.
#define PCLKSEL_GPIOINT                (PCLKSEL1_GPIOINT_S + 32)

//! Peripheral Clock for PCB.
#define PCLKSEL_PCB                    (PCLKSEL1_PCB_S     + 32)

//! Peripheral Clock for I2C1.
#define PCLKSEL_I2C1                   (PCLKSEL1_I2C1_S    + 32)

//! Peripheral Clock for SSP0.
#define PCLKSEL_SSP0                   (PCLKSEL1_SSP0_S    + 32)

//! Peripheral Clock for Timer2.
#define PCLKSEL_TIMER2                 (PCLKSEL1_TIMER2_S  + 32)

//! Peripheral Clock for Timer3.
#define PCLKSEL_TIMER3                 (PCLKSEL1_TIMER3_S  + 32)

//! Peripheral Clock for UART2.
#define PCLKSEL_UART2                  (PCLKSEL1_UART2_S   + 32)

//! Peripheral Clock for UART3.
#define PCLKSEL_UART3                  (PCLKSEL1_UART3_S   + 32)

//! Peripheral Clock for I2C2.
#define PCLKSEL_I2C2                   (PCLKSEL1_I2C2_S    + 32)

//! Peripheral Clock for I2S.
#define PCLKSEL_I2S                    (PCLKSEL1_I2S_S     + 32)

//! Peripheral Clock for RIT.
#define PCLKSEL_RIT                    (PCLKSEL1_RIT_S     + 32)

//! Peripheral Clock for System Control.
#define PCLKSEL_SYSCON                 (PCLKSEL1_SYSCON_S  + 32)

//! Peripheral Clock for Motor Control PWM.
#define PCLKSEL_MC                     (PCLKSEL1_MC_S      + 32)

//! Peripheral clock Divider = Cclk/1
#define PCLK_CCLK_DIV_1                BIT_32_0

//! Peripheral clock Divider = Cclk/2
#define PCLK_CCLK_DIV_2                BIT_32_1

//! Peripheral clock Divider = Cclk/4
#define PCLK_CCLK_DIV_4                BIT_32_ALL_0

//! \brief Peripheral clock Divider = Cclk/6
//! \note  This macro is only suit for CAN1/CAN2.
#define PCLK_CCLK_DIV_6                (BIT_32_1 | BIT_32_0)

//! Peripheral clock Divider = Cclk/8
#define PCLK_CCLK_DIV_8                (BIT_32_1 | BIT_32_0) 

//*****************************************************************************
//
//! @}
//
//***************************************************************************** 

//*****************************************************************************
//
//! \addtogroup LPC17xx_SysCtl_Peripheral_Config LPC17xx Sysctl Peripheral
//!             Configure.
//! \brief      LPC17xx SysCtl Peripheral Configure.
//!
//! Those macro can be used in the following functions:
//!     - \ref SysCtlPeripheralReset
//!     - \ref SysCtlPeripheralEnable
//!     - \ref SysCtlPeripheralDisable
//! @{
//
//***************************************************************************** 

//! LCD Controller power/clock control bit.
#define SYSCTL_PERIPH_LCD              PCONP_PCLCD

//! Timer/Counter 0 power/clock control bit.
#define SYSCTL_PERIPH_TIM0             PCONP_PCTIM0

//! Timer/Counter 1 power/clock control bit.
#define SYSCTL_PERIPH_TIM1             PCONP_PCTIM1

//! UART0 Power/clock control bit.
#define SYSCTL_PERIPH_UART0            PCONP_PCUART0

//! UART1 Power/clock control bit.
#define SYSCTL_PERIPH_UART1            PCONP_PCUART1

//! PWM0 Power/Clock control bit.
#define SYSCTL_PERIPH_PWM0             PCONP_PCPWM0

//! PWM1 Power/Clock control bit.
#define SYSCTL_PERIPH_PWM1             PCONP_PCPWM1

//! I2C0 Interface Power/Clock control bit.
#define SYSCTL_PERIPH_I2C0             PCONP_PCI2C0

//! The SPI interface power/clock control bit.
#define SYSCTL_PERIPH_SPI              PCONP_PCSPI

//! UART4 power/clock control bit.
#define SYSCTL_PERIPH_UART4            PCONP_PCUART4

//! RTC and Event Monitor/Recorder power/clock control bit.
#define SYSCTL_PERIPH_RTC              PCONP_PCRTC

//! SSP 1 interface power/clock control bit.
#define SYSCTL_PERIPH_SSP1             PCONP_PCSSP1

//! External Memory Controller power/clock control bit.
#define SYSCTL_PERIPH_EMC              PCONP_PCEMC

//! A/D converter (ADC) power/clock control bit.
#define SYSCTL_PERIPH_ADC              PCONP_PCADC

//! CAN Controller 1 power/clock control bit.
#define SYSCTL_PERIPH_CAN1             PCONP_PCCAN1

//! CAN Controller 2 power/clock control bit.
#define SYSCTL_PERIPH_CAN2             PCONP_PCCAN2

//! Power/clock control bit for IOCON, GPIO, and GPIO interrupts.
#define SYSCTL_PERIPH_GPIO             PCONP_PCGPIO

//! Repetitive Interrupt Timer power/clock control bit.
#define SYSCTL_PERIPH_RIT              PCONP_PCRIT

//! SPI Flash Interface power/clock control bit.
#define SYSCTL_PERIPH_SPIFI            PCONP_PCSPIFI

//! Motor Control PWM power/clock control bit.
#define SYSCTL_PERIPH_MCPWM            PCONP_PCMCPWM

//! Quadrature Encoder Interface power/clock control bit.
#define SYSCTL_PERIPH_QEI              PCONP_PCQEI

//! I2C1 interface power/clock control bit.
#define SYSCTL_PERIPH_I2C1             PCONP_PCI2C1

//! SSP2 interface power/clock control bit.
#define SYSCTL_PERIPH_SSP2             PCONP_PCSSP2

//! SSP0 interface power/clock control bit.

//! Timer 2 power/clock control bit.
#define SYSCTL_PERIPH_TIM2             PCONP_PCTIM2

//! Timer 3 power/clock control bit.
#define SYSCTL_PERIPH_TIM3             PCONP_PCTIM3

//! UART 2 power/clock control bit.
#define SYSCTL_PERIPH_UART2            PCONP_PCUART2

//! UART 3 power/clock control bit.
#define SYSCTL_PERIPH_UART3            PCONP_PCUART3

//! I2C interface 2 power/clock control bit.
#define SYSCTL_PERIPH_I2C2             PCONP_PCI2C2

//! I2S interface power/clock control bit.
#define SYSCTL_PERIPH_I2S              PCONP_PCI2S

//! SD Card interface power/clock control bit.
#define SYSCTL_PERIPH_SDC              PCONP_PCSDC

//! GPDMA function power/clock control bit.
#define SYSCTL_PERIPH_GPDMA            PCONP_PCGPDMA

//! Ethernet block power/clock control bit.
#define SYSCTL_PERIPH_ETH              PCONP_PCENET

//! USB interface power/clock control bit.
#define SYSCTL_PERIPH_USB              PCONP_PCUSB

//! Reset control bit for the IOCON registers
#define SYSCTL_PERIPH_IOCON            (RSTCON1_RSTIOCON + 32)

//! D/A converter (DAC) reset control bit
#define SYSCTL_PERIPH_DAC              (RSTCON1_RSTDAC + 32)

//! CAN acceptance filter reset control bit
#define SYSCTL_PERIPH_CANACC           (RSTCON1_RSTCANACC + 32)

//*****************************************************************************
//
//! @}
//
//***************************************************************************** 

//*****************************************************************************
//
//! \addtogroup LPC17xx_SysCtl_PowerManager LPC17xx Sysctl PowerManager.
//!
//! \brief      LPC17xx SysCtl PowerManager configuration.
//!
//! Those macro can be used in the following functions:
//!     - \ref SysCtlPwrCfg
//!     - \ref SysCtlPwrFlagCheck
//!     - \ref SysCtlPwrFlagClear
//! @{
//
//***************************************************************************** 

//! Sleep Mode
#define PWR_MODE_SLEEP                 BIT_32_0

//! Deep Sleep Mode
#define PWR_MODE_SLEEP_D               BIT_32_1

//! PowerDown Mode
#define PWR_MODE_PWRDOWN               BIT_32_2

//! Deep PowerDown Mode
#define PWR_MODE_PWRDOWN_D             BIT_32_3

//*****************************************************************************
//
//! @}
//
//***************************************************************************** 

//*****************************************************************************
//
//! \addtogroup LPC17xx_SysCtl_BrownOut_Config LPC17xx Sysctl BrownOut Configure.
//!
//! \brief      LPC17xx SysCtl BrownOut configuration.
//!
//! Those macro can be used in the following functions:
//!     - \ref SysCtlBODCfg
//! @{
//
//***************************************************************************** 

//! Enable Brown-Out Reduced Power Mode.
#define BOD_REDUCE_PWR_EN              BIT_32_18

//! Disable Brown-Out Reduced Power Mode.
#define BOD_REDUCE_PWR_DIS             BIT_32_2

//! Enable Global Brown-Out.
#define BOD_GLOBAL_EN                  BIT_32_19

//! Disable Global Brown-Out.
#define BOD_GLOBAL_DIS                 BIT_32_3

//! Enable Brown-Out Reset.
#define BOD_RESET_EN                   BIT_32_20

//! Enable Brown-Out Reset.
#define BOD_RESET_DIS                  BIT_32_4

//*****************************************************************************
//
//! @}
//
//***************************************************************************** 

//*****************************************************************************
//
//! \addtogroup LPC17xx_SysCtl_Exported_APIs LPC17xx SysCtl APIs
//! \brief LPC17xx SysCtl API Reference
//! @{
//
//*****************************************************************************

extern void SysCtlDelay(unsigned long ulCount);
extern void SysCtlClockSet(unsigned long ulSysClk, unsigned long ulConfig);

extern void SysCtlExtIntCfg(unsigned long ulPin, unsigned long ulCfg);
extern unsigned long SysCtlExtIntFlagGet(void);
extern xtBoolean SysCtlExtIntFlagCheck(unsigned long ulFlag);
extern void SysCtlExtIntFlagClear(unsigned long ulFlag);

extern unsigned long SysCtlResetFlagGet(void);
extern xtBoolean SysCtlResetFlagCheck(unsigned long ulFlag);

extern void SysCtlPeripheralClockCfg(unsigned long ulPeri, unsigned long ulCfg);
extern void SysCtlPeripheralReset(unsigned long ulPeripheral);
extern void SysCtlPeripheralEnable(unsigned long ulPeripheral);
extern void SysCtlPeripheralDisable(unsigned long ulPeripheral);

extern unsigned long SysCtlHClockGet(void);
extern unsigned long SysCtlAPB1ClockGet(void);
extern unsigned long SysCtlAPB2ClockGet(void);

extern unsigned long SysCtlPwrCfg(unsigned long ulMode);
extern void SysCtlBODCfg(unsigned long ulCfg);
extern xtBoolean SysCtlPwrFlagCheck(unsigned long ulFlag);
extern void SysCtlPwrFlagClear(unsigned long ulFlag);

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

