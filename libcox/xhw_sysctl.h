//*****************************************************************************
//
//! \file xhw_sysctl.h
//! \brief Macros used when accessing the system control hardware.
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

#ifndef __xHW_SYSCTL_H__
#define __xHW_SYSCTL_H__

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
//! \addtogroup LPC17xx_Register_SysCtl
//! \brief Here are the detailed info of SysCtl registers.
//!
//! it contains:
//! - Register offset.
//! - detailed bit-field of the registers.
//! - Enum and mask of the registers.
//! .
//! Users can read or write the registers through xHWREG().
//!
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_Offsets Stellaris SysCtl Register Address(Map)
//! \brief Defines for the system control register addresses.
//! @{
//
//*****************************************************************************

#define SYSCTL_PLL0CON          0x400FC080  // PLL0 Control Register
#define SYSCTL_PLL0CFG          0x400FC084  // PLL0 Configuration Register
#define SYSCTL_PLL0STAT         0x400FC088  // PLL0 Status Register
#define SYSCTL_PLL0FEED         0x400FC08C  // PLL0 Feed Register
#define SYSCTL_PLL1CON          0x400FC0A0  // PLL1 Control Register
#define SYSCTL_PLL1CFG          0x400FC0A4  // PLL1 Configuration Register
#define SYSCTL_PLL1STAT         0x400FC0A8  // PLL1 Status Register
#define SYSCTL_PLL1FEED         0x400FC0AC  // PLL1 Feed Register
#define SYSCTL_CLKCFG           0x400FC104  // CPU Clock Configuration Register
#define SYSCTL_USBCLKCFG        0x400FC108  // USB Clock Configuration Register
#define SYSCTL_SCS              0x400FC1A0  // System Controls and Status register
#define SYSCTL_PCLKSEL0         0x400FC1A8  // Peripheral Clock Selection register 0                                            // Channels
#define SYSCTL_PCLKSEL1         0x400FC1AC  // Peripheral Clock Selection register 1
#define SYSCTL_PCON             0x400FC0C0  // Power Control Register
#define SYSCTL_PCONP            0x400FC0C4  // Power Control for Peripherals Register
#define SYSCTL_CLKSRCSEL        0x400FC10C  // Clock Source Select Register
#define SYSCTL_RSID             0x400FC180  // Reset Cause
#define SYSCTL_CLKOUTCFG        0x400FC1C8  // Clock Output Configuration Register
#define FLASH_FLASHCFG          0x400FC000  // Flash Accelerator Configuration Register

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PLL0CON SYSCTL_PLL0CON
//! @{
//
//*****************************************************************************

//
//! PLL0 Enable
//
#define SYSCTL_PLL0CON_PLLE0    0x00000001

//
//! PLL0 Connect
//
#define SYSCTL_PLL0CON_PLLC0    0x00000002

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PLL0CFG SYSCTL_PLL0CFG
//! @{
//
//*****************************************************************************

//
//! PLL0 Pre-Divider value
//
#define SYSCTL_PLL0CFG_NSEL0_M  0x00FF0000
#define SYSCTL_PLL0CFG_NSEL0_S  16

//
//! PLL0 Multiplier value
//
#define SYSCTL_PLL0CFG_MSEL0_M  0x00004FFF
#define SYSCTL_PLL0CFG_MSEL0_S  0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PLL0STAT SYSCTL_PLL0STAT
//! @{
//
//*****************************************************************************

//
//! Reflects the PLL0 Lock status
//
#define SYSCTL_PLL0STAT_PLOCK0  0x04000000

//
//! Read-back for the PLL0 Connect bit
//
#define SYSCTL_PLL0STAT_PLLC0_STAT                                            \
                                0x02000000

//
//! Read-back for the PLL0 Enable bit
//
#define SYSCTL_PLL0STAT_PLLE0_STAT                                            \
                                0x01000000

//
//! Read-back for the PLL0 Pre-Divider value
//
#define SYSCTL_PLL0STAT_NSEL0_M 0x00FF0000
#define SYSCTL_PLL0STAT_NSEL0_S 16

//
//! Read-back for the PLL0 Multiplier value
//
#define SYSCTL_PLL0STAT_MSEL0_M 0x00004FFF
#define SYSCTL_PLL0STAT_MSEL0_S 0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PLL0FEED SYSCTL_PLL0FEED
//! @{
//
//*****************************************************************************

//
//! Read-back for the PLL0 Multiplier value
//
#define SYSCTL_PLL0FEED_PLL0FEED_M                                            \
                                0x000000FF
#define SYSCTL_PLL0FEED_PLL0FEED_S                                            \
                                0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PLL1CON SYSCTL_PLL1CON
//! @{
//
//*****************************************************************************

//
//! PLL1 Enable
//
#define SYSCTL_PLL1CON_PLLE1    0x00000001

//
//! PLL1 Connect
//
#define SYSCTL_PLL1CON_PLLC1    0x00000002

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PLL1CFG SYSCTL_PLL1CFG
//! @{
//
//*****************************************************************************

//
//! PLL1 Divider value
//
#define SYSCTL_PLL1CFG_PSEL1_M  0x00000060
#define SYSCTL_PLL1CFG_PSEL1_S  5

//
//! PLL1 Multiplier value
//
#define SYSCTL_PLL0CFG_MSEL1_M  0x0000001F
#define SYSCTL_PLL0CFG_MSEL1_S  0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PLL1STAT SYSCTL_PLL1STAT
//! @{
//
//*****************************************************************************

//
//! Reflects the PLL0 Lock status
//
#define SYSCTL_PLL1STAT_PLOCK1  0x00000400

//
//! Read-back for the PLL0 Connect bit
//
#define SYSCTL_PLL1STAT_PLLC1_STAT                                            \
                                0x00000200

//
//! Read-back for the PLL1 Enable bit
//
#define SYSCTL_PLL1STAT_PLLE1_STAT                                            \
                                0x00000100

//
//! Read-back for the PLL1 Divider value
//
#define SYSCTL_PLL1STAT_NSEL1_M 0x0000060
#define SYSCTL_PLL1STAT_NSEL1_S 5

//
//! Read-back for the PLL1 Multiplier value
//
#define SYSCTL_PLL1STAT_MSEL1_M 0x0000001F
#define SYSCTL_PLL1STAT_MSEL1_S 0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PLL1FEED SYSCTL_PLL1FEED
//! @{
//
//*****************************************************************************

//
//! Read-back for the PLL1 Multiplier value
//
#define SYSCTL_PLL1FEED_PLL1FEED_M                                            \
                                0x000000FF
#define SYSCTL_PLL1FEED_PLL1FEED_S                                            \
                                0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_CLKCFG SYSCTL_CLKCFG
//! @{
//
//*****************************************************************************

//
//! Selects the divide value for creating the CPU clock from the PLL0 output.
//
#define SYSCTL_CLKCFG_CLKSEL_M  0x000000FF
#define SYSCTL_CLKCFG_CLKSEL_S  0
//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_USBCLKCFG SYSCTL_USBCLKCFG
//! @{
//
//*****************************************************************************

//
//! Selects the divide value for creating the USB clock from the PLL0 output.
//
#define SYSCTL_USBCLKCFG_USBSEL_M                                             \
                                0x0000000F
#define SYSCTL_USBCLKCFG_USBSEL_S                                             \
                                0
//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_SCS SYSCTL_SCS
//! @{
//
//*****************************************************************************

//
//! The frequency range of the main oscillator is 1 MHz to 20 MHz
//
#define SYSCTL_SCS_OSCRANGE_1   0x00000000

//
//! The frequency range of the main oscillator is 15 MHz to 25 MHz
//
#define SYSCTL_SCS_OSCRANGE_15  0x00000010

//
//! The main oscillator is enabled
//
#define SYSCTL_SCS_OSCEN        0x00000020

//
//! The main oscillator is ready to be used as a clock source
//
#define SYSCTL_SCS_OSCSTAT      0x00000040

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PCLKSEL0 SYSCTL_PCLKSEL0
//! @{
//
//*****************************************************************************

#define SYSCTL_PCLKSEL0_ACF_CLK_4                                             \
                                0x00000000
#define SYSCTL_PCLKSEL0_ACF_CLK_1                                             \
                                0x40000000
#define SYSCTL_PCLKSEL0_ACF_CLK_2                                             \
                                0x80000000
#define SYSCTL_PCLKSEL0_ACF_CLK_8                                             \
                                0xC0000000

#define SYSCTL_PCLKSEL0_CAN2_CLK_4                                            \
                                0x00000000
#define SYSCTL_PCLKSEL0_CAN2_CLK_1                                            \
                                0x10000000
#define SYSCTL_PCLKSEL0_CAN2_CLK_2                                            \
                                0x20000000
#define SYSCTL_PCLKSEL0_CAN2_CLK_6                                            \
                                0x30000000

#define SYSCTL_PCLKSEL0_CAN1_CLK_4                                            \
                                0x00000000
#define SYSCTL_PCLKSEL0_CAN1_CLK_1                                            \
                                0x04000000
#define SYSCTL_PCLKSEL0_CAN1_CLK_2                                            \
                                0x08000000
#define SYSCTL_PCLKSEL0_CAN1_CLK_6                                            \
                                0x0C000000

#define SYSCTL_PCLKSEL0_ADC_CLK_4                                             \
                                0x00000000
#define SYSCTL_PCLKSEL0_ADC_CLK_1                                             \
                                0x01000000
#define SYSCTL_PCLKSEL0_ADC_CLK_2                                             \
                                0x02000000
#define SYSCTL_PCLKSEL0_ADC_CLK_8                                             \
                                0x03000000

#define SYSCTL_PCLKSEL0_DAC_CLK_4                                             \
                                0x00000000
#define SYSCTL_PCLKSEL0_DAC_CLK_1                                             \
                                0x00400000
#define SYSCTL_PCLKSEL0_DAC_CLK_2                                             \
                                0x00800000
#define SYSCTL_PCLKSEL0_DAC_CLK_8                                             \
                                0x00C00000

#define SYSCTL_PCLKSEL0_SSP1_CLK_4                                            \
                                0x00000000
#define SYSCTL_PCLKSEL0_SSP1_CLK_1                                            \
                                0x00100000
#define SYSCTL_PCLKSEL0_SSP1_CLK_2                                            \
                                0x00200000
#define SYSCTL_PCLKSEL0_SSP1_CLK_8                                            \
                                0x00300000

#define SYSCTL_PCLKSEL0_SPI_CLK_4                                             \
                                0x00000000
#define SYSCTL_PCLKSEL0_SPI_CLK_1                                             \
                                0x00010000
#define SYSCTL_PCLKSEL0_SPI_CLK_2                                             \
                                0x00020000
#define SYSCTL_PCLKSEL0_SPI_CLK_8                                             \
                                0x00030000

#define SYSCTL_PCLKSEL0_I2C0_CLK_4                                            \
                                0x00000000
#define SYSCTL_PCLKSEL0_I2C0_CLK_1                                            \
                                0x00004000
#define SYSCTL_PCLKSEL0_I2C0_CLK_2                                            \
                                0x00008000
#define SYSCTL_PCLKSEL0_I2C0_CLK_8                                            \
                                0x0000C000

#define SYSCTL_PCLKSEL0_PWM1_CLK_4                                            \
                                0x00000000
#define SYSCTL_PCLKSEL0_PWM1_CLK_1                                            \
                                0x00001000
#define SYSCTL_PCLKSEL0_PWM1_CLK_2                                            \
                                0x00002000
#define SYSCTL_PCLKSEL0_PWM1_CLK_8                                            \
                                0x00003000

#define SYSCTL_PCLKSEL0_UART1_CLK_4                                           \
                                0x00000000
#define SYSCTL_PCLKSEL0_UART1_CLK_1                                           \
                                0x00000100
#define SYSCTL_PCLKSEL0_UART1_CLK_2                                           \
                                0x00000200
#define SYSCTL_PCLKSEL0_UART1_CLK_8                                           \
                                0x00000300

#define SYSCTL_PCLKSEL0_UART0_CLK_4                                           \
                                0x00000000
#define SYSCTL_PCLKSEL0_UART0_CLK_1                                           \
                                0x00000040
#define SYSCTL_PCLKSEL0_UART0_CLK_2                                           \
                                0x00000080
#define SYSCTL_PCLKSEL0_UART0_CLK_8                                           \
                                0x000000C0

#define SYSCTL_PCLKSEL0_TIMER1_CLK_4                                          \
                                0x00000000
#define SYSCTL_PCLKSEL0_TIMER1_CLK_1                                          \
                                0x00000010
#define SYSCTL_PCLKSEL0_TIMER1_CLK_2                                          \
                                0x00000020
#define SYSCTL_PCLKSEL0_TIMER1_CLK_8                                          \
                                0x00000030

#define SYSCTL_PCLKSEL0_TIMER0_CLK_4                                          \
                                0x00000000
#define SYSCTL_PCLKSEL0_TIMER0_CLK_1                                          \
                                0x00000004
#define SYSCTL_PCLKSEL0_TIMER0_CLK_2                                          \
                                0x00000008
#define SYSCTL_PCLKSEL0_TIMER0_CLK_8                                          \
                                0x0000000C

#define SYSCTL_PCLKSEL0_WDT_CLK_4                                             \
                                0x00000000
#define SYSCTL_PCLKSEL0_WDT_CLK_1                                             \
                                0x00000001
#define SYSCTL_PCLKSEL0_WDT_CLK_2                                             \
                                0x00000002
#define SYSCTL_PCLKSEL0_WDT_CLK_8                                             \
                                0x00000003

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PCLKSEL1 SYSCTL_PCLKSEL1
//! @{
//
//*****************************************************************************

#define SYSCTL_PCLKSEL1_MC_CLK_4                                              \
                                0x00000000
#define SYSCTL_PCLKSEL1_MC_CLK_1                                              \
                                0x40000000
#define SYSCTL_PCLKSEL1_MC_CLK_2                                              \
                                0x80000000
#define SYSCTL_PCLKSEL1_MC_CLK_8                                              \
                                0xC0000000

#define SYSCTL_PCLKSEL1_SYSCON_CLK_4                                          \
                                0x00000000
#define SYSCTL_PCLKSEL1_SYSCON_CLK_1                                          \
                                0x10000000
#define SYSCTL_PCLKSEL1_SYSCON_CLK_2                                          \
                                0x20000000
#define SYSCTL_PCLKSEL1_SYSCON_CLK_8                                          \
                                0x30000000

#define SYSCTL_PCLKSEL1_RIT_CLK_4                                             \
                                0x00000000
#define SYSCTL_PCLKSEL1_RIT_CLK_1                                             \
                                0x04000000
#define SYSCTL_PCLKSEL1_RIT_CLK_2                                             \
                                0x08000000
#define SYSCTL_PCLKSEL1_RIT_CLK_8                                             \
                                0x0C000000

#define SYSCTL_PCLKSEL1_I2S_CLK_4                                             \
                                0x00000000
#define SYSCTL_PCLKSEL1_I2S_CLK_1                                             \
                                0x00400000
#define SYSCTL_PCLKSEL1_I2S_CLK_2                                             \
                                0x00800000
#define SYSCTL_PCLKSEL1_I2S_CLK_8                                             \
                                0x00C00000

#define SYSCTL_PCLKSEL1_I2C2_CLK_4                                            \
                                0x00000000
#define SYSCTL_PCLKSEL1_I2C2_CLK_1                                            \
                                0x00100000
#define SYSCTL_PCLKSEL1_I2C2_CLK_2                                            \
                                0x00200000
#define SYSCTL_PCLKSEL1_I2C2_CLK_8                                            \
                                0x00300000

#define SYSCTL_PCLKSEL1_UART3_CLK_4                                           \
                                0x00000000
#define SYSCTL_PCLKSEL1_UART3_CLK_1                                           \
                                0x00040000
#define SYSCTL_PCLKSEL1_UART3_CLK_2                                           \
                                0x00080000
#define SYSCTL_PCLKSEL1_UART3_CLK_8                                           \
                                0x000C0000

#define SYSCTL_PCLKSEL1_UART2_CLK_4                                           \
                                0x00000000
#define SYSCTL_PCLKSEL1_UART2_CLK_1                                           \
                                0x00010000
#define SYSCTL_PCLKSEL1_UART2_CLK_2                                           \
                                0x00020000
#define SYSCTL_PCLKSEL1_UART2_CLK_8                                           \
                                0x00030000

#define SYSCTL_PCLKSEL1_TIMER3_CLK_4                                          \
                                0x00000000
#define SYSCTL_PCLKSEL1_TIMER3_CLK_1                                          \
                                0x00004000
#define SYSCTL_PCLKSEL1_TIMER3_CLK_2                                          \
                                0x00008000
#define SYSCTL_PCLKSEL1_TIMER3_CLK_8                                          \
                                0x0000C000

#define SYSCTL_PCLKSEL1_TIMER2_CLK_4                                          \
                                0x00000000
#define SYSCTL_PCLKSEL1_TIMER2_CLK_1                                          \
                                0x00001000
#define SYSCTL_PCLKSEL1_TIMER2_CLK_2                                          \
                                0x00002000
#define SYSCTL_PCLKSEL1_TIMER2_CLK_8                                          \
                                0x00003000

#define SYSCTL_PCLKSEL1_SSP0_CLK_4                                            \
                                0x00000000
#define SYSCTL_PCLKSEL1_SSP0_CLK_1                                            \
                                0x00000400
#define SYSCTL_PCLKSEL1_SSP0_CLK_2                                            \
                                0x00000800
#define SYSCTL_PCLKSEL1_SSP0_CLK_8                                            \
                                0x00000C00

#define SYSCTL_PCLKSEL1_I2C1_CLK_4                                            \
                                0x00000000
#define SYSCTL_PCLKSEL1_I2C1_CLK_1                                            \
                                0x00000040
#define SYSCTL_PCLKSEL1_I2C1_CLK_2                                            \
                                0x00000080
#define SYSCTL_PCLKSEL1_I2C1_CLK_8                                            \
                                0x000000C0

#define SYSCTL_PCLKSEL1_PCB_CLK_4                                             \
                                0x00000000
#define SYSCTL_PCLKSEL1_PCB_CLK_1                                             \
                                0x00000010
#define SYSCTL_PCLKSEL1_PCB_CLK_2                                             \
                                0x00000020
#define SYSCTL_PCLKSEL1_PCB_CLK_8                                             \
                                0x00000030

#define SYSCTL_PCLKSEL1_GPIOINT_CLK_4                                         \
                                0x00000000
#define SYSCTL_PCLKSEL1_GPIOINT_CLK_1                                         \
                                0x00000004
#define SYSCTL_PCLKSEL1_GPIOINT_CLK_2                                         \
                                0x00000008
#define SYSCTL_PCLKSEL1_GPIOINT_CLK_8                                         \
                                0x0000000C

#define SYSCTL_PCLKSEL1_QEI_CLK_4                                             \
                                0x00000000
#define SYSCTL_PCLKSEL1_QEI_CLK_1                                             \
                                0x00000001
#define SYSCTL_PCLKSEL1_QEI_CLK_2                                             \
                                0x00000002
#define SYSCTL_PCLKSEL1_QEI_CLK_8                                             \
                                0x00000003

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PCON SYSCTL_PCON
//! @{
//
//*****************************************************************************

//
//! Deep Power-down entry flag
//
#define SYSCTL_PCON_DPDFLAG     0x00000800

//
//! Power-down entry flag
//
#define SYSCTL_PCON_PDFLAG      0x00000400

//
//! Deep Sleep entry flag
//
#define SYSCTL_PCON_DSFLAG      0x00000200

//
//! Sleep Mode entry flag
//
#define SYSCTL_PCON_SMFLAG      0x00000100

//
//! Brown-Out Reset Disable
//
#define SYSCTL_PCON_BORD        0x00000010

//
//! Brown-Out Global Disable
//
#define SYSCTL_PCON_BOGD        0x00000008

//
//! Brown-Out Reduced Power Mode
//
#define SYSCTL_PCON_BODRPM      0x00000004

//
//! Power mode control bit 1
//
#define SYSCTL_PCON_PM1         0x00000002

//
//! Power mode control bit 0
//
#define SYSCTL_PCON_PM0         0x00000001

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_PCONP SYSCTL_PCONP
//! @{
//
//*****************************************************************************

#define SYSCTL_PCONP_PCUSB      0x80000000  // USB interface power/clock control bit
#define SYSCTL_PCONP_PCENET     0x40000000  // Ethernet block power/clock control bit
#define SYSCTL_PCONP_PCGPDMA    0x20000000  // GPDMA function power/clock control bit
#define SYSCTL_PCONP_PCI2S      0x08000000  // I2S interface power/clock control bit.
#define SYSCTL_PCONP_PCI2C2     0x04000000  // I2C interface 2 power/clock control bit.
#define SYSCTL_PCONP_PCUART3    0x02000000  // UART 3 power/clock control bit
#define SYSCTL_PCONP_PCUART2    0x01000000  // UART 2 power/clock control bit
#define SYSCTL_PCONP_PCTIM3     0x00800000  // Timer 3 power/clock control bit
#define SYSCTL_PCONP_PCTIM2     0x00400000  // Timer 2 power/clock control bit
#define SYSCTL_PCONP_PCSSP0     0x00200000  // The SSP0 interface power/clock control bit
#define SYSCTL_PCONP_PCI2C1     0x00080000  // The I2C1 interface power/clock control bit
#define SYSCTL_PCONP_PCQEI      0x00040000  // Quadrature Encoder Interface power/clock control bit.
#define SYSCTL_PCONP_PCMCPWM    0x00020000  // Motor Control PWM
#define SYSCTL_PCONP_PCRIT      0x00010000  // Repetitive Interrupt Timer power/clock control bit.
#define SYSCTL_PCONP_PCGPIO     0x00008000  // Power/clock control bit for IOCON, GPIO, and GPIO interrupts
#define SYSCTL_PCONP_PCCAN2     0x00004000  // CAN Controller 2 power/clock control bit.
#define SYSCTL_PCONP_PCCAN1     0x00002000  // CAN Controller 1 power/clock control bit
#define SYSCTL_PCONP_PCADC      0x00001000  // A/D converter (ADC) power/clock control bit
#define SYSCTL_PCONP_PCSSP1     0x00000400  // The SSP 1 interface power/clock control bit
#define SYSCTL_PCONP_PCRTC      0x00000200  // The RTC power/clock control bit
#define SYSCTL_PCONP_PCSPI      0x00000100  // The SPI interface power/clock control bit
#define SYSCTL_PCONP_PCI2C0     0x00000080  // The I2C0 interface power/clock control bit
#define SYSCTL_PCONP_PCPWM1     0x00000040  // PWM1 power/clock control bit
#define SYSCTL_PCONP_PCUART1    0x00000010  // UART1 power/clock control bit
#define SYSCTL_PCONP_PCUART0    0x00000008  // UART0 power/clock control bit
#define SYSCTL_PCONP_PCTIM1     0x00000004  // Timer/Counter 1 power/clock control bit
#define SYSCTL_PCONP_PCTIM0     0x00000002  // Timer/Counter 0 power/clock control bit

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_CLKSRCSEL SYSCTL_CLKSRCSEL
//! @{
//
//*****************************************************************************

//
//!
//
#define SYSCTL_CLKSRCSEL_CLKSRC_M                                             \
                                0x00000003

//
//! Selects the Internal RC oscillator as the PLL0 clock source (default)
//
#define SYSCTL_CLKSRCSEL_INTRC  0x00000000

//
//! Selects the main oscillator as the PLL0 clock source (default)
//
#define SYSCTL_CLKSRCSEL_MAINOSC                                              \
                                0x00000001

//
//! Selects the RTC oscillator as the PLL0 clock source (default)
//
#define SYSCTL_CLKSRCSEL_OSC_32 0x00000002

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_RSID SYSCTL_RSID
//! @{
//
//*****************************************************************************

//
//! BOD reset
//
#define SYSCTL_RSID_BODR        0x00000008

//
//! Watchdog reset
//
#define SYSCTL_RSID_WDTR        0x00000004

//
//! the RESET signal reset
//
#define SYSCTL_RSID_EXTR        0x00000002

//
//! POR signal reset
//
#define SYSCTL_RSID_POR         0x00000001

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_SYSCTL_CLKOUTCFG SYSCTL_CLKOUTCFG
//! @{
//
//*****************************************************************************

//
//! CLKOUT activity indication
//
#define SYSCTL_CLKOUTCFG_CLKOUT_ACT                                           \
                                0x00000200

//
//! CLKOUT enable control
//
#define SYSCTL_CLKOUTCFG_CLKOUT_EN                                            \
                                0x00000100

//
//! Integer value to divide the output clock by, minus one.
//
#define SYSCTL_CLKOUTCFG_CLKOUTDIV_M                                          \
                                0x000000F0
#define SYSCTL_CLKOUTCFG_CLKOUTDIV_S                                          \
                                4

//
//! Selects the clock source for the CLKOUT function
//
#define SYSCTL_CLKOUTCFG_CLKOUTSEL_M                                          \
                                0x0000000F
#define SYSCTL_CLKOUTCFG_CLKOUTSEL_S                                          \
                                0
#define SYSCTL_CLKOUTCFG_CLKOUTSEL_CPUCLK                                     \
                                0x00000000
#define SYSCTL_CLKOUTCFG_CLKOUTSEL_MAINOSC                                    \
                                0x00000001
#define SYSCTL_CLKOUTCFG_CLKOUTSEL_INTRC                                      \
                                0x00000002
#define SYSCTL_CLKOUTCFG_CLKOUTSEL_USBCLK                                     \
                                0x00000003
#define SYSCTL_CLKOUTCFG_CLKOUTSEL_RTCOSC                                     \
                                0x00000004
//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_Register_FLASH_FLASHCFG FLASH_FLASHCFG
//! @{
//
//*****************************************************************************

//
//! Flash access time
//
#define FLASH_FLASHCFG_FLASHTIM_M                                             \
                                0x0000F000
//
//! Flash accesses use 1 CPU clock. Use for up to 20 MHz CPU clock
//
#define FLASH_FLASHCFG_FLASHTIM_1                                             \
                                0x00000000
//
//! Flash accesses use 2 CPU clocks. Use for up to 40 MHz CPU clock
//
#define FLASH_FLASHCFG_FLASHTIM_2                                             \
                                0x00001000
//
//! Flash accesses use 3 CPU clocks. Use for up to 60 MHz CPU clock
//
#define FLASH_FLASHCFG_FLASHTIM_3                                             \
                                0x00002000
//
//! Flash accesses use 4 CPU clocks. Use for up to 80 MHz CPU clock
//
#define FLASH_FLASHCFG_FLASHTIM_4                                             \
                                0x00003000
//
//! Flash accesses use 5 CPU clocks.  Use for up to 100 MHz CPU clock.
//! Use for up to 120 Mhz for LPC1759 and LPC1769 only
//
#define FLASH_FLASHCFG_FLASHTIM_5                                             \
                                0x00004000
//
//! Flash accesses use 6 CPU clocks. This ¡°safe¡± setting will work under any conditions
//
#define FLASH_FLASHCFG_FLASHTIM_6                                             \
                                0x00005000

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

#endif // __XHW_SYSCTL_H__
