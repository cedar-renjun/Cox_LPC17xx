//*****************************************************************************
//
//! \file xhw_sysctl.h
//! \brief Macros used when accessing the system control hardware.
//! \version V2.2.1.0
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
//! 
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


#ifndef __XHW_SYSCTL_H__
#define __XHW_SYSCTL_H__

#include "xhw_types.h"
#include "xhw_memmap.h"



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
//! \addtogroup STM32F1xx_SysCtl_Register STM32F1xx SysCtl Register
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
//! \addtogroup STM32F1xx_SysCtl_Register_Offsets STM32F1xx SysCtl Register Offsets(Map)
//! \brief Defines for the system control register addresses.through 
//! <b>SysCtl_BASE + offset</b>.
//! @{
//
//*****************************************************************************

//
//! Flash Access control register 
//
#define FLASH_ACR               0x40022000

//
//! Clock control register
//
#define RCC_CR                  0x40021000

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup STM32F1xx_SysCtl_Register_RCC_CR SysCtl Register RCC_CR
//! \brief Defines for the bit fields in the GCR_PDID register.
//! @{
//
//*****************************************************************************

//
//! PLL3 clock ready flag
//
#define RCC_CR_PLL3RDY          0x20000000

//
//! PLL3 enable
//
#define RCC_CR_PLL3ON           0x10000000

//
//! PLL2 clock ready flag
//
#define RCC_CR_PLL2RDY          0x08000000

//
//! PLL2 enable
//
#define RCC_CR_PLL2ON           0x04000000

//
//! PLL clock ready flag
//
#define RCC_CR_PLLRDY           0x02000000

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

//! Flash Accelerator Configuration Register Controls flash access timing.
//  SYSCTL_BASE --> 0x400FC000
#define FLASHCFG                (SYSCTL_BASE + (unsigned long)0x000)

//! PLL0 Control
#define PLL0CON                 (SYSCTL_BASE + (unsigned long)0x080)

//! PLL0 Configuration 
#define PLL0CFG                 (SYSCTL_BASE + (unsigned long)0x084)

//! PLL0 Status 
#define PLL0STAT                (SYSCTL_BASE + (unsigned long)0x088)

//! PLL0 Feed 
#define PLL0FEED                (SYSCTL_BASE + (unsigned long)0x08C)

//! PLL1 Control                
#define PLL1CON                 (SYSCTL_BASE + (unsigned long)0x0A0)

//! PLL1 Configuration 
#define PLL1CFG                 (SYSCTL_BASE + (unsigned long)0x0A4)

//! PLL1 Status 
#define PLL1STAT                (SYSCTL_BASE + (unsigned long)0x0A8)

//! PLL1 Feed 
#define PLL1FEED                (SYSCTL_BASE + (unsigned long)0x0AC)

//! Power Control 
#define PCON                    (SYSCTL_BASE + (unsigned long)0x0C0)

//! Power Control for Peripherals
#define PCONP                   (SYSCTL_BASE + (unsigned long)0x0C4)

//! External Memory Controller Clock Selection 
#define EMCCLKSEL               (SYSCTL_BASE + (unsigned long)0x100)

//! CPU Clock Selection 
#define CCLKSEL                 (SYSCTL_BASE + (unsigned long)0x104)

//! USB Clock Selection 
#define USBCLKSEL               (SYSCTL_BASE + (unsigned long)0x108)

//! Clock Source Select Register
#define CLKSRCSEL               (SYSCTL_BASE + (unsigned long)0x10C)

//! Allows clearing the current CAN channel sleep state
//! as well as reading that state.
#define CANSLEEPCLR             (SYSCTL_BASE + (unsigned long)0x110)

//! Allows reading the wake-up state of the CAN channels.
#define CANWAKEFLAGS            (SYSCTL_BASE + (unsigned long)0x114)

//! External Interrupt Flag Register
#define EXTINT                  (SYSCTL_BASE + (unsigned long)0x140)

//! External Interrupt Mode 
#define EXTMODE                 (SYSCTL_BASE + (unsigned long)0x148)

//! External Interrupt Polarity Register
#define EXTPOLAR                (SYSCTL_BASE + (unsigned long)0x14C)

//! Reset Source Ident ification Register
#define RSID                    (SYSCTL_BASE + (unsigned long)0x180)

//! Matrix arbitration           
#define MATRIXARB               (SYSCTL_BASE + (unsigned long)0x188)

//! System Control and Status
#define SCS                     (SYSCTL_BASE + (unsigned long)0x1A0)

//! Peripheral Clock Selection. For LPC 17_7x_8x
#define PCLKSEL                 (SYSCTL_BASE + (unsigned long)0x1A8)

//! Peripheral Clock Selection 0. For LPC 17_nx (n = 5/6/7/8)
#define PCLKSEL0                (SYSCTL_BASE + (unsigned long)0x1A8)

//! Peripheral Clock Selection 1. For LPC 17_5x_6x
#define PCLKSEL1                (SYSCTL_BASE + (unsigned long)0x1AC)

//! Power boost 
#define PBOOST                  (SYSCTL_BASE + (unsigned long)0x1B0)

//! SPIFI Clock Selection 
#define SPIFICLKSEL             (SYSCTL_BASE + (unsigned long)0x1B4)

//! LCD Clock configuration 
#define LCD_CFG                 (SYSCTL_BASE + (unsigned long)0x1B8)

//! USB Interrupt Status
#define USBINTST                (SYSCTL_BASE + (unsigned long)0x1C0)

//! Selects between alternative requests on DMA channels 0 through 7 and 10 through 15
#define DMAREQSEL               (SYSCTL_BASE + (unsigned long)0x1C4)

//! Clock Output Configuration 
#define CLKOUTCFG               (SYSCTL_BASE + (unsigned long)0x1C8)

//! Individual peripheral reset control bits
#define RSTCON0                 (SYSCTL_BASE + (unsigned long)0x1CC)

//! Individual peripheral reset control bits
#define RSTCON1                 (SYSCTL_BASE + (unsigned long)0x1D0)

//! Values for the four programmable delays associated with SDRAM operation.
#define EMCDLYCTL               (SYSCTL_BASE + (unsigned long)0x1DC)

//! Controls the calibration counter for programmable delays and returns the result value.
#define EMCCAL                  (SYSCTL_BASE + (unsigned long)0x1E0)


//FLASHCFG {{

//! Flash access time MASK
#define FLASHCFG_FLASHTIM_M     ((unsigned long)0x0000F000)
//! Flash accesses use 1 CPU clock. Use for up to 20MHz Cpu clock with power boost off.
#define FLASHCFG_FLASHTIM_20M   ((unsigned long)0x00000000)
//! Flash accesses use 2 CPU clock. Use for up to 40MHz Cpu clock with power boost off.
#define FLASHCFG_FLASHTIM_40M   ((unsigned long)0x00001000)
//! Flash accesses use 3 CPU clock. Use for up to 60MHz Cpu clock with power boost off.
#define FLASHCFG_FLASHTIM_60M   ((unsigned long)0x00002000)
//! Flash accesses use 4 CPU clock. Use for up to 80MHz Cpu clock with power boost off.
#define FLASHCFG_FLASHTIM_80M   ((unsigned long)0x00003000)
//! Flash accesses use 5 CPU clock. Use for up to 100MHz Cpu clock with power boost off.
#define FLASHCFG_FLASHTIM_100M  ((unsigned long)0x00004000)
//! Flash accesses use 6 CPU clock. Safe setting for any allowed conditions.
#define FLASHCFG_FLASHTIM_ANY   ((unsigned long)0x00005000)

//FLASHCFG }}


//PLL0CON {{
#define PLL0CON_EN              ((unsigned long)0x00000001)
//PLL0CON }}

//PLL1CON {{
#define PLL1CON_EN              ((unsigned long)0x00000001)
//PLL1CON }}

//PLLCON {{
#define PLLCON_EN              ((unsigned long)0x00000001)
//PLLCON }}

//PLLCFG {{

//! PLL Multiplier value. Supplies the value "M" in the PLL frequency calculations.
//! The value stored here is the M value minus 1.
#define PLLCFG_MSEL_M          ((unsigned long)0x0000001F)
#define PLLCFG_MSEL_S          ((unsigned long)0x00000000)

//! PLL Divider value. Supplies the value "P" in the PLL frequency calculations.
//! The value is encodec as follows:
//! 00  --> divide by 1
//! 01  --> divide by 2
//! 10  --> divide by 4
//! 11  --> divide by 8
#define PLLCFG_PSEL_M          ((unsigned long)0x00000060)
#define PLLCFG_PSEL_S          ((unsigned long)0x00000005)

#define PLLCFG_PSEL_DIV_1      BIT_32_ALL_0
#define PLLCFG_PSEL_DIV_2      BIT_32_5
#define PLLCFG_PSEL_DIV_4      BIT_32_6
#define PLLCFG_PSEL_DIV_8      (BIT_32_5 | BIT_32_6)

//PLLCFG }}

//PLLSTAT {{

#define PLLSTAT_MSEL_M         BIT_MASK(32, 4, 0)
#define PLLSTAT_PSEL_M         BIT_MASK(32, 6, 5)
#define PLLSTAT_PLLE_STAT      BIT_32_8
#define PLLSTAT_PLOCK          BIT_32_10

//PLLSTAT }}

//PLLFEED {{

//! The PLL feed sequence must be written to this register in order for the
//! related PLL's configuration and control register changes to take effect.
#define PLLFEED_PLLFEED         BIT_MASK(32, 7, 0)

//PLLFEED }}


//PCON {{

//! This bit controls entry to the power-down mode.
#define PCON_PM0                BIT_32_0

//! This bit controls entry to the deep power-down mode.
#define PCON_PM1                BIT_32_1

//! Brown-Out Reduced Power Mode.
#define PCON_BODRPM             BIT_32_2

//! Brown-Out global Disable.
#define PCON_BOGD               BIT_32_3

//! Brown-Out Reset Disable
#define PCON_BORD               BIT_32_4

//! Sleep Mode entry flag
#define PCON_SMFLAG             BIT_32_8

//! Deep Sleep Mode entry flag
#define PCON_DSFLAG             BIT_32_9

//! Power-down entry flag
#define PCON_PDFLAG             BIT_32_10

//! Deep Power-down entry flag
#define PCON_DPDFLAG            BIT_32_11

//PCON }}


//PCONP {{

//! LCD Controller power/clock control bit.
#define PCONP_PCLCD             BIT_32_0

//! Timer/Counter 0 power/clock control bit.
#define PCONP_PCTIM0            BIT_32_1

//! Timer/Counter 1 power/clock control bit.
#define PCONP_PCTIM1            BIT_32_2

//! UART0 Power/clock control bit.
#define PCONP_PCUART0           BIT_32_3

//! UART1 Power/clock control bit.
#define PCONP_PCUART1           BIT_32_4

//! PWM0 Power/Clock control bit.
#define PCONP_PCPWM0            BIT_32_5

//! PWM1 Power/Clock control bit.
#define PCONP_PCPWM1            BIT_32_6

//! I2C0 Interface Power/Clock control bit.
#define PCONP_PCI2C0            BIT_32_7

//! UART4 power/clock control bit. 0
#define PCONP_PCUART4 BIT_32_8 

//! RTC and Event Monitor/Recorder power/clock control bit.
#define PCONP_PCRTC             BIT_32_9  

//! SSP 1 interface power/clock control bit.
#define PCONP_PCSSP1            BIT_32_10 

//! External Memory Controller power/clock control bit.
#define PCONP_PCEMC             BIT_32_11 

//! A/D converter (ADC) power/clock control bit.
#define PCONP_PCADC             BIT_32_12 

//! CAN Controller 1 power/clock control bit.
#define PCONP_PCCAN1            BIT_32_13 

//! CAN Controller 2 power/clock control bit.
#define PCONP_PCCAN2            BIT_32_14 

//! Power/clock control bit for IOCON, GPIO, and GPIO interrupts.
#define PCONP_PCGPIO            BIT_32_15 

//! SPI Flash Interface power/clock control bit (LPC1773 only).
#define PCONP_PCSPIFI           BIT_32_16 

//! Motor Control PWM power/clock control bit.
#define PCONP_PCMCPWM           BIT_32_17 

//! Quadrature Encoder Interface power/clock control bit.
#define PCONP_PCQEI             BIT_32_18 

//! I2C1 interface power/clock control bit.
#define PCONP_PCI2C1            BIT_32_19 

//! SSP2 interface power/clock control bit.
#define PCONP_PCSSP2            BIT_32_20 

//! SSP0 interface power/clock control bit.
#define PCONP_PCSSP0            BIT_32_21 

//! Timer 2 power/clock control bit.
#define PCONP_PCTIM2            BIT_32_22 

//! Timer 3 power/clock control bit.
#define PCONP_PCTIM3            BIT_32_23 

//! UART 2 power/clock control bit.
#define PCONP_PCUART2           BIT_32_24 

//! UART 3 power/clock control bit.
#define PCONP_PCUART3           BIT_32_25 

//! I2C interface 2 power/clock control bit.
#define PCONP_PCI2C2            BIT_32_26 

//! I2S interface power/clock control bit.
#define PCONP_PCI2S             BIT_32_27 

//! SD Card interface power/clock control bit.
#define PCONP_PCSDC             BIT_32_28 

//! GPDMA function power/clock control bit.
#define PCONP_PCGPDMA           BIT_32_29 

//! Ethernet block power/clock control bit.
#define PCONP_PCENET            BIT_32_30 

//! USB interface power/clock control bit.
#define PCONP_PCUSB             BIT_32_31 

//PCONP }}

//EMCCLKSEL {{

//! Selects the EMC clock rate relative to the CPU clock.
#define EMCCLKSEL_EMCDIV        BIT_32_0

//EMCCLKSEL }}

//CCLKSEL {{

//! Selects the divide value for creating the CPU clock (CCLK) from the selected
//! clock source.
#define CCLKSEL_CCLKDIV         BIT_MASK(32, 4, 0)

//! Selects the input clock for the CPU clock divider.
#define CCLKSEL_CCLKSEL         BIT_32_8

//CCLKSEL }}

//USBCLKSEL {{

//! Selects the divide value for creating the USB clock from the selected PLL output.
#define USBCLKSEL_USBDIV        BIT_MASK(32, 4, 0)

//! Selects the input clock for the USB clock divider.
#define USBCLKSEL_USBSEL        BIT_MASK(32, 9, 8)

//USBCLKSEL }}

//CLKSRCSEL {{

//! Select the clock source for sysclk and PLL0
#define CLKSRCSEL_CLKSRC        BIT_32_0

//CLKSRCSEL }}

//CANSLEEPCLR {{

//! Sleep status and control for CAN channel 1.
#define CANSLEEPCLR_CAN1SLEEP   BIT_32_1

//! Sleep status and control for CAN channel 2.
#define CANSLEEPCLR_CAN2SLEEP   BIT_32_2

//CANSLEEPCLR }}

//CANWAKEFLAGS {{

//! Wake-up status for CAN channel 1.
#define CANWAKEFLAGS_CAN1WAKE   BIT_32_1

//! Wake-up status for CAN channel 2.
#define CANWAKEFLAGS_CAN2WAKE   BIT_32_2

//CANWAKEFLAGS }}

//EXTINT {{

//! External interrupt flag for INT0
#define EXTINT_EINT0            BIT_32_0

//! External interrupt flag for INT1
#define EXTINT_EINT1            BIT_32_1

//! External interrupt flag for INT2
#define EXTINT_EINT2            BIT_32_2

//! External interrupt flag for INT3
#define EXTINT_EINT3            BIT_32_3

//EXTINT }}

//EXTMODE {{

//! Level or edge sensitivity select for EXINT0.
#define EXTMODE_EXTMODE0        BIT_32_0
#define EXTMODE_EXTMODE0_LEVEL  BIT_32_ALL_0
#define EXTMODE_EXTMODE0_EDGE   BIT_32_0

//! Level or edge sensitivity select for EXINT1.
#define EXTMODE_EXTMODE1        BIT_32_1
#define EXTMODE_EXTMODE1_LEVEL  BIT_32_ALL_0
#define EXTMODE_EXTMODE1_EDGE   BIT_32_1

//! Level or edge sensitivity select for EXINT2.
#define EXTMODE_EXTMODE2        BIT_32_2
#define EXTMODE_EXTMODE2_LEVEL  BIT_32_ALL_0
#define EXTMODE_EXTMODE2_EDGE   BIT_32_2

//! Level or edge sensitivity select for EXINT3.
#define EXTMODE_EXTMODE3        BIT_32_3
#define EXTMODE_EXTMODE3_LEVEL  BIT_32_ALL_0
#define EXTMODE_EXTMODE3_EDGE   BIT_32_3

//EXTMODE }}

//EXTPOLAR {{

//! External interrupt polarity for EINT0
#define EXTPOLAR_EXTPOLAR0      BIT_32_0
//! Low-active or falling-edge sensitive
#define EXTPOLAR_EXTPOLAR0_L_F  BIT_32_ALL_0
//! High-active or Rising-edge sensitive
#define EXTPOLAR_EXTPOLAR0_H_R  BIT_32_0

//! External interrupt polarity for EINT1
#define EXTPOLAR_EXTPOLAR1      BIT_32_1
//! Low-active or falling-edge sensitive
#define EXTPOLAR_EXTPOLAR1_L_F  BIT_32_ALL_0
//! High-active or Rising-edge sensitive
#define EXTPOLAR_EXTPOLAR1_H_R  BIT_32_1

//! External interrupt polarity for EINT2
#define EXTPOLAR_EXTPOLAR2      BIT_32_2
//! Low-active or falling-edge sensitive
#define EXTPOLAR_EXTPOLAR2_L_F  BIT_32_ALL_0
//! High-active or Rising-edge sensitive
#define EXTPOLAR_EXTPOLAR2_H_R  BIT_32_2

//! External interrupt polarity for EINT3
#define EXTPOLAR_EXTPOLAR3      BIT_32_3
//! Low-active or falling-edge sensitive
#define EXTPOLAR_EXTPOLAR3_L_F  BIT_32_ALL_0
//! High-active or Rising-edge sensitive
#define EXTPOLAR_EXTPOLAR3_H_R  BIT_32_3

//EXTPOLAR }}

//RSID {{

//! Power on reset
#define RSID_POR                BIT_32_0

//! External reset signal
#define RSID_EXTR               BIT_32_1

//! Watchdog Timer reset
#define RSID_WDTR               BIT_32_2

//! Brown-out reset
#define RSID_BODR               BIT_32_3

//! System reset requet reset
#define RSID_SYSRESET           BIT_32_4

//! Lockup reset
#define RSID_LOCKUP             BIT_32_5

//RSID }}

//MATRIXARB {{

//! I-code bus priority. Should be lower than PRI_DCODE for proper operation.
#define MATRIXARB_PRI_ICODE     BIT_MASK(32, 1, 0)

//! D-Code bus priority.
#define MATRIXARB_PRI_DCODE     BIT_MASK(32, 3, 2)

//! System bus priority
#define MATRIXARB_PRI_SYS       BIT_MASK(32, 5, 4)

//! General Purpose DMA controller priority
#define MATRIXARB_PRI_GPDMA     BIT_MASK(32, 7, 6)

//! Ethernet DMA priority
#define MATRIXARB_PRI_ETH       BIT_MASK(32, 9, 8)

//! LCD DMA priority
#define MATRIXARB_PRI_LCD       BIT_MASK(32, 11, 10)

//! USB DMA priority
#define MATRIXARB_PRI_USB       BIT_MASK(32, 13, 12)

//! ROM latency select, should awalys be 0.
#define MATRIXARB_ROM_LAT       BIT_32_16

//MATRIXARB }}

//SCS {{

//! EMC Shift control.
#define SCS_EMCSC               BIT_32_0

//! EMC Reset Disable.
#define SCS_EMCRD               BIT_32_1

//! External Memory controller burst control.
#define SCS_EMCBC               BIT_32_2

//! MCIPWR Active level
#define SCS_MCIPWRAL            BIT_32_3

//! Main oscillator range select
#define SCS_OSCRS               BIT_32_4

//! Main oscillator enable
#define SCS_OSCEN               BIT_32_5

//! Main oscillator status
#define SCS_OSCSTAT             BIT_32_6
#define SCS_OSCSTAT_RDY         BIT_32_6
#define SCS_OSCSTAT_NOTRDY      BIT_32_ALL_0

//SCS }}

//PCLKSEL {{

//! Selects the divide value for the clock used for all APB peripherals.
#define PCLKSEL_PCLKDIV_M       BIT_MASK(32, 4, 0)

//PCLKSEL }}

//PCLKSEL0 {{ For LPC 17_5x_6x

//! Peripheral clock selection for WDT.
#define PCLKSEL0_WDT_M         BIT_MASK(32, 1, 0)

//! Peripheral clock selection for TIMER0.
#define PCLKSEL0_TIMER0_M      BIT_MASK(32, 3, 2)

//! Peripheral clock selection for TIMER1.
#define PCLKSEL0_TIMER1_M      BIT_MASK(32, 5, 4)

//! Peripheral clock selection for UART0.
#define PCLKSEL0_UART0_M       BIT_MASK(32, 7, 6)

//! Peripheral clock selection for UART1.
#define PCLKSEL0_UART1_M       BIT_MASK(32, 9, 8)

//! Peripheral clock selection for PWM1.
#define PCLKSEL0_PWM1_M        BIT_MASK(32, 13, 12)

//! Peripheral clock selection for I2C0.
#define PCLKSEL0_I2C0_M        BIT_MASK(32, 15, 14)

//! Peripheral clock selection for SPI.
#define PCLKSEL0_SPI_M         BIT_MASK(32, 17, 16)

//! Peripheral clock selection for SSP1.
#define PCLKSEL0_SSP1_M        BIT_MASK(32, 21, 20)

//! Peripheral clock selection for DAC.
#define PCLKSEL0_DAC_M         BIT_MASK(32, 23, 22) 

//! Peripheral clock selection for ADC.
#define PCLKSEL0_ADC_M         BIT_MASK(32, 25, 24) 

//! Peripheral clock selection for CAN1.
#define PCLKSEL0_CAN1_M        BIT_MASK(32, 27, 26) 

//! Peripheral clock selection for CAN2.
#define PCLKSEL0_CAN2_M        BIT_MASK(32, 29, 28) 

//! Peripheral clock selection for CAN acceptance filtering.
#define PCLKSEL0_ACF_M         BIT_MASK(32, 31, 30) 

//PCLKSEL0 }}


//PCLKSEL1 {{

//! Peripheral clock selection for the Quadrature Encoder Interface.
#define PCLKSEL1_QEI_M         BIT_MASK(32, 1 , 0) 

//! Peripheral clock selection for GPIO interrupts
#define PCLKSEL1_GPIOINT_M     BIT_MASK(32, 3 , 2) 

//! Peripheral clock selection for the Pin Connect block
#define PCLKSEL1_PCB_M         BIT_MASK(32, 5 , 4) 

//! Peripheral clock selection for I2C1
#define PCLKSEL1_I2C1_M        BIT_MASK(32, 7 , 6) 

//! Peripheral clock selection for SSP0
#define PCLKSEL1_SSP0_M        BIT_MASK(32, 11,10) 

//! Peripheral clock selection for TIMER2
#define PCLKSEL1_TIMER2_M      BIT_MASK(32, 13,12) 

//! Peripheral clock selection for TIMER3
#define PCLKSEL1_TIMER3_M      BIT_MASK(32, 15,14) 

//! Peripheral clock selection for UART2
#define PCLKSEL1_UART2_M       BIT_MASK(32, 17,16) 

//! Peripheral clock selection for UART3
#define PCLKSEL1_UART3_M       BIT_MASK(32, 19,18) 

//! Peripheral clock selection for I2C2
#define PCLKSEL1_I2C2_M        BIT_MASK(32, 21,20) 

//! Peripheral clock selection for I2S
#define PCLKSEL1_I2S_M         BIT_MASK(32, 23,22) 

//! Peripheral clock selection for Repetitive Interrupt Timer
#define PCLKSEL1_RIT_M         BIT_MASK(32, 27,26) 

//! Peripheral clock selection for the System Control block
#define PCLKSEL1_SYSCON_M      BIT_MASK(32, 29,28) 

//! Peripheral clock selection for the Motor Control PWM
#define PCLKSEL1_MC_M          BIT_MASK(32, 31,30) 

//PCLKSEL1 }}

//PBOOST {{

//! Boost control bits
#define PBOOST_BOOST_M          BIT_MASK(32, 1, 0)

//! Boost is off, operation must below 100 Mhz
#define PBOOST_BOOST_OFF        BIT_32_ALL_0

//! Boost is on, operation up to 120 Mhz is supported
#define PBOOST_BOOST_ON         (BIT_32_0 | BIT_32_1)

//PBOOST }}

//SPIFICLKSEL {{

//! Selects the divide value for creating the SPIFI clock from the selected clock source.
#define SPIFICLKSEL_SPIFIDIV_M  BIT_MASK(32, 4, 0)

//! Selects the input clock for the USB clock divider
#define SPIFICLKSEL_SPIFISEL_M  BIT_MASK(32, 9, 8)

//! SysClk is used as the input to the SPIFI clock divider
#define SPIFICLKSEL_SPIFISEL_SYSCLK    BIT_32_ALL_0

//! The output of the main PLL is used as the input to the SPIFI clock divider
#define SPIFICLKSEL_SPIFISEL_M_PLL     BIT_32_8

//! The output of the Alt PLL is used as the input to the SPIFI clock divider
#define SPIFICLKSEL_SPIFISEL_A_PLL     BIT_32_9

//SPIFICLKSEL }}

//LCD_CFG {{

//! LCD panel clock prescaler selection. The value in the this register plus 1
//! is used to divide the selected input clock to produce the panel clock.
#define LCD_CFG_CLKDIV_M        BIT_MASK(32, 4, 0)

//LCD_CFG }}

//USBINTST {{
//! Low priority interrupt line status. 
#define USBINTST_USB_INT_REQ_LP  BIT_32_0  
//! High priority interrupt line status.
#define USBINTST_USB_INT_REQ_HP  BIT_32_1  
//! DMA interrupt line status.
#define USBINTST_USB_INT_REQ_DMA BIT_32_2  
//! USB host interrupt line status.
#define USBINTST_USB_HOST_INT    BIT_32_3  
//! External ATX interrupt line status.
#define USBINTST_USB_ATX_INT     BIT_32_4  
//! OTG interrupt line status.
#define USBINTST_USB_OTG_INT     BIT_32_5  
//! I2C module interrupt line status.
#define USBINTST_USB_I2C_INT     BIT_32_6  
//! USB need clock indicator.
#define USBINTST_USB_NEED_CLK    BIT_32_8  
//! Enable all USB interrupts.
#define USBINTST_EN_USB_INTS     BIT_32_31 

//USBINTST }}


//DMAREQSEL {{
//! Selects the DMA request for GPDMA input 0:
#define DMAREQSEL_DMASEL00              BIT_32_0
#define DMAREQSEL_DMASEL00_UNUSED       BIT_32_ALL_0
#define DMAREQSEL_DMASEL00_TIMER0_M0    BIT_32_0

//! Selects the DMA request for GPDMA input 1:
#define DMAREQSEL_DMASEL01              BIT_32_1
#define DMAREQSEL_DMASEL01_SD           BIT_32_ALL_0
#define DMAREQSEL_DMASEL01_TIMER0_M1    BIT_32_1

//! DMASEL02 Selects the DMA request for GPDMA input 2:
#define DMAREQSEL_DMASEL02              BIT_32_2
#define DMAREQSEL_DMASEL02_SSP0_TX      BIT_32_ALL_0
#define DMAREQSEL_DMASEL02_TIMER1_M0    BIT_32_2

//! DMASEL03 Selects the DMA request for GPDMA input 3:
#define DMAREQSEL_DMASEL03              BIT_32_3
#define DMAREQSEL_DMASEL03_SSP0_RX      BIT_32_ALL_0
#define DMAREQSEL_DMASEL03_TIMER1_M1    BIT_32_3

//! DMASEL04 Selects the DMA request for GPDMA input 4:
#define DMAREQSEL_DMASEL04              BIT_32_4
#define DMAREQSEL_DMASEL04_SSP1_TX      BIT_32_ALL_0
#define DMAREQSEL_DMASEL04_TIMER2_M0    BIT_32_4

//! DMASEL05 Selects the DMA request for GPDMA input 5:
#define DMAREQSEL_DMASEL05              BIT_32_5
#define DMAREQSEL_DMASEL05_SSP1_RX      BIT_32_ALL_0
#define DMAREQSEL_DMASEL05_TIMER2_M1    BIT_32_5

//! Selects the DMA request for GPDMA input 6:
#define DMAREQSEL_DMASEL06              BIT_32_6
#define DMAREQSEL_DMASEL06_SSP2_TX      BIT_32_ALL_0
#define DMAREQSEL_DMASEL06_I2C_CH0      BIT_32_6

//! Selects the DMA request for GPDMA input 7:
#define DMAREQSEL_DMASEL07              BIT_32_7
#define DMAREQSEL_DMASEL07_SSP2_RX      BIT_32_ALL_0
#define DMAREQSEL_DMASEL07_I2C_CH1      BIT_32_7

//! Selects the DMA request for GPDMA input 10:
#define DMAREQSEL_DMASEL10              BIT_32_10
#define DMAREQSEL_DMASEL10_UART0_TX     BIT_32_ALL_0
#define DMAREQSEL_DMASEL10_UART3_TX     BIT_32_10

//! Selects the DMA request for GPDMA input 11:
#define DMAREQSEL_DMASEL11              BIT_32_11
#define DMAREQSEL_DMASEL11_UART0_RX     BIT_32_ALL_0
#define DMAREQSEL_DMASEL11_UART3_RX     BIT_32_11

//! Selects the DMA request for GPDMA input 12:
#define DMAREQSEL_DMASEL12              BIT_32_12
#define DMAREQSEL_DMASEL12_UART1_TX     BIT_32_ALL_0
#define DMAREQSEL_DMASEL12_UART4_TX     BIT_32_12

//! Selects the DMA request for GPDMA input 13:
#define DMAREQSEL_DMASEL13              BIT_32_13
#define DMAREQSEL_DMASEL13_UART1_RX     BIT_32_ALL_0
#define DMAREQSEL_DMASEL13_UART4_RX     BIT_32_13

//! Selects the DMA request for GPDMA input 14:
#define DMAREQSEL_DMASEL14              BIT_32_14
#define DMAREQSEL_DMASEL14_UART2_TX     BIT_32_ALL_0
#define DMAREQSEL_DMASEL14_TIMER3_M0    BIT_32_14

//! Selects the DMA request for GPDMA input 15:
#define DMAREQSEL_DMASEL15              BIT_32_15
#define DMAREQSEL_DMASEL15_UART2_RX     BIT_32_ALL_0
#define DMAREQSEL_DMASEL15_TIMER3_M1    BIT_32_15

//DMAREQSEL }}

//CLKOUTCFG {{

//! Selects the clock source for the CLKOUT function.
#define CLKOUTCFG_CLKOUTSEL_M   BIT_MASK(32, 3, 0)

//! Selects the CPU clock as the CLKOUT source.
#define CLKOUTCFG_CLKOUTSEL_CPU      BIT_32_ALL_0

//! Selects the main oscillator as the CLKOUT source.
#define CLKOUTCFG_CLKOUTSEL_M_OSC    BIT_32_0

//! Selects the Internal RC oscillator as the CLKOUT source.
#define CLKOUTCFG_CLKOUTSEL_IRC      BIT_32_1

//! Selects the USB clock as the CLKOUT source.
#define CLKOUTCFG_CLKOUTSEL_USB      (BIT_32_1 | BIT_32_0)

//! Selects the RTC oscillator as the CLKOUT source.
#define CLKOUTCFG_CLKOUTSEL_RTC      BIT_32_2

//! Selects the SPIFI clock as the CLKOUT source.
#define CLKOUTCFG_CLKOUTSEL_SPIFI    (BIT_32_2 | BIT_32_0)

//! Selects the Watchdog oscillator as the CLKOUT source.
#define CLKOUTCFG_CLKOUTSEL_WDT      (BIT_32_2 | BIT_32_1)

//! Integer value to divide the output clock by, minus one.
#define CLKOUTCFG_CLKOUTDIV_M        BIT_MASK(32, 7, 4)

//! CLKOUT enable control, allows switching the CLKOUT source without glitches.
#define CLKOUTCFG_CLKOUT_EN          BIT_32_8

//! CLKOUT activity indication
#define CLKOUTCFG_CLKOUT_ACT         BIT_32_9

//CLKOUTCFG }}

//RSTCON0 {{
//! LCD controller reset control bit
#define RSTCON0_RSTLCD                 BIT_32_0  

//! Timer/Counter 0 reset control bit
#define RSTCON0_RSTTIM0                BIT_32_1  

//! Timer/Counter 1 reset control bit
#define RSTCON0_RSTTIM1                BIT_32_2  

//! UART0 reset control bit
#define RSTCON0_RSTUART0               BIT_32_3  

//! UART1 reset control bit
#define RSTCON0_RSTUART1               BIT_32_4  

//! PWM0 reset control bit
#define RSTCON0_RSTPWM0                BIT_32_5  

//! PWM1 reset control bit
#define RSTCON0_RSTPWM1                BIT_32_6  

//! The I2C0 interface reset control bit
#define RSTCON0_RSTI2C0                BIT_32_7  

//! UART4 reset control bit
#define RSTCON0_RSTUART4               BIT_32_8  

//! RTC and Event Monitor/Recorder reset control bit
#define RSTCON0_RSTRTC                 BIT_32_9  

//! The SSP 1 interface reset control bit
#define RSTCON0_RSTSSP1                BIT_32_10 

//! External Memory Controller reset control bit
#define RSTCON0_RSTEMC                 BIT_32_11 

//! A/D converter (ADC) reset control bit
#define RSTCON0_RSTADC                 BIT_32_12 

//! CAN Controller 1 reset control bit
#define RSTCON0_RSTCAN1                BIT_32_13 

//! CAN Controller 2 reset control bit
#define RSTCON0_RSTCAN2                BIT_32_14 

//! Reset control bit for GPIO, and GPIO interrupts
#define RSTCON0_RSTGPIO                BIT_32_15 

//! SPI Flash Interface reset control bit (LPC1773 only)
#define RSTCON0_RSTSPIFI               BIT_32_16 

//! Motor Control PWM reset control bit
#define RSTCON0_RSTMCPWM               BIT_32_17 

//! Quadrature Encoder Interface reset control bit
#define RSTCON0_RSTQEI                 BIT_32_18 

//! The I2C1 interface reset control bit
#define RSTCON0_RSTI2C1                BIT_32_19 

//! The SSP2 interface reset control bit
#define RSTCON0_RSTSSP2                BIT_32_20 

//! The SSP0 interface reset control bit
#define RSTCON0_RSTSSP0                BIT_32_21 

//! Timer 2 reset control bit
#define RSTCON0_RSTTIM2                BIT_32_22 

//! Timer 3 reset control bit
#define RSTCON0_RSTTIM3                BIT_32_23 

//! UART 2 reset control bit
#define RSTCON0_RSTUART2               BIT_32_24 

//! UART 3 reset control bit
#define RSTCON0_RSTUART3               BIT_32_25 

//! I2C interface 2 reset control bit
#define RSTCON0_RSTI2C2                BIT_32_26 

//! I2S interface reset control bit
#define RSTCON0_RSTI2S                 BIT_32_27 

//! SD Card interface reset control bit
#define RSTCON0_RSTSDC                 BIT_32_28 

//! GPDMA function reset control bit
#define RSTCON0_RSTGPDMA               BIT_32_29 

//! Ethernet block reset control bit
#define RSTCON0_RSTENET                BIT_32_30 

//! USB interface reset control bit
#define RSTCON0_RSTUSB                 BIT_32_31 

//RSTCON0 }}


//RSTCON1 {{

//! Reset control bit for the IOCON registers
#define RSTCON1_RSTIOCON               BIT_32_0

//! D/A converter (DAC) reset control bit
#define RSTCON1_RSTDAC                 BIT_32_1

//! CAN acceptance filter reset control bit
#define RSTCON1_RSTCANACC              BIT_32_2

//RSTCON1 }}

//EMCDLYCTL {{

//! Programmable delay value for EMC outputs in command delayed mode.
#define EMCDLYCTL_CMDDLY_M      BIT_MASK(32, 4, 0)

//! Programmable delay value for the feedback clock that controls input data sampling.
#define EMCDLYCTL_FBCLKDLY_M    BIT_MASK(32, 12, 8)

//! Programmable delay value for the CLKOUT0 output.
#define EMCDLYCTL_CLKOUT0DLY_M  BIT_MASK(32, 20, 16)

//! Programmable delay value for the CLKOUT1 output.
#define EMCDLYCTL_CLKOUT1DLY_M  BIT_MASK(32, 28, 24)

//EMCDLYCTL }}

//EMCCAL {{

//! Returns the count of the approximately 50 MHz ring oscillator that occur
//! during 32 clocks of the IRC oscillator
#define EMCCAL_CALVALUE_M       BIT_MASK(32, 7, 0)

//! Start control bit for the EMC calibration counter
#define EMCCAL_START            BIT_32_14

//! Measurement completion flag
#define EMCCAL_DONE             BIT_32_15

//EMCCAL }}


