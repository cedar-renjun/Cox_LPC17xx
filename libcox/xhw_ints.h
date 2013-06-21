//*****************************************************************************
//
//! \file xhw_ints.h
//! \brief Macros that define the interrupt assignment on NXP LPC17xx.
//! \version V2.1.1.0
//! \date 11/14/2011
//! \todo Update this time information
//! \todo Update the GPIO macro
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

#ifndef __xHW_INTS_H__
#define __xHW_INTS_H__

//! Detect MCU Type.
#if defined(LPC_175x) || defined (LPC_176x) ||       \
    defined(LPC_177x) || defined (LPC_178x)
#else
#error Please select your LPC MCU first!             \
       This value can be one of the following value: \
       LPC_175x or LPC_176x or LPC_177x or LPC_178x
#endif

//*****************************************************************************
//
//! \addtogroup CoX_Peripheral_Lib
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LowLayer
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xLowLayer
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xLowLayer_Interrupt_Assignments xLowLayer Interrupt Assignments
//! \brief Macro definitions for the fault/interrupt assignments.
//!
//! They can be used as ulInterrupt parameters with xIntEnable(), xIntDisable()
//! to enable/disable the interrupt at the core/NVIC level.
//!
//! @{
//
//*****************************************************************************

#define xFAULT_NMI                           TPIU_BASE
#define xFAULT_HARD                          FAULT_HARD
#define xFAULT_MPU                           FAULT_MPU
#define xFAULT_BUS                           FAULT_BUS
#define xFAULT_USAGE                         FAULT_USAGE
#define xFAULT_SVCALL                        FAULT_SVCALL
#define xFAULT_DEBUG                         FAULT_DEBUG
#define xFAULT_PENDSV                        FAULT_PENDSV
#define xFAULT_SYSTICK                       FAULT_SYSTICK

#define xINT_ADC0                            INT_ADC0
#define xINT_DMA                             INT_DMA
//#define xINT_GPIOA                           INT_GPIO
//#define xINT_GPIOB                           INT_GPIO
//#define xINT_GPIOC                           INT_GPIO
//#define xINT_GPIOD                           INT_GPIO
//#define xINT_GPIOE                           INT_GPIO
//#define xINT_GPIOF                           INT_GPIO
#define xINT_SYSCTL                          INT_SYSCTL
#define xINT_TIMER0                          INT_TIMER0
#define xINT_TIMER1                          INT_TIMER1
#define xINT_TIMER2                          INT_TIMER2
#define xINT_TIMER3                          INT_TIMER3

//*****************************************************************************
//
//! \brief Defines for the total number of interrupts.
//
//*****************************************************************************
#define xNUM_INTERRUPTS                      NUM_INTERRUPTS

//*****************************************************************************
//
//! \brief Defines for the total number of priority levels.
//
//*****************************************************************************
#define xNUM_PRIORITY                        NUM_PRIORITY

//*****************************************************************************
//
//! \brief Defines for the total number bits  of priority levels.
//
//*****************************************************************************
#define xNUM_PRIORITY_BITS                   NUM_PRIORITY_BITS

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
//! \addtogroup LPC_LowLayer
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC_LowLayer_Interrupt_Assignments NXP LPC17xx Interrupt Assignments
//! \brief Macros defines for the fault/interrupt assignments.
//! @{
//
//*****************************************************************************

#define FAULT_NMI                            2  // NMI fault
#define FAULT_HARD                           3  // Hard fault
#define FAULT_MPU                            4  // MPU fault
#define FAULT_BUS                            5  // Bus fault
#define FAULT_USAGE                          6  // Usage fault
#define FAULT_SVCALL                         11 // SVCall
#define FAULT_DEBUG                          12 // Debug monitor
#define FAULT_PENDSV                         14 // PendSV
#define FAULT_SYSTICK                        15 // System Tick

#define INT_WDT                              16 // Watchdog timer
#define INT_TIMER0                           17 // Timer 0
#define INT_TIMER1                           18 // Timer 1
#define INT_TIMER2                           19 // Timer 2
#define INT_TIMER3                           20 // Timer 3
#define INT_UART0                            21 // UART0 Rx and Tx
#define INT_UART1                            22 // UART1 Rx and Tx
#define INT_UART2                            23 // UART2 Rx and Tx
#define INT_UART3                            24 // UART3 Rx and Tx
#define INT_PWM1                             25 // PWM Generator 1
#define INT_I2C0                             26 // I2C0 Master and Slave
#define INT_I2C1                             27 // I2C1 Master and Slave
#define INT_I2C2                             28 // I2C2 Master and Slave
#define INT_SPI                              29 // SPI
#define INT_SSP0                             30 // SSP0 Rx and Tx
#define INT_SSP1                             31 // SSP1 Rx and Tx
#define INT_PLL0                             32 // PLL0 Lock (PLOCK0)
#define INT_RTC                              33 // RTC & EV0, EV1, EV2
#define INT_EINT0                            34 // External Interrupt 0
#define INT_EINT1                            35 // External Interrupt 1
#define INT_EINT2                            36 // External Interrupt 2
#define INT_EINT3                            37 // External Interrupt 3
#define INT_ADC                              38 // ADC
#define INT_BOD                              39 // Brown Out detect
#define INT_USB                              40 // USB Controller
#define INT_CAN                              41 // CAN 0
#define INT_DMA                              42 // DMA controller
#define INT_I2S                              43 // I2S
#define INT_ETH                              44 // Ethernet
#if defined(LPC_175x) || defined(LPC_176x)
#define INT_RIT                              45 // RIT
#if defined(LPC_177x) || defined(LPC_178x)
#define INT_SD                               45 // SD Card
#endif
#define INT_MCPWM                            46 // Motor Control PWM
#define INT_QEI                              47 // Quadrature Encoder Interface
#define INT_PLL1                             48 // PLL1 Lock (Alt PLL)
#define INT_USBACT                           49 // USB Activity Interrupt
#define INT_CANACT                           50 // CAN Activity Interrupt
#define INT_UART4                            51 // UART4 Rx and Tx
#define INT_SSP2                             52 // SSP2 Rx and Tx
#define INT_LCD                              53 // LCD Controller
#define INT_GPIO                             54 // GPIO Interrupts
#define INT_PWM0                             55 // PWM Generator 0
#define INT_EEPROM                           56 // FLASH & EEPROM

//*****************************************************************************
//
//! \brief Defines for the total number of interrupts.
//
//*****************************************************************************
#if defined(LPC_175x) || defined(LPC_176x)
#define NUM_INTERRUPTS                       51
#elif defined(LPC_177x) || defined (LPC_178x)
#define NUM_INTERRUPTS                       57
#endif

//*****************************************************************************
//
//! \brief Defines for the total number of priority levels.
//
//*****************************************************************************
#define NUM_PRIORITY                         8

//*****************************************************************************
//
//! \brief Defines for the total number bit of priority levels.
//
//*****************************************************************************
#define NUM_PRIORITY_BITS                    5

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

#endif // __XHW_INTS_H__

