//*****************************************************************************
//
//! \file xhw_memmap.h
//! \brief Macros defining the memory map of NXP LPC17nx (n = 5/6/7/8)MCU.
//! \version V2.1.1.0
//! \date 11/14/2011
//! \todo Update this time information
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

#ifndef __xHW_MEMMAP_H__
#define __xHW_MEMMAP_H__

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
//! \addtogroup xLowLayer_Peripheral_Memmap xLowLayer Peripheral Memmap
//! \brief The following are definitions for the base addresses of the memories
//! and peripherals.
//!
//! They are always used as ulBase parameters in the peripheral library.
//! The name of a macro for the base address of a peripheral is in  general
//! format as $Namen$_BASE, e.g. UART0_BASE.
//!
//! @{
//
//*****************************************************************************

#define xFLASH_BASE             FLASH_BASE
#define xSRAM_BASE              SRAM_BASE
#define xADC0_BASE              ADC0_BASE
#define xGPIO_PORTA_BASE        GPIO_PORTA_BASE
#define xGPIO_PORTB_BASE        GPIO_PORTB_BASE
#define xGPIO_PORTC_BASE        GPIO_PORTC_BASE
#define xGPIO_PORTD_BASE        GPIO_PORTD_BASE
#define xGPIO_PORTE_BASE        GPIO_PORTE_BASE
#define xGPIO_PORTF_BASE        GPIO_PORTF_BASE
#define xI2C0_BASE              I2C0_BASE
#define xI2C1_BASE              I2C1_BASE
#define xNVIC_BASE              NVIC_BASE
#define xTIMER0_BASE            TIMER0_BASE 
#define xTIMER1_BASE            TIMER1_BASE 
#define xTIMER2_BASE            TIMER2_BASE 
#define xTIMER3_BASE            TIMER3_BASE 
#define xSPI0_BASE              SPI0_BASE
#define xSYSCTL_BASE            SYSCTL_BASE
#define xUART0_BASE             UART0_BASE 
#define xUART1_BASE             UART1_BASE
#define xUART2_BASE             UART2_BASE
#define xUART3_BASE             UART3_BASE
#define xUART4_BASE             UART4_BASE
#define xWDT_BASE0              WATCHDOG_BASE
#define xADC0_BASE              ADC0_BASE    
#define xPWM0_BASE              PWM0_BASE   
#define xPWM1_BASE              PWM1_BASE   
#define xDMA0_BASE              DMA0_BASE   
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
//! \addtogroup LPC17xx_LowLayer
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_LowLayer_Peripheral_Memmap NXP LPC17xx LowLayer Peripheral Memmap
//! The following are defines for the base address of the memories and
//! peripherals.
//!
//! This is always used as ulBase parameter in the peripheral library.
//! @{
//
//*****************************************************************************

#define FLASH_BASE              0x00000000  // FLASH memory
#define SRAM_BASE               0x20000000  // SRAM memory
#define DMA_BASE                0x20080000  // General Purpose DMA controller
#define ETH_BASE                0x20084000  // Ethernet MAC
#define LCD_BASE                0x20088000  // LCD controller
#define USB_BASE                0x2008C000  // USB Controller
#define CRC_BASE                0x20090000  // CRC engine
#define GPIO_PORTA_AHB_BASE     0x20098000  // GPIO Port A (high speed)
#define GPIO_PORTB_AHB_BASE     0x20098020  // GPIO Port B (high speed)
#define GPIO_PORTC_AHB_BASE     0x20098040  // GPIO Port C (high speed)
#define GPIO_PORTD_AHB_BASE     0x20098060  // GPIO Port D (high speed)
#define GPIO_PORTE_AHB_BASE     0x20098080  // GPIO Port E (high speed)
#define GPIO_PORTF_AHB_BASE     0x200980A0  // GPIO Port F (high speed)
#define EMC_BASE                0x2009C000  // External Memory Controller
#define WATCHDOG_BASE           0x40000000  // Watchdog
#define TIMER0_BASE             0x40004000  // Timer0
#define TIMER1_BASE             0x40008000  // Timer1
#define UART0_BASE              0x4000C000  // UART0
#define UART1_BASE              0x40010000  // UART1
#define PWM0_BASE               0x40014000  // PWM0
#define PWM1_BASE               0x40018000  // PWM1
#define I2C0_BASE               0x4001C000  // I2C0
#define SPI0_BASE               0x40020000  // SPI0
#define RTC_BASE                0x40024000  // RTC
#define GPIO_INT_BASE           0x40028000  // GPIO Interrupts 
#define GPIO_PORTA_BASE         0x4002C000  // GPIO Port A
#define GPIO_PORTB_BASE         0x4002C080  // GPIO Port B
#define GPIO_PORTC_BASE         0x4002C100  // GPIO Port C
#define GPIO_PORTD_BASE         0x4002C180  // GPIO Port D
#define GPIO_PORTE_BASE         0x4002C200  // GPIO Port E
#define GPIO_PORTF_BASE         0x4002C280  // GPIO Port F
#define SSP1_BASE               0x40030000  // SSP1
#define ADC0_BASE               0x40034000  // ADC0
#define CAN_RAM_BASE            0x40038000  // CAN Acceptance Filter RAM
#define CAN_REGISTAER_BASE      0x4003C000  // CAN Acceptance Filter Registers
#define CAN_COM_BASE            0x40040000  // CAN Common Registers
#define CAN0_BASE               0x40044000  // CAN Controller 1
#define CAN1_BASE               0x40048000  // CAN Controller 2
#define I2C1_BASE               0x4005C000  // I2C1
#define SSP0_BASE               0x40088000  // SSP0
#define DAC0_BASE               0x4008C000  // DAC0
#define TIMER2_BASE             0x40090000  // Timer2
#define TIMER3_BASE             0x40094000  // Timer3
#define UART2_BASE              0x40098000  // UART2
#define UART3_BASE              0x4009C000  // UART3
#define I2C2_BASE               0x400A0000  // I2C2
#define UART4_BASE              0x400A4000  // UART4
#define I2S0_BASE               0x400A8000  // I2S0
#define SSP2_BASE               0x400AC000  // SSP2
#define RIT_BASE                0x400B0000  // Repetitive interrupt timer
#define MCPWM_BASE              0x400B8000  // Motor control PWM
#define QEI_BASE                0x400BC000  // Quadrature Encoder Interface
#define SDC_BASE                0x400C0000  // SD card interface
#define SYSCTL_BASE             0x400FC000  // System Control
#define ITM_BASE                0xE0000000  // Instrumentation Trace Macrocell
#define DWT_BASE                0xE0001000  // Data Watchpoint and Trace
#define FPB_BASE                0xE0002000  // FLASH Patch and Breakpoint
#define NVIC_BASE               0xE000E000  // Nested Vectored Interrupt Ctrl
#define TPIU_BASE               0xE0040000  // Trace Port Interface Unit
#define COREDEBUG_BASE          0xE000EDF0  // Core Debug Base Address 

//5x 6x

//AHB
#define ETH_BASE                0x50000000
#define DMA_BASE                0x50004000
#define USB_BASE                0x50004000

//APB0
#define WDT_BASE                0x40000000
#define TIMER0_BASE             0x40004000
#define TIMER1_BASE             0x40008000
#define UART0_BASE              0x4000C000
#define UART1_BASE              0x40010000
#define PWM1_BASE               0x40018000
#define I2C0_BASE               0x4001C000
#define SPI_BASE                0x40020000
#define RTC_BASE                0x40024000
#define GPIO_INT_BASE           0x40028000
#define PIN_CON_BASE            0x4002C000
#define SSP1_BASE               0x40030000
#define ADC_BASE                0x40034000
#define CAN_AF_RAM_BASE         0x40038000
#define CAN_AF_REG_BASE         0x4003C000
#define CAN_COMM_BASE           0x40040000
#define CAN1_BASE               0x40044000
#define CAN2_BASE               0x40048000
#define I2C1_BASE               0x4005C000

//APB1
#define SSP0_BASE               0x40088000
#define DAC_BASE                0x4008C000
#define TIMER2_BASE             0x40090000
#define TIMER3_BASE             0x40094000
#define UART2_BASE              0x40098000
#define UART3_BASE              0x4009C000
#define I2C2_BASE               0x400A0000
#define I2S_BASE                0x400A8000
#define RIT_BASE                0x400B0000
#define PWM_BASE                0x400B8000
#define QEI_BASE                0x400BC000
#define SYSCTL_BASE             0x400FC000

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

#endif // __xHW_MEMMAP_H__
