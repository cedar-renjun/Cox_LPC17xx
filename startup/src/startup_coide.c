//*****************************************************************************
//
//! \file startup_coide.c
//! \brief LPC17nx MCU Startup code for CooCox CoIDE.
//!        (n can be 5,6,7,8)
//!        This module performs:
//!           - Set the initial SP
//!           - Set the vector table entries with the exceptions ISR address
//!           - Initialize data and bss
//!           - Setup the microcontroller system.
//!           - Call the application's entry point.
//!           .
//! \version V2.1.1.0
//! \todo Need to update time information.
//! \date
//! \author CooCox
//! \note You MUST define LPC MCU type before use this startup code.
//!       The MCU type can be one of the following value:
//!       - LPC_175x
//!       - LPC_176x
//!       - LPC_177x
//!       - LPC_178x
//!
//! \copy
//!
//! Copyright (c)  2013, CooCox
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

//*****************************************************************************
//
// Stack Configuration
//
//*****************************************************************************
//
// Stack size (in Words)
//
#define STACK_SIZE       0x00000200
__attribute__ ((section(".co_stack")))
unsigned long pulStack[STACK_SIZE];

#define WEAK __attribute__ ((weak))

//*****************************************************************************
//
// Declaration of the default fault handlers
//
//*****************************************************************************
void WEAK ResetHandler           (void);
void WEAK NMIIntHandler          (void);
void WEAK HardFaultIntHandler    (void);
void WEAK MemManageIntHandler    (void);
void WEAK BusFaultIntHandler     (void);
void WEAK UsageFaultIntHandler   (void);
void WEAK SVCIntHandler          (void);
void WEAK DebugMonIntHandler     (void);
void WEAK PendSVIntHandler       (void);
void WEAK SysTickIntHandler      (void);

// External Interrupts
void WEAK WDT_IRQHandler         (void);     // 16: Watchdog Timer
void WEAK TIMER0_IRQHandler      (void);     // 17: Timer0
void WEAK TIMER1_IRQHandler      (void);     // 18: Timer1
void WEAK TIMER2_IRQHandler      (void);     // 19: Timer2
void WEAK TIMER3_IRQHandler      (void);     // 20: Timer3
void WEAK UART0_IRQHandler       (void);     // 21: UART0
void WEAK UART1_IRQHandler       (void);     // 22: UART1
void WEAK UART2_IRQHandler       (void);     // 23: UART2
void WEAK UART3_IRQHandler       (void);     // 24: UART3
void WEAK PWM1_IRQHandler        (void);     // 25: PWM1
void WEAK I2C0_IRQHandler        (void);     // 26: I2C0
void WEAK I2C1_IRQHandler        (void);     // 27: I2C1
void WEAK I2C2_IRQHandler        (void);     // 28: I2C2
void WEAK SPI_IRQHandler         (void);     // 29: SPI
void WEAK SSP0_IRQHandler        (void);     // 30: SSP0
void WEAK SSP1_IRQHandler        (void);     // 31: SSP1
void WEAK PLL0_IRQHandler        (void);     // 32: PLL0 Lock (Main PLL)
void WEAK RTC_IRQHandler         (void);     // 33: Real Time Clock
void WEAK EINT0_IRQHandler       (void);     // 34: External Interrupt 0
void WEAK EINT1_IRQHandler       (void);     // 35: External Interrupt 1
void WEAK EINT2_IRQHandler       (void);     // 36: External Interrupt 2
void WEAK EINT3_IRQHandler       (void);     // 37: External Interrupt 3
void WEAK ADC_IRQHandler         (void);     // 38: A/D Converter
void WEAK BOD_IRQHandler         (void);     // 39: Brown-Out Detect
void WEAK USB_IRQHandler         (void);     // 40: USB
void WEAK CAN_IRQHandler         (void);     // 41: CAN
void WEAK DMA_IRQHandler         (void);     // 42: General Purpose DMA
void WEAK I2S_IRQHandler         (void);     // 43: I2S
void WEAK ENET_IRQHandler        (void);     // 44: Ethernet
#if defined(LPC_175x) || defined (LPC_176x)
void WEAK RIT_IRQHandler         (void);     // 45: Repetitive Interrupt Timer
#elif defined(LPC_177x) || defined (LPC_178x)
void WEAK MCI_IRQHandler         (void);     // 45: SD/MMC Card
#else
#error Please select your LPC MCU first!             \
       This value can be one of the following value: \
       LPC_175x or LPC_176x or LPC_177x or LPC_178x
#endif
void WEAK MCPWM_IRQHandler       (void);     // 46: Motor Control PWM
void WEAK QEI_IRQHandler         (void);     // 47: Quadrature Encoder Interface
void WEAK PLL1_IRQHandler        (void);     // 48: PLL1 Lock (USB PLL)
void WEAK USBActivity_IRQHandler (void);     // 49: USB Activity
void WEAK CANActivity_IRQHandler (void);     // 50: CAN Activity
void WEAK UART4_IRQHandler       (void);     // 51: UART4
void WEAK SSP2_IRQHandler        (void);     // 52: SSP2
void WEAK LCD_IRQHandler         (void);     // 53: LCD
void WEAK GPIO_IRQHandler        (void);     // 54: GPIO
void WEAK PWM0_IRQHandler        (void);     // 55: PWM0
void WEAK EEPROM_IRQHandler      (void);     // 56: EEPROM
//*****************************************************************************
//
// Symbols defined in linker script
//
//*****************************************************************************
//
// Start address for the initialization values of the .data section.
//
extern unsigned long _sidata;

//
// Start address for the .data section
//
extern unsigned long _sdata;

//
// End address for the .data section
//
extern unsigned long _edata;

//
// Start address for the .bss section
//
extern unsigned long _sbss;

//
// End address for the .bss section
//
extern unsigned long _ebss;

//
// End address for ram
//
extern void _eram;

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
extern int main(void);
__attribute__ ((used))
void ResetHandler(void);
static void DefaultIntHandler(void);

//
// The minimal vector table for a Cortex M3.  Note that the proper constructs
// must be placed on this to ensure that it ends up at physical address
// 0x00000000.
//
__attribute__ ((used,section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
    (void *)&pulStack[STACK_SIZE],          // The initial stack pointer
    ResetHandler                 ,          // The reset handler
    NMIIntHandler                ,          // The NMI handler
    HardFaultIntHandler          ,          // The hard fault handler
    MemManageIntHandler          ,          // The MPU fault handler
    BusFaultIntHandler           ,          // The bus fault handler
    UsageFaultIntHandler         ,          // The usage fault handler
    0                            ,          // Reserved
    0                            ,          // Reserved
    0                            ,          // Reserved
    0                            ,          // Reserved
    SVCIntHandler                ,          // SVCall handler
    DebugMonIntHandler           ,          // Debug monitor handler
    0                            ,          // Reserved
    PendSVIntHandler             ,          // The PendSV handler
    SysTickIntHandler            ,          // The SysTick handler

    // External Interrupts
    WDT_IRQHandler               ,          // 16: Watchdog Timer
    TIMER0_IRQHandler            ,          // 17: Timer0
    TIMER1_IRQHandler            ,          // 18: Timer1
    TIMER2_IRQHandler            ,          // 19: Timer2
    TIMER3_IRQHandler            ,          // 20: Timer3
    UART0_IRQHandler             ,          // 21: UART0
    UART1_IRQHandler             ,          // 22: UART1
    UART2_IRQHandler             ,          // 23: UART2
    UART3_IRQHandler             ,          // 24: UART3
    PWM1_IRQHandler              ,          // 25: PWM1
    I2C0_IRQHandler              ,          // 26: I2C0
    I2C1_IRQHandler              ,          // 27: I2C1
    I2C2_IRQHandler              ,          // 28: I2C2
    SPI_IRQHandler               ,          // 29: SPI
    SSP0_IRQHandler              ,          // 30: SSP0
    SSP1_IRQHandler              ,          // 31: SSP1
    PLL0_IRQHandler              ,          // 32: PLL0 Lock (Main PLL)
    RTC_IRQHandler               ,          // 33: Real Time Clock
    EINT0_IRQHandler             ,          // 34: External Interrupt 0
    EINT1_IRQHandler             ,          // 35: External Interrupt 1
    EINT2_IRQHandler             ,          // 36: External Interrupt 2
    EINT3_IRQHandler             ,          // 37: External Interrupt 3
    ADC_IRQHandler               ,          // 38: A/D Converter
    BOD_IRQHandler               ,          // 39: Brown-Out Detect
    USB_IRQHandler               ,          // 40: USB
    CAN_IRQHandler               ,          // 41: CAN
    DMA_IRQHandler               ,          // 42: General Purpose DMA
    I2S_IRQHandler               ,          // 43: I2S
    ENET_IRQHandler              ,          // 44: Ethernet
#if defined(LPC_175x) || defined (LPC_176x)
    RIT_IRQHandler               ,          // 45: Repetitive Interrupt Timer
#elif defined(LPC_177x) || defined (LPC_178x)
    MCI_IRQHandler               ,          // 45: SD/MMC Card
#else
#error Please select your LPC MCU first!             \
       This value can be one of the following value: \
       LPC_175x or LPC_176x or LPC_177x or LPC_178x
#endif
    MCPWM_IRQHandler             ,          // 46: Motor Control PWM
    QEI_IRQHandler               ,          // 47: Quadrature Encoder Interface
    PLL1_IRQHandler              ,          // 48: PLL1 Lock (USB PLL)
    USBActivity_IRQHandler       ,          // 49: USB Activity
    CANActivity_IRQHandler       ,          // 50: CAN Activity
    UART4_IRQHandler             ,          // 51: UART4
    SSP2_IRQHandler              ,          // 52: SSP2
    LCD_IRQHandler               ,          // 53: LCD
    GPIO_IRQHandler              ,          // 54: GPIO
    PWM0_IRQHandler              ,          // 55: PWM0
    EEPROM_IRQHandler            ,          // 56: EEPROM
};

//*****************************************************************************
//
//! \brief This is the code that gets called when the processor first
//! starts execution following a reset event.
//!
//! \param None.
//!
//! Only the absolutely necessary set is performed, after which the
//! application supplied main() routine is called.
//!
//! \return None.
//
//*****************************************************************************
void Default_ResetHandler(void)
{
    //
    // Initialize data and bss
    //
    unsigned long *pulSrc, *pulDest;

    //
    // Copy the data segment initializers from flash to SRAM
    //
    pulSrc = &_sidata;

    for(pulDest = &_sdata; pulDest < &_edata; )
    {
        *(pulDest++) = *(pulSrc++);
    }

    //
    // Zero fill the bss segment.
    //
    for(pulDest = &_sbss; pulDest < &_ebss; )
    {
        *(pulDest++) = 0;
    }

    //
    // Call the application's entry point.
    //
    main();
}

//*****************************************************************************
//
// Provide weak aliases for each Exception handler to the DefaultIntHandler.
// As they are weak aliases, any function with the same name will override
// this definition.
//
//*****************************************************************************

#pragma weak ResetHandler            = Default_ResetHandler
#pragma weak NMIIntHandler           = DefaultIntHandler
#pragma weak HardFaultIntHandler     = DefaultIntHandler
#pragma weak MemManageIntHandler     = DefaultIntHandler
#pragma weak BusFaultIntHandler      = DefaultIntHandler
#pragma weak UsageFaultIntHandler    = DefaultIntHandler
#pragma weak SVCIntHandler           = DefaultIntHandler
#pragma weak DebugMonIntHandler      = DefaultIntHandler
#pragma weak PendSVIntHandler        = DefaultIntHandler
#pragma weak SysTickIntHandler       = DefaultIntHandler

// External Interrupts
#pragma weak WDT_IRQHandler          = DefaultIntHandler    // 16: Watchdog Timer
#pragma weak TIMER0_IRQHandler       = DefaultIntHandler    // 17: Timer0
#pragma weak TIMER1_IRQHandler       = DefaultIntHandler    // 18: Timer1
#pragma weak TIMER2_IRQHandler       = DefaultIntHandler    // 19: Timer2
#pragma weak TIMER3_IRQHandler       = DefaultIntHandler    // 20: Timer3
#pragma weak UART0_IRQHandler        = DefaultIntHandler    // 21: UART0
#pragma weak UART1_IRQHandler        = DefaultIntHandler    // 22: UART1
#pragma weak UART2_IRQHandler        = DefaultIntHandler    // 23: UART2
#pragma weak UART3_IRQHandler        = DefaultIntHandler    // 24: UART3
#pragma weak PWM1_IRQHandler         = DefaultIntHandler    // 25: PWM1
#pragma weak I2C0_IRQHandler         = DefaultIntHandler    // 26: I2C0
#pragma weak I2C1_IRQHandler         = DefaultIntHandler    // 27: I2C1
#pragma weak I2C2_IRQHandler         = DefaultIntHandler    // 28: I2C2
#pragma weak SPI_IRQHandler          = DefaultIntHandler    // 29: SPI
#pragma weak SSP0_IRQHandler         = DefaultIntHandler    // 30: SSP0
#pragma weak SSP1_IRQHandler         = DefaultIntHandler    // 31: SSP1
#pragma weak PLL0_IRQHandler         = DefaultIntHandler    // 32: PLL0 Lock (Main PLL)
#pragma weak RTC_IRQHandler          = DefaultIntHandler    // 33: Real Time Clock
#pragma weak EINT0_IRQHandler        = DefaultIntHandler    // 34: External Interrupt 0
#pragma weak EINT1_IRQHandler        = DefaultIntHandler    // 35: External Interrupt 1
#pragma weak EINT2_IRQHandler        = DefaultIntHandler    // 36: External Interrupt 2
#pragma weak EINT3_IRQHandler        = DefaultIntHandler    // 37: External Interrupt 3
#pragma weak ADC_IRQHandler          = DefaultIntHandler    // 38: A/D Converter
#pragma weak BOD_IRQHandler          = DefaultIntHandler    // 39: Brown-Out Detect
#pragma weak USB_IRQHandler          = DefaultIntHandler    // 40: USB
#pragma weak CAN_IRQHandler          = DefaultIntHandler    // 41: CAN
#pragma weak DMA_IRQHandler          = DefaultIntHandler    // 42: General Purpose DMA
#pragma weak I2S_IRQHandler          = DefaultIntHandler    // 43: I2S
#pragma weak ENET_IRQHandler         = DefaultIntHandler    // 44: Ethernet

#if defined(LPC_175x) || defined (LPC_176x)
#pragma weak RIT_IRQHandler          = DefaultIntHandler    // 45: Repetitive Interrupt Timer
#elif defined(LPC_177x) || defined (LPC_178x)
#pragma weak MCI_IRQHandler          = DefaultIntHandler    // 45: SD/MMC Card
#else
#error Please select your LPC MCU first!             \
       This value can be one of the following value: \
       LPC_175x or LPC_176x or LPC_177x or LPC_178x
#endif

#pragma weak MCPWM_IRQHandler        = DefaultIntHandler    // 46: Motor Control PWM
#pragma weak QEI_IRQHandler          = DefaultIntHandler    // 47: Quadrature Encoder Interface
#pragma weak PLL1_IRQHandler         = DefaultIntHandler    // 48: PLL1 Lock (USB PLL)
#pragma weak USBActivity_IRQHandler  = DefaultIntHandler    // 49: USB Activity
#pragma weak CANActivity_IRQHandler  = DefaultIntHandler    // 50: CAN Activity
#pragma weak UART4_IRQHandler        = DefaultIntHandler    // 51: UART4
#pragma weak SSP2_IRQHandler         = DefaultIntHandler    // 52: SSP2
#pragma weak LCD_IRQHandler          = DefaultIntHandler    // 53: LCD
#pragma weak GPIO_IRQHandler         = DefaultIntHandler    // 54: GPIO
#pragma weak PWM0_IRQHandler         = DefaultIntHandler    // 55: PWM0
#pragma weak EEPROM_IRQHandler       = DefaultIntHandler    // 56: EEPROM
//*****************************************************************************
//
//! \brief This is the code that gets called when the processor receives an
//! unexpected interrupt.
//!
//! \param None.
//!
//! This simply enters an infinite loop, preserving the system state for
//! examination by a debugger.
//!
//! \return None.
//*****************************************************************************
static void DefaultIntHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while (1)
    {
    }
}

