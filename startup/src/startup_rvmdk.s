;/*****************************************************************************
; * @file:    startup_LPC17nx.s
; * @purpose: CMSIS Cortex-M3 Core Device Startup File
; *           for the NXP LPC17nx Device Series
; * @note:    n can be 5/6/7/8
; *           You must define LPC MCU type in assembler of MDK,
; *           This value can be one of the following value
; *           - LPC_175x
; *           - LPC_176x
; *           - LPC_177x
; *           - LPC_178x
; * @version: V1.20
; * @date:    07. October 2010
; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------
; *
; * Copyright (C) 2010 ARM Limited. All rights reserved.
; * ARM Limited (ARM) is supplying this software for use with Cortex-M3
; * processor based microcontrollers.  This file can be freely distributed
; * within development tools that are supporting such ARM based processors.
; *
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; *****************************************************************************/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000200

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     ResetIntHandler             ; Reset Handler
                DCD     NMIIntHandler               ; NMI Handler
                DCD     HardFaultIntHandler         ; Hard Fault Handler
                DCD     MemManageIntHandler         ; MPU Fault Handler
                DCD     BusFaultIntHandler          ; Bus Fault Handler
                DCD     UsageFaultIntHandler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVCIntHandler               ; SVCall Handler
                DCD     DebugMonIntHandler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSVIntHandler            ; PendSV Handler
                DCD     SysTickIntHandler           ; SysTick Handler

                ; External Interrupts
                DCD     WDTIntHandler            ; 16: Watchdog Timer
                DCD     TIMER0IntHandler         ; 17: Timer0
                DCD     TIMER1IntHandler         ; 18: Timer1
                DCD     TIMER2IntHandler         ; 19: Timer2
                DCD     TIMER3IntHandler         ; 20: Timer3
                DCD     UART0IntHandler          ; 21: UART0
                DCD     UART1IntHandler          ; 22: UART1
                DCD     UART2IntHandler          ; 23: UART2
                DCD     UART3IntHandler          ; 24: UART3
                DCD     PWM1IntHandler           ; 25: PWM1
                DCD     I2C0IntHandler           ; 26: I2C0
                DCD     I2C1IntHandler           ; 27: I2C1
                DCD     I2C2IntHandler           ; 28: I2C2
                DCD     SPIIntHandler            ; 29: SPI
                DCD     SSP0IntHandler           ; 30: SSP0
                DCD     SSP1IntHandler           ; 31: SSP1
                DCD     PLL0IntHandler           ; 32: PLL0 Lock (Main PLL)
                DCD     RTCIntHandler            ; 33: Real Time Clock
                DCD     EINT0IntHandler          ; 34: External Interrupt 0
                DCD     EINT1IntHandler          ; 35: External Interrupt 1
                DCD     EINT2IntHandler          ; 36: External Interrupt 2
                DCD     EINT3IntHandler          ; 37: External Interrupt 3
                DCD     ADCIntHandler            ; 38: A/D Converter
                DCD     BODIntHandler            ; 39: Brown-Out Detect
                DCD     USBIntHandler            ; 40: USB
                DCD     CANIntHandler            ; 41: CAN
                DCD     DMAIntHandler            ; 42: General Purpose DMA
                DCD     I2SIntHandler            ; 43: I2S
                DCD     ENETIntHandler           ; 44: Ethernet
#if defined(LPC_175x) || defined (LPC_176x)
                DCD     RITIntHandler            ; 45: Repetitive Interrupt Timer
#elif defined(LPC_177x) || defined (LPC_178x)
                DCD     MCIIntHandler            ; 45: SD/MMC card I/F
#else
#error Please select your LPC MCU first!             \
       This value can be one of the following value: \
       LPC_175x or LPC_176x or LPC_177x or LPC_178x
#endif
                DCD     RITIntHandler            ; 45: Repetitive Interrupt Timer
                DCD     MCPWMIntHandler          ; 46: Motor Control PWM
                DCD     QEIIntHandler            ; 47: Quadrature Encoder Interface
                DCD     PLL1IntHandler           ; 48: PLL1 Lock (USB PLL)
                DCD     USBActivityIntHandler    ; 49: USB Activity interrupt to wakeup
                DCD     CANActivityIntHandler    ; 50: CAN Activity interrupt to wakeup
                DCD     UART4IntHandler          ; 51: UART4
                DCD     SSP2IntHandler           ; 52: SSP2
                DCD     LCDIntHandler            ; 53: LCD
                DCD     GPIOIntHandler           ; 54: GPIO
                DCD     PWM0IntHandler           ; 55: PWM0
                DCD     EEPROMIntHandler         ; 56: EEPROM


                IF      :LNOT::DEF:NO_CRP
                AREA    |.ARM.__at_0x02FC|, CODE, READONLY
CRP_Key         DCD     0xFFFFFFFF
                ENDIF


                AREA    |.text|, CODE, READONLY


; Reset Handler

ResetIntHandler   PROC
                EXPORT  ResetIntHandler             [WEAK]
                IMPORT  __main
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMIIntHandler     PROC
                EXPORT  NMIIntHandler               [WEAK]
                B       .
                ENDP
HardFaultIntHandler\
                PROC
                EXPORT  HardFaultIntHandler         [WEAK]
                B       .
                ENDP
MemManageIntHandler\
                PROC
                EXPORT  MemManageIntHandler         [WEAK]
                B       .
                ENDP
BusFaultIntHandler\
                PROC
                EXPORT  BusFaultIntHandler          [WEAK]
                B       .
                ENDP
UsageFaultIntHandler\
                PROC
                EXPORT  UsageFaultIntHandler        [WEAK]
                B       .
                ENDP
SVCIntHandler     PROC
                EXPORT  SVCIntHandler               [WEAK]
                B       .
                ENDP
DebugMonIntHandler\
                PROC
                EXPORT  DebugMonIntHandler          [WEAK]
                B       .
                ENDP
PendSVIntHandler  PROC
                EXPORT  PendSVIntHandler            [WEAK]
                B       .
                ENDP
SysTickIntHandler PROC
                EXPORT  SysTickIntHandler           [WEAK]
                B       .
                ENDP

DefaultIntHandler PROC

                EXPORT  WDTIntHandler            [WEAK]
                EXPORT  TIMER0IntHandler         [WEAK]
                EXPORT  TIMER1IntHandler         [WEAK]
                EXPORT  TIMER2IntHandler         [WEAK]
                EXPORT  TIMER3IntHandler         [WEAK]
                EXPORT  UART0IntHandler          [WEAK]
                EXPORT  UART1IntHandler          [WEAK]
                EXPORT  UART2IntHandler          [WEAK]
                EXPORT  UART3IntHandler          [WEAK]
                EXPORT  PWM1IntHandler           [WEAK]
                EXPORT  I2C0IntHandler           [WEAK]
                EXPORT  I2C1IntHandler           [WEAK]
                EXPORT  I2C2IntHandler           [WEAK]
                EXPORT  SPIIntHandler            [WEAK]
                EXPORT  SSP0IntHandler           [WEAK]
                EXPORT  SSP1IntHandler           [WEAK]
                EXPORT  PLL0IntHandler           [WEAK]
                EXPORT  RTCIntHandler            [WEAK]
                EXPORT  EINT0IntHandler          [WEAK]
                EXPORT  EINT1IntHandler          [WEAK]
                EXPORT  EINT2IntHandler          [WEAK]
                EXPORT  EINT3IntHandler          [WEAK]
                EXPORT  ADCIntHandler            [WEAK]
                EXPORT  BODIntHandler            [WEAK]
                EXPORT  USBIntHandler            [WEAK]
                EXPORT  CANIntHandler            [WEAK]
                EXPORT  DMAIntHandler            [WEAK]
                EXPORT  I2SIntHandler            [WEAK]
                EXPORT  ENETIntHandler           [WEAK]
#if defined(LPC_175x) || defined (LPC_176x)
                EXPORT  RITIntHandler            [WEAK]

#elif defined(LPC_177x) || defined (LPC_178x)
                EXPORT  MCIIntHandler            [WEAK]
#else
#error Please select your LPC MCU first!             \
       This value can be one of the following value: \
       LPC_175x or LPC_176x or LPC_177x or LPC_178x
#endif
                EXPORT  MCIIntHandler            [WEAK]
                EXPORT  MCPWMIntHandler          [WEAK]
                EXPORT  QEIIntHandler            [WEAK]
                EXPORT  PLL1IntHandler           [WEAK]
                EXPORT  USBActivityIntHandler    [WEAK]
                EXPORT  CANActivityIntHandler    [WEAK]
                EXPORT  UART4IntHandler          [WEAK]
                EXPORT  SSP2IntHandler           [WEAK]
                EXPORT  LCDIntHandler            [WEAK]
                EXPORT  GPIOIntHandler           [WEAK]
                EXPORT  PWM0IntHandler           [WEAK]
                EXPORT  EEPROMIntHandler         [WEAK]

WDTIntHandler
TIMER0IntHandler
TIMER1IntHandler
TIMER2IntHandler
TIMER3IntHandler
UART0IntHandler
UART1IntHandler
UART2IntHandler
UART3IntHandler
PWM1IntHandler
I2C0IntHandler
I2C1IntHandler
I2C2IntHandler
SPIIntHandler
SSP0IntHandler
SSP1IntHandler
PLL0IntHandler
RTCIntHandler
EINT0IntHandler
EINT1IntHandler
EINT2IntHandler
EINT3IntHandler
ADCIntHandler
BODIntHandler
USBIntHandler
CANIntHandler
DMAIntHandler
I2SIntHandler
ENETIntHandler
#if defined(LPC_175x) || defined (LPC_176x)
    RITIntHandler
#elif defined(LPC_177x) || defined (LPC_178x)
    MCIIntHandler
#else
#error Please select your LPC MCU first!             \
       This value can be one of the following value: \
       LPC_175x or LPC_176x or LPC_177x or LPC_178x
#endif
MCPWMIntHandler
QEIIntHandler
PLL1IntHandler
USBActivityIntHandler
CANActivityIntHandler
UART4IntHandler
SSP2IntHandler
LCDIntHandler
GPIOIntHandler
PWM0IntHandler
EEPROMIntHandler

                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END

