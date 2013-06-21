//*****************************************************************************
//
//! \file xhw_nvic.h
//! \brief Macros used when accessing the NVIC(Cortex M3) hardware.
//! \version V2.1.1.1
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

#ifndef __xHW_NVIC_H__
#define __xHW_NVIC_H__

//*****************************************************************************
//
//! \addtogroup CoX_Peripheral_Lib
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xCORE
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register
//! \brief Here are the detailed info of NVIC registers. 
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
//! \addtogroup CORE_NVIC_Register_Address NVIC Address Register(NVIC_Address)
//! \brief Defines for the bit fields in the NVIC_Register_Address register
//! @{
//
//*****************************************************************************
#define NVIC_INT_TYPE           0xE000E004  // Interrupt Controller Type Reg
#define NVIC_ACTLR              0xE000E008  // Auxiliary Control
#define NVIC_ST_CTRL            0xE000E010  // SysTick Control and Status
                                            // Register
#define NVIC_ST_RELOAD          0xE000E014  // SysTick Reload Value Register
#define NVIC_ST_CURVAL          0xE000E018  // SysTick Current Value Register
#define NVIC_ST_CALIB           0xE000E01C  // SysTick Calibration Value Reg
#define NVIC_ISER0              0xE000E100  // Interrupt 0-31 Set Enable
#define NVIC_ISER1              0xE000E104  // Interrupt 32-63 Set Enable
#define NVIC_ISER2              0xE000E108  // Interrupt 64-95 Set Enable
#define NVIC_ISER3              0xE000E10C  // Interrupt 96-127 Set Enable
#define NVIC_ICER0              0xE000E180  // Interrupt 0-31 Clear Enable
#define NVIC_ICER1              0xE000E184  // Interrupt 32-63 Clear Enable
#define NVIC_ICER2              0xE000E188  // Interrupt 64-95 Clear Enable
#define NVIC_ICER3              0xE000E18C  // Interrupt 96-127 Clear Enable
#define NVIC_ISPR0              0xE000E200  // Interrupt 0-31 Set Pending
#define NVIC_ISPR1              0xE000E204  // Interrupt 32-63 Set Pending
#define NVIC_ISPR2              0xE000E208  // Interrupt 64-95 Set Pending
#define NVIC_ISPR3              0xE000E20C  // Interrupt 96-127 Set Pending
#define NVIC_ICPR0              0xE000E280  // Interrupt 0-31 Clear Pending
#define NVIC_ICPR1              0xE000E284  // Interrupt 32-63 Clear Pending
#define NVIC_ICPR2              0xE000E288  // Interrupt 64-95 Clear Pending
#define NVIC_ICPR3              0xE000E28C  // Interrupt 96-127 Clear Pending
#define NVIC_IABR0              0xE000E300  // Interrupt 0-31 Active Bit
#define NVIC_IABR1              0xE000E304  // Interrupt 32-63 Active Bit
#define NVIC_IABR2              0xE000E308  // Interrupt 64-95 Active Bit
#define NVIC_IABR3              0xE000E30C  // Interrupt 96-127 Active Bit
#define NVIC_PRI0               0xE000E400  // Interrupt 0-3 Priority
#define NVIC_PRI1               0xE000E404  // Interrupt 4-7 Priority
#define NVIC_PRI2               0xE000E408  // Interrupt 8-11 Priority
#define NVIC_PRI3               0xE000E40C  // Interrupt 12-15 Priority
#define NVIC_PRI4               0xE000E410  // Interrupt 16-19 Priority
#define NVIC_PRI5               0xE000E414  // Interrupt 20-23 Priority
#define NVIC_PRI6               0xE000E418  // Interrupt 24-27 Priority
#define NVIC_PRI7               0xE000E41C  // Interrupt 28-31 Priority
#define NVIC_PRI8               0xE000E420  // Interrupt 32-35 Priority
#define NVIC_PRI9               0xE000E424  // Interrupt 36-39 Priority
#define NVIC_PRI10              0xE000E428  // Interrupt 40-43 Priority
#define NVIC_PRI11              0xE000E42C  // Interrupt 44-47 Priority
#define NVIC_PRI12              0xE000E430  // Interrupt 48-51 Priority
#define NVIC_PRI13              0xE000E434  // Interrupt 52-55 Priority
#define NVIC_PRI14              0xE000E438  // Interrupt 56-59 Priority
#define NVIC_PRI15              0xE000E43C  // Interrupt 60-63 Priority
#define NVIC_PRI16              0xE000E440  // Interrupt 64-67 Priority
#define NVIC_PRI17              0xE000E444  // Interrupt 68-71 Priority
#define NVIC_PRI18              0xE000E448  // Interrupt 72-75 Priority
#define NVIC_PRI19              0xE000E44C  // Interrupt 76-79 Priority
#define NVIC_PRI20              0xE000E450  // Interrupt 80-83 Priority
#define NVIC_PRI21              0xE000E454  // Interrupt 84-87 Priority
#define NVIC_PRI22              0xE000E458  // Interrupt 88-91 Priority
#define NVIC_PRI23              0xE000E45C  // Interrupt 92-95 Priority
#define NVIC_PRI24              0xE000E460  // Interrupt 96-99 Priority
#define NVIC_PRI25              0xE000E464  // Interrupt 100-103 Priority
#define NVIC_PRI26              0xE000E468  // Interrupt 104-107 Priority
#define NVIC_PRI27              0xE000E46C  // Interrupt 108-111 Priority
#define NVIC_CPUID              0xE000ED00  // CPU ID Base
#define NVIC_ICSR               0xE000ED04  // Interrupt Control and State
#define NVIC_VTOR               0xE000ED08  // Vector Table Offset
#define NVIC_AIRCR              0xE000ED0C  // Application Interrupt and Reset Control 
#define NVIC_SCR                0xE000ED10  // System Control
#define NVIC_CCR                0xE000ED14  // Configuration and Control
#define NVIC_SHPR1              0xE000ED18  // System Handler Priority 1
#define NVIC_SHPR2              0xE000ED1C  // System Handler Priority 2
#define NVIC_SHPR3              0xE000ED20  // System Handler Priority 3
#define NVIC_SHCSR              0xE000ED24  // System Handler Control and State
#define NVIC_CFSR               0xE000ED28  // Configurable Fault Status
#define NVIC_MMSR               0xE000ED28  //
#define NVIC_BFSR               0xE000ED29  //
#define NVIC_UFSR               0xE000ED2A  //
#define NVIC_HFSR               0xE000ED2C  // Hard Fault Status
#define NVIC_DEBUG_STAT         0xE000ED30  // Debug Status Register
#define NVIC_MMFAR              0xE000ED34  // Memory Management Fault Address
#define NVIC_BFAR               0xE000ED38  // Bus Fault Address
#define NVIC_MPU_TYPE           0xE000ED90  // MPU Type
#define NVIC_MPU_CTRL           0xE000ED94  // MPU Control
#define NVIC_MPU_RNR            0xE000ED98  // MPU Region Number
#define NVIC_MPU_RBAR           0xE000ED9C  // MPU Region Base Address
#define NVIC_MPU_RASR           0xE000EDA0  // MPU Region Attribute and Size
#define NVIC_MPU_RBAR1          0xE000EDA4  // MPU Region Base Address Alias 1
#define NVIC_MPU_RASR1          0xE000EDA8  // MPU Region Attribute and Size Alias 1
#define NVIC_MPU_RBAR2          0xE000EDAC  // MPU Region Base Address Alias 2
#define NVIC_MPU_RASR2          0xE000EDB0  // MPU Region Attribute and Size Alias 2
#define NVIC_MPU_RBAR3          0xE000EDB4  // MPU Region Base Address Alias 3
#define NVIC_MPU_RASR3          0xE000EDB8  // MPU Region Attribute and Size Alias 3
                                            
#define NVIC_DBG_CTRL           0xE000EDF0  // Debug Control and Status Reg
#define NVIC_DBG_XFER           0xE000EDF4  // Debug Core Reg. Transfer Select
#define NVIC_DBG_DATA           0xE000EDF8  // Debug Core Register Data
#define NVIC_DBG_INT            0xE000EDFC  // Debug Reset Interrupt Control
#define NVIC_STIR               0xE000EF00  // Software Trigger Interrupt

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_INT_TYPE NVIC Interrupt Type Register(NVIC_INT_TYPE)
//! \brief Defines for the bit fields in the NVIC_INT_TYPE register.
//! @{
//
//*****************************************************************************
#define NVIC_INT_TYPE_LINES_M   0x0000001F  // Number of interrupt lines (x32)
#define NVIC_INT_TYPE_LINES_S   0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ACTLR NVIC ACTLR Register(NVIC_ACTLR)
//! \brief Defines for the bit fields in the NVIC_ACTLR register.
//! @{
//
//*****************************************************************************

#define NVIC_ACTLR_DISFOLD      0x00000004  // Disable IT Folding
#define NVIC_ACTLR_DISWBUF      0x00000002  // Disable Write Buffer
#define NVIC_ACTLR_DISMCYC      0x00000001  // Disable Interrupts of Multiple
                                            // Cycle Instructions

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ST_CTRL NVIC Status Control Register(NVIC_ST_CTRL)
//! \brief Defines for the bit fields in the NVIC_ST_CTRL register.
//! @{
//
//*****************************************************************************
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count Flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt Enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Enable
//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ST_RELOAD NVIC Status Reload Register(NVIC_ST_RELOAD)
//! \brief Defines for the bit fields in the NVIC_ST_RELOAD register.
//! @{
//
//*****************************************************************************
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Reload Value
#define NVIC_ST_RELOAD_S        0
//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ST_CURVAL NVIC Status Current Register(NVIC_ST_CURVAL)
//! \brief Defines for the bit fields in the NVIC_ST_CURVAL register.
//! @{
//
//*****************************************************************************
#define NVIC_ST_CURVAL_M       0x00FFFFFF  // Current Value
#define NVIC_ST_CURVAL_S       0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ST_CALIB NVIC Status Cal Register(NVIC_ST_CALIB)
//! \brief Defines for the bit fields in the NVIC_ST_CALIB register.
//! @{
//
//*****************************************************************************
#define NVIC_ST_CALIB_NOREF     0x80000000  // No reference clock
#define NVIC_ST_CALIB_SKEW      0x40000000  // Clock skew
#define NVIC_ST_CALIB_TEEMS_M   0x00FFFFFF  // 1ms reference value
#define NVIC_ST_CALIB_TEEMS_S   0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ISER0 NVIC Ebanle0 Register(NVIC_ISER0)
//! \brief Defines for the bit fields in the NVIC_ISER0 register.
//! @{
//
//*****************************************************************************
#define NVIC_ISER0_INT_M          0xFFFFFFFF  // Interrupt Enable
#define NVIC_ISER0_INT0           0x00000001  // Interrupt 0 enable
#define NVIC_ISER0_INT1           0x00000002  // Interrupt 1 enable
#define NVIC_ISER0_INT2           0x00000004  // Interrupt 2 enable
#define NVIC_ISER0_INT3           0x00000008  // Interrupt 3 enable
#define NVIC_ISER0_INT4           0x00000010  // Interrupt 4 enable
#define NVIC_ISER0_INT5           0x00000020  // Interrupt 5 enable
#define NVIC_ISER0_INT6           0x00000040  // Interrupt 6 enable
#define NVIC_ISER0_INT7           0x00000080  // Interrupt 7 enable
#define NVIC_ISER0_INT8           0x00000100  // Interrupt 8 enable
#define NVIC_ISER0_INT9           0x00000200  // Interrupt 9 enable
#define NVIC_ISER0_INT10          0x00000400  // Interrupt 10 enable
#define NVIC_ISER0_INT11          0x00000800  // Interrupt 11 enable
#define NVIC_ISER0_INT12          0x00001000  // Interrupt 12 enable
#define NVIC_ISER0_INT13          0x00002000  // Interrupt 13 enable
#define NVIC_ISER0_INT14          0x00004000  // Interrupt 14 enable
#define NVIC_ISER0_INT15          0x00008000  // Interrupt 15 enable
#define NVIC_ISER0_INT16          0x00010000  // Interrupt 16 enable
#define NVIC_ISER0_INT17          0x00020000  // Interrupt 17 enable
#define NVIC_ISER0_INT18          0x00040000  // Interrupt 18 enable
#define NVIC_ISER0_INT19          0x00080000  // Interrupt 19 enable
#define NVIC_ISER0_INT20          0x00100000  // Interrupt 20 enable
#define NVIC_ISER0_INT21          0x00200000  // Interrupt 21 enable
#define NVIC_ISER0_INT22          0x00400000  // Interrupt 22 enable
#define NVIC_ISER0_INT23          0x00800000  // Interrupt 23 enable
#define NVIC_ISER0_INT24          0x01000000  // Interrupt 24 enable
#define NVIC_ISER0_INT25          0x02000000  // Interrupt 25 enable
#define NVIC_ISER0_INT26          0x04000000  // Interrupt 26 enable
#define NVIC_ISER0_INT27          0x08000000  // Interrupt 27 enable
#define NVIC_ISER0_INT28          0x10000000  // Interrupt 28 enable
#define NVIC_ISER0_INT29          0x20000000  // Interrupt 29 enable
#define NVIC_ISER0_INT30          0x40000000  // Interrupt 30 enable
#define NVIC_ISER0_INT31          0x80000000  // Interrupt 31 enable

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ISER1 NVIC Enable1 Register(NVIC_ISER1)
//! \brief Defines for the bit fields in the NVIC_ISER1 register.
//! @{
//
//*****************************************************************************
#define NVIC_ISER1_INT_M          0x007FFFFF  // Interrupt Enable
#define NVIC_ISER1_INT32          0x00000001  // Interrupt 32 enable
#define NVIC_ISER1_INT33          0x00000002  // Interrupt 33 enable
#define NVIC_ISER1_INT34          0x00000004  // Interrupt 34 enable
#define NVIC_ISER1_INT35          0x00000008  // Interrupt 35 enable
#define NVIC_ISER1_INT36          0x00000010  // Interrupt 36 enable
#define NVIC_ISER1_INT37          0x00000020  // Interrupt 37 enable
#define NVIC_ISER1_INT38          0x00000040  // Interrupt 38 enable
#define NVIC_ISER1_INT39          0x00000080  // Interrupt 39 enable
#define NVIC_ISER1_INT40          0x00000100  // Interrupt 40 enable
#define NVIC_ISER1_INT41          0x00000200  // Interrupt 41 enable
#define NVIC_ISER1_INT42          0x00000400  // Interrupt 42 enable
#define NVIC_ISER1_INT43          0x00000800  // Interrupt 43 enable
#define NVIC_ISER1_INT44          0x00001000  // Interrupt 44 enable
#define NVIC_ISER1_INT45          0x00002000  // Interrupt 45 enable
#define NVIC_ISER1_INT46          0x00004000  // Interrupt 46 enable
#define NVIC_ISER1_INT47          0x00008000  // Interrupt 47 enable
#define NVIC_ISER1_INT48          0x00010000  // Interrupt 48 enable
#define NVIC_ISER1_INT49          0x00020000  // Interrupt 49 enable
#define NVIC_ISER1_INT50          0x00040000  // Interrupt 50 enable
#define NVIC_ISER1_INT51          0x00080000  // Interrupt 51 enable
#define NVIC_ISER1_INT52          0x00100000  // Interrupt 52 enable
#define NVIC_ISER1_INT53          0x00200000  // Interrupt 53 enable
#define NVIC_ISER1_INT54          0x00400000  // Interrupt 54 enable

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ICER0 NVIC Disable0 Register(NVIC_ICER0)
//! \brief Defines for the bit fields in the NVIC_ICER0 register.
//! @{
//
//*****************************************************************************
#define NVIC_ICER0_INT_M         0xFFFFFFFF  // Interrupt Disable
#define NVIC_ICER0_INT0          0x00000001  // Interrupt 0 disable
#define NVIC_ICER0_INT1          0x00000002  // Interrupt 1 disable
#define NVIC_ICER0_INT2          0x00000004  // Interrupt 2 disable
#define NVIC_ICER0_INT3          0x00000008  // Interrupt 3 disable
#define NVIC_ICER0_INT4          0x00000010  // Interrupt 4 disable
#define NVIC_ICER0_INT5          0x00000020  // Interrupt 5 disable
#define NVIC_ICER0_INT6          0x00000040  // Interrupt 6 disable
#define NVIC_ICER0_INT7          0x00000080  // Interrupt 7 disable
#define NVIC_ICER0_INT8          0x00000100  // Interrupt 8 disable
#define NVIC_ICER0_INT9          0x00000200  // Interrupt 9 disable
#define NVIC_ICER0_INT10         0x00000400  // Interrupt 10 disable
#define NVIC_ICER0_INT11         0x00000800  // Interrupt 11 disable
#define NVIC_ICER0_INT12         0x00001000  // Interrupt 12 disable
#define NVIC_ICER0_INT13         0x00002000  // Interrupt 13 disable
#define NVIC_ICER0_INT14         0x00004000  // Interrupt 14 disable
#define NVIC_ICER0_INT15         0x00008000  // Interrupt 15 disable
#define NVIC_ICER0_INT16         0x00010000  // Interrupt 16 disable
#define NVIC_ICER0_INT17         0x00020000  // Interrupt 17 disable
#define NVIC_ICER0_INT18         0x00040000  // Interrupt 18 disable
#define NVIC_ICER0_INT19         0x00080000  // Interrupt 19 disable
#define NVIC_ICER0_INT20         0x00100000  // Interrupt 20 disable
#define NVIC_ICER0_INT21         0x00200000  // Interrupt 21 disable
#define NVIC_ICER0_INT22         0x00400000  // Interrupt 22 disable
#define NVIC_ICER0_INT23         0x00800000  // Interrupt 23 disable
#define NVIC_ICER0_INT24         0x01000000  // Interrupt 24 disable
#define NVIC_ICER0_INT25         0x02000000  // Interrupt 25 disable
#define NVIC_ICER0_INT26         0x04000000  // Interrupt 26 disable
#define NVIC_ICER0_INT27         0x08000000  // Interrupt 27 disable
#define NVIC_ICER0_INT28         0x10000000  // Interrupt 28 disable
#define NVIC_ICER0_INT29         0x20000000  // Interrupt 29 disable
#define NVIC_ICER0_INT30         0x40000000  // Interrupt 30 disable
#define NVIC_ICER0_INT31         0x80000000  // Interrupt 31 disable

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ICER1 NVIC Disable1 Register(NVIC_ICER1)
//! \brief Defines for the bit fields in the NVIC_ICER1 register.
//! @{
//
//*****************************************************************************
#define NVIC_ICER1_INT_M         0x00FFFFFF  // Interrupt Disable
#define NVIC_ICER1_INT32         0x00000001  // Interrupt 32 disable
#define NVIC_ICER1_INT33         0x00000002  // Interrupt 33 disable
#define NVIC_ICER1_INT34         0x00000004  // Interrupt 34 disable
#define NVIC_ICER1_INT35         0x00000008  // Interrupt 35 disable
#define NVIC_ICER1_INT36         0x00000010  // Interrupt 36 disable
#define NVIC_ICER1_INT37         0x00000020  // Interrupt 37 disable
#define NVIC_ICER1_INT38         0x00000040  // Interrupt 38 disable
#define NVIC_ICER1_INT39         0x00000080  // Interrupt 39 disable
#define NVIC_ICER1_INT40         0x00000100  // Interrupt 40 disable
#define NVIC_ICER1_INT41         0x00000200  // Interrupt 41 disable
#define NVIC_ICER1_INT42         0x00000400  // Interrupt 42 disable
#define NVIC_ICER1_INT43         0x00000800  // Interrupt 43 disable
#define NVIC_ICER1_INT44         0x00001000  // Interrupt 44 disable
#define NVIC_ICER1_INT45         0x00002000  // Interrupt 45 disable
#define NVIC_ICER1_INT46         0x00004000  // Interrupt 46 disable
#define NVIC_ICER1_INT47         0x00008000  // Interrupt 47 disable
#define NVIC_ICER1_INT48         0x00010000  // Interrupt 48 disable
#define NVIC_ICER1_INT49         0x00020000  // Interrupt 49 disable
#define NVIC_ICER1_INT50         0x00040000  // Interrupt 50 disable
#define NVIC_ICER1_INT51         0x00080000  // Interrupt 51 disable
#define NVIC_ICER1_INT52         0x00100000  // Interrupt 52 disable
#define NVIC_ICER1_INT53         0x00200000  // Interrupt 53 disable
#define NVIC_ICER1_INT54         0x00400000  // Interrupt 54 disable
#define NVIC_ICER1_INT55         0x00800000  // Interrupt 55 disable

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ISPR0 NVIC PEND 0 Register(NVIC_ISPR0)
//! \brief Defines for the bit fields in the NVIC_ISPR0 register.
//! @{
//
//*****************************************************************************
#define NVIC_ISPR0_INT_M        0xFFFFFFFF  // Interrupt Set Pending
#define NVIC_ISPR0_INT0         0x00000001  // Interrupt 0 pend
#define NVIC_ISPR0_INT1         0x00000002  // Interrupt 1 pend
#define NVIC_ISPR0_INT2         0x00000004  // Interrupt 2 pend
#define NVIC_ISPR0_INT3         0x00000008  // Interrupt 3 pend
#define NVIC_ISPR0_INT4         0x00000010  // Interrupt 4 pend
#define NVIC_ISPR0_INT5         0x00000020  // Interrupt 5 pend
#define NVIC_ISPR0_INT6         0x00000040  // Interrupt 6 pend
#define NVIC_ISPR0_INT7         0x00000080  // Interrupt 7 pend
#define NVIC_ISPR0_INT8         0x00000100  // Interrupt 8 pend
#define NVIC_ISPR0_INT9         0x00000200  // Interrupt 9 pend
#define NVIC_ISPR0_INT10        0x00000400  // Interrupt 10 pend
#define NVIC_ISPR0_INT11        0x00000800  // Interrupt 11 pend
#define NVIC_ISPR0_INT12        0x00001000  // Interrupt 12 pend
#define NVIC_ISPR0_INT13        0x00002000  // Interrupt 13 pend
#define NVIC_ISPR0_INT14        0x00004000  // Interrupt 14 pend
#define NVIC_ISPR0_INT15        0x00008000  // Interrupt 15 pend
#define NVIC_ISPR0_INT16        0x00010000  // Interrupt 16 pend
#define NVIC_ISPR0_INT17        0x00020000  // Interrupt 17 pend
#define NVIC_ISPR0_INT18        0x00040000  // Interrupt 18 pend
#define NVIC_ISPR0_INT19        0x00080000  // Interrupt 19 pend
#define NVIC_ISPR0_INT20        0x00100000  // Interrupt 20 pend
#define NVIC_ISPR0_INT21        0x00200000  // Interrupt 21 pend
#define NVIC_ISPR0_INT22        0x00400000  // Interrupt 22 pend
#define NVIC_ISPR0_INT23        0x00800000  // Interrupt 23 pend
#define NVIC_ISPR0_INT24        0x01000000  // Interrupt 24 pend
#define NVIC_ISPR0_INT25        0x02000000  // Interrupt 25 pend
#define NVIC_ISPR0_INT26        0x04000000  // Interrupt 26 pend
#define NVIC_ISPR0_INT27        0x08000000  // Interrupt 27 pend
#define NVIC_ISPR0_INT28        0x10000000  // Interrupt 28 pend
#define NVIC_ISPR0_INT29        0x20000000  // Interrupt 29 pend
#define NVIC_ISPR0_INT30        0x40000000  // Interrupt 30 pend
#define NVIC_ISPR0_INT31        0x80000000  // Interrupt 31 pend

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ISPR1 NVIC PEND 1 Register(NVIC_ISPR1)
//! \brief Defines for the bit fields in the NVIC_ISPR1 register.
//! @{
//
//*****************************************************************************

#define NVIC_ISPR1_INT_M        0x00FFFFFF  // Interrupt Set Pending
#define NVIC_ISPR1_INT32        0x00000001  // Interrupt 32 pend
#define NVIC_ISPR1_INT33        0x00000002  // Interrupt 33 pend
#define NVIC_ISPR1_INT34        0x00000004  // Interrupt 34 pend
#define NVIC_ISPR1_INT35        0x00000008  // Interrupt 35 pend
#define NVIC_ISPR1_INT36        0x00000010  // Interrupt 36 pend
#define NVIC_ISPR1_INT37        0x00000020  // Interrupt 37 pend
#define NVIC_ISPR1_INT38        0x00000040  // Interrupt 38 pend
#define NVIC_ISPR1_INT39        0x00000080  // Interrupt 39 pend
#define NVIC_ISPR1_INT40        0x00000100  // Interrupt 40 pend
#define NVIC_ISPR1_INT41        0x00000200  // Interrupt 41 pend
#define NVIC_ISPR1_INT42        0x00000400  // Interrupt 42 pend
#define NVIC_ISPR1_INT43        0x00000800  // Interrupt 43 pend
#define NVIC_ISPR1_INT44        0x00001000  // Interrupt 44 pend
#define NVIC_ISPR1_INT45        0x00002000  // Interrupt 45 pend
#define NVIC_ISPR1_INT46        0x00004000  // Interrupt 46 pend
#define NVIC_ISPR1_INT47        0x00008000  // Interrupt 47 pend
#define NVIC_ISPR1_INT48        0x00010000  // Interrupt 48 pend
#define NVIC_ISPR1_INT49        0x00020000  // Interrupt 49 pend
#define NVIC_ISPR1_INT50        0x00040000  // Interrupt 50 pend
#define NVIC_ISPR1_INT51        0x00080000  // Interrupt 51 pend
#define NVIC_ISPR1_INT52        0x00100000  // Interrupt 52 pend
#define NVIC_ISPR1_INT53        0x00200000  // Interrupt 53 pend
#define NVIC_ISPR1_INT54        0x00400000  // Interrupt 54 pend
#define NVIC_ISPR1_INT55        0x00800000  // Interrupt 55 pend

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ICPR0 NVIC UNPEND 0 Register(NVIC_ICPR0)
//! \brief Defines for the bit fields in the NVIC_ICPR0 register.
//! @{
//
//*****************************************************************************

#define NVIC_ICPR0_INT_M        0xFFFFFFFF  // Interrupt Clear Pending
#define NVIC_ICPR0_INT0         0x00000001  // Interrupt 0 unpend
#define NVIC_ICPR0_INT1         0x00000002  // Interrupt 1 unpend
#define NVIC_ICPR0_INT2         0x00000004  // Interrupt 2 unpend
#define NVIC_ICPR0_INT3         0x00000008  // Interrupt 3 unpend
#define NVIC_ICPR0_INT4         0x00000010  // Interrupt 4 unpend
#define NVIC_ICPR0_INT5         0x00000020  // Interrupt 5 unpend
#define NVIC_ICPR0_INT6         0x00000040  // Interrupt 6 unpend
#define NVIC_ICPR0_INT7         0x00000080  // Interrupt 7 unpend
#define NVIC_ICPR0_INT8         0x00000100  // Interrupt 8 unpend
#define NVIC_ICPR0_INT9         0x00000200  // Interrupt 9 unpend
#define NVIC_ICPR0_INT10        0x00000400  // Interrupt 10 unpend
#define NVIC_ICPR0_INT11        0x00000800  // Interrupt 11 unpend
#define NVIC_ICPR0_INT12        0x00001000  // Interrupt 12 unpend
#define NVIC_ICPR0_INT13        0x00002000  // Interrupt 13 unpend
#define NVIC_ICPR0_INT14        0x00004000  // Interrupt 14 unpend
#define NVIC_ICPR0_INT15        0x00008000  // Interrupt 15 unpend
#define NVIC_ICPR0_INT16        0x00010000  // Interrupt 16 unpend
#define NVIC_ICPR0_INT17        0x00020000  // Interrupt 17 unpend
#define NVIC_ICPR0_INT18        0x00040000  // Interrupt 18 unpend
#define NVIC_ICPR0_INT19        0x00080000  // Interrupt 19 unpend
#define NVIC_ICPR0_INT20        0x00100000  // Interrupt 20 unpend
#define NVIC_ICPR0_INT21        0x00200000  // Interrupt 21 unpend
#define NVIC_ICPR0_INT22        0x00400000  // Interrupt 22 unpend
#define NVIC_ICPR0_INT23        0x00800000  // Interrupt 23 unpend
#define NVIC_ICPR0_INT24        0x01000000  // Interrupt 24 unpend
#define NVIC_ICPR0_INT25        0x02000000  // Interrupt 25 unpend
#define NVIC_ICPR0_INT26        0x04000000  // Interrupt 26 unpend
#define NVIC_ICPR0_INT27        0x08000000  // Interrupt 27 unpend
#define NVIC_ICPR0_INT28        0x10000000  // Interrupt 28 unpend
#define NVIC_ICPR0_INT29        0x20000000  // Interrupt 29 unpend
#define NVIC_ICPR0_INT30        0x40000000  // Interrupt 30 unpend
#define NVIC_ICPR0_INT31        0x80000000  // Interrupt 31 unpend

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_ICPR1 NVIC UNPEND 1 Register(NVIC_ICPR1)
//! \brief Defines for the bit fields in the NVIC_ICPR1 register.
//! @{
//
//*****************************************************************************

#define NVIC_ICPR1_INT_M        0x00FFFFFF  // Interrupt Clear Pending
#define NVIC_ICPR1_INT32        0x00000001  // Interrupt 32 unpend
#define NVIC_ICPR1_INT33        0x00000002  // Interrupt 33 unpend
#define NVIC_ICPR1_INT34        0x00000004  // Interrupt 34 unpend
#define NVIC_ICPR1_INT35        0x00000008  // Interrupt 35 unpend
#define NVIC_ICPR1_INT36        0x00000010  // Interrupt 36 unpend
#define NVIC_ICPR1_INT37        0x00000020  // Interrupt 37 unpend
#define NVIC_ICPR1_INT38        0x00000040  // Interrupt 38 unpend
#define NVIC_ICPR1_INT39        0x00000080  // Interrupt 39 unpend
#define NVIC_ICPR1_INT40        0x00000100  // Interrupt 40 unpend
#define NVIC_ICPR1_INT41        0x00000200  // Interrupt 41 unpend
#define NVIC_ICPR1_INT42        0x00000400  // Interrupt 42 unpend
#define NVIC_ICPR1_INT43        0x00000800  // Interrupt 43 unpend
#define NVIC_ICPR1_INT44        0x00001000  // Interrupt 44 unpend
#define NVIC_ICPR1_INT45        0x00002000  // Interrupt 45 unpend
#define NVIC_ICPR1_INT46        0x00004000  // Interrupt 46 unpend
#define NVIC_ICPR1_INT47        0x00008000  // Interrupt 47 unpend
#define NVIC_ICPR1_INT48        0x00010000  // Interrupt 48 unpend
#define NVIC_ICPR1_INT49        0x00020000  // Interrupt 49 unpend
#define NVIC_ICPR1_INT50        0x00040000  // Interrupt 50 unpend
#define NVIC_ICPR1_INT51        0x00080000  // Interrupt 51 unpend
#define NVIC_ICPR1_INT52        0x00100000  // Interrupt 52 unpend
#define NVIC_ICPR1_INT53        0x00200000  // Interrupt 53 unpend
#define NVIC_ICPR1_INT54        0x00400000  // Interrupt 54 unpend
#define NVIC_ICPR1_INT55        0x00800000  // Interrupt 55 unpend

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_IABR0 NVIC ACTIVE 0 Register(NVIC_IABR0)
//! \brief Defines for the bit fields in the NVIC_IABR0 register.
//! @{
//
//*****************************************************************************

#define NVIC_IABR0_INT_M        0xFFFFFFFF  // Interrupt Active
#define NVIC_IABR0_INT0         0x00000001  // Interrupt 0 active
#define NVIC_IABR0_INT1         0x00000002  // Interrupt 1 active
#define NVIC_IABR0_INT2         0x00000004  // Interrupt 2 active
#define NVIC_IABR0_INT3         0x00000008  // Interrupt 3 active
#define NVIC_IABR0_INT4         0x00000010  // Interrupt 4 active
#define NVIC_IABR0_INT5         0x00000020  // Interrupt 5 active
#define NVIC_IABR0_INT6         0x00000040  // Interrupt 6 active
#define NVIC_IABR0_INT7         0x00000080  // Interrupt 7 active
#define NVIC_IABR0_INT8         0x00000100  // Interrupt 8 active
#define NVIC_IABR0_INT9         0x00000200  // Interrupt 9 active
#define NVIC_IABR0_INT10        0x00000400  // Interrupt 10 active
#define NVIC_IABR0_INT11        0x00000800  // Interrupt 11 active
#define NVIC_IABR0_INT12        0x00001000  // Interrupt 12 active
#define NVIC_IABR0_INT13        0x00002000  // Interrupt 13 active
#define NVIC_IABR0_INT14        0x00004000  // Interrupt 14 active
#define NVIC_IABR0_INT15        0x00008000  // Interrupt 15 active
#define NVIC_IABR0_INT16        0x00010000  // Interrupt 16 active
#define NVIC_IABR0_INT17        0x00020000  // Interrupt 17 active
#define NVIC_IABR0_INT18        0x00040000  // Interrupt 18 active
#define NVIC_IABR0_INT19        0x00080000  // Interrupt 19 active
#define NVIC_IABR0_INT20        0x00100000  // Interrupt 20 active
#define NVIC_IABR0_INT21        0x00200000  // Interrupt 21 active
#define NVIC_IABR0_INT22        0x00400000  // Interrupt 22 active
#define NVIC_IABR0_INT23        0x00800000  // Interrupt 23 active
#define NVIC_IABR0_INT24        0x01000000  // Interrupt 24 active
#define NVIC_IABR0_INT25        0x02000000  // Interrupt 25 active
#define NVIC_IABR0_INT26        0x04000000  // Interrupt 26 active
#define NVIC_IABR0_INT27        0x08000000  // Interrupt 27 active
#define NVIC_IABR0_INT28        0x10000000  // Interrupt 28 active
#define NVIC_IABR0_INT29        0x20000000  // Interrupt 29 active
#define NVIC_IABR0_INT30        0x40000000  // Interrupt 30 active
#define NVIC_IABR0_INT31        0x80000000  // Interrupt 31 active

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_IABR1 NVIC ACTIVE 1 Register(NVIC_IABR1)
//! \brief Defines for the bit fields in the NVIC_IABR1 register.
//! @{
//
//*****************************************************************************

#define NVIC_IABR1_INT_M        0x00FFFFFF  // Interrupt Active
#define NVIC_IABR1_INT32        0x00000001  // Interrupt 32 active
#define NVIC_IABR1_INT33        0x00000002  // Interrupt 33 active
#define NVIC_IABR1_INT34        0x00000004  // Interrupt 34 active
#define NVIC_IABR1_INT35        0x00000008  // Interrupt 35 active
#define NVIC_IABR1_INT36        0x00000010  // Interrupt 36 active
#define NVIC_IABR1_INT37        0x00000020  // Interrupt 37 active
#define NVIC_IABR1_INT38        0x00000040  // Interrupt 38 active
#define NVIC_IABR1_INT39        0x00000080  // Interrupt 39 active
#define NVIC_IABR1_INT40        0x00000100  // Interrupt 40 active
#define NVIC_IABR1_INT41        0x00000200  // Interrupt 41 active
#define NVIC_IABR1_INT42        0x00000400  // Interrupt 42 active
#define NVIC_IABR1_INT43        0x00000800  // Interrupt 43 active
#define NVIC_IABR1_INT44        0x00001000  // Interrupt 44 active
#define NVIC_IABR1_INT45        0x00002000  // Interrupt 45 active
#define NVIC_IABR1_INT46        0x00004000  // Interrupt 46 active
#define NVIC_IABR1_INT47        0x00008000  // Interrupt 47 active
#define NVIC_IABR1_INT48        0x00010000  // Interrupt 48 active
#define NVIC_IABR1_INT49        0x00020000  // Interrupt 49 active
#define NVIC_IABR1_INT50        0x00040000  // Interrupt 50 active
#define NVIC_IABR1_INT51        0x00080000  // Interrupt 51 active
#define NVIC_IABR1_INT52        0x00100000  // Interrupt 52 active
#define NVIC_IABR1_INT53        0x00200000  // Interrupt 53 active
#define NVIC_IABR1_INT54        0x00400000  // Interrupt 54 active
#define NVIC_IABR1_INT55        0x00800000  // Interrupt 55 active

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI0 NVIC Priority 0 Register(NVIC_PRI0)
//! \brief Defines for the bit fields in the NVIC_PRI0 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI0_INT3_M        0xF8000000  // Interrupt 3 Priority Mask
#define NVIC_PRI0_INT2_M        0x00F80000  // Interrupt 2 Priority Mask
#define NVIC_PRI0_INT1_M        0x0000F80  // Interrupt 1 Priority Mask
#define NVIC_PRI0_INT0_M        0x000000F8  // Interrupt 0 Priority Mask
#define NVIC_PRI0_INT3_S        27
#define NVIC_PRI0_INT2_S        19
#define NVIC_PRI0_INT1_S        11
#define NVIC_PRI0_INT0_S        3

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI1 NVIC Priority 1 Register(NVIC_PRI1)
//! \brief Defines for the bit fields in the NVIC_PRI1 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI1_INT7_M        0xE0000000  // Interrupt 7 Priority Mask
#define NVIC_PRI1_INT6_M        0x00E00000  // Interrupt 6 Priority Mask
#define NVIC_PRI1_INT5_M        0x0000E000  // Interrupt 5 Priority Mask
#define NVIC_PRI1_INT4_M        0x000000E0  // Interrupt 4 Priority Mask
#define NVIC_PRI1_INT7_S        29
#define NVIC_PRI1_INT6_S        21
#define NVIC_PRI1_INT5_S        13
#define NVIC_PRI1_INT4_S        5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI2 NVIC Priority 2 Register(NVIC_PRI2)
//! \brief Defines for the bit fields in the NVIC_PRI2 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI2_INT11_M       0xE0000000  // Interrupt 11 Priority Mask
#define NVIC_PRI2_INT10_M       0x00E00000  // Interrupt 10 Priority Mask
#define NVIC_PRI2_INT9_M        0x0000E000  // Interrupt 9 Priority Mask
#define NVIC_PRI2_INT8_M        0x000000E0  // Interrupt 8 Priority Mask
#define NVIC_PRI2_INT11_S       29
#define NVIC_PRI2_INT10_S       21
#define NVIC_PRI2_INT9_S        13
#define NVIC_PRI2_INT8_S        5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI3 NVIC Priority 3 Register(NVIC_PRI3)
//! \brief Defines for the bit fields in the NVIC_PRI3 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI3_INT15_M       0xE0000000  // Interrupt 15 Priority Mask
#define NVIC_PRI3_INT14_M       0x00E00000  // Interrupt 14 Priority Mask
#define NVIC_PRI3_INT13_M       0x0000E000  // Interrupt 13 Priority Mask
#define NVIC_PRI3_INT12_M       0x000000E0  // Interrupt 12 Priority Mask
#define NVIC_PRI3_INT15_S       29
#define NVIC_PRI3_INT14_S       21
#define NVIC_PRI3_INT13_S       13
#define NVIC_PRI3_INT12_S       5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI4 NVIC Priority 4 Register(NVIC_PRI4)
//! \brief Defines for the bit fields in the NVIC_PRI4 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI4_INT19_M       0xE0000000  // Interrupt 19 Priority Mask
#define NVIC_PRI4_INT18_M       0x00E00000  // Interrupt 18 Priority Mask
#define NVIC_PRI4_INT17_M       0x0000E000  // Interrupt 17 Priority Mask
#define NVIC_PRI4_INT16_M       0x000000E0  // Interrupt 16 Priority Mask
#define NVIC_PRI4_INT19_S       29
#define NVIC_PRI4_INT18_S       21
#define NVIC_PRI4_INT17_S       13
#define NVIC_PRI4_INT16_S       5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI5 NVIC Priority 5 Register(NVIC_PRI5)
//! \brief Defines for the bit fields in the NVIC_PRI5 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI5_INT23_M       0xE0000000  // Interrupt 23 Priority Mask
#define NVIC_PRI5_INT22_M       0x00E00000  // Interrupt 22 Priority Mask
#define NVIC_PRI5_INT21_M       0x0000E000  // Interrupt 21 Priority Mask
#define NVIC_PRI5_INT20_M       0x000000E0  // Interrupt 20 Priority Mask
#define NVIC_PRI5_INT23_S       29
#define NVIC_PRI5_INT22_S       21
#define NVIC_PRI5_INT21_S       13
#define NVIC_PRI5_INT20_S       5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI6 NVIC Priority 6 Register(NVIC_PRI6)
//! \brief Defines for the bit fields in the NVIC_PRI6 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI6_INT27_M       0xE0000000  // Interrupt 27 Priority Mask
#define NVIC_PRI6_INT26_M       0x00E00000  // Interrupt 26 Priority Mask
#define NVIC_PRI6_INT25_M       0x0000E000  // Interrupt 25 Priority Mask
#define NVIC_PRI6_INT24_M       0x000000E0  // Interrupt 24 Priority Mask
#define NVIC_PRI6_INT27_S       29
#define NVIC_PRI6_INT26_S       21
#define NVIC_PRI6_INT25_S       13
#define NVIC_PRI6_INT24_S       5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI7 NVIC Priority 7 Register(NVIC_PRI7)
//! \brief Defines for the bit fields in the NVIC_PRI7 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI7_INT31_M       0xE0000000  // Interrupt 31 Priority Mask
#define NVIC_PRI7_INT30_M       0x00E00000  // Interrupt 30 Priority Mask
#define NVIC_PRI7_INT29_M       0x0000E000  // Interrupt 29 Priority Mask
#define NVIC_PRI7_INT28_M       0x000000E0  // Interrupt 28 Priority Mask
#define NVIC_PRI7_INT31_S       29
#define NVIC_PRI7_INT30_S       21
#define NVIC_PRI7_INT29_S       13
#define NVIC_PRI7_INT28_S       5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI8 NVIC Priority 8 Register(NVIC_PRI8)
//! \brief Defines for the bit fields in the NVIC_PRI8 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI8_INT35_M       0xE0000000  // Interrupt 35 Priority Mask
#define NVIC_PRI8_INT34_M       0x00E00000  // Interrupt 34 Priority Mask
#define NVIC_PRI8_INT33_M       0x0000E000  // Interrupt 33 Priority Mask
#define NVIC_PRI8_INT32_M       0x000000E0  // Interrupt 32 Priority Mask
#define NVIC_PRI8_INT35_S       29
#define NVIC_PRI8_INT34_S       21
#define NVIC_PRI8_INT33_S       13
#define NVIC_PRI8_INT32_S       5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI9 NVIC Priority 9 Register(NVIC_PRI9)
//! \brief Defines for the bit fields in the NVIC_PRI9 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI9_INT39_M       0xE0000000  // Interrupt 39 Priority Mask
#define NVIC_PRI9_INT38_M       0x00E00000  // Interrupt 38 Priority Mask
#define NVIC_PRI9_INT37_M       0x0000E000  // Interrupt 37 Priority Mask
#define NVIC_PRI9_INT36_M       0x000000E0  // Interrupt 36 Priority Mask
#define NVIC_PRI9_INT39_S       29
#define NVIC_PRI9_INT38_S       21
#define NVIC_PRI9_INT37_S       13
#define NVIC_PRI9_INT36_S       5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI10 NVIC Priority 10 Register(NVIC_PRI10)
//! \brief Defines for the bit fields in the NVIC_PRI10 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI10_INT43_M      0xE0000000  // Interrupt 43 Priority Mask
#define NVIC_PRI10_INT42_M      0x00E00000  // Interrupt 42 Priority Mask
#define NVIC_PRI10_INT41_M      0x0000E000  // Interrupt 41 Priority Mask
#define NVIC_PRI10_INT40_M      0x000000E0  // Interrupt 40 Priority Mask
#define NVIC_PRI10_INT43_S      29
#define NVIC_PRI10_INT42_S      21
#define NVIC_PRI10_INT41_S      13
#define NVIC_PRI10_INT40_S      5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI11 NVIC Priority 11 Register(NVIC_PRI11)
//! \brief Defines for the bit fields in the NVIC_PRI11 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI11_INT47_M      0xE0000000  // Interrupt 47 Priority Mask
#define NVIC_PRI11_INT46_M      0x00E00000  // Interrupt 46 Priority Mask
#define NVIC_PRI11_INT45_M      0x0000E000  // Interrupt 45 Priority Mask
#define NVIC_PRI11_INT44_M      0x000000E0  // Interrupt 44 Priority Mask
#define NVIC_PRI11_INT47_S      29
#define NVIC_PRI11_INT46_S      21
#define NVIC_PRI11_INT45_S      13
#define NVIC_PRI11_INT44_S      5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI12 NVIC Priority 12 Register(NVIC_PRI12)
//! \brief Defines for the bit fields in the NVIC_PRI12 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI12_INT51_M      0xE0000000  // Interrupt 51 Priority Mask
#define NVIC_PRI12_INT50_M      0x00E00000  // Interrupt 50 Priority Mask
#define NVIC_PRI12_INT49_M      0x0000E000  // Interrupt 49 Priority Mask
#define NVIC_PRI12_INT48_M      0x000000E0  // Interrupt 48 Priority Mask
#define NVIC_PRI12_INT51_S      29
#define NVIC_PRI12_INT50_S      21
#define NVIC_PRI12_INT49_S      13
#define NVIC_PRI12_INT48_S      5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_PRI13 NVIC Priority 13 Register(NVIC_PRI13)
//! \brief Defines for the bit fields in the NVIC_PRI13 register.
//! @{
//
//*****************************************************************************

#define NVIC_PRI13_INT55_M      0xE0000000  // Interrupt 55 Priority Mask
#define NVIC_PRI13_INT54_M      0x00E00000  // Interrupt 54 Priority Mask
#define NVIC_PRI13_INT53_M      0x0000E000  // Interrupt 53 Priority Mask
#define NVIC_PRI13_INT52_M      0x000000E0  // Interrupt 52 Priority Mask
#define NVIC_PRI13_INT55_S      29
#define NVIC_PRI13_INT54_S      21
#define NVIC_PRI13_INT53_S      13
#define NVIC_PRI13_INT52_S      5

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_CPUID  NVIC CPUID Register(NVIC_CPUID)
//! \brief Defines for the bit fields in the NVIC_CPUID register.
//! @{
//
//*****************************************************************************

#define NVIC_CPUID_IMP_M        0xFF000000  // Implementer Code
#define NVIC_CPUID_IMP_ARM      0x41000000  // ARM
#define NVIC_CPUID_VAR_M        0x00F00000  // Variant Number
#define NVIC_CPUID_CON_M        0x000F0000  // Constant
#define NVIC_CPUID_PARTNO_M     0x0000FFF0  // Part Number
#define NVIC_CPUID_PARTNO_CM3   0x0000C230  // Cortex-M3 processor
#define NVIC_CPUID_REV_M        0x0000000F  // Revision Number

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_ICSR_CTRL NVIC Interrupt Control Register(NVIC_ICSR)
//! \brief Defines for the bit fields in the NVIC_ICSR register.
//! @{
//
//*****************************************************************************

#define NVIC_ICSR_NMI_SET       0x80000000  // NMI Set Pending
#define NVIC_ICSR_PENDSV_SET    0x10000000  // PendSV Set Pending
#define NVIC_ICSR_PENDSV_CLR    0x08000000  // PendSV Clear Pending
#define NVIC_ICSR_PENDST_SET    0x04000000  // SysTick Set Pending
#define NVIC_ICSR_PENDST_CLR    0x02000000  // SysTick Clear Pending
#define NVIC_ICSR_ISR_PRE       0x00800000  // Debug Interrupt Handling
#define NVIC_ICSR_ISR_PEND      0x00400000  // Interrupt Pending
#define NVIC_ICSR_VEC_PEN_M     0x0003F000  // Interrupt Pending Vector Number
#define NVIC_ICSR_VEC_PEN_NMI   0x00002000  // NMI
#define NVIC_ICSR_VEC_PEN_HARD  0x00003000  // Hard fault
#define NVIC_ICSR_VEC_PEN_MEM   0x00004000  // Memory management fault
#define NVIC_ICSR_VEC_PEN_BUS   0x00005000  // Bus fault
#define NVIC_ICSR_VEC_PEN_USG   0x00006000  // Usage fault
#define NVIC_ICSR_VEC_PEN_SVC   0x0000B000  // SVCall
#define NVIC_ICSR_VEC_PEN_PNDSV 0x0000E000  // PendSV
#define NVIC_ICSR_VEC_PEN_TICK  0x0000F000  // SysTick
#define NVIC_ICSR_RET_BASE      0x00000800  // Return to Base
#define NVIC_ICSR_VEC_ACT_M     0x000001FF  // Interrupt Pending Vector Number
#define NVIC_ICSR_VEC_PEN_S     12
#define NVIC_ICSR_VEC_ACT_S     0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_VTOR NVIC Interrupt Verctor Table Register(NVIC_VTOR)
//! \brief Defines for the bit fields in the NVIC_VTOR register.
//! @{
//
//*****************************************************************************

#define NVIC_VTOR_BASE        0x20000000  // Vector Table Base
#define NVIC_VTOR_OFFSET_M    0x3FFFFF00  // Vector Table Offset
#define NVIC_VTOR_OFFSET_S    8

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_AIRCR NVIC AIRCR Register(NVIC_AIRCR)
//! \brief Defines for the bit fields in the NVIC_AIRCR register.
//! @{
//
//*****************************************************************************

#define NVIC_AIRCR_VECTKEY_M    0xFFFF0000  // Register Key
#define NVIC_AIRCR_VECTKEY      0x05FA0000  // Vector key
#define NVIC_AIRCR_ENDIANESS    0x00008000  // Data Endianess
#define NVIC_AIRCR_PRIGROUP_M   0x00000700  // Interrupt Priority Grouping
#define NVIC_AIRCR_PRIGROUP_7_1 0x00000000  // Priority group 7.1 split
#define NVIC_AIRCR_PRIGROUP_6_2 0x00000100  // Priority group 6.2 split
#define NVIC_AIRCR_PRIGROUP_5_3 0x00000200  // Priority group 5.3 split
#define NVIC_AIRCR_PRIGROUP_4_4 0x00000300  // Priority group 4.4 split
#define NVIC_AIRCR_PRIGROUP_3_5 0x00000400  // Priority group 3.5 split
#define NVIC_AIRCR_PRIGROUP_2_6 0x00000500  // Priority group 2.6 split
#define NVIC_AIRCR_PRIGROUP_1_7 0x00000600  // Priority group 1.7 split
#define NVIC_AIRCR_PRIGROUP_0_8 0x00000700  // Priority group 0.8 split
#define NVIC_AIRCR_SYSRESETREQ  0x00000004  // System Reset Request
#define NVIC_AIRCR_VECT_CLR_ACT 0x00000002  // Clear Active NMI / Fault
#define NVIC_AIRCR_VECT_RESET   0x00000001  // System Reset

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_SCR NVIC System Control Register(NVIC_SCR)
//! \brief Defines for the bit fields in the NVIC_SCR register.
//! @{
//
//*****************************************************************************

#define NVIC_SCR_SEVONPEND      0x00000010  // Wake Up on Pending
#define NVIC_SCR_SLEEPDEEP      0x00000004  // Deep Sleep Enable
#define NVIC_SCR_SLEEPEXIT      0x00000002  // Sleep on ISR Exit

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_CCR NVIC Cfg Control Register(NVIC_CCR)
//! \brief Defines for the bit fields in the NVIC_CCR register.
//! @{
//
//*****************************************************************************

#define NVIC_CCR_STKALIGN       0x00000200  // Stack Alignment on Exception Entry
#define NVIC_CCR_BFHFNMIGN      0x00000100  // Ignore Bus Fault in NMI and Fault
#define NVIC_CCR_DIV0           0x00000010  // Trap on Divide by 0
#define NVIC_CCR_UNALIGNED      0x00000008  // Trap on Unaligned Access
#define NVIC_CCR_MAIN_PEND      0x00000002  // Allow Main Interrupt Trigger
#define NVIC_CCR_BASE_THR       0x00000001  // Thread State Control

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_SHPR1 NVIC System Priority 1 Register(NVIC_SHPR1)
//! \brief Defines for the bit fields in the NVIC_SHPR1 register.
//! @{
//
//*****************************************************************************

#define NVIC_SHPR1_USAGE_M   0x00FF0000  // Usage Fault Priority
#define NVIC_SHPR1_BUS_M     0x0000FF00  // Bus Fault Priority
#define NVIC_SHPR1_MEM_M     0x000000FF  // Memory Management Fault Priority
#define NVIC_SHPR1_USAGE_S   16
#define NVIC_SHPR1_BUS_S     8
#define NVIC_SHPR1_MEM_S     0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_SHPR2 NVIC System Priority 2 Register(NVIC_SHPR2)
//! \brief Defines for the bit fields in the NVIC_SHPR2 register.
//! @{
//
//*****************************************************************************

#define NVIC_SHPR2_SVC_M     0xFF000000  // SVCall Priority
#define NVIC_SHPR2_SVC_S     24

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_SHPR3 NVIC System Priority 3 Register(NVIC_SHPR3)
//! \brief Defines for the bit fields in the NVIC_SHPR3 register.
//! @{
//
//*****************************************************************************

#define NVIC_SHPR3_TICK_M    0xFF000000  // SysTick Exception Priority
#define NVIC_SHPR3_PENDSV_M  0x00FF0000  // PendSV Priority
#define NVIC_SHPR3_TICK_S    24
#define NVIC_SHPR3_PENDSV_S  16

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_SHCSR NVIC HND Control Register(NVIC_SHCSR)
//! \brief Defines for the bit fields in the NVIC_SHCSR register.
//! @{
//
//*****************************************************************************

#define NVIC_SHCSR_USAGE        0x00040000  // Usage Fault Enable
#define NVIC_SHCSR_BUS          0x00020000  // Bus Fault Enable
#define NVIC_SHCSR_MEM          0x00010000  // Memory Management Fault Enable
#define NVIC_SHCSR_SVC          0x00008000  // SVC Call Pending
#define NVIC_SHCSR_BUSP         0x00004000  // Bus Fault Pending
#define NVIC_SHCSR_MEMP         0x00002000  // Memory Management Fault Pending
#define NVIC_SHCSR_USAGEP       0x00001000  // Usage Fault Pending
#define NVIC_SHCSR_TICK         0x00000800  // SysTick Exception Active
#define NVIC_SHCSR_PNDSV        0x00000400  // PendSV Exception Active
#define NVIC_SHCSR_MON          0x00000100  // Debug Monitor Active
#define NVIC_SHCSR_SVCA         0x00000080  // SVC Call Active
#define NVIC_SHCSR_USGA         0x00000008  // Usage Fault Active
#define NVIC_SHCSR_BUSA         0x00000002  // Bus Fault Active
#define NVIC_SHCSR_MEMA         0x00000001  // Memory Management Fault Active

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_FAULT_STAT NVIC Fault State Register(NVIC_FAULT_STAT)
//! \brief Defines for the bit fields in the NVIC_FAULT_STAT register.
//! @{
//
//*****************************************************************************

#define NVIC_CFSR_DIV0    0x02000000  // Divide-by-Zero Usage Fault
#define NVIC_CFSR_UNALIGN 0x01000000  // Unaligned Access Usage Fault
#define NVIC_CFSR_NOCP    0x00080000  // No Coprocessor Usage Fault
#define NVIC_CFSR_INVPC   0x00040000  // Invalid PC Load Usage Fault
#define NVIC_CFSR_INVSTAT 0x00020000  // Invalid State Usage Fault
#define NVIC_CFSR_UNDEF   0x00010000  // Undefined Instruction Usage Fault
                                            
#define NVIC_CFSR_BFARV   0x00008000  // Bus Fault Address Register Valid
#define NVIC_CFSR_STKE    0x00001000  // Stack Bus Fault
#define NVIC_CFSR_UNSTKE  0x00000800  // Unstack Bus Fault
#define NVIC_CFSR_IMPRE   0x00000400  // Imprecise Data Bus Error
#define NVIC_CFSR_PRECISE 0x00000200  // Precise Data Bus Error
#define NVIC_CFSR_IBUS    0x00000100  // Instruction Bus Error

#define NVIC_CFSR_MMARV   0x00000080  // Memory Management Fault Address Register Valid                                         
#define NVIC_CFSR_MSTKE   0x00000010  // Stack Access Violation
#define NVIC_CFSR_MUSTKE  0x00000008  // Unstack Access Violation
#define NVIC_CFSR_DACC    0x00000002  // Data Access Violation Flag
#define NVIC_CFSR_IACC    0x00000001  // Instruction Access Violation Flag

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_HFSR NVIC Hard Fault Status Register
//! \brief Defines for the bit fields in the NVIC_HFSR register.
//! @{
//
//*****************************************************************************

#define NVIC_HFSR_DEBUGEVT      0x80000000  // Debug Event
#define NVIC_HFSR_FORCED        0x40000000  // Forced Hard Fault
#define NVIC_HFSR_VECTTBL       0x00000002  // Vector Table Read Fault

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MMFAR NVIC Memory Management Fault Address Register
//! \brief Defines for the bit fields in the NVIC_MMFAR register.
//! @{
//
//*****************************************************************************

#define NVIC_MMFAR_M            0xFFFFFFFF  // Fault Address
#define NVIC_MMFAR_S            0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_BFAR NVIC Bus Fault Address Register
//! \brief Defines for the bit fields in the NVIC_BFAR register.
//! @{
//
//*****************************************************************************

#define NVIC_BFAR_M       0xFFFFFFFF  // Fault Address
#define NVIC_BFAR_S       0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MPU_TYPE NVIC_MPU_TYPE
//! \brief Defines for the bit fields in the NVIC_MPU_TYPE register.
//! @{
//
//*****************************************************************************

#define NVIC_MPU_TYPE_IREGION_M 0x00FF0000  // Number of I Regions
#define NVIC_MPU_TYPE_DREGION_M 0x0000FF00  // Number of D Regions
#define NVIC_MPU_TYPE_SEPARATE  0x00000001  // Separate or Unified MPU
#define NVIC_MPU_TYPE_IREGION_S 16
#define NVIC_MPU_TYPE_DREGION_S 8

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MPU_CTRL NVIC_MPU_CTRL
//! \brief Defines for the bit fields in the NVIC_MPU_CTRL register.
//! @{
//
//*****************************************************************************

#define NVIC_MPU_CTRL_PRIVDEFEN 0x00000004  // MPU Default Region
#define NVIC_MPU_CTRL_HFNMIENA  0x00000002  // MPU Enabled During Faults
#define NVIC_MPU_CTRL_ENABLE    0x00000001  // MPU Enable

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MPU_RNR NVIC MPU Region Number Register
//! \brief Defines for the bit fields in the NVIC_MPU_RNR register.
//! @{
//
//*****************************************************************************

#define NVIC_MPU_RNR_M          0x00000007  // MPU Region to Access
#define NVIC_MPU_RNR_S          0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MPU_RBAR NVIC MPU Region Base Address Register
//! \brief Defines for the bit fields in the NVIC_MPU_RBAR register.
//! @{
//
//*****************************************************************************

#define NVIC_MPU_RBAR_ADDR_M    0xFFFFFFE0  // Base Address Mask
#define NVIC_MPU_RBAR_VALID     0x00000010  // Region Number Valid
#define NVIC_MPU_RBAR_REGION_M  0x00000007  // Region Number
#define NVIC_MPU_RBAR_ADDR_S    5
#define NVIC_MPU_RBAR_REGION_S  0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MPU_RASR NVIC MPU Region Attribute and Size Register
//! \brief Defines for the bit fields in the NVIC_MPU_RASR register.
//! @{
//
//*****************************************************************************

#define NVIC_MPU_RASR_M         0xFFFF0000  // Attributes
#define NVIC_MPU_RASR_XN        0x10000000  // Instruction Access Disable
#define NVIC_MPU_RASR_AP_M      0x07000000  // Access Privilege
#define NVIC_MPU_RASR_AP_NO_NO  0x00000000  // prv: no access, usr: no access
#define NVIC_MPU_RASR_AP_RW_NO  0x01000000  // prv: rw, usr: none
#define NVIC_MPU_RASR_AP_RW_RO  0x02000000  // prv: rw, usr: read-only
#define NVIC_MPU_RASR_AP_RW_RW  0x03000000  // prv: rw, usr: rw
#define NVIC_MPU_RASR_AP_RO_NO  0x05000000  // prv: ro, usr: none
#define NVIC_MPU_RASR_AP_RO_RO  0x06000000  // prv: ro, usr: ro
#define NVIC_MPU_RASR_TEX_M     0x00380000  // Type Extension Mask
#define NVIC_MPU_RASR_SHAREABLE 0x00040000  // Shareable
#define NVIC_MPU_RASR_CACHEABLE 0x00020000  // Cacheable
#define NVIC_MPU_RASR_BUFFRABLE 0x00010000  // Bufferable
#define NVIC_MPU_RASR_SRD_M     0x0000FF00  // Subregion Disable Bits
#define NVIC_MPU_RASR_SRD_0     0x00000100  // Sub-region 0 disable
#define NVIC_MPU_RASR_SRD_1     0x00000200  // Sub-region 1 disable
#define NVIC_MPU_RASR_SRD_2     0x00000400  // Sub-region 2 disable
#define NVIC_MPU_RASR_SRD_3     0x00000800  // Sub-region 3 disable
#define NVIC_MPU_RASR_SRD_4     0x00001000  // Sub-region 4 disable
#define NVIC_MPU_RASR_SRD_5     0x00002000  // Sub-region 5 disable
#define NVIC_MPU_RASR_SRD_6     0x00004000  // Sub-region 6 disable
#define NVIC_MPU_RASR_SRD_7     0x00008000  // Sub-region 7 disable
#define NVIC_MPU_RASR_SIZE_M    0x0000003E  // Region Size Mask
#define NVIC_MPU_RASR_SIZE_32B  0x00000008  // Region size 32 bytes
#define NVIC_MPU_RASR_SIZE_64B  0x0000000A  // Region size 64 bytes
#define NVIC_MPU_RASR_SIZE_128B 0x0000000C  // Region size 128 bytes
#define NVIC_MPU_RASR_SIZE_256B 0x0000000E  // Region size 256 bytes
#define NVIC_MPU_RASR_SIZE_512B 0x00000010  // Region size 512 bytes
#define NVIC_MPU_RASR_SIZE_1K   0x00000012  // Region size 1 Kbytes
#define NVIC_MPU_RASR_SIZE_2K   0x00000014  // Region size 2 Kbytes
#define NVIC_MPU_RASR_SIZE_4K   0x00000016  // Region size 4 Kbytes
#define NVIC_MPU_RASR_SIZE_8K   0x00000018  // Region size 8 Kbytes
#define NVIC_MPU_RASR_SIZE_16K  0x0000001A  // Region size 16 Kbytes
#define NVIC_MPU_RASR_SIZE_32K  0x0000001C  // Region size 32 Kbytes
#define NVIC_MPU_RASR_SIZE_64K  0x0000001E  // Region size 64 Kbytes
#define NVIC_MPU_RASR_SIZE_128K 0x00000020  // Region size 128 Kbytes
#define NVIC_MPU_RASR_SIZE_256K 0x00000022  // Region size 256 Kbytes
#define NVIC_MPU_RASR_SIZE_512K 0x00000024  // Region size 512 Kbytes
#define NVIC_MPU_RASR_SIZE_1M   0x00000026  // Region size 1 Mbytes
#define NVIC_MPU_RASR_SIZE_2M   0x00000028  // Region size 2 Mbytes
#define NVIC_MPU_RASR_SIZE_4M   0x0000002A  // Region size 4 Mbytes
#define NVIC_MPU_RASR_SIZE_8M   0x0000002C  // Region size 8 Mbytes
#define NVIC_MPU_RASR_SIZE_16M  0x0000002E  // Region size 16 Mbytes
#define NVIC_MPU_RASR_SIZE_32M  0x00000030  // Region size 32 Mbytes
#define NVIC_MPU_RASR_SIZE_64M  0x00000032  // Region size 64 Mbytes
#define NVIC_MPU_RASR_SIZE_128M 0x00000034  // Region size 128 Mbytes
#define NVIC_MPU_RASR_SIZE_256M 0x00000036  // Region size 256 Mbytes
#define NVIC_MPU_RASR_SIZE_512M 0x00000038  // Region size 512 Mbytes
#define NVIC_MPU_RASR_SIZE_1G   0x0000003A  // Region size 1 Gbytes
#define NVIC_MPU_RASR_SIZE_2G   0x0000003C  // Region size 2 Gbytes
#define NVIC_MPU_RASR_SIZE_4G   0x0000003E  // Region size 4 Gbytes
#define NVIC_MPU_RASR_ENABLE    0x00000001  // Region Enable

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MPU_BASE1 NVIC_MPU_BASE1
//! \brief Defines for the bit fields in the NVIC_MPU_BASE1 register.
//! @{
//
//*****************************************************************************

#define NVIC_MPU_RBAR1_ADDR_M   0xFFFFFFE0  // Base Address Mask
#define NVIC_MPU_RBAR1_VALID    0x00000010  // Region Number Valid
#define NVIC_MPU_RBAR1_REGION_M 0x00000007  // Region Number
#define NVIC_MPU_RBAR1_ADDR_S   5
#define NVIC_MPU_RBAR1_REGION_S 0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MPU_ATTR1 NVIC_MPU_ATTR1
//! \brief Defines for the bit fields in the NVIC_MPU_ATTR1 register.
//! @{
//
//*****************************************************************************

#define NVIC_MPU_RASR1_XN       0x10000000  // Instruction Access Disable
#define NVIC_MPU_RASR1_AP_M     0x07000000  // Access Privilege
#define NVIC_MPU_RASR1_TEX_M    0x00380000  // Type Extension Mask
#define NVIC_MPU_RASR1_SHAREABLE \
                                0x00040000  // Shareable
#define NVIC_MPU_RASR1_CACHEABLE \
                                0x00020000  // Cacheable
#define NVIC_MPU_RASR1_BUFFRABLE \
                                0x00010000  // Bufferable
#define NVIC_MPU_RASR1_SRD_M    0x0000FF00  // Subregion Disable Bits
#define NVIC_MPU_RASR1_SIZE_M   0x0000003E  // Region Size Mask
#define NVIC_MPU_RASR1_ENABLE   0x00000001  // Region Enable

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MPU_BASE2 NVIC_MPU_BASE2
//! \brief Defines for the bit fields in the NVIC_MPU_BASE2 register.
//! @{
//
//*****************************************************************************

#define NVIC_MPU_RBAR2_ADDR_M   0xFFFFFFE0  // Base Address Mask
#define NVIC_MPU_RBAR2_VALID    0x00000010  // Region Number Valid
#define NVIC_MPU_RBAR2_REGION_M 0x00000007  // Region Number
#define NVIC_MPU_RBAR2_ADDR_S   5
#define NVIC_MPU_RBAR2_REGION_S 0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MPU_ATTR2 NVIC_MPU_ATTR2
//! \brief Defines for the bit fields in the NVIC_MPU_ATTR2 register.
//! @{
//
//*****************************************************************************

#define NVIC_MPU_RASR2_XN       0x10000000  // Instruction Access Disable
#define NVIC_MPU_RASR2_AP_M     0x07000000  // Access Privilege
#define NVIC_MPU_RASR2_TEX_M    0x00380000  // Type Extension Mask
#define NVIC_MPU_RASR2_SHAREABLE \
                                0x00040000  // Shareable
#define NVIC_MPU_RASR2_CACHEABLE \
                                0x00020000  // Cacheable
#define NVIC_MPU_RASR2_BUFFRABLE \
                                0x00010000  // Bufferable
#define NVIC_MPU_RASR2_SRD_M    0x0000FF00  // Subregion Disable Bits
#define NVIC_MPU_RASR2_SIZE_M   0x0000003E  // Region Size Mask
#define NVIC_MPU_RASR2_ENABLE   0x00000001  // Region Enable

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MPU_BASE3 NVIC_MPU_BASE3
//! \brief Defines for the bit fields in the NVIC_MPU_BASE3 register.
//! @{
//
//*****************************************************************************

#define NVIC_MPU_RBAR3_ADDR_M   0xFFFFFFE0  // Base Address Mask
#define NVIC_MPU_RBAR3_VALID    0x00000010  // Region Number Valid
#define NVIC_MPU_RBAR3_REGION_M 0x00000007  // Region Number
#define NVIC_MPU_RBAR3_ADDR_S   5
#define NVIC_MPU_RBAR3_REGION_S 0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_MPU_ATTR3 NVIC_MPU_ATTR3
//! \brief Defines for the bit fields in the NVIC_MPU_ATTR3 register.
//! @{
//
//*****************************************************************************

#define NVIC_MPU_RASR3_XN       0x10000000  // Instruction Access Disable
#define NVIC_MPU_RASR3_AP_M     0x07000000  // Access Privilege
#define NVIC_MPU_RASR3_TEX_M    0x00380000  // Type Extension Mask
#define NVIC_MPU_RASR3_SHAREABLE \
                                0x00040000  // Shareable
#define NVIC_MPU_RASR3_CACHEABLE \
                                0x00020000  // Cacheable
#define NVIC_MPU_RASR3_BUFFRABLE \
                                0x00010000  // Bufferable
#define NVIC_MPU_RASR3_SRD_M    0x0000FF00  // Subregion Disable Bits
#define NVIC_MPU_RASR3_SIZE_M   0x0000003E  // Region Size Mask
#define NVIC_MPU_RASR3_ENABLE   0x00000001  // Region Enable

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_DBG_CTRL NVIC_DBG_CTRL
//! \brief Defines for the bit fields in the NVIC_DBG_CTRL register.
//! @{
//
//*****************************************************************************

#define NVIC_DBG_CTRL_DBGKEY_M  0xFFFF0000  // Debug key mask
#define NVIC_DBG_CTRL_DBGKEY    0xA05F0000  // Debug key
#define NVIC_DBG_CTRL_S_RESET_ST \
                                0x02000000  // Core has reset since last read
#define NVIC_DBG_CTRL_S_RETIRE_ST \
                                0x01000000  // Core has executed insruction
                                            // since last read
#define NVIC_DBG_CTRL_S_LOCKUP  0x00080000  // Core is locked up
#define NVIC_DBG_CTRL_S_SLEEP   0x00040000  // Core is sleeping
#define NVIC_DBG_CTRL_S_HALT    0x00020000  // Core status on halt
#define NVIC_DBG_CTRL_S_REGRDY  0x00010000  // Register read/write available
#define NVIC_DBG_CTRL_C_SNAPSTALL \
                                0x00000020  // Breaks a stalled load/store
#define NVIC_DBG_CTRL_C_MASKINT 0x00000008  // Mask interrupts when stepping
#define NVIC_DBG_CTRL_C_STEP    0x00000004  // Step the core
#define NVIC_DBG_CTRL_C_HALT    0x00000002  // Halt the core
#define NVIC_DBG_CTRL_C_DEBUGEN 0x00000001  // Enable debug

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_DBG_XFER NVIC_DBG_XFER
//! \brief Defines for the bit fields in the NVIC_DBG_XFER register.
//! @{
//
//*****************************************************************************

#define NVIC_DBG_XFER_REG_WNR   0x00010000  // Write or not read
#define NVIC_DBG_XFER_REG_SEL_M 0x0000001F  // Register
#define NVIC_DBG_XFER_REG_R0    0x00000000  // Register R0
#define NVIC_DBG_XFER_REG_R1    0x00000001  // Register R1
#define NVIC_DBG_XFER_REG_R2    0x00000002  // Register R2
#define NVIC_DBG_XFER_REG_R3    0x00000003  // Register R3
#define NVIC_DBG_XFER_REG_R4    0x00000004  // Register R4
#define NVIC_DBG_XFER_REG_R5    0x00000005  // Register R5
#define NVIC_DBG_XFER_REG_R6    0x00000006  // Register R6
#define NVIC_DBG_XFER_REG_R7    0x00000007  // Register R7
#define NVIC_DBG_XFER_REG_R8    0x00000008  // Register R8
#define NVIC_DBG_XFER_REG_R9    0x00000009  // Register R9
#define NVIC_DBG_XFER_REG_R10   0x0000000A  // Register R10
#define NVIC_DBG_XFER_REG_R11   0x0000000B  // Register R11
#define NVIC_DBG_XFER_REG_R12   0x0000000C  // Register R12
#define NVIC_DBG_XFER_REG_R13   0x0000000D  // Register R13
#define NVIC_DBG_XFER_REG_R14   0x0000000E  // Register R14
#define NVIC_DBG_XFER_REG_R15   0x0000000F  // Register R15
#define NVIC_DBG_XFER_REG_FLAGS 0x00000010  // xPSR/Flags register
#define NVIC_DBG_XFER_REG_MSP   0x00000011  // Main SP
#define NVIC_DBG_XFER_REG_PSP   0x00000012  // Process SP
#define NVIC_DBG_XFER_REG_DSP   0x00000013  // Deep SP
#define NVIC_DBG_XFER_REG_CFBP  0x00000014  // Control/Fault/BasePri/PriMask

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_DBG_DATA NVIC_DBG_DATA
//! \brief Defines for the bit fields in the NVIC_DBG_DATA register.
//! @{
//
//*****************************************************************************

#define NVIC_DBG_DATA_M         0xFFFFFFFF  // Data temporary cache
#define NVIC_DBG_DATA_S         0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_DBG_INT NVIC_DBG_INT
//! \brief Defines for the bit fields in the NVIC_DBG_INT register.
//! @{
//
//*****************************************************************************

#define NVIC_DBG_INT_HARDERR    0x00000400  // Debug trap on hard fault
#define NVIC_DBG_INT_INTERR     0x00000200  // Debug trap on interrupt errors
#define NVIC_DBG_INT_BUSERR     0x00000100  // Debug trap on bus error
#define NVIC_DBG_INT_STATERR    0x00000080  // Debug trap on usage fault state
#define NVIC_DBG_INT_CHKERR     0x00000040  // Debug trap on usage fault check
#define NVIC_DBG_INT_NOCPERR    0x00000020  // Debug trap on coprocessor error
#define NVIC_DBG_INT_MMERR      0x00000010  // Debug trap on mem manage fault
#define NVIC_DBG_INT_RESET      0x00000008  // Core reset status
#define NVIC_DBG_INT_RSTPENDCLR 0x00000004  // Clear pending core reset
#define NVIC_DBG_INT_RSTPENDING 0x00000002  // Core reset is pending
#define NVIC_DBG_INT_RSTVCATCH  0x00000001  // Reset vector catch

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup CORE_NVIC_Register_NVIC_STIR NVIC Software Trigger Interrupt Register
//! \brief Defines for the bit fields in the NVIC_STIR register.
//! @{
//
//*****************************************************************************

#define NVIC_STIR_INTID_M    0x000001FF  // Interrupt ID
#define NVIC_STIR_INTID_S    0

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

#endif // __XHW_NVIC_H__

