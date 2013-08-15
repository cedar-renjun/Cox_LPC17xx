//*****************************************************************************
//
//! \file xgpio.h
//! \brief Prototypes for the GPIO Driver.
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

#ifndef __xGPIO_H__
#define __xGPIO_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplu
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
//! \addtogroup GPIO
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xGPIO
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xGPIO_General_Pin_IDs xGPIO General Pin ID
//! 
//! \section xGPIO_General_Pin_IDs 1. Where to use this group
//! The following values define the bit field for the ulPins argument to several
//! of the APIs. So all the API which have a ulPins argument must use this group.
//! 
//! \section xGPIO_General_Pin_IDs_CoX 2.CoX Port Details 
//! \verbatim
//! +--------------------------+----------------+------------------------+
//! |  xGPIO General Pin ID    |       CoX      |         STM32F1xx      |
//! |--------------------------|----------------|------------------------|
//! |  xGPIO_PIN_n             |    Mandatory   |       xGPIO_PIN_0      |
//! |                          |                |------------------------|
//! |                          |                |       xGPIO_PIN_1      |
//! |                          |                |------------------------|
//! |                          |                |         ...            |
//! |                          |                |------------------------|
//! |                          |                |      xGPIO_PIN_30      |
//! |                          |                |------------------------|
//! |                          |                |      xGPIO_PIN_31      |
//! +--------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//! GPIO Pin 0
#define xGPIO_PIN_0             GPIO_PIN_0 

//! GPIO Pin 1
#define xGPIO_PIN_1             GPIO_PIN_1 

//! GPIO Pin 2
#define xGPIO_PIN_2             GPIO_PIN_2 

//! GPIO Pin 3
#define xGPIO_PIN_3             GPIO_PIN_3 

//! GPIO Pin 4
#define xGPIO_PIN_4             GPIO_PIN_4 

//! GPIO Pin 5
#define xGPIO_PIN_5             GPIO_PIN_5 

//! GPIO Pin 6
#define xGPIO_PIN_6             GPIO_PIN_6 

//! GPIO Pin 7
#define xGPIO_PIN_7             GPIO_PIN_7 

//! GPIO Pin 8
#define xGPIO_PIN_8             GPIO_PIN_8 

//! GPIO Pin 9
#define xGPIO_PIN_9             GPIO_PIN_9 

//! GPIO Pin 10
#define xGPIO_PIN_10            GPIO_PIN_10

//! GPIO Pin 11
#define xGPIO_PIN_11            GPIO_PIN_11

//! GPIO Pin 12
#define xGPIO_PIN_12            GPIO_PIN_12

//! GPIO Pin 13
#define xGPIO_PIN_13            GPIO_PIN_13

//! GPIO Pin 14
#define xGPIO_PIN_14            GPIO_PIN_14

//! GPIO Pin 15
#define xGPIO_PIN_15            GPIO_PIN_15

//! GPIO Pin 16
#define xGPIO_PIN_16            GPIO_PIN_16

//! GPIO Pin 17
#define xGPIO_PIN_17            GPIO_PIN_17

//! GPIO Pin 18
#define xGPIO_PIN_18            GPIO_PIN_18

//! GPIO Pin 19
#define xGPIO_PIN_19            GPIO_PIN_19

//! GPIO Pin 20
#define xGPIO_PIN_20            GPIO_PIN_20

//! GPIO Pin 21
#define xGPIO_PIN_21            GPIO_PIN_21

//! GPIO Pin 22
#define xGPIO_PIN_22            GPIO_PIN_22

//! GPIO Pin 23
#define xGPIO_PIN_23            GPIO_PIN_23

//! GPIO Pin 24
#define xGPIO_PIN_24            GPIO_PIN_24

//! GPIO Pin 25
#define xGPIO_PIN_25            GPIO_PIN_25

//! GPIO Pin 26
#define xGPIO_PIN_26            GPIO_PIN_26

//! GPIO Pin 27
#define xGPIO_PIN_27            GPIO_PIN_27

//! GPIO Pin 28
#define xGPIO_PIN_28            GPIO_PIN_28

//! GPIO Pin 29
#define xGPIO_PIN_29            GPIO_PIN_29

//! GPIO Pin 30
#define xGPIO_PIN_30            GPIO_PIN_30

//! GPIO Pin 31
#define xGPIO_PIN_31            GPIO_PIN_31

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xGPIO_Dir_Mode xGPIO Dir Mode
//! 
//! \section xGPIO_Dir_Mode_S1 1. Where to use this group
//! Values that can be passed to xGPIODirModeSet as the ulPinIO parameter, and
//! returned from xGPIODirModeGet.
//! 
//! \section xGPIO_Dir_Mode_CoX 2.CoX Port Details 
//! \verbatim
//! +------------------------+----------------+------------------------+
//! |xGPIO Dir Mode          |       CoX      |         STM32F1xx      |
//! |------------------------|----------------|------------------------|
//! |xGPIO_DIR_MODE_IN       |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xGPIO_DIR_MODE_OUT      |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xGPIO_DIR_MODE_HW       |    Mandatory   |            N           |
//! |------------------------|----------------|------------------------|
//! |xGPIO_DIR_MODE_QB       |  Non-Mandatory |            N           |
//! |------------------------|----------------|------------------------|
//! |xGPIO_DIR_MODE_OD       |  Non-Mandatory |            Y           |
//! +------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Pin is a GPIO input
//
#define xGPIO_DIR_MODE_IN       PIN_MODE_INPUT

//
//! Pin is a GPIO output
//
#define xGPIO_DIR_MODE_OUT      PIN_MODE_OUTPUT

//
//! Pin is a peripheral function
//
#define xGPIO_DIR_MODE_HW       0

//
//! Pin is in Quasi-bidirectional mode
//
#define xGPIO_DIR_MODE_QB       0  

//
//! Pin is in Open-Drain mode.
//
#define xGPIO_DIR_MODE_OD       PIN_MODE_OD_EN

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xGPIO_Int_Type xGPIO Int Type
//! 
//! \section xGPIO_Int_Type_S1 1. Where to use this group
//! Values that can be passed to xGPIOIntTypeSet as the ulIntType parameter, and
//! returned from xGPIOIntTypeGet.
//! 
//! \section xGPIO_Int_Type_CoX 2.CoX Port Details 
//! \verbatim
//! +------------------------+----------------+------------------------+
//! |xGPIO Int Type          |       CoX      |         STM32F1xx      |
//! |------------------------|----------------|------------------------|
//! |xGPIO_FALLING_EDGE      |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xGPIO_RISING_EDGE       |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xGPIO_LOW_LEVEL         |    Mandatory   |            N           |
//! |------------------------|----------------|------------------------|
//! |xGPIO_HIGH_LEVEL        |    Mandatory   |            N           |
//! |------------------------|----------------|------------------------|
//! |xGPIO_BOTH_LEVEL        |  Non-Mandatory |            N           |
//! |------------------------|----------------|------------------------|
//! |xGPIO_BOTH_EDGES        |  Non-Mandatory |            Y           |
//! |------------------------|----------------|------------------------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Interrupt on falling edge
//
#define xGPIO_FALLING_EDGE      INT_TYPE_FALLING         

//
//! Interrupt on rising edge
//
#define xGPIO_RISING_EDGE       INT_TYPE_RISING

//
//! Interrupt on low level
//
#define xGPIO_LOW_LEVEL         0  

//
//! Interrupt on high level
//
#define xGPIO_HIGH_LEVEL        0 

//
//! Interrupt on both edge
//
#define xGPIO_BOTH_EDGES        (INT_TYPE_RISING | INT_TYPE_FALLING)

//
//! Interrupt on both level
//
#define xGPIO_BOTH_LEVEL        0
//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xGPIO_Pad_Config_Strength xGPIO Pad Config Strength
//! 
//! \section xGPIO_Pad_Config_Strength_S1 1. Where to use this group
//! Values that can be passed to xGPIOPadConfigSet as the ulStrength parameter,
//! and returned by xGPIOPadConfigGet in the *pulStrength parameter.
//! 
//! \section xGPIO_Pad_Config_Strength_CoX 2.CoX Port Details 
//! \verbatim
//! +------------------------+----------------+------------------------+
//! |xGPIO Pad Strength      |       CoX      |         STM32F1xx      |
//! |------------------------|----------------|------------------------|
//! |xGPIO_STRENGTH_nMA      | Non-Mandatory  |            N           |
//! +------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xGPIO_GP_Short_Pin xGPIO General Purpose Short Pin
//! @{
//
//*****************************************************************************
#define GPA0                    GPIOA_BASE, GPIO_PIN_0 
#define GPA1                    GPIOA_BASE, GPIO_PIN_1 
#define GPA2                    GPIOA_BASE, GPIO_PIN_2 
#define GPA3                    GPIOA_BASE, GPIO_PIN_3 
#define GPA4                    GPIOA_BASE, GPIO_PIN_4 
#define GPA5                    GPIOA_BASE, GPIO_PIN_5 
#define GPA6                    GPIOA_BASE, GPIO_PIN_6 
#define GPA7                    GPIOA_BASE, GPIO_PIN_7 
#define GPA8                    GPIOA_BASE, GPIO_PIN_8 
#define GPA9                    GPIOA_BASE, GPIO_PIN_9 
#define GPA10                   GPIOA_BASE, GPIO_PIN_10 
#define GPA11                   GPIOA_BASE, GPIO_PIN_11 
#define GPA12                   GPIOA_BASE, GPIO_PIN_12 
#define GPA13                   GPIOA_BASE, GPIO_PIN_13 
#define GPA14                   GPIOA_BASE, GPIO_PIN_14 
#define GPA15                   GPIOA_BASE, GPIO_PIN_15 
#define GPA16                   GPIOA_BASE, GPIO_PIN_16
#define GPA17                   GPIOA_BASE, GPIO_PIN_17
#define GPA18                   GPIOA_BASE, GPIO_PIN_18
#define GPA19                   GPIOA_BASE, GPIO_PIN_19
#define GPA20                   GPIOA_BASE, GPIO_PIN_20
#define GPA21                   GPIOA_BASE, GPIO_PIN_21
#define GPA22                   GPIOA_BASE, GPIO_PIN_22
#define GPA23                   GPIOA_BASE, GPIO_PIN_23
#define GPA24                   GPIOA_BASE, GPIO_PIN_24
#define GPA25                   GPIOA_BASE, GPIO_PIN_25
#define GPA26                   GPIOA_BASE, GPIO_PIN_26 
#define GPA27                   GPIOA_BASE, GPIO_PIN_27 
#define GPA28                   GPIOA_BASE, GPIO_PIN_28 
#define GPA29                   GPIOA_BASE, GPIO_PIN_29 
#define GPA30                   GPIOA_BASE, GPIO_PIN_30 
#define GPA31                   GPIOA_BASE, GPIO_PIN_31 

#define GPB0                    GPIOB_BASE, GPIO_PIN_0 
#define GPB1                    GPIOB_BASE, GPIO_PIN_1 
#define GPB2                    GPIOB_BASE, GPIO_PIN_2 
#define GPB3                    GPIOB_BASE, GPIO_PIN_3 
#define GPB4                    GPIOB_BASE, GPIO_PIN_4 
#define GPB5                    GPIOB_BASE, GPIO_PIN_5 
#define GPB6                    GPIOB_BASE, GPIO_PIN_6 
#define GPB7                    GPIOB_BASE, GPIO_PIN_7 
#define GPB8                    GPIOB_BASE, GPIO_PIN_8 
#define GPB9                    GPIOB_BASE, GPIO_PIN_9 
#define GPB10                   GPIOB_BASE, GPIO_PIN_10 
#define GPB11                   GPIOB_BASE, GPIO_PIN_11 
#define GPB12                   GPIOB_BASE, GPIO_PIN_12 
#define GPB13                   GPIOB_BASE, GPIO_PIN_13 
#define GPB14                   GPIOB_BASE, GPIO_PIN_14 
#define GPB15                   GPIOB_BASE, GPIO_PIN_15 
#define GPB16                   GPIOB_BASE, GPIO_PIN_16
#define GPB17                   GPIOB_BASE, GPIO_PIN_17
#define GPB18                   GPIOB_BASE, GPIO_PIN_18
#define GPB19                   GPIOB_BASE, GPIO_PIN_19
#define GPB20                   GPIOB_BASE, GPIO_PIN_20
#define GPB21                   GPIOB_BASE, GPIO_PIN_21
#define GPB22                   GPIOB_BASE, GPIO_PIN_22
#define GPB23                   GPIOB_BASE, GPIO_PIN_23
#define GPB24                   GPIOB_BASE, GPIO_PIN_24
#define GPB25                   GPIOB_BASE, GPIO_PIN_25
#define GPB26                   GPIOB_BASE, GPIO_PIN_26 
#define GPB27                   GPIOB_BASE, GPIO_PIN_27 
#define GPB28                   GPIOB_BASE, GPIO_PIN_28 
#define GPB29                   GPIOB_BASE, GPIO_PIN_29 
#define GPB30                   GPIOB_BASE, GPIO_PIN_30 
#define GPB31                   GPIOB_BASE, GPIO_PIN_31 

#define GPC0                    GPIOC_BASE, GPIO_PIN_0 
#define GPC1                    GPIOC_BASE, GPIO_PIN_1 
#define GPC2                    GPIOC_BASE, GPIO_PIN_2 
#define GPC3                    GPIOC_BASE, GPIO_PIN_3 
#define GPC4                    GPIOC_BASE, GPIO_PIN_4 
#define GPC5                    GPIOC_BASE, GPIO_PIN_5 
#define GPC6                    GPIOC_BASE, GPIO_PIN_6 
#define GPC7                    GPIOC_BASE, GPIO_PIN_7 
#define GPC8                    GPIOC_BASE, GPIO_PIN_8 
#define GPC9                    GPIOC_BASE, GPIO_PIN_9 
#define GPC10                   GPIOC_BASE, GPIO_PIN_10 
#define GPC11                   GPIOC_BASE, GPIO_PIN_11 
#define GPC12                   GPIOC_BASE, GPIO_PIN_12 
#define GPC13                   GPIOC_BASE, GPIO_PIN_13 
#define GPC14                   GPIOC_BASE, GPIO_PIN_14 
#define GPC15                   GPIOC_BASE, GPIO_PIN_15 
#define GPC16                   GPIOC_BASE, GPIO_PIN_16
#define GPC17                   GPIOC_BASE, GPIO_PIN_17
#define GPC18                   GPIOC_BASE, GPIO_PIN_18
#define GPC19                   GPIOC_BASE, GPIO_PIN_19
#define GPC20                   GPIOC_BASE, GPIO_PIN_20
#define GPC21                   GPIOC_BASE, GPIO_PIN_21
#define GPC22                   GPIOC_BASE, GPIO_PIN_22
#define GPC23                   GPIOC_BASE, GPIO_PIN_23
#define GPC24                   GPIOC_BASE, GPIO_PIN_24
#define GPC25                   GPIOC_BASE, GPIO_PIN_25
#define GPC26                   GPIOC_BASE, GPIO_PIN_26 
#define GPC27                   GPIOC_BASE, GPIO_PIN_27 
#define GPC28                   GPIOC_BASE, GPIO_PIN_28 
#define GPC29                   GPIOC_BASE, GPIO_PIN_29 
#define GPC30                   GPIOC_BASE, GPIO_PIN_30 
#define GPC31                   GPIOC_BASE, GPIO_PIN_31 

#define GPD0                    GPIOD_BASE, GPIO_PIN_0 
#define GPD1                    GPIOD_BASE, GPIO_PIN_1 
#define GPD2                    GPIOD_BASE, GPIO_PIN_2 
#define GPD3                    GPIOD_BASE, GPIO_PIN_3 
#define GPD4                    GPIOD_BASE, GPIO_PIN_4 
#define GPD5                    GPIOD_BASE, GPIO_PIN_5 
#define GPD6                    GPIOD_BASE, GPIO_PIN_6 
#define GPD7                    GPIOD_BASE, GPIO_PIN_7 
#define GPD8                    GPIOD_BASE, GPIO_PIN_8 
#define GPD9                    GPIOD_BASE, GPIO_PIN_9 
#define GPD10                   GPIOD_BASE, GPIO_PIN_10 
#define GPD11                   GPIOD_BASE, GPIO_PIN_11 
#define GPD12                   GPIOD_BASE, GPIO_PIN_12 
#define GPD13                   GPIOD_BASE, GPIO_PIN_13 
#define GPD14                   GPIOD_BASE, GPIO_PIN_14 
#define GPD15                   GPIOD_BASE, GPIO_PIN_15 
#define GPD16                   GPIOD_BASE, GPIO_PIN_16
#define GPD17                   GPIOD_BASE, GPIO_PIN_17
#define GPD18                   GPIOD_BASE, GPIO_PIN_18
#define GPD19                   GPIOD_BASE, GPIO_PIN_19
#define GPD20                   GPIOD_BASE, GPIO_PIN_20
#define GPD21                   GPIOD_BASE, GPIO_PIN_21
#define GPD22                   GPIOD_BASE, GPIO_PIN_22
#define GPD23                   GPIOD_BASE, GPIO_PIN_23
#define GPD24                   GPIOD_BASE, GPIO_PIN_24
#define GPD25                   GPIOD_BASE, GPIO_PIN_25
#define GPD26                   GPIOD_BASE, GPIO_PIN_26 
#define GPD27                   GPIOD_BASE, GPIO_PIN_27 
#define GPD28                   GPIOD_BASE, GPIO_PIN_28 
#define GPD29                   GPIOD_BASE, GPIO_PIN_29 
#define GPD30                   GPIOD_BASE, GPIO_PIN_30 
#define GPD31                   GPIOD_BASE, GPIO_PIN_31 

#define GPE0                    GPIOE_BASE, GPIO_PIN_0 
#define GPE1                    GPIOE_BASE, GPIO_PIN_1 
#define GPE2                    GPIOE_BASE, GPIO_PIN_2 
#define GPE3                    GPIOE_BASE, GPIO_PIN_3 
#define GPE4                    GPIOE_BASE, GPIO_PIN_4 
#define GPE5                    GPIOE_BASE, GPIO_PIN_5 
#define GPE6                    GPIOE_BASE, GPIO_PIN_6 
#define GPE7                    GPIOE_BASE, GPIO_PIN_7 
#define GPE8                    GPIOE_BASE, GPIO_PIN_8 
#define GPE9                    GPIOE_BASE, GPIO_PIN_9 
#define GPE10                   GPIOE_BASE, GPIO_PIN_10 
#define GPE11                   GPIOE_BASE, GPIO_PIN_11 
#define GPE12                   GPIOE_BASE, GPIO_PIN_12 
#define GPE13                   GPIOE_BASE, GPIO_PIN_13 
#define GPE14                   GPIOE_BASE, GPIO_PIN_14 
#define GPE15                   GPIOE_BASE, GPIO_PIN_15 
#define GPE16                   GPIOE_BASE, GPIO_PIN_16
#define GPE17                   GPIOE_BASE, GPIO_PIN_17
#define GPE18                   GPIOE_BASE, GPIO_PIN_18
#define GPE19                   GPIOE_BASE, GPIO_PIN_19
#define GPE20                   GPIOE_BASE, GPIO_PIN_20
#define GPE21                   GPIOE_BASE, GPIO_PIN_21
#define GPE22                   GPIOE_BASE, GPIO_PIN_22
#define GPE23                   GPIOE_BASE, GPIO_PIN_23
#define GPE24                   GPIOE_BASE, GPIO_PIN_24
#define GPE25                   GPIOE_BASE, GPIO_PIN_25
#define GPE26                   GPIOE_BASE, GPIO_PIN_26 
#define GPE27                   GPIOE_BASE, GPIO_PIN_27 
#define GPE28                   GPIOE_BASE, GPIO_PIN_28 
#define GPE29                   GPIOE_BASE, GPIO_PIN_29 
#define GPE30                   GPIOE_BASE, GPIO_PIN_30 
#define GPE31                   GPIOE_BASE, GPIO_PIN_31 
//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xGPIO_Short_Pin xGPIO Short Pin ID
//! 
//! \section xGPIO_Short_Pin_S1 1. Where to use this group
//! The following values define the short pin argument(dShortPin) to several
//! of the \b XPinTypexxx APIs and all the API which have a eShortPin argument.
//! Such as \ref xGPIOSPinRead(),\ref xGPIOSPinWrite().
//! 
//! \section xGPIO_Short_Pin_CoX 2.CoX Port Details 
//! \verbatim
//! +------------------------+----------------+------------------------+
//! |xGPIO Short Pin ID      |       CoX      |         STM32F1xx      |
//! |------------------------|----------------|------------------------|
//! |PXn                     |    Mandatory   |    PA0 PA1 ... PA31    |
//! | (X = A/B/...)          |                |------------------------|
//! | (n = 0/1/...)          |                |    PB0 PB1 ... PB31    |
//! |                        |                |------------------------|
//! |                        |                |     ...                |
//! |                        |                |------------------------|
//! |                        |                |    PE0 PE1 ... PE31    |  
//! +------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

#define PA0                      PA0 
#define PA1                      PA1 
#define PA2                      PA2 
#define PA3                      PA3 
#define PA4                      PA4 
#define PA5                      PA5 
#define PA6                      PA6 
#define PA7                      PA7 
#define PA8                      PA8 
#define PA9                      PA9 
#define PA10                     PA10
#define PA11                     PA11
#define PA12                     PA12
#define PA13                     PA13
#define PA14                     PA14
#define PA15                     PA15
#define PA16                     PA16
#define PA17                     PA17
#define PA18                     PA18
#define PA19                     PA19
#define PA20                     PA20
#define PA21                     PA21
#define PA22                     PA22
#define PA23                     PA23
#define PA24                     PA24
#define PA25                     PA25
#define PA26                     PA26
#define PA27                     PA27
#define PA28                     PA28
#define PA29                     PA29
#define PA30                     PA30
#define PA31                     PA31

#define PB0                      PB0 
#define PB1                      PB1 
#define PB2                      PB2 
#define PB3                      PB3 
#define PB4                      PB4 
#define PB5                      PB5 
#define PB6                      PB6 
#define PB7                      PB7 
#define PB8                      PB8 
#define PB9                      PB9 
#define PB10                     PB10
#define PB11                     PB11
#define PB12                     PB12
#define PB13                     PB13
#define PB14                     PB14
#define PB15                     PB15
#define PB16                     PB16
#define PB17                     PB17
#define PB18                     PB18
#define PB19                     PB19
#define PB20                     PB20
#define PB21                     PB21
#define PB22                     PB22
#define PB23                     PB23
#define PB24                     PB24
#define PB25                     PB25
#define PB26                     PB26
#define PB27                     PB27
#define PB28                     PB28
#define PB29                     PB29
#define PB30                     PB30
#define PB31                     PB31

#define PC0                      PC0 
#define PC1                      PC1 
#define PC2                      PC2 
#define PC3                      PC3 
#define PC4                      PC4 
#define PC5                      PC5 
#define PC6                      PC6 
#define PC7                      PC7 
#define PC8                      PC8 
#define PC9                      PC9 
#define PC10                     PC10
#define PC11                     PC11
#define PC12                     PC12
#define PC13                     PC13
#define PC14                     PC14
#define PC15                     PC15
#define PC16                     PC16
#define PC17                     PC17
#define PC18                     PC18
#define PC19                     PC19
#define PC20                     PC20
#define PC21                     PC21
#define PC22                     PC22
#define PC23                     PC23
#define PC24                     PC24
#define PC25                     PC25
#define PC26                     PC26
#define PC27                     PC27
#define PC28                     PC28
#define PC29                     PC29
#define PC30                     PC30
#define PC31                     PC31

#define PD0                      PD0 
#define PD1                      PD1 
#define PD2                      PD2 
#define PD3                      PD3 
#define PD4                      PD4 
#define PD5                      PD5 
#define PD6                      PD6 
#define PD7                      PD7 
#define PD8                      PD8 
#define PD9                      PD9 
#define PD10                     PD10
#define PD11                     PD11
#define PD12                     PD12
#define PD13                     PD13
#define PD14                     PD14
#define PD15                     PD15
#define PD16                     PD16
#define PD17                     PD17
#define PD18                     PD18
#define PD19                     PD19
#define PD20                     PD20
#define PD21                     PD21
#define PD22                     PD22
#define PD23                     PD23
#define PD24                     PD24
#define PD25                     PD25
#define PD26                     PD26
#define PD27                     PD27
#define PD28                     PD28
#define PD29                     PD29
#define PD30                     PD30
#define PD31                     PD31

#define PE0                      PE0 
#define PE1                      PE1 
#define PE2                      PE2 
#define PE3                      PE3 
#define PE4                      PE4 
#define PE5                      PE5 
#define PE6                      PE6 
#define PE7                      PE7 
#define PE8                      PE8 
#define PE9                      PE9 
#define PE10                     PE10
#define PE11                     PE11
#define PE12                     PE12
#define PE13                     PE13
#define PE14                     PE14
#define PE15                     PE15
#define PE16                     PE16
#define PE17                     PE17
#define PE18                     PE18
#define PE19                     PE19
#define PE20                     PE20
#define PE21                     PE21
#define PE22                     PE22
#define PE23                     PE23
#define PE24                     PE24
#define PE25                     PE25
#define PE26                     PE26
#define PE27                     PE27
#define PE28                     PE28
#define PE29                     PE29
#define PE30                     PE30
#define PE31                     PE31

//*****************************************************************************
//
//! @}
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup xGPIO_Peripheral_Pin xGPIO General Peripheral Pin
//! \brief General Peripheral Pin Name.
//!
//! \section xGPIO_Peripheral_Pin_define xGPIO Peripheral Pin define?
//! The macros of General Peripheral Pin Name always like:
//! <b> ModuleName + n + PinName </b>, such as CAN0RX, SPI1CLK.
//!
//! \section xGPIO_Peripheral_Pin_Port CoX Port Detail
//! \verbatim
//! +-------------------------+----------------+-------------------------+
//! | General Peripheral Pin  |       CoX      |         STM32F1xx       |
//! |-------------------------|----------------|-------------------------|
//! | ADCn                    |    Mandatory   |   ADC0 ADC1 ... ADC15   |
//! |-------------------------|----------------|-------------------------|
//! | CANnRX                  |    Mandatory   |      CAN0RX CAN1RX      |
//! |-------------------------|----------------|-------------------------|
//! | CANnTX                  |    Mandatory   |      CAN0TX CAN1TX      |
//! |-------------------------|----------------|-------------------------|
//! | I2CnSCK                 |    Mandatory   | I2C0SCK I2C1SCK I2C2SCK |
//! |-------------------------|----------------|-------------------------|
//! | I2CnSDA                 |    Mandatory   | I2C0SDA I2C1SDA I2C2SDA |
//! |-------------------------|----------------|-------------------------|
//! | I2SnRXSCK               |    Mandatory   |        I2S0RXSCK        |
//! |-------------------------|----------------|-------------------------|
//! | I2SnRXMCLK              |    Mandatory   |       I2S0RXMCLK        |
//! |-------------------------|----------------|-------------------------|
//! | I2SnRXSD                |    Mandatory   |        I2S0RXSD         |
//! |-------------------------|----------------|-------------------------|
//! | I2SnRXWS                |    Mandatory   |        I2S0RXWS         |
//! |-------------------------|----------------|-------------------------|
//! | I2SnTXSCK               |    Mandatory   |        I2S0TXSCK        |
//! |-------------------------|----------------|-------------------------|
//! | I2SnTXMCLK              |    Mandatory   |       I2S0TXMCLK        |
//! |-------------------------|----------------|-------------------------|
//! | I2SnTXSD                |    Mandatory   |        I2S0TXSD         |
//! |-------------------------|----------------|-------------------------|
//! | I2SnTXWS                |    Mandatory   |        I2S0TXWS         |
//! |-------------------------|----------------|-------------------------|
//! | SPInCLK                 |    Mandatory   |     SPI0CLK SPI1CLK     |
//! |                         |                |------------------------ |
//! |                         |                |     SPI2CLK SPI3CLK     |
//! |-------------------------|----------------|-------------------------|
//! | SPInMOSI                |    Mandatory   |    SPI0MOSI SPI1MOSI    |
//! |                         |                |------------------------ |
//! |                         |                |    SPI2MOSI SPI3MOSI    |
//! |-------------------------|----------------|-------------------------|
//! | SPInMISO                |    Mandatory   |    SPI0MISO SPI1MISO    |
//! |                         |                |------------------------ |
//! |                         |                |    SPI2MISO SPI3MISO    |
//! |-------------------------|----------------|-------------------------|
//! | SPInCS                  |    Mandatory   |      SPI0CS SPI1CS      |
//! |                         |                |------------------------ |
//! |                         |                |      SPI2CS SPI3CS      |
//! |-------------------------|----------------|-------------------------|
//! | UARTnRX                 |    Mandatory   |     UART0RX UART1RX     |
//! |                         |                |------------------------ |
//! |                         |                |     UART2RX             |
//! |-------------------------|----------------|-------------------------|
//! | UARTnTX                 |    Mandatory   |     UART0TX UART1TX     |
//! |                         |                |------------------------ |
//! |                         |                |     UART2TX             |
//! |-------------------------|----------------|-------------------------|
//! | UARTnCTS                |    Mandatory   |    UART0CTS UART1CTS    |
//! |                         |                |------------------------ |
//! |                         |                |    UART2CTS             |
//! |-------------------------|----------------|-------------------------|
//! | UARTnDCD                |    Mandatory   |    UART0DCD UART1DCD    |
//! |                         |                |------------------------ |
//! |                         |                |    UART2DCD             |
//! |-------------------------|----------------|-------------------------|
//! | UARTnDSR                |    Mandatory   |    UART0DSR UART1DSR    |
//! |                         |                |------------------------ |
//! |                         |                |    UART2DSR             |
//! |-------------------------|----------------|-------------------------|
//! | UARTnDTR                |    Mandatory   |    UART0DTR UART1DTR    |
//! |                         |                |------------------------ |
//! |                         |                |    UART2DTR             |
//! |-------------------------|----------------|-------------------------|
//! | CMPnN                   |    Mandatory   |       CMP0N CMP1N       |
//! |-------------------------|----------------|-------------------------|
//! | CMPnP                   |    Mandatory   |       CMP0P CMP1P       |
//! |-------------------------|----------------|-------------------------|
//! | CMPnO                   |    Mandatory   |       CMP0O CMP1O       |
//! |-------------------------|----------------|-------------------------|
//! | PWMn                    |    Mandatory   |      PWM0 ... PWM7      |
//! |-------------------------|----------------|-------------------------|
//! | TIMCCPn                 |    Mandatory   |   TIMCCP0 ... TIMCCP7   |
//! +-------------------------+----------------+-------------------------+
//! \endverbatim
//!
//!  
//! @{
//
//*****************************************************************************

#define PA0                     PA0         
#define RD1                     RD1         
#define RXD3                    RXD3        
#define I2C1SDA                 I2C1SDA         
#define PA1                     PA1         
#define TD1                     TD1         
#define RXD3                    RXD3        
#define I2C1SCL                 I2C1SCL         
#define PA2                     PA2         
#define TXD0                    TXD0        
#define AD_CH_7                 AD_CH_7
#define PA3                     PA3         
#define RXD0                    RXD0        
#define AD_CH_6                 AD_CH_6
#define PA4                     PA4         
#define I2SRX_CLK               I2SRX_CLK   
#define RD2                     RD2         
#define CAP2_0                  CAP2_0      
#define PA5                     PA5         
#define I2SRX_WS                I2SRX_WS    
#define TD2                     TD2         
#define CAP2_1                  CAP2_1      
#define PA6                     PA6         
#define I2SRX_SDA               I2SRX_SDA   
#define SSEL1                   SSEL1       
#define MAT2_0                  MAT2_0      
#define PA7                     PA7         
#define I2STX_CLK               I2STX_CLK   
#define SCK1                    SCK1        
#define MAT2_1                  MAT2_1      
#define PA8                     PA8         
#define I2STX_WS                I2STX_WS    
#define MISO1                   MISO1       
#define MAT2_2                  MAT2_2      
#define PA9                     PA9         
#define I2STX_SDA               I2STX_SDA   
#define MOSI1                   MOSI1       
#define MAT2_3                  MAT2_3      
#define PA10                    PA10        
#define TXD2                    TXD2        
#define I2C2SDA                 I2C2SDA         
#define MAT3_0                  MAT3_0      
#define PA11                    PA11        
#define RXD2                    RXD2        
#define I2C2SCL                 I2C2SCL         
#define MAT3_1                  MAT3_1      
#define PA15                    PA15        
#define TXD1                    TXD1        
#define SCK0                    SCK0        
#define SCK                     SCK         
#define PA16                    PA16        
#define RXD1                    RXD1        
#define SSEL0                   SSEL0       
#define SSEL                    SSEL        
#define PA17                    PA17        
#define CTS1                    CTS1        
#define MISO0                   MISO0       
#define MISO                    MISO        
#define PA18                    PA18        
#define DCD1                    DCD1        
#define MOSI0                   MOSI0       
#define MOSI                    MOSI        
#define PA19                    PA19        
#define DSR1                    DSR1        
#define PA20                    PA20        
#define DTR1                    DTR1        
#define PA21                    PA21        
#define RI1                     RI1         
#define RD1                     RD1         
#define PA22                    PA22        
#define RTS1                    RTS1        
#define TD1                     TD1         
#define PA23                    PA23        
#define AD_CH_0                 AD_CH_0
#define I2SRX_CLK               I2SRX_CLK   
#define CAP3_0                  CAP3_0      
#define PA24                    PA24        
#define AD_CH_1                 AD_CH_1
#define I2SRX_WS                I2SRX_WS    
#define CAP3_1                  CAP3_1      
#define PA25                    PA25        
#define AD_CH_2                 AD_CH_2
#define I2SRX_SDA               I2SRX_SDA   
#define TXD3                    TXD3        
#define PA26                    PA26        
#define AD_CH_3                 AD_CH_3
#define AOUT                    AOUT        
#define RXD3                    RXD3        
#define PA27                    PA27        
#define I2C0SDA                 I2C0SDA        
#define USB_SDA                 USB_SDA     
#define PA28                    PA28        
#define I2C0SCL                 I2C0SCL         
#define USB_SCL                 USB_SCL     
#define PA29                    PA29        
#define USB_D_P                 USB_D_P     
#define PA30                    PA30        
#define USB_D_N                 USB_D_N     
#define PB0                     PB0         
#define ETH_TXD0                ETH_TXD0    
#define PB1                     PB1         
#define ETH_TXD1                ETH_TXD1    
#define PB4                     PB4         
#define ETH_TX_EN               ETH_TX_EN   
#define PB8                     PB8         
#define ETH_CRS                 ETH_CRS     
#define PB9                     PB9         
#define ETH_RXD0                ETH_RXD0    
#define PB10                    PB10        
#define ETH_RXD1                ETH_RXD1    
#define PB14                    PB14        
#define ETH_RX_ER               ETH_RX_ER   
#define PB15                    PB15        
#define ETH_REF_CLK             ETH_REF_CLK 
#define PB16                    PB16        
#define ENET_MDC                ENET_MDC    
#define PB17                    PB17        
#define ENET_MDIO               ENET_MDIO   
#define PB18                    PB18        
#define USB_UP_LED              USB_UP_LED  
#define PWM1_CH1                PWM1_CH1    
#define CAP1_0                  CAP1_0      
#define PB19                    PB19        
#define MCOA0                   MCOA0       
#define USB_PPWR                USB_PPWR    
#define CAP1_1                  CAP1_1      
#define PB20                    PB20        
#define MCI0                    MCI0        
#define PWM1_CH2                PWM1_CH2    
#define SCK0                    SCK0        
#define PB21                    PB21        
#define MCABORT                 MCABORT     
#define PWM1_CH3                PWM1_CH3    
#define SSEL0                   SSEL0       
#define PB22                    PB22        
#define MCOB0                   MCOB0       
#define USB_PWRD                USB_PWRD    
#define MAT1_0                  MAT1_0      
#define PB23                    PB23        
#define MCI1                    MCI1        
#define PWM1_CH4                PWM1_CH4    
#define MISO0                   MISO0       
#define PB24                    PB24        
#define MCI2                    MCI2        
#define PWM1_CH5                PWM1_CH5    
#define MOSI0                   MOSI0       
#define PB25                    PB25        
#define MCOA1                   MCOA1       
#define MAT1_1                  MAT1_1      
#define PB26                    PB26        
#define MCOB1                   MCOB1       
#define PWM1_CH6                PWM1_CH6    
#define CAP0_0                  CAP0_0      
#define PB27                    PB27        
#define CLKOUT                  CLKOUT      
#define USB_OVRCR               USB_OVRCR   
#define CAP0_1                  CAP0_1      
#define PB28                    PB28        
#define MCOA2                   MCOA2       
#define PWM_CAP_CH0             PWM_CAP_CH0 
#define MAT0_0                  MAT0_0      
#define PB29                    PB29        
#define MCOB2                   MCOB2       
#define PWM_CAP_CH1             PWM_CAP_CH1 
#define MAT0_1                  MAT0_1      
#define PB30                    PB30        
#define VBUS                    VBUS        
#define AD_CH_4                 AD_CH_4
#define PB31                    PB31        
#define SCK1                    SCK1        
#define AD_CH_5                 AD_CH_5
#define PC0                     PC0         
#define TXD1                    TXD1        
#define PC1                     PC1         
#define RXD1                    RXD1        
#define PC2                     PC2         
#define CTS1                    CTS1        
#define PC3                     PC3         
#define DCD1                    DCD1        
#define PC4                     PC4         
#define DSR1                    DSR1        
#define PC5                     PC5         
#define DTR1                    DTR1        
#define PC6                     PC6         
#define RI1                     RI1         
#define PC7                     PC7         
#define RD2                     RD2         
#define RTS1                    RTS1        
#define PC8                     PC8         
#define TD2                     TD2         
#define TXD2                    TXD2        
#define ENET_MDC                ENET_MDC    
#define PC9                     PC9         
#define USB_CONNECT             USB_CONNECT 
#define RXD2                    RXD2        
#define ENET_MDIO               ENET_MDIO   
#define PC10                    PC10        
#define EINT0                   EINT0       
#define NMI                     NMI         
#define PC11                    PC11        
#define EINT1                   EINT1       
#define I2STX_CLK               I2STX_CLK   
#define PC12                    PC12        
#define EINT2                   EINT2       
#define I2STX_WS                I2STX_WS    
#define PC13                    PC13        
#define EINT3                   EINT3       
#define I2STX_SDA               I2STX_SDA   
#define PD25                    PD25        
#define MAT0_0                  MAT0_0      
#define PD26                    PD26        
#define STCLK                   STCLK       
#define MAT0_1                  MAT0_1      
#define PE28                    PE28        
#define RX_MCLK                 RX_MCLK     
#define MAT2_0                  MAT2_0      
#define TXD3                    TXD3        
#define PE29                    PE29        
#define TX_MCLK                 TX_MCLK     
#define MAT2_1                  MAT2_1      
#define RXD3                    RXD3        

//*****************************************************************************
//
//! @}
//
//*****************************************************************************



//*****************************************************************************
//
//! \addtogroup xGPIO_Exported_APIs xGPIO API
//! \brief xGPIO API Reference.
//!
//! \section xGPIO_Exported_APIs_Port CoX Port Detail
//!
//! \verbatim
//! +--------------------------+----------------+------------------------+
//! |xGPIO API                 |       CoX      |         STM32F1xx      |
//! |--------------------------|----------------|------------------------|
//! |xGPIODirModeSet           |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinToPeripheralId   |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinToPort           |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinToPin            |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinDirModeSet       |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIODirModeGet           |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOPinIntCallbackInit   |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOPinIntEnable         |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinIntEnable        |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOPinIntDisable        |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinIntDisable       |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOPinIntStatus         |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOPinIntClear          |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinIntClear         |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOPinRead              |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinRead             |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOPinWrite             |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinWrite            |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOPinConfigure         |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinTypeGPIOInput    |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinTypeGPIOOutput   |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinTypeGPIOOutputOD |  Non-Mandatory |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xGPIOSPinTypeGPIOOutputQB |  Non-Mandatory |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypeADC              |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypeCAN              |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypeI2C              |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypeI2S              |  Non-Mandatory |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypePWM              |  Non-Mandatory |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypeSPI              |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypeTimer            |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypeUART             |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypeACMP             |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypeCLKO             |  Non-Mandatory |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypeEXTINT           |        N       |            Y           |
//! |--------------------------|----------------|------------------------|
//! |xSPinTypeEBI              |  Non-Mandatory |            Y           |
//! +--------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief   Set the direction and mode of the specified pin(s).
//!
//! This function will set the specified pin(s) on the selected GPIO port
//! as either an input or output under software control, or it will set the
//! pin to be under hardware control.
//!
//! \param [in] ulPort is the base address of the GPIO port, this value can
//!             be one of the following value:
//!             - \ref GPIOA_BASE
//!             - \ref GPIOB_BASE
//!             - ...
//!             More Information, please refer to \ref xLowLayer_Peripheral_Memmap.
//!
//! \param [in] ulPins is the bit-packed representation of the pin(s).
//!             elemnt can be one of the following value:
//!             - \ref GPIO_PIN0
//!             - \ref GPIO_PIN1
//!             - \ref ...
//!             More Information, please refer to \ref xGPIO_General_Pin_IDs.
//!  
//! \param [in] ulPinIO is the pin direction and/or mode.
//! Details please refer to \ref xGPIO_Dir_Mode.
//!
//!
//! The parameter \e ulPinIO is an enumerated data type that can be one of
//! the following values:
//! 
//! Details please refer to \ref xGPIO_Dir_Mode_CoX.
//!
//! Where \b xGPIO_DIR_MODE_IN specifies that the pin will be programmed a
//! a software controlled input, \b xGPIO_DIR_MODE_OUT specifies that the pin
//! will be programmed as a software controlled output, and
//! \b xGPIO_DIR_MODE_HW specifies that the pin will be placed under
//! hardware control.
//!
//! The pin(s) are specified using a bit-packed byte, where each bit that i
//! set identifies the pin to be accessed, and where bit 0 of the byte
//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
//! 
//! Details please refer to \ref xGPIO_General_Pin_IDs_CoX.
//!
//! \note xGPIOPadConfigSet() must also be used to configure the corresponding
//! pad(s) in order for them to propagate the signal to/from the GPIO.
//!
//! \return None.
//
//*****************************************************************************
extern void xGPIODirModeSet(unsigned long ulPort, unsigned long ulPins,
                            unsigned long ulPinIO);   

//*****************************************************************************
//
//! \brief Get the GPIO port from a Pin.
//!
//! \param eShortPin is the base address of the GPIO port
//! 
//! \return GPIO port code which is used by \ref xSysCtlPeripheralEnable,
//! \ref xSysCtlPeripheralDisable, \ref xSysCtlPeripheralReset.
//! Details please refer to \ref xSysCtl_Peripheral_ID.
//
//*****************************************************************************
#define xGPIOSPinToPeripheralId(eShortPin)                                    \
        GPIOPinToPeripheralId(G##eShortPin)

//*****************************************************************************
//
//! \brief Get the GPIO port from a short Pin.
//!
//! \param eShortPin is the base address of the GPIO port
//!
//! \note None.
//! 
//! \return GPIO port address which is used by GPIO API.
//
//*****************************************************************************
#define xGPIOSPinToPort(eShortPin)                                            \
        GPIOPinToPort(G##eShortPin)


//*****************************************************************************
//
//! \brief Get the GPIO port and Pin number from a short Pin.
//!
//! \param eShortPin is the base address of the GPIO port
//!
//! \note None.
//! 
//! \return GPIO port address which is used by GPIO API.
//
//*****************************************************************************
#define xGPIOSPinToPortPin(eShortPin)                                         \
        GPIOSPinToPortPin(eShortPin)
        
//*****************************************************************************
//
//! \brief Get the GPIO pin number from a short Pin.
//!
//! \param eShortPin is the base address of the GPIO port
//!
//! \note None.
//! 
//! \return GPIO pin number which is used by GPIO API.
//
//*****************************************************************************
#define xGPIOSPinToPin(eShortPin)                                             \
        GPIOPinToPin(G##eShortPin)

//*****************************************************************************
//
//! \brief Set the direction and mode of the specified pin(s).
//!
//! \param eShortPin Specified port and pin.
//! Details please refer to \ref xGPIO_Short_Pin.
//! \param ulPinIO is the pin direction and/or mode.
//! Details please refer to \ref xGPIO_Dir_Mode.
//!
//! This function will set the specified pin(s) on the selected GPIO port
//! as either an input or output under software control, or it will set the
//! pin to be under hardware control.
//!
//! The parameter \e ulPinIO is an enumerated data type that can be one of
//! the following values:
//!
//! Details please refer to \ref xGPIO_Dir_Mode_CoX.
//!
//! where \b xGPIO_DIR_MODE_IN specifies that the pin will be programmed a
//! a software controlled input, \b xGPIO_DIR_MODE_OUT specifies that the pin
//! will be programmed as a software controlled output, and
//! \b xGPIO_DIR_MODE_HW specifies that the pin will be placed under
//! hardware control.
//!
//! The pin is specified by eShortPin, which can only be one pin.
//! Details please refer to \ref xGPIO_Short_Pin_CoX.
//!
//! \note xGPIOPadConfigSet() must also be used to configure the corresponding
//! pad(s) in order for them to propagate the signal to/from the GPIO.
//!
//! \return None.
//
//*****************************************************************************
#define xGPIOSPinDirModeSet(eShortPin, ulPinIO)                               \
        xGPIODirModeSet(G##eShortPin, ulPinIO)

//*****************************************************************************
//
//! \brief Get the direction and mode of a pin.
//!
//! \param ulPort is the base address of the GPIO port
//! Details please refer to \ref xLowLayer_Peripheral_Memmap.
//! \param ulPin is the bit-packed representation of the pin.
//! Details please refer to \ref xGPIO_General_Pin_IDs.
//!
//! This function gets the direction and control mode for a specified pin on
//! the selected GPIO port.  The pin can be configured as either an input or
//! output under software control, or it can be under hardware control.  The
//! type of control and direction are returned as an enumerated data type.
//!
//! The pin are specified using a bit-packed byte, where each bit that i
//! set identifies the pin to be accessed, and where bit 0 of the byte
//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
//! 
//! Details please refer to \ref xGPIO_General_Pin_IDs_CoX.
//!
//! \return Returns one of the enumerated data types described for
//! \ref xGPIODirModeSet().Details please refer to \ref xGPIO_Dir_Mode_CoX.
//
//*****************************************************************************
extern unsigned long xGPIODirModeGet(unsigned long ulPort, 
                                     unsigned long ulPin);

//*****************************************************************************
//
//! \brief Init the GPIO Port X Interrupt Callback function.
//!
//! \param ulPort is the base address of the GPIO port.
//! Details please refer to \ref xLowLayer_Peripheral_Memmap.
//! \param ulPin is the bit-packed representation of the pin.
//! Details please refer to \ref xGPIO_General_Pin_IDs.
//! \param pfnCallback is the callback function.
//! Details please refer to \ref xLowLayer_Exported_Types.
//!
//! When there is any pins interrupt occrus, Interrupt Handler will 
//! call the callback function. 
//! 
//! param of pfnCallback
//! - pvCBData not used, always 0.
//! - ulEvent not used, always 0.
//! - ulMsgParam is pins which have an event.
//! Details please refer to \ref xGPIO_General_Pin_IDs_CoX.
//! - pvMsgData not used, always 0.
//!
//! \return None.
//
//*****************************************************************************
extern void xGPIOPinIntCallbackInit(unsigned long ulPort, unsigned long ulPin, 
                                   xtEventCallback xtPortCallback);
        
//*****************************************************************************
//
//! \brief Set the interrupt type and Enable interrupt for the specified pin(s).
//!
//! \param ulPort is the base address of the GPIO port
//! Details please refer to \ref xLowLayer_Peripheral_Memmap.
//! \param ulPins is the bit-packed representation of the pin(s).
//! Details please refer to \ref xGPIO_General_Pin_IDs.
//! \param ulIntType specifies the type of interrupt trigger mechanism.
//! Details please refer to \ref xGPIO_Int_Type.
//!
//! This function sets up the various interrupt trigger mechanisms for the
//! specified pin(s) on the selected GPIO port.
//!
//! The parameter \e ulIntType is an enumerated data type that can be one of
//! the following values:
//!
//! Details please refer to \ref xGPIO_Int_Type_CoX.
//!
//! Where the different values describe the interrupt detection mechanism
//! (edge or level) and the particular triggering event (falling, rising,
//! or both edges for edge detect, low or high for level detect).
//!
//! The pin(s) are specified using a bit-packed byte, where each bit that i
//! set identifies the pin to be accessed, and where bit 0 of the byte
//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
//! 
//! Details please refer to \ref xGPIO_General_Pin_IDs_CoX.
//!
//! \note In order to avoid any spurious interrupts, the user must
//! ensure that the GPIO inputs remain stable for the duration of
//! this function.
//!
//! \return None.
//
//*****************************************************************************
extern void xGPIOPinIntEnable(unsigned long ulPort, unsigned long ulPins,
        unsigned long ulIntType);
         
//*****************************************************************************
//
//! \brief Set the interrupt type and Enable interrupt for the specified pin(s).
//!
//! \param eShortPin Specified port and pin.
//! Details please refer to \ref xGPIO_Short_Pin.
//! \param ulIntType specifies the type of interrupt trigger mechanism.
//! Details please refer to \ref xGPIO_Int_Type.
//!
//! This function sets up the various interrupt trigger mechanisms for the
//! specified pin(s) on the selected GPIO port.
//!
//! The parameter \e ulIntType is an enumerated data type that can be one of
//! the following values:
//!
//! Details please refer to \ref xGPIO_Int_Type_CoX.
//!
//! Where the different values describe the interrupt detection mechanism
//! (edge or level) and the particular triggering event (falling, rising,
//! or both edges for edge detect, low or high for level detect).
//!
//! The pin is specified by eShortPin, which can only be one pin.
//! Details please refer to \ref xGPIO_Short_Pin_CoX.
//!
//! \note In order to avoid any spurious interrupts, the user must
//! ensure that the GPIO inputs remain stable for the duration of
//! this function.
//!
//! \return None.
//
//*****************************************************************************
#define xGPIOSPinIntEnable(eShortPin, ulIntType)                              \
        xGPIOPinIntEnable(G##eShortPin, ulIntType)

//*****************************************************************************
//
//! \brief Disables interrupts for the specified pin(s).
//!
//! \param ulPort is the base address of the GPIO port
//! Details please refer to \ref xLowLayer_Peripheral_Memmap.
//! \param ulPins is the bit-packed representation of the pin(s).
//! Details please refer to \ref xGPIO_General_Pin_IDs.
//!
//! Mask the interrupt for the specified pin(s).
//!
//! The pin(s) are specified using a bit-packed byte, where each bit that i
//! set identifies the pin to be accessed, and where bit 0 of the byte
//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
//!
//! Details please refer to \ref xGPIO_General_Pin_IDs_CoX.
//!
//! \return None.
//
//*****************************************************************************       
extern void xGPIOPinIntDisable(unsigned long ulPort, unsigned long ulPins);

//*****************************************************************************
//
//! \brief Disables interrupts for the specified pin.
//!
//! \param eShortPin Specified port and pin.
//! Details please refer to \ref xGPIO_Short_Pin.
//!
//! Mask the interrupt for the specified pin(s).
//!
//! The pin is specified by eShortPin, which can only be one pin.
//! Details please refer to \ref xGPIO_Short_Pin_CoX.
//!
//! \return None.
//
//*****************************************************************************       
#define xGPIOSPinIntDisable(eShortPin)                                        \
        xGPIOPinIntDisable(G##eShortPin)

//*****************************************************************************
//
//! \brief Get interrupt status for the specified GPIO port.
//!
//! \param ulPort is the base address of the GPIO port.
//! Details please refer to \ref xLowLayer_Peripheral_Memmap.
//! 
//! \return Returns a bit-packed byte, where each bit that is set identifie
//! an active masked or raw interrupt, and where bit 0 of the byte
//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
//! Bits 31:16 should be ignored.
//
//*****************************************************************************
#define xGPIOPinIntStatus(ulPort)

//*****************************************************************************
//
//! \brief Clear the interrupt for the specified pin(s).
//!
//! \param ulPort is the base address of the GPIO port
//! Details please refer to \ref xLowLayer_Peripheral_Memmap.
//! \param ulPins is the bit-packed representation of the pin(s).
//! Details please refer to \ref xGPIO_General_Pin_IDs.
//!
//! Clear the interrupt for the specified pin(s).
//!
//! The pin(s) are specified using a bit-packed byte, where each bit that i
//! set identifies the pin to be accessed, and where bit 0 of the byte
//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
//!
//! Details please refer to \ref xGPIO_General_Pin_IDs_CoX.
//!
//! \note Because there is a write buffer in the Cortex-M0 processor, it may
//! take several clock cycles before the interrupt source is actually cleared.
//! Therefore, it is recommended that the interrupt source be cleared early in
//! the interrupt handler (as opposed to the very last action) to avoid
//! returning from the interrupt handler before the interrupt source i
//! actually cleared.  Failure to do so may result in the interrupt handler
//! being immediately reentered (because the interrupt controller still see
//! the interrupt source asserted).
//!
//! \return None.
//
//*****************************************************************************
extern void xGPIOPinIntClear(unsigned long ulPort, unsigned long ulPins);

//*****************************************************************************
//
//! \brief Clear the interrupt for the specified pin.
//!
//! \param eShortPin Specified port and pin.
//! Details please refer to \ref xGPIO_Short_Pin.
//!
//! Clear the interrupt for the specified pin.
//!
//! The pin is specified by eShortPin, which can only be one pin.
//! Details please refer to \ref xGPIO_Short_Pin_CoX.
//!
//! \note Because there is a write buffer in the Cortex-M0 processor, it may
//! take several clock cycles before the interrupt source is actually cleared.
//! Therefore, it is recommended that the interrupt source be cleared early in
//! the interrupt handler (as opposed to the very last action) to avoid
//! returning from the interrupt handler before the interrupt source i
//! actually cleared.  Failure to do so may result in the interrupt handler
//! being immediately reentered (because the interrupt controller still see
//! the interrupt source asserted).
//!
//! \return None.
//
//*****************************************************************************
#define xGPIOSPinIntClear(eShortPin)                                          \
        xGPIOPinIntClear(G##eShortPin)

//*****************************************************************************
//
//! \brief Reads the values present of the specified pin(s).
//!
//! \param ulPort is the base address of the GPIO port
//! Details please refer to \ref xLowLayer_Peripheral_Memmap.
//! \param ulPins is the bit-packed representation of the pin(s).
//! Details please refer to \ref xGPIO_General_Pin_IDs.
//!
//! The values at the specified pin(s) are read, as specified by \e ucPins.
//! Values are returned for both input and output pin(s), and the value
//! for pin(s) that are not specified by \e ucPins are set to 0.
//!
//! The pin(s) are specified using a bit-packed byte, where each bit that i
//! set identifies the pin to be accessed, and where bit 0 of the byte
//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
//!
//! Details please refer to \ref xGPIO_General_Pin_IDs_CoX.
//!
//! \return Returns a bit-packed byte providing the state of the specified
//! pin, where bit 0 of the byte represents GPIO port pin 0, bit 1 represent
//! GPIO port pin 1, and so on.  Any bit that is not specified by \e ucPin
//! is returned as a 0.  Bits 31:8 should be ignored.
//
//*****************************************************************************
extern unsigned long xGPIOPinRead(unsigned long ulPort, unsigned long ulPins);

//*****************************************************************************
//
//! \brief Reads the values present of the specified Port.
//!
//! \param ulPort is the base address of the GPIO port.
//!
//! The values at the specified Port are read
//!
//! \return Returns a bit-packed byte providing the state of the specified
//! Port
//
//*****************************************************************************
#define xGPIOPortRead(ulPort)                                                 \
        GPIOPortRead(ulPort)

//*****************************************************************************
//
//! \brief Reads the values present of the specified pin.
//!
//! \param eShortPin Specified port and pin.
//! Details please refer to \ref xGPIO_Short_Pin.
//!
//! The values at the specified pin are read, as specified by \e eShortPin.
//! Values are returned for both input and output pin(s), and the value
//! for pin(s) that are not specified by \e eShortPin are set to 0.
//!
//! The pin is specified by eShortPin, which can only be one pin.
//! Details please refer to \ref xGPIO_Short_Pin_CoX.
//!
//! \return Returns the value of specified port and pin.
//
//*****************************************************************************
#define xGPIOSPinRead(eShortPin)                                              \
        GPIOPinRead(G##eShortPin)

//*****************************************************************************
//
//! \brief Write a value to the specified pin(s).
//!
//! \param ulPort is the base address of the GPIO port
//! Details please refer to \ref xLowLayer_Peripheral_Memmap.
//! \param ulPins is the bit-packed representation of the pin(s).
//! Details please refer to \ref xGPIO_General_Pin_IDs.
//! \param ucVal is the value to write to the pin(s), 0 or 1.
//!
//! Write the corresponding bit values to the output pin(s) specified by
//! \e ucPins.  Writing to a pin configured as an input pin has no effect.
//!
//! The pin(s) are specified using a bit-packed byte, where each bit that i
//! set identifies the pin to be accessed, and where bit 0 of the byte
//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
//!
//! Details please refer to \ref xGPIO_General_Pin_IDs_CoX.
//!
//! \return None.
//
//*****************************************************************************
extern void xGPIOPinWrite(unsigned long ulPort, unsigned long ulPins,
        unsigned long ucVal);

//*****************************************************************************
//
//! \brief Writes a value to the specified Port.
//!
//! \param ulPort is the base address of the GPIO port.
//! \param ucVal is the value to write to the Port.
//!
//! Writes the corresponding bit values to the output Port
//!
//! \return None.
//
//*****************************************************************************
#define xGPIOPortWrite(ulPort, ulVal)                                         \
        GPIOPortWrite(ulPort, ulVal)

        
//*****************************************************************************
//
//! \brief Write a value to the specified pin.
//!
//! \param eShortPin Specified port and pin.
//! Details please refer to \ref xGPIO_Short_Pin.
//! \param ucVal is the value to write to the pin(s), 0 or 1.
//!
//! Write the corresponding bit values to the output pin specified by
//! \e eShortPin.  Writing to a pin configured as an input pin has no effect.
//!
//! The pin is specified by eShortPin, which can only be one pin.
//! Details please refer to \ref xGPIO_Short_Pin_CoX.
//!
//! \return None.
//
//*****************************************************************************
#define xGPIOSPinWrite(eShortPin, ulVal)                                      \
        GPIOPinWrite(G##eShortPin, ulVal)

//*****************************************************************************
//
//! \brief Configure the alternate function of a GPIO pin.
//!
//! \param ulPinConfig is the pin configuration value, specified as only one of
//! the \b GPIO_P??_??? values.
//!
//! This function configures the pin mux that selects the peripheral function
//! associated with a particular GPIO pin.  Only one peripheral function at a
//! time can be associated with a GPIO pin, and each peripheral function should
//! only be associated with a single GPIO pin at a time (despite the fact that
//! many of them can be associated with more than one GPIO pin).
//!
//! \note This function is only valid on Tempest-class devices.
//!
//! \return None.
//
//*****************************************************************************
#define xGPIOPinConfigure(eShortPin, ulPinConfig)                             \
        GPIOPinFunCfg(G##eShortPin, ulPinConfig)

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO Input pin.
//!
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin.
//!
//! This function configures a pin for use as an GPIO Input pin device and turns 
//! the pin into a GPIO input pin.
//!
//! \return None.
//
//*****************************************************************************
#define xGPIOSPinTypeGPIOInput(eShortPin)                                     \
        do                                                                    \
        {                                                                     \
            GPIOPinFunCfg(G##eShortPin, GPIO_##eShortPin##_##eShortPin);      \
            GPIOPinModeCfg(G##eShortPin, PIN_MODE_INPUT);                     \
        }                                                                     \
        while(0)

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO Output(push-pull) pin.
//!
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin.
//!
//! This function configures a pin for use as an GPIO Output pin device and  
//! turns the pin into a GPIO Output pin.
//!
//! \return None.
//
//*****************************************************************************
#define xGPIOSPinTypeGPIOOutput(eShortPin)                                    \
        do                                                                    \
        {                                                                     \
            GPIOPinFunCfg(G##eShortPin, GPIO_##eShortPin##_##eShortPin);      \
            GPIOPinModeCfg(G##eShortPin, PIN_MODE_OUTPUT);                    \
        }                                                                     \
        while(0)  
            
//*****************************************************************************
//
//! \brief Turn a pin to a GPIO Output(open drain) pin.
//!
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin.
//!
//! This function configures a pin for use as an GPIO Output pin device and turns 
//! the pin into a GPIO Output(open drain) pin.
//!
//! \return None.
//
//*****************************************************************************
#define xGPIOSPinTypeGPIOOutputOD(eShortPin)                                  \
        do                                                                    \
        {                                                                     \
            GPIOPinFunCfg(G##eShortPin, GPIO_##eShortPin##_eShortPin);        \
            GPIOPinModeCfg(G##eShortPin, PIN_MODE_OD_EN);                     \
        }                                                                     \
        while(0)  

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO ADC input pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as ADC0. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin.
//!
//! This function configures a pin for use as an ADC function device and turns 
//! the pin into a GPIO ADC input pin.
//!
//! Table shows what the ePeripheralPin and eShortPin should be, only the 
//! argument which are in the same line can be combined. For eaxmple(TI):<br/>
//! xSPinTypeADC(ADC0, PE7) or xSPinTypeADC(ADC1, PE6) is correct;<br/>
//! But xSPinTypeADC(ADC0, PE6) or xSPinTypeADC(ADC0, PE5) is error.
//! \verbatim
//! +--------------------+------------------------+------------------------+
//! |    manufacturer    |ePeripheralPin          |eShortPin               |
//! |--------------------|------------------------|------------------------|
//! |    CoX             |This parameter is a     |This parameter is a     |
//! |                    |mandatory.The mandatory |mandatory. the mandatory|
//! |                    |is the format of        |is the format of        |
//! |                    |Variable naming.So it   |Variable naming.So it   |
//! |                    |should be: ADCn         |should be: PXn          |
//! |                    |n indicate the pin      |XX indicate the GPIO    |
//! |                    |number such as          |PORT,Such as            |
//! |                    |0 1 2 3 ....            |A B C D E ...           |
//! |                    |                        |n indicate the pin      |
//! |                    |                        |number such as          |
//! |                    |                        |0 1 2 3 ....            |
//! |--------------------|------------------------|------------------------|
//! |      STM32F1xx     |    ADC0                |    PA0 (ADC123)        |
//! |                    |    ADC1                |    PA1 (ADC123)        |
//! |                    |    ADC2                |    PA2 (ADC123)        |
//! |                    |    ADC3                |    PA3 (ADC123)        |
//! |                    |    ADC4                | PA4(ADC12)  PF6(ADC3)  |
//! |                    |    ADC5                | PA5(ADC12)  PF7(ADC3)  |
//! |                    |    ADC6                | PA6(ADC12)  PF8(ADC3)  |
//! |                    |    ADC7                | PA7(ADC12)  PF9(ADC3)  |
//! |                    |    ADC8                | PB0(ADC12)  PF10(ADC3) |
//! |                    |    ADC9                |    PB1 (ADC12)         |
//! |                    |    ADC10               |    PC0 (ADC123)        |
//! |                    |    ADC11               |    PC1 (ADC123)        |
//! |                    |    ADC12               |    PC2 (ADC123)        |
//! |                    |    ADC13               |    PC3 (ADC123)        |
//! |                    |    ADC14               |    PC4 (ADC12)         |
//! |                    |    ADC15               |    PC5 (ADC12)         |
//! |--------------------|------------------------|------------------------|
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************
#define xSPinTypeADC(ePeripheralPin, eShortPin)                               \
        GPIOSPinConfigure(ePeripheralPin, eShortPin)  

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO CAN input or output pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as CAN0RX. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin. 
//!
//! This function configures a pin for use as a CAN function and turns 
//! the pin into a GPIO CAN input or output pin.
//!
//! Table shows what the ePeripheralPin and eShortPin should be,only the 
//! argument which are in the same line can be combined.For eaxmple(TI):<br/>
//! xSPinTypeCAN(CAN0RX, PA4) or xSPinTypeCAN(CAN0RX, PB4) is correct;<br/>
//! But xSPinTypeCAN(CAN0RX, PA5) or xSPinTypeCAN(CAN0RX, PF0) is error.
//! \verbatim
//! +--------------------+------------------------+------------------------+
//! |    manufacturer    |ePeripheralPin          |eShortPin               |
//! |--------------------|------------------------|------------------------|
//! |    CoX             |This parameter is a     |This parameter is a     |
//! |                    |mandatory.The mandatory |mandatory. the mandatory|
//! |                    |is the format of        |is the format of        |
//! |                    |Variable naming.So it   |Variable naming.So it   |
//! |                    |should be: CANnRX       |should be: PXn          |
//! |                    |or CANnTX               |XX indicate the GPIO    |
//! |                    |n indicate the pin      |PORT,Such as            |
//! |                    |number such as          |A B C D E ...           |
//! |                    |0 1 2 3 ....            |n indicate the pin      |
//! |                    |                        |number such as          |
//! |                    |                        |0 1 2 3 ....            |
//! |--------------------|------------------------|------------------------|
//! |      STM32F1xx     |    CAN0RX              |    PA11  PD0  PB8      |
//! |                    |    CAN0TX              |    PA12  PD1  PB9      |
//! |--------------------|------------------------|------------------------|
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************             
#define xSPinTypeCAN(ePeripheralPin, eShortPin)                               \
        GPIOSPinConfigure(ePeripheralPin, eShortPin)

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO I2C input or output pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as I2C0SDA. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin.  
//!
//! This function configures a pin for use as an I2C function device and turns 
//! the pin into a GPIO I2C input or output pin.
//!
//! Table shows what the ePeripheralPin and eShortPin should be,only the 
//! argument which are in the same line can be combined.For eaxmple(TI):<br/>
//! xSPinTypeI2C(CAN0RX, PA4) or xSPinTypeI2C(CAN0RX, PB4) is correct;<br/>
//! But xSPinTypeI2C(CAN0RX, PA5) or xSPinTypeI2C(CAN0RX, PF0) is error.
//! \verbatim
//! +--------------------+------------------------+------------------------+
//! |    manufacturer    |ePeripheralPin          |eShortPin               |
//! |--------------------|------------------------|------------------------|
//! |    CoX             |This parameter is a     |This parameter is a     |
//! |                    |mandatory.The mandatory |mandatory. the mandatory|
//! |                    |is the format of        |is the format of        |
//! |                    |Variable naming.So it   |Variable naming.So it   |
//! |                    |should be: I2CnSCK      |should be: PXn          |
//! |                    |or I2CnSDA              |XX indicate the GPIO    |
//! |                    |n indicate the pin      |PORT,Such as            |
//! |                    |number such as          |A B C D E ...           |
//! |                    |0 1 2 3 ....            |n indicate the pin      |
//! |                    |                        |number such as          |
//! |                    |                        |0 1 2 3 ....            |
//! |--------------------|------------------------|------------------------|
//! |      STM32F1xx     |    I2C1SCK             |    PB6   PB8           |
//! |                    |    I2C1SDA             |    PB7   PB9           |
//! |                    |    I2C1SMBA            |    PB5                 |
//! |                    |    I2C2SCK             |    PB10                |
//! |                    |    I2C2SDA             |    PB11                |
//! |                    |    I2C2SMBA            |    PB12                |
//! |--------------------|------------------------|------------------------|
//! \endverbatim
//!
//! \return None.
//
//***************************************************************************** 
 #define xSPinTypeI2C(ePeripheralPin, eShortPin)                              \
         GPIOSPinConfigure(ePeripheralPin, eShortPin)

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO I2S input or output pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as I2C0SDA. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin. 
//!
//! This function configures a pin for use as an I2S function device and turns 
//!  the pin into a GPIO I2S input or output pin.
//!
//! Table shows what the ePeripheralPin and eShortPin should be,only the 
//! argument which are in the same line can be combined.For eaxmple(TI):<br/>
//! xSPinTypeI2S(I2S0RXSD, PA2) or xSPinTypeI2S(I2S0RXSD, PD4) is correct;<br/>
//! But xSPinTypeI2S(I2S0RXSD, PD1) or xSPinTypeI2S(I2S0RXWS, PA2) is error.
//! \verbatim
//! +--------------------+------------------------+------------------------+
//! |    manufacturer    |ePeripheralPin          |eShortPin               |
//! |--------------------|------------------------|------------------------|
//! |    CoX             |This parameter is a     |This parameter is a     |
//! |                    |mandatory.The mandatory |mandatory. the mandatory|
//! |                    |is the format of        |is the format of        |
//! |                    |Variable naming.So it   |Variable naming.So it   |
//! |                    |should be: I2SnRXSCK,   |should be: PXn          |
//! |                    |I2SnRXMCLK,I2S0RXSD,    |XX indicate the GPIO    |
//! |                    |I2S0RXWS,I2S0TXSCK,     |PORT,Such as            |
//! |                    |I2S0TXMCLK,I2S0TXSD,    |A B C D E ...           |
//! |                    |or I2S0TXWS.            |n indicate the pin      |
//! |                    |n indicate the pin      |number such as          |
//! |                    |number such as          |0 1 2 3 ....            |
//! |                    |0 1 2 3 ....            |                        |
//! |--------------------|------------------------|------------------------|
//! |      STM32F1xx     |    I2S2RXMCLK          |    PC6                 |
//! |                    |    I2S2RXSCK           |    PB13                |
//! |                    |    I2S2RXSD            |    PB15                |
//! |                    |    I2S2RXWS            |    PB12                |
//! |                    |    I2S2TXMCLK          |    PC6                 |
//! |                    |    I2S2TXSCK           |    PB13                |
//! |                    |    I2S2TXSD            |    PB15                |
//! |                    |    I2S2TXWS            |    PB12                |
//! |                    |    I2S3RXMCLK          |    PC7                 |
//! |                    |    I2S3RXSCK           |    PB3                 |
//! |                    |    I2S3RXSD            |    PB5                 |
//! |                    |    I2S3RXWS            |    PA15                |
//! |                    |    I2S3TXMCLK          |    PC7                 |
//! |                    |    I2S3TXSCK           |    PB3                 |
//! |                    |    I2S3TXSD            |    PB5                 |
//! |                    |    I2S3TXWS            |    PA15                |
//! |--------------------|------------------------|------------------------|
//! \endverbatim
//!
//! \return None.
//
//***************************************************************************** 
#define xSPinTypeI2S(ePeripheralPin, eShortPin)                               \
        GPIOSPinConfigure(ePeripheralPin, eShortPin)

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO SPI input or output pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as I2C0SDA. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin.
//!
//! This function configures a pin for use as a SPI function and turns 
//!  the pin into a GPIO SPI input or output pin.
//!
//! Table shows what the ePeripheralPin and eShortPin should be,only the 
//! argument which are in the same line can be combined.For eaxmple(TI):<br/>
//! xSPinTypeSPI(SPI0CLK, PA2) or xSPinTypeSPI(SPI0MOSI, PA5) is correct;<br/>
//! But xSPinTypeSPI(SPI0CLK, PA5) or xSPinTypeSPI(SPI0CLK, PA4) is error.
//! \verbatim
//! +--------------------+------------------------+------------------------+
//! |    manufacturer    |ePeripheralPin          |eShortPin               |
//! |--------------------|------------------------|------------------------|
//! |    CoX             |This parameter is a     |This parameter is a     |
//! |                    |mandatory.The mandatory |mandatory. the mandatory|
//! |                    |is the format of        |is the format of        |
//! |                    |Variable naming.So it   |Variable naming.So it   |
//! |                    |should be: SPInCLK,     |should be: PXn          |
//! |                    |SPInMISO, SPInMOSI,     |XX indicate the GPIO    |
//! |                    |or SPInCS,              |PORT,Such as            |
//! |                    |n indicate the pin      |A B C D E ...           |
//! |                    |number such as          |n indicate the pin      |
//! |                    |0 1 2 3 ....            |number such as          |
//! |                    |                        |0 1 2 3 ....            |
//! |--------------------|------------------------|------------------------|
//! |      STM32F1xx     |    SPI1CLK             |    PA5 PB3             |
//! |                    |    SPI1MOSI            |    PA7,PB5             |
//! |                    |    SPI1MISO            |    PA6,PB4             |
//! |                    |    SPI1CS              |    PA4,PA15            |
//! |                    |    SPI2CLK             |    PB13                |
//! |                    |    SPI2MOSI            |    PB15                |
//! |                    |    SPI2MISO            |    PB14                |
//! |                    |    SPI2CS              |    PB12                |
//! |                    |    SPI3CLK             |    PB3                 |
//! |                    |    SPI3MOSI            |    PB5                 |
//! |                    |    SPI3MISO            |    PB4                 |
//! |                    |    SPI3CS              |    PA15                |
//! |--------------------|------------------------|------------------------|
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************
#define xSPinTypeSPI(ePeripheralPin, eShortPin)                               \
        GPIOSPinConfigure(ePeripheralPin, eShortPin)

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO Timer input or output pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as I2C0SDA. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin. 
//!
//! This function configures a pin for use as a Timer function and turns 
//!  the pin into a GPIO Timer input or output pin.
//!
//! Table shows what the ePeripheralPin and eShortPin should be,only the 
//! argument which are in the same line can be combined.For eaxmple(TI):<br/>
//! xSPinTypeTimer(TIMCCP0, PD3) or xSPinTypeTimer(TIMCCP0, PJ7) is correct;<br/>
//! But xSPinTypeTimer(TIMCCP0, PC5) or xSPinTypeTimer(TIMCCP0, PB6) is error.
//! \verbatim
//! +--------------------+------------------------+------------------------+
//! |    manufacturer    |ePeripheralPin          |eShortPin               |
//! |--------------------|------------------------|------------------------|
//! |    CoX             |This parameter is a     |This parameter is a     |
//! |                    |mandatory.The mandatory |mandatory. the mandatory|
//! |                    |is the format of        |is the format of        |
//! |                    |Variable naming.So it   |Variable naming.So it   |
//! |                    |should be: TIMCCPn,     |should be: PXn          |
//! |                    |n indicate the pin      |XX indicate the GPIO    |
//! |                    |number such as          |PORT,Such as            |
//! |                    |0 1 2 3 ....            |A B C D E ...           |
//! |                    |                        |n indicate the pin      |
//! |                    |                        |number such as          |
//! |                    |                        |0 1 2 3 ....            |
//! |--------------------|------------------------|------------------------|
//! |      STM32F1xx     |    TIM1ETR             |    PA12 PE7            |
//! |                    |    TIM1CH1             |    PA8  PE9            |
//! |                    |    TIM1CH1N            |    PA7  PB13   PE8     |
//! |                    |    TIM1CH2             |    PA9  PE11           |
//! |                    |    TIM1CH2N            |    PB14 PB0   PE10     |
//! |                    |    TIM1CH3             |    PA10 PE13           |
//! |                    |    TIM1CH3N            |    PB15 PB1   PE12     |
//! |                    |    TIM1CH4             |    PA11 PE14           |
//! |                    |    TIM1CH4N            |                        |
//! |                    |    TIM1BKIN            |    PB12 PA6   PE15     |
//! |                    |    TIM8ETR             |    PA0                 |
//! |                    |    TIM8CH1             |    PC6                 |
//! |                    |    TIM8CH1N            |    PA7                 |
//! |                    |    TIM8CH2             |    PC7                 |
//! |                    |    TIM8CH2N            |    PB0                 |
//! |                    |    TIM8CH3             |    PC8                 |
//! |                    |    TIM8CH3N            |    PB1                 |
//! |                    |    TIM8CH4             |    PC9                 |
//! |                    |    TIM8CH4N            |                        |
//! |                    |    TIM8BKIN            |    PA6                 |
//! |                    |    TIM2ETR             |    PA0  PA15           |
//! |                    |    TIM2CH1             |    PA0  PA15           |
//! |                    |    TIM2CH2             |    PA1  PB3            |
//! |                    |    TIM2CH3             |    PA2  PB10           |
//! |                    |    TIM2CH4             |    PA3  PB11           |
//! |                    |    TIM3ETR             |    PD2                 |
//! |                    |    TIM3CH1             |    PA6  PB4   PC6  PF8 |
//! |                    |    TIM3CH2             |    PA7  PB5   PC7      |
//! |                    |    TIM3CH3             |    PB0  PC8            |
//! |                    |    TIM3CH4             |    PB1  PC9            |
//! |                    |    TIM4ETR             |    PE0                 |
//! |                    |    TIM4CH1             |    PB6  PD12           |
//! |                    |    TIM4CH2             |    PB7  PD13           |
//! |                    |    TIM4CH3             |    PB8  PD14           |
//! |                    |    TIM4CH4             |    PB9  PD15           |
//! |                    |    TIM5ETR             |                        |
//! |                    |    TIM5CH1             |    PA0                 |
//! |                    |    TIM5CH2             |    PA1                 |
//! |                    |    TIM5CH3             |    PA2                 |
//! |                    |    TIM5CH4             |    PA3                 |
//! |                    |    TIM9CH1             |    PE5  PA2            |
//! |                    |    TIM9CH2             |    PE6  PA3            |
//! |                    |    TIM10CH1            |    PF6                 |
//! |                    |    TIM10CH2            |                        |
//! |                    |    TIM11CH1            |    PF7                 |
//! |                    |    TIM11CH2            |                        |
//! |                    |    TIM12CH1            |    PB14                |
//! |                    |    TIM12CH2            |    PB15                |
//! |                    |    TIM13CH1            |    PA6                 |
//! |                    |    TIM13CH2            |                        |
//! |                    |    TIM14CH1            |    PA7  PF9            |
//! |                    |    TIM14CH2            |                        |
//! |--------------------|------------------------|------------------------|
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************            
#define xSPinTypeTimer(ePeripheralPin, eShortPin)                             \
        GPIOSPinConfigure(ePeripheralPin, eShortPin)

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO UART input or output pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as I2C0SDA. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin. 
//!
//! This function configures a pin for use as a UART function and turns 
//!  the pin into a GPIO UART input or output pin.
//!
//! Table shows what the ePeripheralPin and eShortPin should be,only the 
//! argument which are in the same line can be combined.For eaxmple(TI):<br/>
//! xSPinTypeUART(UART0RX, PA0) or xSPinTypeUART(UART0TX, PA1) is correct;<br/>
//! But xSPinTypeUART(UART0RX, PA1) or xSPinTypeUART(UART0RX, PE6) is error.
//! \verbatim
//! +--------------------+-------------------------+--------------------------+
//! |    manufacturer    | ePeripheralPin          | eShortPin                |
//! |--------------------|-------------------------|--------------------------|
//! |    CoX             | This parameter is a     | This parameter is a      |
//! |                    | mandatory.The mandatory | mandatory. the mandatory |
//! |                    | is the format of        | is the format of         |
//! |                    | Variable naming.So it   | Variable naming.So it    |
//! |                    | should be: UARTnRX,     | should be: PXn           |
//! |                    | UARTnTX, UARTnCTS,      | XX indicate the GPIO     |
//! |                    | ......,                 | PORT,Such as             |
//! |                    | n indicate the pin      | A B C D E ...            |
//! |                    | number such as          | n indicate the pin       |
//! |                    | 0 1 2 3 ....            | number such as           |
//! |                    |                         | 0 1 2 3 ....             |
//! |--------------------|-------------------------|--------------------------|
//! |                    |     UART0TX             |     PA2                  |
//! |                    |     UART0RX             |     PA3                  |
//! |                    | ------------------------- ------------------------ |
//! |                    |     UART1TX             |     PA15  PC0            |
//! |                    |     UART1RX             |     PA16  PC1            |
//! |                    |     UART1CTS            |     PA17  PC2            |
//! |                    |     UART1DCD            |     PA18  PC3            |
//! |                    |     UART1DSR            |     PA19  PC4            |
//! |      LPC17xx       |     UART1DTR            |     PA20  PC5            |
//! |                    |     UART1RI             |     PA21  PC6            |
//! |                    |     UART1RTS            |     PA22  PC7            |
//! |                    | ------------------------- ------------------------ |
//! |                    |     UART2TX             |     PA10  PC8            |
//! |                    |     UART2RX             |     PA11  PC9            |
//! |                    | ------------------------- ------------------------ |
//! |                    |     UART3TX             |     PA0   PA25  PE28     |
//! |                    |     UART3RX             |     PA1   PA26  PE29     |
//! |-------------------------------------------------------------------------|
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************
#define xSPinTypeUART(ePeripheralPin, eShortPin)                              \
        GPIOSPinConfigure(ePeripheralPin, eShortPin)

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO SDIO input or output pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as I2C0SDA. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin. 
//!
//! This function configures a pin for use as an SDIO function and turns 
//!  the pin into a GPIO SDIO input or output pin.
//!
//! \verbatim
//! +--------------------+------------------------+------------------------+
//! |    manufacturer    |ePeripheralPin          |eShortPin               |
//! |--------------------|------------------------|------------------------|
//! |    CoX             |This parameter is a     |This parameter is a     |
//! |                    |mandatory.The mandatory |mandatory. the mandatory|
//! |                    |is the format of        |is the format of        |
//! |                    |Variable naming.So it   |Variable naming.So it   |
//! |                    |should be: SDIOnDx,     |should be: PXn          |
//! |                    |SDIOnCK or SDIOnCMD     |X  indicate the GPIO    |
//! |                    |n x indicate the pin    |PORT,Such as            |
//! |                    |number such as          |A B C D E ...           |
//! |                    |0 1 2 3 ....            |n indicate the pin      |
//! |                    |                        |number such as          |
//! |                    |                        |0 1 2 3 ....            |
//! |--------------------|------------------------|------------------------|
//! |      STM32F1xx     |    SDIO1D0             |    PC8                 |
//! |                    |    SDIO1D1             |    PC9                 |
//! |                    |    SDIO1D2             |    PC10                |
//! |                    |    SDIO1D3             |    PC11                |
//! |                    |    SDIO1D4             |    PB8                 |
//! |                    |    SDIO1D5             |    PB9                 |
//! |                    |    SDIO1D6             |    PC6                 |
//! |                    |    SDIO1D7             |    PC7                 |
//! |                    |    SDIO1CK             |    PC12                |
//! |                    |    SDIO1CMD            |    PD2                 |
//! |--------------------|------------------------|------------------------|
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************            
#define xSPinTypeSDIO(ePeripheralPin, eShortPin)                              \
        GPIOSPinConfigure(ePeripheralPin, eShortPin)

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO CLKO output pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as I2C0SDA. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin. 
//!
//! This function configures a pin for use as a CLKO function and turns 
//!  the pin into a GPIO CLKO output pin.
//!
//! \verbatim
//! +--------------------+------------------------+------------------------+
//! |    manufacturer    |ePeripheralPin          |eShortPin               |
//! |--------------------|------------------------|------------------------|
//! |    CoX             |This parameter is a     |This parameter is a     |
//! |                    |mandatory.The mandatory |mandatory. the mandatory|
//! |                    |is the format of        |is the format of        |
//! |                    |Variable naming.So it   |Variable naming.So it   |
//! |                    |should be: MCO ,        |should be: PXn          |
//! |                    |                        |X  indicate the GPIO    |
//! |                    |                        |PORT,Such as            |
//! |                    |                        |A B C D E ...           |
//! |                    |                        |n indicate the pin      |
//! |                    |                        |number such as          |
//! |                    |                        |0 1 2 3 ....            |
//! |--------------------|------------------------|------------------------|
//! |      STM32F1xx     |    MCO                 |    PA8                 |
//! |--------------------|------------------------|------------------------|
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************
#define xSPinTypeCLKO(ePeripheralPin, eShortPin)                              \
        GPIOSPinConfigure(ePeripheralPin, eShortPin)
            
//*****************************************************************************
//
//! \brief Turn a pin to a GPIO DAC output pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as I2C0SDA. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin. 
//!
//! This function configures a pin for use as an GPIO DAC function and  
//! turns the pin into a GPIO DAC output  pin.
//!
//! \verbatim
//! +--------------------+------------------------+------------------------+
//! |    manufacturer    |ePeripheralPin          |eShortPin               |
//! |--------------------|------------------------|------------------------|
//! |    CoX             |This parameter is a     |This parameter is a     |
//! |                    |mandatory.The mandatory |mandatory. the mandatory|
//! |                    |is the format of        |is the format of        |
//! |                    |Variable naming.So it   |Variable naming.So it   |
//! |                    |should be: DACOUTn,     |should be: PXn          |
//! |                    |                        |X  indicate the GPIO    |
//! |                    |n x indicate the pin    |PORT,Such as            |
//! |                    |number such as          |A B C D E ...           |
//! |                    |0 1 2 3 ....            |n indicate the pin      |
//! |                    |                        |number such as          |
//! |                    |                        |0 1 2 3 ....            |
//! |--------------------|------------------------|------------------------|
//! |      STM32F1xx     |    DACOUT1             |    PA4                 |
//! |                    |    DACOUT2             |    PA5                 |
//! |--------------------|------------------------|------------------------|
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************
#define xSPinTypeDAC(ePeripheralPin, eShortPin)                               \
        GPIOSPinConfigure(ePeripheralPin, eShortPin)

//*****************************************************************************
//
//! \brief Turn a pin to a GPIO PWM output pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as I2C0SDA. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin. 
//!
//! This function configures a pin for use as an GPIO DAC function and  
//! turns the pin into a GPIO DAC output  pin.
//!
//! \verbatim
//! +--------------------+------------------------+------------------------+
//! |    manufacturer    |ePeripheralPin          |eShortPin               |
//! |--------------------|------------------------|------------------------|
//! |    CoX             |This parameter is a     |This parameter is a     |
//! |                    |mandatory.The mandatory |mandatory. the mandatory|
//! |                    |is the format of        |is the format of        |
//! |                    |Variable naming.So it   |Variable naming.So it   |
//! |                    |should be: DACOUTn,     |should be: PXn          |
//! |                    |                        |X  indicate the GPIO    |
//! |                    |n x indicate the pin    |PORT,Such as            |
//! |                    |number such as          |A B C D E ...           |
//! |                    |0 1 2 3 ....            |n indicate the pin      |
//! |                    |                        |number such as          |
//! |                    |                        |0 1 2 3 ....            |
//! |--------------------|------------------------|------------------------|
//! |      STM32F1xx     |    DACOUT1             |    PA4                 |
//! |                    |    DACOUT2             |    PA5                 |
//! |--------------------|------------------------|------------------------|
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************
#define xSPinTypeDAC(ePeripheralPin, eShortPin)                               \
        GPIOSPinConfigure(ePeripheralPin, eShortPin)


//*****************************************************************************
//
//! \brief Turn a pin to a GPIO FSMC input or output pin.
//!
//! \param ePeripheralPin is the GPIO Peripheral name such as I2C0SDA. 
//! Details please refer to \ref xGPIO_Pin_Config.
//! \param eShortPin is the GPIO short pin name such as PA0. 
//! Details please refer to \ref xGPIO_Short_Pin. 
//!
//! This function configures a pin for use as an FSMC function and turns 
//!  the pin into a GPIO FSMC input or output pin.
//!
//! \verbatim
//! +--------------------+------------------------+------------------------+
//! |    manufacturer    |ePeripheralPin          |eShortPin               |
//! |--------------------|------------------------|------------------------|
//! |    CoX             |This parameter is a     |This parameter is a     |
//! |                    |mandatory.The mandatory |mandatory. the mandatory|
//! |                    |is the format of        |is the format of        |
//! |                    |Variable naming.So it   |Variable naming.So it   |
//! |                    |should be: FSMC1An,     |should be: PXn          |
//! |                    |FSMC1Dn or ........     |X  indicate the GPIO    |
//! |                    |n x indicate the pin    |PORT,Such as            |
//! |                    |number such as          |A B C D E ...           |
//! |                    |0 1 2 3 ....            |n indicate the pin      |
//! |                    |                        |number such as          |
//! |                    |                        |0 1 2 3 ....            |
//! |--------------------|------------------------|------------------------|
//! |      STM32F1xx     |    FSMC1NE1            |    PD7                 |
//! |                    |    FSMC1NE2            |    PG9                 |
//! |                    |    FSMC1NE3            |    PG10                |
//! |                    |    FSMC1NE4            |    PG12                |
//! |                    |    FSMC1NADV           |    PB7                 |
//! |                    |    FSMC1NBL0           |    PE0                 |
//! |                    |    FSMC1NBL1           |    PE1                 |
//! |                    |    FSMC1CLK            |    PD3                 |
//! |                    |    FSMC1A0             |    PF0                 |
//! |                    |    FSMC1A1             |    PF1                 |
//! |                    |    FSMC1A2             |    PF2                 |
//! |                    |    FSMC1A3             |    PF3                 |
//! |                    |    FSMC1A4             |    PF4                 |
//! |                    |    FSMC1A5             |    PF5                 |
//! |                    |    FSMC1A6             |    PF12                |
//! |                    |    FSMC1A7             |    PF13                |
//! |                    |    FSMC1A8             |    PF14                |
//! |                    |    FSMC1A9             |    PF15                |
//! |                    |    FSMC1A10            |    PG0                 |
//! |                    |    FSMC1A11            |    PG1                 |
//! |                    |    FSMC1A12            |    PG2                 |
//! |                    |    FSMC1A13            |    PG3                 |
//! |                    |    FSMC1A14            |    PG4                 |
//! |                    |    FSMC1A15            |    PG5                 |
//! |                    |    FSMC1A16            |    PD11                |
//! |                    |    FSMC1A17            |    PD12                |
//! |                    |    FSMC1A18            |    PD13                |
//! |                    |    FSMC1A19            |    PE3                 |
//! |                    |    FSMC1A20            |    PE4                 |
//! |                    |    FSMC1A21            |    PE5                 |
//! |                    |    FSMC1A22            |    PE6                 |
//! |                    |    FSMC1A23            |    PE2                 |
//! |                    |    FSMC1A24            |    PG13                |
//! |                    |    FSMC1A25            |    PG14                |
//! |                    |    FSMC1D0             |    PD14                |
//! |                    |    FSMC1D1             |    PD14                |
//! |                    |    FSMC1D2             |    PD0                 |
//! |                    |    FSMC1D3             |    PD1                 |
//! |                    |    FSMC1D4             |    PE7                 |
//! |                    |    FSMC1D5             |    PE8                 |
//! |                    |    FSMC1D6             |    PE9                 |
//! |                    |    FSMC1D7             |    PE10                |
//! |                    |    FSMC1D8             |    PE11                |
//! |                    |    FSMC1D9             |    PE12                |
//! |                    |    FSMC1D10            |    PE13                |
//! |                    |    FSMC1D11            |    PE14                |
//! |                    |    FSMC1D12            |    PE15                |
//! |                    |    FSMC1D13            |    PD8                 |
//! |                    |    FSMC1D14            |    PD9                 |
//! |                    |    FSMC1D15            |    PD10                |
//! |                    |    FSMC1NOE            |    PD4                 |
//! |                    |    FSMC1NWE            |    PD5                 |
//! |                    |    FSMC1NWAIT          |    PD6                 |
//! |                    |    FSMC1NCE3           |    PG9                 |
//! |                    |    FSMC1NCE2           |    PD7                 |
//! |                    |    FSMC1INT3           |    PG7                 |
//! |                    |    FSMC1INT2           |    PG6                 |
//! |                    |    FSMC1INTR           |    PF10                |
//! |                    |    FSMC1NCE4_1         |    PG10                |
//! |                    |    FSMC1NCE4_2         |    PG11                |
//! |                    |    FSMC1NIORD          |    PF6                 |
//! |                    |    FSMC1NIOWR          |    PF8                 |
//! |                    |    FSMC1NIOS16         |    PF11                |
//! |                    |    FSMC1NREG           |    PF7                 |
//! |                    |    FSMC1CD             |    PF9                 |
//! |--------------------|------------------------|------------------------|
//! \endverbatim
//!
//! \return None.
//
//*****************************************************************************
#define xSPinTypeFSMC(ePeripheralPin, eShortPin)                              \
        GPIOSPinConfigure(ePeripheralPin, eShortPin)


//*****************************************************************************
//
//! @}
//
//*****************************************************************************


 #define GPIOSPinConfigure(ePeripheralPin, eShortPin)                          \
        GPIOPinFunCfg(G##eShortPin, GPIO_##eShortPin##_##ePeripheralPin)






//*****************************************************************************
//
//! \addtogroup STM32F1xx_GPIO_General_Pin_IDs STM32F1xx GPIO General Pin ID
//! \brief The following values define the bit field for the ucPins argument 
//! to several of the APIs.
//! @{
//
//*****************************************************************************

//! GPIO Pin 0
#define GPIO_PIN_0              BIT_32_0

//! GPIO Pin 1
#define GPIO_PIN_1              BIT_32_1

//! GPIO Pin 2
#define GPIO_PIN_2              BIT_32_2

//! GPIO Pin 3
#define GPIO_PIN_3              BIT_32_3

//! GPIO Pin 4
#define GPIO_PIN_4              BIT_32_4

//! GPIO Pin 5
#define GPIO_PIN_5              BIT_32_5

//! GPIO Pin 6
#define GPIO_PIN_6              BIT_32_6

//! GPIO Pin 7
#define GPIO_PIN_7              BIT_32_7

//! GPIO Pin 8
#define GPIO_PIN_8              BIT_32_8

//! GPIO Pin 9
#define GPIO_PIN_9              BIT_32_9

//! GPIO Pin 10
#define GPIO_PIN_10             BIT_32_10

//! GPIO Pin 11
#define GPIO_PIN_11             BIT_32_11

//! GPIO Pin 12
#define GPIO_PIN_12             BIT_32_12

//! GPIO Pin 13
#define GPIO_PIN_13             BIT_32_13

//! GPIO Pin 14
#define GPIO_PIN_14             BIT_32_14

//! GPIO Pin 15
#define GPIO_PIN_15             BIT_32_15

//! GPIO Pin 16
#define GPIO_PIN_16             BIT_32_16

//! GPIO Pin 17
#define GPIO_PIN_17             BIT_32_17

//! GPIO Pin 18
#define GPIO_PIN_18             BIT_32_18

//! GPIO Pin 19
#define GPIO_PIN_19             BIT_32_19

//! GPIO Pin 20
#define GPIO_PIN_20             BIT_32_20

//! GPIO Pin 21
#define GPIO_PIN_21             BIT_32_21

//! GPIO Pin 22
#define GPIO_PIN_22             BIT_32_22

//! GPIO Pin 23
#define GPIO_PIN_23             BIT_32_23

//! GPIO Pin 24
#define GPIO_PIN_24             BIT_32_24

//! GPIO Pin 25
#define GPIO_PIN_25             BIT_32_25

//! GPIO Pin 26
#define GPIO_PIN_26             BIT_32_26

//! GPIO Pin 27
#define GPIO_PIN_27             BIT_32_27

//! GPIO Pin 28
#define GPIO_PIN_28             BIT_32_28

//! GPIO Pin 29
#define GPIO_PIN_29             BIT_32_29

//! GPIO Pin 30
#define GPIO_PIN_30             BIT_32_30

//! GPIO Pin 31
#define GPIO_PIN_31             BIT_32_31

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

#define GPIO_PA0_PA0            ((unsigned long)0x00 << 0 )
#define GPIO_PA0_RD1            ((unsigned long)0x01 << 0 )
#define GPIO_PA0_UART3TX        ((unsigned long)0x02 << 0 )
#define GPIO_PA0_I2C1SDA        ((unsigned long)0x03 << 0 )
#define GPIO_PA1_PA1            ((unsigned long)0x00 << 2 )
#define GPIO_PA1_TD1            ((unsigned long)0x01 << 2 )
#define GPIO_PA1_UART3RX        ((unsigned long)0x02 << 2 )
#define GPIO_PA1_I2C1SCL        ((unsigned long)0x03 << 2 )
#define GPIO_PA2_PA2            ((unsigned long)0x00 << 4 )
#define GPIO_PA2_UART0TX        ((unsigned long)0x01 << 4 )
#define GPIO_PA2_AD_CH_7        ((unsigned long)0x02 << 4 )
#define GPIO_PA3_PA3            ((unsigned long)0x00 << 6 )
#define GPIO_PA3_UART0RX        ((unsigned long)0x01 << 6 )
#define GPIO_PA3_AD_CH_6        ((unsigned long)0x02 << 6 )
#define GPIO_PA4_PA4            ((unsigned long)0x00 << 8 )
#define GPIO_PA4_I2SRX_CLK      ((unsigned long)0x01 << 8 )
#define GPIO_PA4_RD2            ((unsigned long)0x02 << 8 )
#define GPIO_PA4_CAP2_0         ((unsigned long)0x03 << 8 )
#define GPIO_PA5_PA5            ((unsigned long)0x00 << 10)
#define GPIO_PA5_I2SRX_WS       ((unsigned long)0x01 << 10)
#define GPIO_PA5_TD2            ((unsigned long)0x02 << 10)
#define GPIO_PA5_CAP2_1         ((unsigned long)0x03 << 10)
#define GPIO_PA6_PA6            ((unsigned long)0x00 << 12)
#define GPIO_PA6_I2SRX_SDA      ((unsigned long)0x01 << 12)
#define GPIO_PA6_SSEL1          ((unsigned long)0x02 << 12)
#define GPIO_PA6_MAT2_0         ((unsigned long)0x03 << 12)
#define GPIO_PA7_PA7            ((unsigned long)0x00 << 14)
#define GPIO_PA7_I2STX_CLK      ((unsigned long)0x01 << 14)
#define GPIO_PA7_SCK1           ((unsigned long)0x02 << 14)
#define GPIO_PA7_MAT2_1         ((unsigned long)0x03 << 14)
#define GPIO_PA8_PA8            ((unsigned long)0x00 << 16)
#define GPIO_PA8_I2STX_WS       ((unsigned long)0x01 << 16)
#define GPIO_PA8_MISO1          ((unsigned long)0x02 << 16)
#define GPIO_PA8_MAT2_2         ((unsigned long)0x03 << 16)
#define GPIO_PA9_PA9            ((unsigned long)0x00 << 18)
#define GPIO_PA9_I2STX_SDA      ((unsigned long)0x01 << 18)
#define GPIO_PA9_MOSI1          ((unsigned long)0x02 << 18)
#define GPIO_PA9_MAT2_3         ((unsigned long)0x03 << 18)
#define GPIO_PA10_PA10          ((unsigned long)0x00 << 20)
#define GPIO_PA10_UART2TX       ((unsigned long)0x01 << 20)
#define GPIO_PA10_I2C2SDA       ((unsigned long)0x02 << 20)
#define GPIO_PA10_MAT3_0        ((unsigned long)0x03 << 20)
#define GPIO_PA11_PA11          ((unsigned long)0x00 << 22)
#define GPIO_PA11_UART2RX       ((unsigned long)0x01 << 22)
#define GPIO_PA11_I2C2SCL       ((unsigned long)0x02 << 22)
#define GPIO_PA11_MAT3_1        ((unsigned long)0x03 << 22)
#define GPIO_PA15_PA15          ((unsigned long)0x00 << 30)
#define GPIO_PA15_UART1TX       ((unsigned long)0x01 << 30)
#define GPIO_PA15_SCK0          ((unsigned long)0x02 << 30)
#define GPIO_PA15_SCK           ((unsigned long)0x03 << 30)
#define GPIO_PA16_PA16          ((unsigned long)0x00 << 0 )
#define GPIO_PA16_UART1RX       ((unsigned long)0x01 << 0 )
#define GPIO_PA16_SSEL0         ((unsigned long)0x02 << 0 )
#define GPIO_PA16_SSEL          ((unsigned long)0x03 << 0 )
#define GPIO_PA17_PA17          ((unsigned long)0x00 << 2 )
#define GPIO_PA17_UART1CTS      ((unsigned long)0x01 << 2 )
#define GPIO_PA17_MISO0         ((unsigned long)0x02 << 2 )
#define GPIO_PA17_MISO          ((unsigned long)0x03 << 2 )
#define GPIO_PA18_PA18          ((unsigned long)0x00 << 4 )
#define GPIO_PA18_UART1DCD      ((unsigned long)0x01 << 4 )
#define GPIO_PA18_MOSI0         ((unsigned long)0x02 << 4 )
#define GPIO_PA18_MOSI          ((unsigned long)0x03 << 4 )
#define GPIO_PA19_PA19          ((unsigned long)0x00 << 6 )
#define GPIO_PA19_UART1DSR      ((unsigned long)0x01 << 6 )
#define GPIO_PA19_I2C1SDA       ((unsigned long)0x03 << 6 )
#define GPIO_PA20_PA20          ((unsigned long)0x00 << 8 )
#define GPIO_PA20_UART1DTR      ((unsigned long)0x01 << 8 )
#define GPIO_PA20_I2C1SCL       ((unsigned long)0x03 << 8 )
#define GPIO_PA21_PA21          ((unsigned long)0x00 << 10)
#define GPIO_PA21_UART1RI       ((unsigned long)0x01 << 10)
#define GPIO_PA21_RD1           ((unsigned long)0x03 << 10)
#define GPIO_PA22_PA22          ((unsigned long)0x00 << 12)
#define GPIO_PA22_UART1RTS      ((unsigned long)0x01 << 12)
#define GPIO_PA22_TD1           ((unsigned long)0x03 << 12)
#define GPIO_PA23_PA23          ((unsigned long)0x00 << 14)
#define GPIO_PA23_AD_CH_0       ((unsigned long)0x01 << 14)
#define GPIO_PA23_I2SRX_CLK     ((unsigned long)0x02 << 14)
#define GPIO_PA23_CAP3_0        ((unsigned long)0x03 << 14)
#define GPIO_PA24_PA24          ((unsigned long)0x00 << 16)
#define GPIO_PA24_AD_CH_1       ((unsigned long)0x01 << 16)
#define GPIO_PA24_I2SRX_WS      ((unsigned long)0x02 << 16)
#define GPIO_PA24_CAP3_1        ((unsigned long)0x03 << 16)
#define GPIO_PA25_PA25          ((unsigned long)0x00 << 18)
#define GPIO_PA25_AD_CH_2       ((unsigned long)0x01 << 18)
#define GPIO_PA25_I2SRX_SDA     ((unsigned long)0x02 << 18)
#define GPIO_PA25_UART3TX       ((unsigned long)0x03 << 18)
#define GPIO_PA26_PA26          ((unsigned long)0x00 << 20)
#define GPIO_PA26_AD_CH_3       ((unsigned long)0x01 << 20)
#define GPIO_PA26_AOUT          ((unsigned long)0x02 << 20)
#define GPIO_PA26_UART3RX       ((unsigned long)0x03 << 20)
#define GPIO_PA27_PA27          ((unsigned long)0x00 << 22)
#define GPIO_PA27_I2C0SDA       ((unsigned long)0x01 << 22)
#define GPIO_PA27_USB_SDA       ((unsigned long)0x02 << 22)
#define GPIO_PA28_PA28          ((unsigned long)0x00 << 28)
#define GPIO_PA28_I2C0SCL       ((unsigned long)0x01 << 28)
#define GPIO_PA28_USB_SCL       ((unsigned long)0x02 << 28)
#define GPIO_PA29_PA29          ((unsigned long)0x00 << 26)
#define GPIO_PA29_USB_D_P       ((unsigned long)0x01 << 26)
#define GPIO_PA30_PA30          ((unsigned long)0x00 << 28)
#define GPIO_PA30_USB_D_N       ((unsigned long)0x01 << 28)
#define GPIO_PB0_PB0            ((unsigned long)0x00 << 0 )
#define GPIO_PB0_ETH_TXD0       ((unsigned long)0x01 << 0 )
#define GPIO_PB1_PB1            ((unsigned long)0x00 << 2 )
#define GPIO_PB1_ETH_TXD1       ((unsigned long)0x01 << 2 )
#define GPIO_PB4_PB4            ((unsigned long)0x00 << 8 )
#define GPIO_PB4_ETH_TX_EN      ((unsigned long)0x01 << 8 )
#define GPIO_PB8_PB8            ((unsigned long)0x00 << 16)
#define GPIO_PB8_ETH_CRS        ((unsigned long)0x01 << 16)
#define GPIO_PB9_PB9            ((unsigned long)0x00 << 18)
#define GPIO_PB9_ETH_RXD0       ((unsigned long)0x01 << 18)
#define GPIO_PB10_PB10          ((unsigned long)0x00 << 20)
#define GPIO_PB10_ETH_RXD1      ((unsigned long)0x01 << 20)
#define GPIO_PB14_PB14          ((unsigned long)0x00 << 28)
#define GPIO_PB14_ETH_RX_ER     ((unsigned long)0x01 << 28)
#define GPIO_PB15_PB15          ((unsigned long)0x00 << 30)
#define GPIO_PB15_ETH_REF_CLK   ((unsigned long)0x01 << 30)
#define GPIO_PB16_PB16          ((unsigned long)0x00 << 0 )
#define GPIO_PB16_ENET_MDC      ((unsigned long)0x01 << 0 )
#define GPIO_PB17_PB17          ((unsigned long)0x00 << 2 )
#define GPIO_PB17_ENET_MDIO     ((unsigned long)0x01 << 2 )
#define GPIO_PB18_PB18          ((unsigned long)0x00 << 4 )
#define GPIO_PB18_USB_UP_LED    ((unsigned long)0x01 << 4 )
#define GPIO_PB18_PWM1_CH1      ((unsigned long)0x02 << 4 )
#define GPIO_PB18_CAP1_0        ((unsigned long)0x03 << 4 )
#define GPIO_PB19_PB19          ((unsigned long)0x00 << 6 )
#define GPIO_PB19_MCOA0         ((unsigned long)0x01 << 6 )
#define GPIO_PB19_USB_PPWR      ((unsigned long)0x02 << 6 )
#define GPIO_PB19_CAP1_1        ((unsigned long)0x03 << 6 )
#define GPIO_PB20_PB20          ((unsigned long)0x00 << 8 )
#define GPIO_PB20_MCI0          ((unsigned long)0x01 << 8 )
#define GPIO_PB20_PWM1_CH2      ((unsigned long)0x02 << 8 )
#define GPIO_PB20_SCK0          ((unsigned long)0x03 << 8 )
#define GPIO_PB21_PB21          ((unsigned long)0x00 << 10)
#define GPIO_PB21_MCABORT       ((unsigned long)0x01 << 10)
#define GPIO_PB21_PWM1_CH3      ((unsigned long)0x02 << 10)
#define GPIO_PB21_SSEL0         ((unsigned long)0x03 << 10)
#define GPIO_PB22_PB22          ((unsigned long)0x00 << 12)
#define GPIO_PB22_MCOB0         ((unsigned long)0x01 << 12)
#define GPIO_PB22_USB_PWRD      ((unsigned long)0x02 << 12)
#define GPIO_PB22_MAT1_0        ((unsigned long)0x03 << 12)
#define GPIO_PB23_PB23          ((unsigned long)0x00 << 14)
#define GPIO_PB23_MCI1          ((unsigned long)0x01 << 14)
#define GPIO_PB23_PWM1_CH4      ((unsigned long)0x02 << 14)
#define GPIO_PB23_MISO0         ((unsigned long)0x03 << 14)
#define GPIO_PB24_PB24          ((unsigned long)0x00 << 16)
#define GPIO_PB24_MCI2          ((unsigned long)0x01 << 16)
#define GPIO_PB24_PWM1_CH5      ((unsigned long)0x02 << 16)
#define GPIO_PB24_MOSI0         ((unsigned long)0x03 << 16)
#define GPIO_PB25_PB25          ((unsigned long)0x00 << 18)
#define GPIO_PB25_MCOA1         ((unsigned long)0x01 << 18)
#define GPIO_PB25_MAT1_1        ((unsigned long)0x03 << 18)
#define GPIO_PB26_PB26          ((unsigned long)0x00 << 20)
#define GPIO_PB26_MCOB1         ((unsigned long)0x01 << 20)
#define GPIO_PB26_PWM1_CH6      ((unsigned long)0x02 << 20)
#define GPIO_PB26_CAP0_0        ((unsigned long)0x03 << 20)
#define GPIO_PB27_PB27          ((unsigned long)0x00 << 22)
#define GPIO_PB27_CLKOUT        ((unsigned long)0x01 << 22)
#define GPIO_PB27_USB_OVRCR     ((unsigned long)0x02 << 22)
#define GPIO_PB27_CAP0_1        ((unsigned long)0x03 << 22)
#define GPIO_PB28_PB28          ((unsigned long)0x00 << 24)
#define GPIO_PB28_MCOA2         ((unsigned long)0x01 << 24)
#define GPIO_PB28_PWM_CAP_CH0   ((unsigned long)0x02 << 24)
#define GPIO_PB28_MAT0_0        ((unsigned long)0x03 << 24)
#define GPIO_PB29_PB29          ((unsigned long)0x00 << 26)
#define GPIO_PB29_MCOB2         ((unsigned long)0x01 << 26)
#define GPIO_PB29_PWM_CAP_CH1   ((unsigned long)0x02 << 26)
#define GPIO_PB29_MAT0_1        ((unsigned long)0x03 << 26)
#define GPIO_PB30_PB30          ((unsigned long)0x00 << 28)
#define GPIO_PB30_VBUS          ((unsigned long)0x02 << 28)
#define GPIO_PB30_AD_CH_4       ((unsigned long)0x03 << 28)
#define GPIO_PB31_PB31          ((unsigned long)0x00 << 30)
#define GPIO_PB31_SCK1          ((unsigned long)0x02 << 30)
#define GPIO_PB31_AD_CH_5       ((unsigned long)0x03 << 30)
#define GPIO_PC0_PC0            ((unsigned long)0x00 << 0 )
#define GPIO_PC0_PWM1_CH1       ((unsigned long)0x01 << 0 )
#define GPIO_PC0_UART1TX        ((unsigned long)0x02 << 0 )
#define GPIO_PC1_PC1            ((unsigned long)0x00 << 2 )
#define GPIO_PC1_PWM1_CH2       ((unsigned long)0x01 << 2 )
#define GPIO_PC1_UART1RX        ((unsigned long)0x02 << 2 )
#define GPIO_PC2_PC2            ((unsigned long)0x00 << 4 )
#define GPIO_PC2_PWM1_CH3       ((unsigned long)0x01 << 4 )
#define GPIO_PC2_UART1CTS       ((unsigned long)0x02 << 4 )
#define GPIO_PC3_PC3            ((unsigned long)0x00 << 6 )
#define GPIO_PC3_PWM1_CH4       ((unsigned long)0x01 << 6 )
#define GPIO_PC3_UART1DCD       ((unsigned long)0x02 << 6 )
#define GPIO_PC4_PC4            ((unsigned long)0x00 << 8 )
#define GPIO_PC4_PWM1_CH5       ((unsigned long)0x01 << 8 )
#define GPIO_PC4_UART1DSR       ((unsigned long)0x02 << 8 )
#define GPIO_PC5_PC5            ((unsigned long)0x00 << 10)
#define GPIO_PC5_PWM1_CH6       ((unsigned long)0x01 << 10)
#define GPIO_PC5_UART1DTR       ((unsigned long)0x02 << 10)
#define GPIO_PC6_PC6            ((unsigned long)0x00 << 12)
#define GPIO_PC6_PWM_CAP_CH0    ((unsigned long)0x01 << 12)
#define GPIO_PC6_UART1RI        ((unsigned long)0x02 << 12)
#define GPIO_PC7_PC7            ((unsigned long)0x00 << 14)
#define GPIO_PC7_RD2            ((unsigned long)0x01 << 14)
#define GPIO_PC7_UART1RTS       ((unsigned long)0x02 << 14)
#define GPIO_PC8_PC8            ((unsigned long)0x00 << 16)
#define GPIO_PC8_TD2            ((unsigned long)0x01 << 16)
#define GPIO_PC8_UART2TX        ((unsigned long)0x02 << 16)
#define GPIO_PC8_ENET_MDC       ((unsigned long)0x03 << 16)
#define GPIO_PC9_PC9            ((unsigned long)0x00 << 18)
#define GPIO_PC9_USB_CONNECT    ((unsigned long)0x01 << 18)
#define GPIO_PC9_UART2RX        ((unsigned long)0x02 << 18)
#define GPIO_PC9_ENET_MDIO      ((unsigned long)0x03 << 18)
#define GPIO_PC10_PC10          ((unsigned long)0x00 << 20)
#define GPIO_PC10_EINT0         ((unsigned long)0x01 << 20)
#define GPIO_PC10_NMI           ((unsigned long)0x02 << 20)
#define GPIO_PC11_PC11          ((unsigned long)0x00 << 22)
#define GPIO_PC11_EINT1         ((unsigned long)0x01 << 22)
#define GPIO_PC11_I2STX_CLK     ((unsigned long)0x03 << 22)
#define GPIO_PC12_PC12          ((unsigned long)0x00 << 24)
#define GPIO_PC12_EINT2         ((unsigned long)0x01 << 24)
#define GPIO_PC12_I2STX_WS      ((unsigned long)0x03 << 24)
#define GPIO_PC13_PC13          ((unsigned long)0x00 << 26)
#define GPIO_PC13_EINT3         ((unsigned long)0x01 << 26)
#define GPIO_PC13_I2STX_SDA     ((unsigned long)0x03 << 26)
#define GPIO_PD25_PD25          ((unsigned long)0x00 << 18)
#define GPIO_PD25_MAT0_0        ((unsigned long)0x02 << 18)
#define GPIO_PD25_PWM1_CH2      ((unsigned long)0x03 << 18)
#define GPIO_PD26_PD26          ((unsigned long)0x00 << 20)
#define GPIO_PD26_STCLK         ((unsigned long)0x01 << 20)
#define GPIO_PD26_MAT0_1        ((unsigned long)0x02 << 20)
#define GPIO_PD26_PWM1_CH3      ((unsigned long)0x03 << 20)
#define GPIO_PE28_PE28          ((unsigned long)0x00 << 24)
#define GPIO_PE28_RX_MCLK       ((unsigned long)0x01 << 24)
#define GPIO_PE28_MAT2_0        ((unsigned long)0x02 << 24)
#define GPIO_PE28_UART3TX       ((unsigned long)0x03 << 24)
#define GPIO_PE29_PE29          ((unsigned long)0x00 << 26)
#define GPIO_PE29_TX_MCLK       ((unsigned long)0x01 << 26)
#define GPIO_PE29_MAT2_1        ((unsigned long)0x02 << 26)
#define GPIO_PE29_UART3RX       ((unsigned long)0x03 << 26)

#define PIN_MODE_OD_DIS         (BIT_32_6                       )
#define PIN_MODE_OD_EN          (BIT_32_6 | BIT_32_5            )
#define PIN_MODE_PULL_UP        (BIT_32_4                       )
#define PIN_MODE_REPEATER       (BIT_32_4 | BIT_32_2            )
#define PIN_MODE_NONE           (BIT_32_4 | BIT_32_3            )
#define PIN_MODE_PULL_DOWN      (BIT_32_4 | BIT_32_3 | BIT_32_2 )
#define PIN_MODE_INPUT          (BIT_32_1                       )
#define PIN_MODE_OUTPUT         (BIT_32_1 | BIT_32_0            )

#define INT_TYPE_RISING         BIT_32_0
#define INT_TYPE_FALLING        BIT_32_1


extern unsigned long  GPIOPinToPeripheralId(unsigned long ulPort, unsigned long ulPin);
extern unsigned long  GPIOPinToPort(unsigned long ulPort, unsigned long ulPin);
extern unsigned long  GPIOPinToPin(unsigned long ulPort, unsigned long ulPin);

extern void          GPIOPinFunCfg(unsigned long ulPort, unsigned long ulPin, unsigned long ulCfg);
extern void          GPIOPinModeCfg(unsigned long ulPort, unsigned long ulPin, unsigned long ulCfg);
extern void          GPIOPinSet(unsigned long ulPort, unsigned long ulPins);
extern void          GPIOPinClr(unsigned long ulPort, unsigned long ulPins);
extern void          GPIOPinWrite(unsigned long ulPort, unsigned long ulPins, unsigned long ulVal);
extern unsigned long GPIOPinRead(unsigned long ulPort, unsigned long ulPin);
extern unsigned long GPIOPortRead(unsigned long ulPort);
extern void          GPIOPortWrite(unsigned long ulPort, unsigned long ulVal);
extern void          GPIOPinIntCfg(unsigned long ulPort, unsigned long ulPin, unsigned long ulCfg);
extern unsigned long GPIOPinIntFlagGet(unsigned long ulPort, unsigned long ulPin);
extern void          GPIOPinIntFlagClear(unsigned long ulPort, unsigned long ulPin);
extern void          GPIOPinIntEnable(unsigned long ulPort, unsigned long ulPin);
extern void          GPIOPinIntDisable(unsigned long ulPort, unsigned long ulPin);


#endif


// For write n(>=1) pin with the same value, please use GPIOPinSet/GPIOPinClr
// For Write pin value in parial, please use GPIOPinWrite
// For control whole GPIO, please use GPIOPortWrite.

