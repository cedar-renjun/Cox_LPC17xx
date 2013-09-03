//*****************************************************************************
//
//! \file xpwm.h
//! \brief Prototypes for the PWM Driver.
//! \version V2.2.1.0
//! \date 6/25/2012
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

#ifndef __xPWM_H__
#define __xPWM_H__

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
//! \addtogroup PWM
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xPWM
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xPWM_Int_Type xPWM Interrupt Type
//! \brief      Values that show xPWM Interrupt Type
//! 
//! \section    xPWM_Int_Type_Section 1. Where to use this group
//!             Values that can be passed to 
//!             \ref xPWMIntEnable()
//!             \ref xPWMIntDisable()
//!             as the ulIntType parameter. 
//! 
//! \section    xPWM_Int_Type_CoX     2. CoX Mandatory and CoX Non-mandatory 
//! \verbatim
//! +--------------------------+----------------+-----------+
//! |  xPWM Interrupt Source   |       CoX      |  LPC17xx  |
//! |--------------------------|----------------|-----------|
//! |  xPWM_INT_PWM            |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! PWM channels Interrupt 
//
#define xPWM_INT_PWM            0x00000002

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xPWM_Event_Type xPWM Event Type
//! \brief      Values that show xPWM Event Type
//! \n
//! \section    xPWM_Event_Type_Section 1. Where to use this group
//!             PWM Event/Error Flag, Used by IntHandle's Event Callback
//!             Function as ulEvent parameter. User Callback function can
//!             use this to detect what event happened. 
//!
//! \section    xPWM_Event_Type_CoX     2. CoX Mandatory and CoX Non-mandatory 
//! \verbatim
//! +--------------------------+----------------+-----------+
//! |  xPWM Event Source       |       CoX      |  LPC17xx  |
//! |--------------------------|----------------|-----------|
//! |  xPWM_EVENT_PWM          |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWM_EVENT_CAP          |  non-Mandatory |     Y     |
//! |--------------------------|----------------|-----------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! The Interrupt event is PWM
//
#define xPWM_EVENT_PWM      

//
//! The Interrupt event is input capture.
//
#define xPWM_EVENT_CAP   

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xPWM_Freq_Config xPWM Frequency Configure
//! \brief      xPWM Frequency Configure
//! \n
//! \section    xPWM_Event_Type_Section 1. Where to use this group
//!             Values that can be passed to xPWMFrequencyConfig() as
//!             ulConfig parameter. 
//! 
//! \section    xPWM_Event_Type_CoX     2. CoX Mandatory and CoX Non-mandatory 
//! \verbatim
//! +--------------------------+----------------+-----------+
//! |  xPWM Freq Config        |       CoX      |  LPC17xx  |
//! |--------------------------|----------------|-----------|
//! |  xPWM_FREQ_CONFIG(a,b,c) |    Mandatory   |     Y     |
//! |  a is The Divider value  |                |           |
//! |  b is The PreScale value |                |           |
//! |  c is PWM Counter value  |                |           |
//! |--------------------------|----------------|-----------|
//! \endverbatim
//! @{
//
//*****************************************************************************

#define xPWM_FREQ_CONFIG(a,b,c) (1 | b | (c<<16))

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xPWM_Channel xPWM Channel
//! \brief      Values that show xPWM Channel
//! 
//! \section    xPWM_Channel_Section 1. Where to use this group
//!             Values that can be passed to all the function in xpwm.c as the
//!             ulChannel parameter. 
//!
//!             PWM Event Channel Flag, Used by IntHandle's Event Callback
//!             Function as ulMsgParam parmeter. User Callback function can
//!             use this to detect what Channel event happened. 
//!
//! \section    xPWM_Channel_CoX 2.CoX Mandatory and CoX Non-mandatory 
//! \verbatim
//! +-----------------------+----------------+---------------+
//! | xPWM Channel Number   |       CoX      |     LPC17xx   |
//! |-----------------------|----------------|---------------|
//! | xPWM_CHANNEL$x$       |  Non-Mandatory | xPWM_CHANNEL0 |
//! |                       |                | ------------- |
//! |                       |                | xPWM_CHANNEL1 |
//! |                       |                | ------------- |
//! |                       |                | xPWM_CHANNEL2 |
//! |                       |                | ------------- |
//! |                       |                | xPWM_CHANNEL3 |
//! |                       |                | ------------- |
//! |                       |                | xPWM_CHANNEL4 |
//! |                       |                | ------------- |
//! |                       |                | xPWM_CHANNEL5 |
//! |                       |                | ------------- |
//! |                       |                | xPWM_CHANNEL6 |
//! |-----------------------|----------------|---------------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Channel 0
//
#define xPWM_CHANNEL0           PWM_CHANNEL0

//
//! Channel 1
//
#define xPWM_CHANNEL1           PWM_CHANNEL1

//
//! Channel 2
//
#define xPWM_CHANNEL2           PWM_CHANNEL2

//
//! Channel 3
//
#define xPWM_CHANNEL3           PWM_CHANNEL3

//
//! Channel 4
//
#define xPWM_CHANNEL4           PWM_CHANNEL4 

//
//! Channel 5
//
#define xPWM_CHANNEL5           PWM_CHANNEL5 

//
//! Channel 6
//
#define xPWM_CHANNEL6           PWM_CHANNEL6 

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xPWM_Config xPWM Configuration
//! \brief      Values that show xPWM Configuration
//!             Values that can be passed to PWMConfigure().
//!
//! \section    xPWM_Config_Section 1. Where to use this group
//!             Values that can be passed to xPWMInitConfigure() as the ulConfig
//!             parameter. 
//!
//! \section    xPWM_Config_CoX 2.CoX Mandatory and CoX Non-mandatory 
//! \verbatim
//! +--------------------------+----------------+-----------+
//! | xPWM Config              |       CoX      |  LPC17xx  |
//! |--------------------------|----------------|-----------|
//! | xPWM_ONE_SHOT_MODE       |    Mandatory   |     N     |
//! |--------------------------|----------------|-----------|
//! | xPWM_TOGGLE_MODE         |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! | xPWM_OUTPUT_INVERTER_EN  |  Non-Mandatory |     Y     |
//! |--------------------------|----------------|-----------|
//! | xPWM_OUTPUT_INVERTER_DIS |  Non-Mandatory |     Y     |
//! |--------------------------|----------------|-----------|
//! | xPWM_DEAD_ZONE_EN        |  Non-Mandatory |     N     |
//! |--------------------------|----------------|-----------|
//! | xPWM_DEAD_ZONE_DIS       |  Non-Mandatory |     N     |
//! |--------------------------|----------------|-----------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! One-Shot Mode
//
#define xPWM_ONE_SHOT_MODE     

//
//! Auto-reload Mode
//
#define xPWM_TOGGLE_MODE       

//
//! Inverter enable
//
#define xPWM_OUTPUT_INVERTER_EN

//
//! Inverter disable
//
#define xPWM_OUTPUT_INVERTER_DIS

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xPWM_Exported_APIs xPWM APIs
//! \brief      xPWM API Reference.
//! \verbatim
//! +--------------------------+----------------+-----------+
//! |  xPWM API                |       CoX      |  LPC17xx  |
//! |--------------------------|----------------|-----------|
//! |  xPWMInitConfigure       |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMFrequencySet        |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMFrequencyConfig     |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMFrequencyGet        |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMOutputEnable        |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMOutputDisable       |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMStart               |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMStop                |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMDutySet             |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMDutyGet             |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMIntEnable           |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMIntDisable          |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMIntFlagGet          |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! |  xPWMIntCallbackInit     |    Mandatory   |     Y     |
//! |--------------------------|----------------|-----------|
//! \endverbatim
//! @{
//
//*****************************************************************************


//*****************************************************************************
//
//! \brief  Initialize and configure the PWM module. 
//!         This function is to initialize and configure channel of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//!
//! \param  [in] ulConfig is the configuration of PWM channel.
//!
//! \return None.
//
//*****************************************************************************
#define xPWMInitConfigure(ulBase, ulChannel, ulConfig)
  
//*****************************************************************************
//
//! \brief  Set the PWM frequency of the PWM module. 
//!         This function is to set the PWM frequency of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//! \param  [in] ulFrequency is the PWM frequency of PWM channel.
//!
//! \return The Actual Frequency of PWM Module.
//
//*****************************************************************************
#define xPWMFrequencySet(ulBase, ulChannel, ulFrequency)                      

//*****************************************************************************
//
//! \brief  Set the PWM frequency of the PWM module. 
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//! \param  [in] ulConfig is the configuration of PWM channel's frequency.
//!
//! \return the Actual Frequency of PWM.
//
//*****************************************************************************
#define xPWMFrequencyConfig(ulBase, ulChannel, ulConfig)                      

//*****************************************************************************
//
//! \brief  Get the PWM frequency of the PWM module. 
//!         This function is to get the PWM frequency of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//!
//! \return The Actual Frequency of PWM.
//
//*****************************************************************************
#define xPWMFrequencyGet(ulBase, ulChannel)                                  

//*****************************************************************************
//
//! \brief  Enable the PWM output of the PWM module. 
//!         This function is to enable the PWM output of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//!
//! \return None.
//
//*****************************************************************************
#define xPWMOutputEnable(ulBase, ulChannel)                                   

//*****************************************************************************
//
//! \brief  Disable the PWM output of the PWM module. 
//!         This function is to disable the PWM output of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//!
//! \return None.
//
//*****************************************************************************
#define xPWMOutputDisable(ulBase, ulChannel)                                  

//*****************************************************************************
//
//! \brief  Start the PWM of the PWM module. 
//!         This function is to start the PWM of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//!
//! \return None.
//
//*****************************************************************************
#define xPWMStart(ulBase, ulChannel)                                          

//*****************************************************************************
//
//! \brief  Stop the PWM of the PWM module. 
//!         This function is to stop the PWM of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//!
//! \return None.
//
//*****************************************************************************  
#define xPWMStop(ulBase, ulChannel) 

//*****************************************************************************
//
//! \brief  Set the PWM duty of the PWM module. 
//!         This function is to set the PWM duty of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//! \param  [in] ulDuty is the duty of PWM channel.
//!
//! \return None.
//!
//! \note   Duty should not be 0;
//
//*****************************************************************************
#define xPWMDutySet(ulBase, ulChannel, ulDuty) 

//*****************************************************************************
//
//! \brief  Get the PWM duty of the PWM module. 
//!
//!         This function is to get the PWM duty of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//!
//! \return the Actual duty of PWM.
//
//*****************************************************************************
#define xPWMDutyGet(ulBase, ulChannel)  

//*****************************************************************************
//
//! \brief  Enable the PWM interrupt of the PWM module. 
//!         This function is to enable the PWM interrupt of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//! \param  [in] ulIntType is the PWM channel interrupt type.
//!
//! \return None.
//
//*****************************************************************************
#define xPWMIntEnable(ulBase, ulChannel, ulIntType)                           

//*****************************************************************************
//
//! \brief  Disable the PWM interrupt of the PWM module. 
//!         This function is to disable the PWM interrupt of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//! \param  [in] ulIntType is the PWM channel interrupt type.
//!
//! \return None.
//
//***************************************************************************** 
#define xPWMIntDisable(ulBase, ulChannel, ulIntType)

//*****************************************************************************
//
//! \brief  Get the PWM interrupt flag of the PWM module. 
//!         This function is to get the PWM interrupt flag of the PWM module.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] ulChannel is the PWM channel.
//! \param  [in] ulIntType is the PWM channel interrupt type.
//!
//! \return None.
//
//*****************************************************************************
#define xPWMIntFlagGet(ulBase, ulChannel, ulIntType) 

//*****************************************************************************
//
//! \brief  Init interrupts callback for the PWM timer.
//!         This function is to init interrupts callback for the PWM timer.
//!
//! \param  [in] ulBase is the base address of the PWM port.
//!              For LPC17xx, This value can must be \ref xPWM0_BASE.
//!
//! \param  [in] xtPortCallback is callback for the PWM timer.
//!
//! \return None.
//
//*****************************************************************************
#define xPWMIntCallbackInit(ulBase, xtPWMCallback) 
        
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
//! \addtogroup  LPC17xx _PWM
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup  LPC17xx _PWM_Int_Type  LPC17xx  PWM Interrupt Type
//! Values that can be passed to PWMIntEnable(), PWMIntDisable(), PWMIntFlagGet(),
//! PWMIntFlagClear().
//! @{
//
//*****************************************************************************

#define PWM_CH_0                BIT_32_0           
#define PWM_CH_1                BIT_32_1           
#define PWM_CH_2                BIT_32_2           
#define PWM_CH_3                BIT_32_3           
#define PWM_CH_4                BIT_32_4           
#define PWM_CH_5                BIT_32_5           
#define PWM_CH_6                BIT_32_6
#define PWM_CAP_0               BIT_32_7           
#define PWM_CAP_1               BIT_32_8                 

#define PWM_INT_CH_0            BIT_32_0           
#define PWM_INT_CH_1            BIT_32_1           
#define PWM_INT_CH_2            BIT_32_2           
#define PWM_INT_CH_3            BIT_32_3           
#define PWM_INT_CH_4            BIT_32_8           
#define PWM_INT_CH_5            BIT_32_9           
#define PWM_INT_CH_6            BIT_32_10           
#define PWM_INT_CAP_0           BIT_32_4           
#define PWM_INT_CAP_1           BIT_32_5           

#define PWM_MATCH_INT_EN        BIT_32_0         
#define PWM_MATCH_INT_DIS       BIT_32_8         
#define PWM_MATCH_RESET_EN      BIT_32_1         
#define PWM_MATCH_RESET_DIS     BIT_32_9         
#define PWM_MATCH_STOP_EN       BIT_32_2         
#define PWM_MATCH_STOP_DIS      BIT_32_10  


#define PWM_EDGE_DOUBLE         BIT_32_0
#define PWM_EDGE_SINGLE         BIT_32_1


#define CH0_FALLING_SAMPLE_EN   BIT_32_0
#define CH0_FALLING_SAMPLE_DIS  BIT_32_8
#define CH0_RISING_SAMPLE_EN    BIT_32_1
#define CH0_RISING_SAMPLE_DIS   BIT_32_9
#define CH0_EDGE_EVENT_INT_EN   BIT_32_2
#define CH0_EDGE_EVENT_INT_DIS  BIT_32_10
#define CH1_FALLING_SAMPLE_EN   BIT_32_3
#define CH1_FALLING_SAMPLE_DIS  BIT_32_11
#define CH1_RISING_SAMPLE_EN    BIT_32_4
#define CH1_RISING_SAMPLE_DIS   BIT_32_12
#define CH1_EDGE_EVENT_INT_EN   BIT_32_5
#define CH1_EDGE_EVENT_INT_DIS  BIT_32_13

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup  LPC17xx _PWM_Exported_APIs  LPC17xx  PWM API
//! \brief  LPC17xx  PWM API Reference.
//! @{
//
//*****************************************************************************

extern void PWMIntStatusClear(unsigned long ulBase, unsigned long ulIntFlags);
extern unsigned long PWMIntStatusGet(unsigned long ulBase);
extern xtBoolean PWMIntStatusCheck(unsigned long ulBase, unsigned long ulIntFlags);
extern void PWMCounterEnable(unsigned long ulBase);
extern void PWMCounterDisable(unsigned long ulBase);
extern void PWMCounterReset(unsigned long ulBase);
extern void PWMMatchCfg(unsigned long ulBase, unsigned long ulCh, unsigned long ulCfg);
extern void PWMMatchUpdate(unsigned long ulBase, unsigned long ulCh, unsigned long ulValue);
extern void PWMOutPutEnable(unsigned long ulBase, unsigned long ulChs);
extern void PWMOutPutDisable(unsigned long ulBase, unsigned long ulChs);
extern void PWMEdgeCfg(unsigned long ulBase, unsigned long ulChs, unsigned long ulCfg);
extern void PWMCapCfg(unsigned long ulBase, unsigned long ulCfg);

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

#endif // __xPWM_H__
