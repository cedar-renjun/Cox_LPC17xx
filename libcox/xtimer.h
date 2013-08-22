//*****************************************************************************
//
//! \file xtimer.h
//! \brief Prototypes for the TIMER Driver.
//! \version V2.2.1.0
//! \todo Need to update time information.
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

#ifndef __xTIMER_H__
#define __xTIMER_H__

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
//! \addtogroup TIMER
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xTIMER
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xTIMER_Mode_Type xTIMER Mode Type 
//! \brief      Values that show xTIMER Mode Type.
//! \n
//! \section    xTIMER_Mode_Type_Section 1. Where to use this group
//!             Values that can be passed to xTimerInitConfig() as the ulConfig
//!             parameter.
//! \n
//! \section    xTIMER_Mode_Type_CoX 2. CoX Port Details 
//! \verbatim
//!     +--------------------------+----------------+------------------------+
//!     |  xTIMER Mode             |       CoX      |         LPC17xx        |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_MODE_ONESHOT     |    Mandatory   |            Y           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_MODE_PERIODIC    |    Mandatory   |            Y           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_MODE_TOGGLE      |    Mandatory   |            Y           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_MODE_CONTINUOUS  |  Non-Mandatory |            N           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_MODE_CAPTURE     |  Non-Mandatory |            Y           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_MODE_PWM         |  Non-Mandatory |            Y           |
//!     +--------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! The timer is operating at the one-shot mode.
//
#define xTIMER_MODE_ONESHOT     

//
//! The timer is operating at the periodic mode.
//
#define xTIMER_MODE_PERIODIC    

//
//! The timer is operating at the toggle mode.
//
#define xTIMER_MODE_TOGGLE      

//
//! The timer is operating at continuous counting mode.
//
#define xTIMER_MODE_CONTINUOUS 

//
//! The timer is operating as capture.
//
#define xTIMER_MODE_CAPTURE   

//
//! The timer is operating at PWM mode.
//
#define xTIMER_MODE_PWM      


//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xTIMER_INT_Type xTIMER Interrupt
//! \brief      Values that show xTIMER Interrupt Type.
//! \n
//! \section    xTIMER_INT_Type_Section 1. Where to use this group
//!             Values that can be passed to
//!             \b \ref xTimerIntEnable(),
//!             \b \ref xTimerIntDisable() ,
//!             \b \ref xTimerIntStatus(),
//!             \b \ref xTimerIntClear() as ulIntFlags. 
//! \n
//! \section    xTIMER_INT_Type_CoX 2. CoX Port Details 
//! \verbatim
//!     +--------------------------+----------------+------------------------+
//!     |  xTIMER Interrupt        |       CoX      |         LPC17xx        |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_INT_MATCH        |    Mandatory   |            Y           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_INT_CAP_EVENT    |    Mandatory   |            Y           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_INT_CAP_MATCH    |  Non-Mandatory |            N           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_INT_OVERFLOW     |  Non-Mandatory |            N           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_INT_PWM          |  Non-Mandatory |            N           |
//!     +--------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Timer match interrupt.
//
#define xTIMER_INT_MATCH        

//
//! Timer External capture event interrupt.
//
#define xTIMER_INT_CAP_EVENT    

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xTIMER_INT_Event xTIMER Interrupt Event
//! \brief      Values that show xTIMER Interrupt Events.
//! \n
//! \section    xTIMER_INT_Type_Section 1. Where to use this group
//!             Values that can be passed to
//!             \b \ref xTimerIntEnable(),
//!             \b \ref xTimerIntDisable() ,
//!             \b \ref xTimerIntStatus(),
//!             \b \ref xTimerIntClear() as ulIntFlags. 
//! \n
//! \section    xTIMER_INT_Event_CoX 2. CoX Port Details 
//! \verbatim
//!     +--------------------------+----------------+------------------------+
//!     |  xTIMER Interrupt        |       CoX      |         LPC17xx        |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_EVENT_MATCH      |    Mandatory   |            Y           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_EVENT_CAP_EVENT  |    Mandatory   |            Y           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_EVENT_CAP_MATCH  |  Non-Mandatory |            N           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_EVENT_OVERFLOW   |  Non-Mandatory |            N           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_EVENT_PWM        |  Non-Mandatory |            N           |
//!     +--------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Timer match interrupt.
//
#define xTIMER_EVENT_MATCH

//
//! Timer External capture event interrupt.
//
#define xTIMER_EVENT_CAP_EVENT

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xTIMER_Counter_Pase_Type xTIMER Counter Pase Type
//! \brief      Values that show xTIMER Counter Pase Type.
//! \n
//! \section    xTIMER_Counter_Pase_Type_Section 1. Where to use this group
//!             Values that can be passed to xTimerCounterDetectPhaseSelect() as
//!             ulPhase. 
//! \n
//! \section    xTIMER_Counter_Pase_Type_CoX 2. CoX Port Details 
//! \verbatim
//!     +----------------------------+----------------+------------------------+
//!     |  xTIMER Counter Pase Type  |       CoX      |         LPC17xx        |
//!     |----------------------------|----------------|------------------------|
//!     |  xTIMER_COUNTER_RISING     |    Mandatory   |            Y           |
//!     |----------------------------|----------------|------------------------|
//!     |  xTIMER_COUNTER_FALLING    |    Mandatory   |            Y           |
//!     +----------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! A rising edge of external count pin will be counted.
//
#define xTIMER_COUNTER_RISING   

//
//! A falling edge of external count pin will be counted.
//
#define xTIMER_COUNTER_FALLING  

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xTIMER_Capture_Edge_Type xTIMER Capture Edge Type
//! \brief      Values that show xTIMER Capture Edge Type.
//! \n
//! \section    xTIMER_Capture_Edge_Type_Section 1. Where to use this group
//!             Values that can be passed to xTimerCaptureEdgeSelect() as ulPhase.
//! \n
//! \section    xTIMER_Capture_Edge_Type_CoX 2. CoX Port Details 
//! \verbatim
//!     +--------------------------+----------------+------------------------+
//!     |  xTIMER Capture Edge     |       CoX      |         LPC17xx        |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_CAP_RISING       |    Mandatory   |            Y           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_CAP_FALLING      |    Mandatory   |            Y           |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_CAP_BOTH         |  Non-Mandatory |            Y           |
//!     +--------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! a 0 to 1 transition on TEX will be detected.
//
#define xTIMER_CAP_RISING       

//
//! a 1 to 0 transition on TEX will be detected.
//
#define xTIMER_CAP_FALLING     

//
//! Any transition on TEX will be detected.
//
#define xTIMER_CAP_BOTH       

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xTIMER_Capture_Mode_Type xTIMER Capture Mode Type
//! \brief      Values that show xTIMER Capture Mode Type.
//! \n
//! \section    xTIMER_Capture_Mode_Type_Section 1. Where to use this group
//!             Values that can be passed to xTimerCaptureEdgeSelect() as ulPhase.
//! \n
//! \section    xTIMER_Capture_Mode_Type_CoX 2. CoX Port Details 
//! \verbatim
//!     +------------------------+----------------+------------------------+
//!     |  xTIMER Capture Mode   |       CoX      |         LPC17xx        |
//!     |------------------------|----------------|------------------------|
//!     |  xTIMER_CAP_MODE_CAP   |    Mandatory   |            Y           |
//!     |------------------------|----------------|------------------------|
//!     |  xTIMER_CAP_MODE_RST   |  Non-Mandatory |            N           |
//!     +------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! TEX transition is using as the timer capture function.
//
#define xTIMER_CAP_MODE_CAP     0x00000001


//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xTIMER_Count_Direction xTIMER Count Direction
//! \brief      Values that show the count direction
//! \n
//! \section    xTIMER_Count_Direction_Section 1. Where to use this group
//!             Values that can be passed to xTimerInitConfig() as ulConfig.
//! \n
//! \section    xTIMER_Count_Direction_CoX 2. CoX Port Details 
//! \verbatim
//!     +--------------------------+-----------------+------------------------+
//!     |  xTIMER Count Direction  |       CoX       |         LPC17xx        |
//!     |--------------------------|-----------------|------------------------|
//!     |  xTIMER_COUNT_UP         |    Mandatory    |            Y           |
//!     |--------------------------|-----------------|------------------------|
//!     |  xTIMER_COUNT_DOWN       |  Non-Mandatory  |            N           |
//!     +--------------------------+-----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! the timer counts up.
//
#define xTIMER_COUNT_UP

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xTIMER_Count_Channel xTIMER Count Channel
//! \brief      Timer Count Channels.
//! \n
//! \section    xTIMER_Count_Channel_Section 1. Where to use this group
//!             Values that can be passed to all xAPI as ulChannel.
//! \n
//! \section    xTIMER_Count_Channel_CoX 2. CoX Port Details 
//! \verbatim
//!     +--------------------------+----------------+------------------------+
//!     |  xTIMER Count Channel    |       CoX      |         LPC17xx        |
//!     |--------------------------|----------------|------------------------|
//!     |  xTIMER_CHANNELn         |    Mandatory   |     xTIMER_CHANNEL0    |
//!     |                          |                |     xTIMER_CHANNEL1    |
//!     |                          |                |     xTIMER_CHANNEL2    |
//!     |                          |                |     xTIMER_CHANNEL3    |  
//!     +--------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Channel 0.
//
#define xTIMER_CHANNEL0         
  
//
//! Channel 1.
//
#define xTIMER_CHANNEL1        
  
//
//! Channel 2.
//
#define xTIMER_CHANNEL2       
  
//
//! Channel 3.
//
#define xTIMER_CHANNEL3      

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xTIMER_Exported_APIs xTIMER API
//! \brief      xTIMER API Reference.
//!
//! \section    xTIMER_Exported_APIs_Port CoX Port Details
//! \verbatim
//!     +--------------------------------+----------------+------------------+
//!     |  xTIMER API                    |       CoX      |      LPC17xx     |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerInitConfig              |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerStart                   |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerStop                    |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerCounterEnable           |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerCounterDisable          |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerCaptureModeSet          |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerPrescaleSet             |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerPrescaleGet             |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerLoadSet                 |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerLoadGet                 |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerValueGet                |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerMatchSet                |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerMatchGet                |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerIntCallbackInit         |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerIntEnable               |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerIntDisable              |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerStatusGet               |    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerCounterDetectPhaseSelect|    Mandatory   |         Y        |
//!     |--------------------------------|----------------|------------------|
//!     |  xTimerCaptureEdgeSelect       |    Mandatory   |         Y        |
//!     +--------------------------------+----------------+------------------+
//! \endverbatim
//!
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief Configurate The Timer's mode and tick frequency. 
//!
//!        This function is to configurate The Timer's mode and tick frequency.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//! \param [in] ulConfig is the mode Configuratation of the Timer port.
//! \param [in] ulTickFreq is the tick frequency of the Timer port.
//!
//! \return None.
//
//*****************************************************************************
extern void xTimerInitConfig(unsigned long ulBase, 
     unsigned long ulChannel, unsigned long ulConfig, unsigned long ulTickFreq);

//*****************************************************************************
//
//! \brief Start The Timer counter. 
//!        This function is to start The Timer counter.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//!
//! \return None.
//
//*****************************************************************************
#define xTimerStart(ulBase, ulChannel)                                        

//*****************************************************************************
//
//! \brief Stop The Timer counter. 
//!        This function is to stop The Timer counter.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//!
//! \return None.
//
//*****************************************************************************
#define xTimerStop(ulBase, ulChannel)     

//*****************************************************************************
//
//! \brief Enable The Timer counter as a counter. 
//!        This function is to enable The Timer counter as a counter.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//!
//! \return None.
//
//*****************************************************************************
#define xTimerCounterEnable(ulBase, ulChannel)

//*****************************************************************************
//
//! \brief Disable The Timer counter as a counter. 
//!        This function is to disable The Timer counter as a counter.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//!
//! \return None.
//
//*****************************************************************************
#define xTimerCounterDisable(ulBase, ulChannel)

//*****************************************************************************
//
//! \brief Enable The Timer counter as a capture. 
//!        This function is to enable The Timer counter as a capture.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//! \param [in] ulCapMode is the capture mode of the Timer port.
//!             Which can be the values: \b TIMER_CAP_MODE_CAP, \b TIMER_CAP_MODE_RST.
//!
//! \return None.
//
//*****************************************************************************
extern void xTimerCaptureModeSet(unsigned long ulBase, unsigned long ulChannel, 
                                 unsigned long ulCapMode);

//*****************************************************************************
//
//! \brief Set The Timer counter Prescale Value. 
//!        This function is to Set The Timer counter Prescale Value.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//! \param [in] ulValue is the Prescale Value will be set.
//!
//! \return None.
//
//*****************************************************************************
#define xTimerPrescaleSet(ulBase, ulChannel, ulValue)                         

//*****************************************************************************
//
//! \brief Get The Timer counter Prescale Value. 
//!        This function is to get The Timer counter Prescale Value.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//!
//! \return the Prescale Value will be get.
//
//*****************************************************************************
#define xTimerPrescaleGet(ulBase, ulChannel)  

//*****************************************************************************
//
//! \brief Set The Timer counter Value. 
//!        This function is to Set The Timer counter Value.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//! \param [in] ulValue is the Proload Value will be set.
//!
//! \return None.
//
//*****************************************************************************
#define xTimerLoadSet(ulBase, ulChannel, ulValue)  

//*****************************************************************************
//
//! \brief Get The Timer counter Value. 
//!        This function is to get The Timer counter Value.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//!
//! \return the counter Value will be get.
//
//*****************************************************************************
#define xTimerLoadGet(ulBase, ulChannel)  

//*****************************************************************************
//
//! \brief Get The Timer counter current up timer or up event counter value. 
//!        This function is to get The Timer counter current up timer or up event
//!        counter value.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//!
//! \return current up timer or up event counter value will be set.
//
//*****************************************************************************
#define xTimerValueGet(ulBase, ulChannel) 

//*****************************************************************************
//
//! \brief Set The Timer counter Compare Match Value. 
//!        This function is to Set The Timer counter Compare Match Value.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//! \param [in] ulValue is Timer counter Compare Match Value.
//!
//! \return None.
//
//*****************************************************************************
#define xTimerMatchSet(ulBase, ulChannel, ulValue)

//*****************************************************************************
//
//! \brief Get The Timer counter Compare Match Value. 
//!        This function is to get The Timer counter Compare Match Value.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//!
//! \return the Prescale Value will be set.
//
//*****************************************************************************
#define xTimerMatchGet(ulBase, ulChannel, ulValue)

//*****************************************************************************
//
//! \brief Init interrupts callback for the timer.
//!        Init interrupts callback for the timer.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] xtPortCallback is callback for the timer.
//!
//! \return None.
//
//*****************************************************************************
#define xTimerIntCallbackInit(ulBase, xtTimerCallback)

//*****************************************************************************
//
//! \brief Enable The Timer counter interrupt. 
//!        This function is to enable The Timer counter interrupt.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//! \param [in] ulIntFlags is the interrupt type of the Timer port.
//!
//! \return None.
//
//*****************************************************************************
#define xTimerIntEnable(ulBase, ulChannel, ulIntFlags)

//*****************************************************************************
//
//! \brief Disable The Timer counter interrupt. 
//!        This function is to disable The Timer counter interrupt.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//! \param [in] ulIntFlags is the interrupt type of the Timer port.
//!
//! \return None.
//
//*****************************************************************************
#define xTimerIntDisable(ulBase, ulChannel, ulIntFlags)

//*****************************************************************************
//
//! \brief Get The Timer counter interrupt Status. 
//!        This function is to get timer interrupt status.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulChannel is the channel of the Timer port.
//! \param [in] ulIntFlags is the interrupt type of the Timer port.
//!
//! \return the Status of The Timer counter interrupt.
//
//*****************************************************************************
#define xTimerStatusGet(ulBase, ulChannel, ulIntFlags)

//*****************************************************************************
//
//! \brief Select The Timer counter detect phase. 
//!        This function is to select The Timer counter detect phase.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulPhase is the counter detect phase of the Timer port.
//! 
//! \return None.
//
//*****************************************************************************
#define xTimerCounterDetectPhaseSelect(ulBase, ulChannel, ulPhase)

//*****************************************************************************
//
//! \brief Select The Timer counter capture detect edge. 
//!        This function is to select The Timer counter capture detect edge.
//!
//! \param [in] ulBase is the base address of the Timer port.
//! \param [in] ulEdge is the capture detect edge of the Timer port.
//!
//! \return None.
//
//*****************************************************************************
#define xTimerCaptureEdgeSelect(ulBase, ulChannel, ulEdge)

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

#define TIMER_MAT_CH_0          BIT_32_0 
#define TIMER_MAT_CH_1          BIT_32_1 
#define TIMER_MAT_CH_2          BIT_32_2 
#define TIMER_MAT_CH_3          BIT_32_3 

#define TIMER_CAP_CH_0          BIT_32_4 
#define TIMER_CAP_CH_1          BIT_32_5 

#define TIMER_INT_MAT_CH_0      BIT_32_0 
#define TIMER_INT_MAT_CH_1      BIT_32_1 
#define TIMER_INT_MAT_CH_2      BIT_32_2 
#define TIMER_INT_MAT_CH_3      BIT_32_3 

#define TIMER_INT_CAP_CH_0      BIT_32_4 
#define TIMER_INT_CAP_CH_1      BIT_32_5 

#define TIMER_MAT_MASK          BIT_MASK(32, 2, 0)
//! An interrupt is generated when MR matches the value in the TC. 
#define TIMER_MAT_INT           BIT_32_0

//! The TC will be reset if MR matches it.
#define TIMER_MAT_RESET         BIT_32_1

//! The TC and PC will be stopped and TCR will be set to 0 if MR matches the TC.
#define TIMER_MAT_STOP          BIT_32_2

#define TIMER_MAT_PIN_MASK      BIT_MASK(32, 14, 12)
#define TIMER_MAT_PIN_NONE      BIT_32_14
#define TIMER_MAT_PIN_LOW       BIT_32_12
#define TIMER_MAT_PIN_HIGH      BIT_32_13
#define TIMER_MAT_PIN_TOGGLE    (BIT_32_13 | BIT_32_12)

//*****************************************************************************
//
//! \addtogroup LPC17xx_TIMER_Cfg_Capture LPC17xx TimerCapture Configure parameters.
//! \brief      Parameters for Timer Capture mode.
//!             The value below can be used in the function \ref TimerCaptureCfg.
//! @{
//
//*****************************************************************************

//! Rising Edge Capture.
#define TIMER_CFG_CAP_RISING    BIT_32_0

//! Falling Edge Capture.
#define TIMER_CFG_CAP_FALLING   BIT_32_1

//! Rising or Falling Edge Capture.
#define TIMER_CFG_CAP_INT       BIT_32_2

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup LPC17xx_TIMER_Cfg_Counter LPC17xx TimerCounter Configure parameters.
//! \brief      Parameters for Timer Counter mode.
//!             The value below can be used in the function \ref TimerCounterCfg.
//! @{
//
//*****************************************************************************

//! Counter clock --> Capture Pin 0 Rising Edge pluse 
#define TIMER_CFG_CNT_CAP0_RISING      (BIT_32_0)

//! Counter clock --> Capture Pin 0 Falling Edge pluse 
#define TIMER_CFG_CNT_CAP0_FALLING     (BIT_32_1)

//! Counter clock --> Capture Pin 0 Rising or Falling Edge pluse 
#define TIMER_CFG_CNT_CAP0_BOTH        (BIT_32_1 | BIT_32_0)

//! Counter clock --> Capture Pin 1 Rising Edge pluse 
#define TIMER_CFG_CNT_CAP1_RISING      (BIT_32_2 | BIT_32_0)

//! Counter clock --> Capture Pin 1 Falling Edge pluse 
#define TIMER_CFG_CNT_CAP1_FALLING     (BIT_32_2 | BIT_32_1)

//! Counter clock --> Capture Pin 1 Rising or Falling Edge pluse 
#define TIMER_CFG_CNT_CAP1_BOTH        (BIT_32_2 | BIT_32_1 | BIT_32_0)

//*****************************************************************************
//
//! @}
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup LPC17xx_TIMER_Exported_APIs LPC17xx TIMER API
//! \brief      LPC17xx TIMER API Reference.
//! @{
//
//*****************************************************************************

extern unsigned long TimerIntStatusGet(unsigned long ulBase);
extern xtBoolean TimerIntStatusCheck(unsigned long ulBase, unsigned long ulIntFlags);
extern void TimerIntStatusClear(unsigned long ulBase, unsigned long ulIntFlags);
extern void TimerStart(unsigned long ulBase);
extern void TimerStop(unsigned long ulBase);
extern void TimerReset(unsigned long ulBase);
extern void TimerPrescaleSet(unsigned long ulBase, unsigned long ulValue);
extern unsigned long TimerPrescaleGet(unsigned long ulBase);
extern void TimerLoadSet(unsigned long ulBase, unsigned long ulValue);
extern unsigned long TimerLoadGet(unsigned long ulBase);
extern unsigned long TimerValueGet(unsigned long ulBase);
extern void TimerMatchValueSet(unsigned long ulBase, unsigned long ulChs, unsigned long ulValue);
extern unsigned long TimerMatchValueGet(unsigned long ulBase, unsigned long ulChs);
extern void TimerMatchCfg(unsigned long ulBase, unsigned long ulChs, unsigned long ulCfgs);
extern void TimerCaptureCfg(unsigned long ulBase, unsigned long ulChs, unsigned long ulCfgs);
extern unsigned long TimerCapValueGet(unsigned long ulBase, unsigned long ulChs);
extern void TimerIntCallbackInit(unsigned long ulBase, xtEventCallback pfnCallback);
extern void TimerCounterCfg(unsigned long ulBase, unsigned long ulCfg);

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

#endif // __xTIMER_H__
