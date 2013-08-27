//*****************************************************************************
//
//! \file xrtc.h
//! \brief Prototypes for the RTC Driver.
//! \version V2.2.1.0
//! \date 06/01/2012
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

#ifndef __xRTC_H__
#define __xRTC_H__

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
//! \addtogroup RTC
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xRTC
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xRTC_INT_Type xRTC Interrupt Type
//! \brief Values that show xRTC Interrupt Type
//! \n
//! \section xRTC_INT_Type_Section 1. Where to use this group
//! Values that can be passed to xRTCIntEnable(),xRTCIntDisable() and 
//! xRTCIntClear() as the ulIntFlags parameter. 
//! \n
//! \section xRTC_INT_Type_CoX 2.CoX Port Details 
//! \verbatim
//! +--------------------------+----------------+------------------------+
//! |  xRTC Interrupts         |       CoX      |         LPC17xx        |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_INT_SECOND         |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_INT_ALARM          |  Non-Mandatory |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_INT_OVERFLOW       |  Non-Mandatory |            N           |
//! +--------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************
#define xRTC_INT_SECOND         RTC_INT_INC
#define xRTC_INT_ALARM          RTC_INT_ALARM

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xRTC_INT_Event xRTC Interrupt Event
//! \brief Values that show xRTC Interrupt Event
//! \n
//! \section xRTC_INT_Event_Section 1. Where to use this group
//! RTC Event/Error Flag, Used by IntHandle's Event Callback Function as 
//! ulMsgParam parmeter. User Callback function can user this to detect what 
//! event happened.  
//! \n
//! \section xRTC_INT_Event_CoX 2.CoX Port Details 
//! \verbatim
//! +------------------------+----------------+------------------------+
//! |xRTC Interrupts         |       CoX      |         LPC17xx        |
//! |------------------------|----------------|------------------------|
//! |xRTC_EVENT_SECOND       |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xRTC_EVENT_ALARM        |  Non-Mandatory |            Y           |
//! |------------------------|----------------|------------------------|
//! |xRTC_EVENT_OVERFLOW     |  Non-Mandatory |            N           |
//! +------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************
#define xRTC_EVENT_SECOND       BIT_32_0
#define xRTC_EVENT_ALARM        BIT_32_1

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xRTC_Day_Week xRTC Day Week
//! \brief Values that show xRTC Day Week
//! \n
//! \section xRTC_Day_Week_Section 1. Where to use this group
//! Values that can be passed to xRTCTimeRead(),xRTCTimeWrite() 
//! as the tTime.ulWDay parameter. 
//! \n
//! \section xRTC_Day_Week_CoX 2.CoX Port Details 
//! \verbatim
//! +--------------------------+----------------+------------------------+
//! |  tTime.ulWDay            |       CoX      |         LPC17xx        |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_WEEK_SUNDAY        |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_WEEK_MONDAY        |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_WEEK_TUESDAY       |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_WEEK_WEDNESDAY     |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_WEEK_THURSDAY      |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_WEEK_FRIDAY        |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_WEEK_SATURDAY      |    Mandatory   |            Y           |
//! +--------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Sunday
//
#define xRTC_WEEK_SUNDAY        

//
//! Monday
//
#define xRTC_WEEK_MONDAY        

//
//! Tuesday
//
#define xRTC_WEEK_TUESDAY       

//
//! Wednesday
//
#define xRTC_WEEK_WEDNESDAY     

//
//! Thursday
//
#define xRTC_WEEK_THURSDAY     

//
//! Friday
//
#define xRTC_WEEK_FRIDAY      

//
//! Saturday
//
#define xRTC_WEEK_SATURDAY   

//*****************************************************************************
//
//! @}
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup xRTC_Year_Offset xRTC Year Offset
//! \Values that show xRTC Year Offset
//! \n
//! \section xRTC_Year_Offset_Section 1. Where to use this group
//! Values that is the offset of the year. 
//! \n
//! \section xRTC_Year_Offset_CoX 2.CoX Port Details 
//! \verbatim

//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Initiative time is 00:00:00 1/1 / 2000
//
#define xRTC_YEAR_OFFSET        

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xRTC_Time_Type xRTC Time Type
//! \brief Values that show xRTC Time Type
//! \n
//! \section xRTC_Year_Offset_Section 1. Where to use this group
//! Values that can be passed to RTCTimeRead() and RTCTimeWrite()
//! as the ulTimeAlarm parameter. 
//! \n
//! \section xRTC_Year_Offset_CoX 2.CoX Port Details 
//! \verbatim
//! +--------------------------+----------------+------------------------+
//! |  ulTimeAlarm             |       CoX      |         LPC17xx        |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_TIME_CURRENT       |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xRTC_TIME_ALARM         |  Non-Mandatory |            Y           |
//! |--------------------------|----------------|------------------------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Read or write current time and date
//
#define xRTC_TIME_CURRENT       BIT_32_0

//
//! Read or write alarm time and date 
//
#define xRTC_TIME_ALARM         BIT_32_1

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xRTC_Exported_Types xRTC Exported Types
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! xRTC Time and Calendar Definition definitions
//
//*****************************************************************************
typedef struct 
{
    //
    //! Seconds of time 
    // 
    unsigned long ulSecond;     
    
    //
    //! Minutes of time 
    // 
    unsigned long ulMinute; 
    
    //
    //! Hours of time 
    // 
    unsigned long ulHour; 
    
    //
    //! Day of Month
    // 
    unsigned long ulMDay;  
    
    //
    //! Month
    // 
    unsigned long ulMonth;   
    
    //
    //! Years
    // 
    unsigned long ulYear;    
    
    //
    //! Day of Week  
    // 
    unsigned long ulWDay;   
} xtTime;

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xRTC_Exported_APIs xRTC API
//! \brief xRTC API Reference.
//!
//! \section xRTC_Exported_APIs_Port CoX Port Details
//! \verbatim
//! +------------------------+----------------+------------------------+
//! |xRTC API                |       CoX      |         LPC17xx        |
//! |------------------------|----------------|------------------------|
//! |xRTCTimeInit            |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xRTCTimeRead            |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xRTCTimeWrite           |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xRTCIntEnable           |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xRTCIntCallbackInit     |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xRTCIntDisable          |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xRTCStart               |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |xRTCStop                |    Mandatory   |            Y           |
//! +------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief  Initializes the RTC peripheral. 
//!         This function is to initializes the RTC peripheral.
//!
//! \param  None.
//!
//! \return The result of operation.
//!         - xtrue  Initializes successfully
//!         - xfalse Initializes fail
//
//*****************************************************************************
#define xRTCTimeInit()          RTCCounterReset()

//*****************************************************************************
//
//! \brief  Read current date/time or alarm date/time from RTC setting. 
//!         This function is to Read current date/time or alarm date/time from RTC
//!         setting.
//!
//! \param  [out] xtTime specifies the point of time and data.
//! \param  [in]  ulTimeAlarm specifies which will be read current time or alarm time.
//!               This parameter is the one of any of the following:
//!               \ref xRTC_TIME_CURRENT  Get Current time.
//!               \ref xRTC_TIME_ALARM    Get System Alarm.
//!
//! \return None.
//
//*****************************************************************************
extern void xRTCTimeRead(xtTime * pxtTime, unsigned long ulTimeAlarm);

//*****************************************************************************
//
//! \brief  Write current date/time or alarm date/time to RTC Module.
//!         This function is to configure current date/time or alarm date/time.
//!
//! \param  [out] xtTime specifies the point of time and data.
//! \param  [in]  ulTimeAlarm specifies which will be read current time or alarm time.
//!               This parameter is the one of any of the following:
//!               \ref xRTC_TIME_CURRENT  Get Current time.
//!               \ref xRTC_TIME_ALARM    Get System Alarm.
//!
//! \return None.
//
//*****************************************************************************
extern void xRTCTimeWrite(xtTime * pxtTime, unsigned long ulTimeAlarm);

//*****************************************************************************
//
//! \brief  Enable the time tick or alarm interrupt of RTC. 
//!         This function is to enable the time tick or alarm interrupt of RTC.
//!
//! \param  [in] ulIntType is the bit mask of the interrupt sources to be enabled.
//!              This value can be the logical OR of the following value:
//!              \ref xRTC_INT_SECOND      Tick interrupt
//!              \ref xRTC_INT_ALARM       Alarm interrupt
//!
//! \return None.
//
//*****************************************************************************        
extern void xRTCIntEnable(unsigned long ulIntType);

//*****************************************************************************
//
//! \brief  Disable the time tick or alarm interrupt of RTC. 
//!         This function is to disable the time tick or alarm interrupt of RTC.
//!
//! \param  [in] ulIntType is the bit mask of the interrupt sources to be enabled.
//!              This value can be the logical OR of the following value:
//!              \ref xRTC_INT_SECOND      Tick interrupt
//!              \ref xRTC_INT_ALARM       Alarm interrupt
//!
//! \return None.
//
//*****************************************************************************  
extern void xRTCIntDisable(unsigned long ulIntType);

//*****************************************************************************
//
//! \brief  Register user ISR callback function for the RTC.
//!
//! \param  [in] xtPortCallback is user ISR callback function for the RTC.
//!
//! \return None.
//
//*****************************************************************************
#define xRTCIntCallbackInit(xtRTCCallback)                                    \
         RTCIntCallbackInit(xtRTCCallback)

//*****************************************************************************
//
//! \brief  Start the RTC timer. 
//!
//! \param  None.
//!
//! \return None.
//
//*****************************************************************************
#define xRTCStart()             RTCEnable()

//*****************************************************************************
//
//! \brief  Stop the RTC timer. 
//!
//! \param  None.
//!
//! \return None.
//
//*****************************************************************************
#define xRTCStop()              RTCDisable()

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
//! \addtogroup LPC17xx_RTC
//! @{
//
//*****************************************************************************

#define RTC_TIMETYPE_SECOND     BIT_32_0
#define RTC_TIMETYPE_MINUTE     BIT_32_1
#define RTC_TIMETYPE_HOUR       BIT_32_2
#define RTC_TIMETYPE_DAYOFWEEK  BIT_32_3
#define RTC_TIMETYPE_DAYOFMONTH BIT_32_4
#define RTC_TIMETYPE_DAYOFYEAR  BIT_32_5
#define RTC_TIMETYPE_MONTH      BIT_32_6
#define RTC_TIMETYPE_YEAR       BIT_32_7

#define RTC_INT_INC             ILR_CIF
#define RTC_INT_ALARM           ILR_CALF

#define INT_SEC_EN              BIT_32_0
#define INT_MIN_EN              BIT_32_1
#define INT_HOUR_EN             BIT_32_2
#define INT_DOM_EN              BIT_32_3
#define INT_DOW_EN              BIT_32_4
#define INT_DOY_EN              BIT_32_5
#define INT_MON_EN              BIT_32_6
#define INT_YEAR_EN             BIT_32_7

#define INT_SEC_DIS             BIT_32_8
#define INT_MIN_DIS             BIT_32_9
#define INT_HOUR_DIS            BIT_32_10
#define INT_DOM_DIS             BIT_32_11
#define INT_DOW_DIS             BIT_32_12
#define INT_DOY_DIS             BIT_32_13
#define INT_MON_DIS             BIT_32_14
#define INT_YEAR_DIS            BIT_32_15

#define INT_ALARM_SEC_EN        BIT_32_24
#define INT_ALARM_MIN_EN        BIT_32_25
#define INT_ALARM_HOUR_EN       BIT_32_26
#define INT_ALARM_DOM_EN        BIT_32_27
#define INT_ALARM_DOW_EN        BIT_32_28
#define INT_ALARM_DOY_EN        BIT_32_29
#define INT_ALARM_MON_EN        BIT_32_30
#define INT_ALARM_YEAR_EN       BIT_32_31

#define INT_ALARM_SEC_DIS       BIT_32_16
#define INT_ALARM_MIN_DIS       BIT_32_17
#define INT_ALARM_HOUR_DIS      BIT_32_18
#define INT_ALARM_DOM_DIS       BIT_32_19
#define INT_ALARM_DOW_DIS       BIT_32_20
#define INT_ALARM_DOY_DIS       BIT_32_21
#define INT_ALARM_MON_DIS       BIT_32_22
#define INT_ALARM_YEAR_DIS      BIT_32_23




//*****************************************************************************
//
//! \addtogroup LPC17xx_RTC_Exported_APIs LPC17xx API
//! \brief LPC17xx RTC API Reference.
//! @{
//
//*****************************************************************************

extern void RTCTimeSet(unsigned long ulType, unsigned long ulValue);
extern unsigned long RTCTimeGet(unsigned long ulType);
extern void RTCAlarmSet(unsigned long ulType, unsigned long ulValue);
extern unsigned long RTCAlarmGet(unsigned long ulType);
extern void RTCGenRegWrite(unsigned long ulID, unsigned long ulValue);
extern unsigned long RTCGenRegRead(unsigned long ulID);
extern unsigned long RTCIntFlagGet(void);
extern xtBoolean RTCIntFlagCheck(unsigned long ulFlags);
extern void RTCIntFlagClear(unsigned long ulFlags);
extern void RTCEnable(void);
extern void RTCDisable(void);
extern void RTCCounterReset(void);
extern void RTCCaliEnable(void);
extern void RTCCaliDisable(void);
extern void RTCIntCfg(unsigned long ulCfg);
extern unsigned long RTCIntCallbackInit(xtEventCallback pfnCallback);



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

#endif // __xRTC_H__

