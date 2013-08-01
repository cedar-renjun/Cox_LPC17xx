#include "xhw_types.h"
#include "xhw_ints.h"
#include "xcore.h"
#include "xhw_memmap.h"
#include "xhw_nvic.h"
#include "xhw_sysctl.h"
#include "xdebug.h"
#include "xsysctl.h"
#include "xhw_rtc.h"
#include "xrtc.h"

//*****************************************************************************
//
// An array is RTC callback function point
//
//*****************************************************************************
static xtEventCallback g_pfnRTCHandlerCallbacks = 0;

#define SEC_MASK                BIT_MASK(32, 5, 0)
#define MIN_MASK                BIT_MASK(32, 5, 0)
#define HOUR_MASK               BIT_MASK(32, 4, 0)
#define DOM_MASK                BIT_MASK(32, 4, 0)
#define DOW_MASK                BIT_MASK(32, 2, 0)
#define DOY_MASK                BIT_MASK(32, 8, 0)
#define MONTH_MASK              BIT_MASK(32, 3, 0)
#define YEAR_MASK               BIT_MASK(32, 11, 0)

//*****************************************************************************
//
//! \brief  RTC interrupt handler.
//!
//!         This function is the RTC interrupt handler, it simple execute the
//!         callback function if there be one.
//!
//! \param  None.
//! \return None.
//!
//
//*****************************************************************************
void RTCIntHandler(void)
{
    if(g_pfnRTCHandlerCallbacks != 0)
    {
        g_pfnRTCHandlerCallbacks(0, 0, 0, 0);
    }
    else
    {
        while(1);
    }
}

void RTCTimeSet(unsigned long ulType, unsigned long ulValue)
{
    switch(ulType)
    {
        case RTC_TIMETYPE_SECOND:          // Second
            {
                if(ulValue > 59)
                {
                    ulValue = 59;
                }  
                xHWREG(RTC_BASE + RTC_SEC) = ulValue;
                break;
            }
        case RTC_TIMETYPE_MINUTE:          // Minute
            {
                if(ulValue > 59)
                {
                    ulValue = 59;
                }  
                xHWREG(RTC_BASE + RTC_MIN) = ulValue;
                break;
            }
        case RTC_TIMETYPE_HOUR:            // Hour
            {
                if(ulValue > 23)
                {
                    ulValue = 23;
                }  
                xHWREG(RTC_BASE + RTC_HOUR) = ulValue;
                break;
            }
        case RTC_TIMETYPE_DAYOFWEEK:       // Day of week
            {
                if(ulValue > 6)
                {
                    ulValue = 6;
                } 
                xHWREG(RTC_BASE + RTC_DOW) = ulValue;
                break;
            }
        case RTC_TIMETYPE_DAYOFMONTH:      // Day of month
            {
                if(ulValue < 1)
                {
                    ulValue = 1;
                }
                if(ulValue > 31)
                {
                    ulValue = 31;
                } 
                xHWREG(RTC_BASE + RTC_DOM) = ulValue;
                break;
            }
        case RTC_TIMETYPE_DAYOFYEAR:       // Day of year
            {
                if(ulValue < 1)
                {
                    ulValue = 1;
                }

                if(ulValue > 366)
                {
                    ulValue = 366;
                } 
                xHWREG(RTC_BASE + RTC_DOY) = ulValue;
                break;
            }
        case RTC_TIMETYPE_MONTH:           // Month
            {
                if(ulValue < 1)
                {
                    ulValue = 1;
                }

                if(ulValue > 12)
                {
                    ulValue = 12;
                } 
                xHWREG(RTC_BASE + RTC_MONTH) = ulValue;
                break;
            }
        case RTC_TIMETYPE_YEAR:            // Year
            {
                if(ulValue > 4095)
                {
                    ulValue = 4095;
                }  
                xHWREG(RTC_BASE + RTC_YEAR) = ulValue;
                break;
            }
        default:
            {
                while(1);
            }
    }
}

unsigned long RTCTimeGet(unsigned long ulType)
{
    switch(ulType)
    {
        case RTC_TIMETYPE_SECOND:          // Second
            {
                return xHWREG(RTC_BASE + RTC_SEC) & SEC_MASK;
                break;
            }
        case RTC_TIMETYPE_MINUTE:          // Minute
            {
                return xHWREG(RTC_BASE + RTC_MIN) & MIN_MASK;
                break;
            }
        case RTC_TIMETYPE_HOUR:            // Hour
            {
                return xHWREG(RTC_BASE + RTC_HOUR) & HOUR_MASK;
                break;
            }
        case RTC_TIMETYPE_DAYOFWEEK:       // Day of week
            {
                return xHWREG(RTC_BASE + RTC_DOW) & DOW_MASK;
                break;
            }
        case RTC_TIMETYPE_DAYOFMONTH:      // Day of month
            {
                return xHWREG(RTC_BASE + RTC_DOM) & DOM_MASK;
                break;
            }
        case RTC_TIMETYPE_DAYOFYEAR:       // Day of year
            {
                return xHWREG(RTC_BASE + RTC_DOY) & DOY_MASK;
                break;
            }
        case RTC_TIMETYPE_MONTH:           // Month
            {
                return xHWREG(RTC_BASE + RTC_MONTH) & MONTH_MASK;
                break;
            }
        case RTC_TIMETYPE_YEAR:            // Year
            {
                return xHWREG(RTC_BASE + RTC_YEAR) & YEAR_MASK;
                break;
            }
        default:
            {
                while(1);
            }
    }
}


void RTCAlarmSet(unsigned long ulType, unsigned long ulValue)
{
     switch(ulType)
    {
        case RTC_TIMETYPE_SECOND:          // Second
            {
                if(ulValue > 59)
                {
                    ulValue = 59;
                }
                xHWREG(RTC_BASE + RTC_ALSEC) = ulValue;
                break;
            }
        case RTC_TIMETYPE_MINUTE:          // Minute
            {         
                if(ulValue > 59)
                {
                    ulValue = 59;
                }  
                xHWREG(RTC_BASE + RTC_ALMIN) = ulValue;
                break;
            }
        case RTC_TIMETYPE_HOUR:            // Hour
            {
                if(ulValue > 23)
                {
                    ulValue = 23;
                } 
                xHWREG(RTC_BASE + RTC_ALHOUR) = ulValue;
                break;
            }
        case RTC_TIMETYPE_DAYOFWEEK:       // Day of week
            {
                if(ulValue > 6)
                {
                    ulValue = 6;
                } 
                xHWREG(RTC_BASE + RTC_ALDOW) = ulValue;
                break;
            }
        case RTC_TIMETYPE_DAYOFMONTH:      // Day of month
            {       
                if(ulValue < 1)
                {
                    ulValue = 1;
                }
                if(ulValue > 31)
                {
                    ulValue = 31;
                }  
                xHWREG(RTC_BASE + RTC_ALDOM) = ulValue;
                break;
            }
        case RTC_TIMETYPE_DAYOFYEAR:       // Day of year
            {
                if(ulValue < 1)
                {
                    ulValue = 1;
                }

                if(ulValue > 366)
                {
                    ulValue = 366;
                }

                xHWREG(RTC_BASE + RTC_ALDOY) = ulValue;
                break;
            }
        case RTC_TIMETYPE_MONTH:           // Month
            {
                if(ulValue < 1)
                {
                    ulValue = 1;
                }

                if(ulValue > 12)
                {
                    ulValue = 12;
                }
                xHWREG(RTC_BASE + RTC_ALMONTH) = ulValue;
                break;
            }
        case RTC_TIMETYPE_YEAR:            // Year
            {
                if(ulValue > 4095)
                {
                    ulValue = 4095;
                }  
                xHWREG(RTC_BASE + RTC_ALYEAR) = ulValue;
                break;
            }
        default:
            {
                while(1);
            }
    }
}

unsigned long RTCAlarmGet(unsigned long ulType)
{
     switch(ulType)
    {
        case RTC_TIMETYPE_SECOND:          // Second
            {
                return xHWREG(RTC_BASE + RTC_ALSEC) & SEC_MASK;
                break;
            }
        case RTC_TIMETYPE_MINUTE:          // Minute
            {
                return xHWREG(RTC_BASE + RTC_ALMIN) & MIN_MASK;
                break;
            }
        case RTC_TIMETYPE_HOUR:            // Hour
            {
                return xHWREG(RTC_BASE + RTC_ALHOUR) & HOUR_MASK;
                break;
            }
        case RTC_TIMETYPE_DAYOFWEEK:       // Day of week
            {
                return xHWREG(RTC_BASE + RTC_ALDOW) & DOW_MASK;
                break;
            }
        case RTC_TIMETYPE_DAYOFMONTH:      // Day of month
            {
                return xHWREG(RTC_BASE + RTC_ALDOM) & DOM_MASK;
                break;
            }
        case RTC_TIMETYPE_DAYOFYEAR:       // Day of year
            {
                return xHWREG(RTC_BASE + RTC_ALDOY) & DOY_MASK;
                break;
            }
        case RTC_TIMETYPE_MONTH:           // Month
            {
                return xHWREG(RTC_BASE + RTC_ALMONTH) & MONTH_MASK;
                break;
            }
        case RTC_TIMETYPE_YEAR:            // Year
            {
                return xHWREG(RTC_BASE + RTC_ALYEAR) & YEAR_MASK;
                break;
            }
        default:
            {
                while(1);
            }
    }
}

/*
RTC_GPREG0
RTC_GPREG1
RTC_GPREG2
RTC_GPREG3
RTC_GPREG4
*/

void RTCGenRegWrite(unsigned long ulID, unsigned long ulValue)
{
    xHWREG(RTC_BASE + ulID) = ulValue;
}

unsigned long RTCGenRegRead(unsigned long ulID)
{
    return xHWREG(RTC_BASE + ulID);
}

unsigned long RTCIntFlagGet(void)
{
    return xHWREG(RTC_BASE + RTC_ILR);
}

xtBoolean RTCIntFlagCheck(unsigned long ulFlags)
{
    xASSERT( !(ulFlags & ~( RTC_INT_INC | RTC_INT_ALARM )));

    if(xHWREG(RTC_BASE + RTC_ILR) & ulFlags)
    {
        return (xtrue);
    }
    else
    {
        return (xfalse);
    }
}

void RTCIntFlagClear(unsigned long ulFlags)
{
    xASSERT( !(ulFlags & ~( RTC_INT_INC | RTC_INT_ALARM )));
    xHWREG(RTC_BASE + RTC_ILR) |= ulFlags;
}


void RTCEnable(void)
{
    xHWREG(RTC_BASE + RTC_CCR) |= CCR_CLKEN;
}

void RTCDisable(void)
{
    xHWREG(RTC_BASE + RTC_CCR) &= ~CCR_CLKEN;
}

void RTCCounterReset(void)
{
    xHWREG(RTC_BASE + RTC_CCR) |= CCR_CTCRST;
    xHWREG(RTC_BASE + RTC_CCR) &= ~CCR_CTCRST;
}

void RTCCaliEnable(void)
{
    xHWREG(RTC_BASE + RTC_CCR) |= CCR_CCALEN;
}

void RTCCaliDisable(void)
{
    xHWREG(RTC_BASE + RTC_CCR) &= ~CCR_CCALEN;
}

void RTCIntCfg(unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    // Check the parameters.
    xASSERT(ulCfg != 0);

    // Configure Increment Interrupt.
    if(ulCfg & 0x0000FFFF)
    {
        ulTmpReg = xHWREG(RTC_BASE + RTC_CIIR);
        ulTmpReg &= ~((ulCfg >> 8) & 0xFF);
        ulTmpReg |=  ((ulCfg >> 0) & 0xFF);
        xHWREG(RTC_BASE + RTC_CIIR) = ulTmpReg;
    }

    // Configure Alarm Interrupt.
    if(ulCfg & 0xFFFF0000)
    {
        ulTmpReg = xHWREG(RTC_BASE + RTC_AMR);
        ulTmpReg &= ~((ulCfg >> 24) & 0xFF);
        ulTmpReg |=  ((ulCfg >> 16) & 0xFF);
        xHWREG(RTC_BASE + RTC_AMR) = ulTmpReg;
    }
}

unsigned long RTCIntCallbackInit(xtEventCallback pfnCallback)
{
    // Check the parameters.
    xASSERT(pfnCallback != 0);

    g_pfnRTCHandlerCallbacks = pfnCallback;

    return (0);

}

