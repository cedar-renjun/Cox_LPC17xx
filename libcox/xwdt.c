#include "xhw_types.h"
#include "xhw_ints.h"
#include "xcore.h"
#include "xhw_memmap.h"
#include "xhw_nvic.h"
#include "xhw_sysctl.h"
#include "xdebug.h"
#include "xsysctl.h"
#include "xhw_wdt.h"
#include "xwdt.h"

#define WDT_CFG_MODE_M          (BIT_32_1 | BIT_32_0)
#define WDT_CFG_CLKSRC_M        (BIT_32_3 | BIT_32_2)

//*****************************************************************************
//
// An array is Watchdog callback function point
//
//*****************************************************************************
static xtEventCallback g_pfnWDTHandlerCallbacks = 0;

//*****************************************************************************
//
//! \brief  WDT interrupt handler.
//!
//!         This function is the WDT interrupt handler, it simple execute the
//!         callback function if there be one.
//!
//! \param  None.
//! \return None.
//!
//
//*****************************************************************************
void WDTIntHandler(void)
{
    if(g_pfnWDTHandlerCallbacks != 0)
    {
        g_pfnWDTHandlerCallbacks(0, 0, 0, 0);
    }
    else
    {
        while(1);
    }
}

unsigned long WDTIntCallbackInit(xtEventCallback pfnCallback)
{
    // Check the parameters.
    xASSERT(pfnCallback != 0);

    g_pfnWDTHandlerCallbacks = pfnCallback;

    return (0);

}


void WDTCfg(unsigned long ulCfg, unsigned long ulValue)
{
    unsigned long ulTmpReg = 0;

/************ Configure Watchdog Mode and Clock source **************/
    switch(ulCfg & WDT_CFG_MODE_M)
    {
        case WDT_CFG_INT_MODE:                             // Triggle Interrupt when underflow.
            {
                ulTmpReg = xHWREG(WDT_BASE + WDT_MOD);
                //ulTmpReg |= WDMOD_EN;
                ulTmpReg &= ~WDMOD_RESET;
                xHWREG(WDT_BASE + WDT_MOD) = ulTmpReg;  
                break;
            }
        case WDT_CFG_RESET_MODE:                           // Reset MCU when underflow.
            {
                ulTmpReg = xHWREG(WDT_BASE + WDT_MOD);
                ulTmpReg |= /*WDMOD_EN |*/ WDMOD_RESET;
                xHWREG(WDT_BASE + WDT_MOD) = ulTmpReg;  
                break;
            }
    }

    switch(ulCfg & WDT_CFG_CLKSRC_M)
    {
        case WDT_CFG_CLKSRC_IRC:                           // Internal RC clock.
            {
                ulTmpReg = xHWREG(WDT_BASE + WDT_CLKSEL);
                ulTmpReg &= ~WDCLKSEL_WDSEL_M;
                ulTmpReg |= WDCLKSEL_WDSEL_IRC;
                xHWREG(WDT_BASE + WDT_CLKSEL) = ulTmpReg;  
                break;
            }
        case WDT_CFG_CLKSRC_APB:                           // APB Clock source.
            {
                ulTmpReg = xHWREG(WDT_BASE + WDT_CLKSEL);
                ulTmpReg &= ~WDCLKSEL_WDSEL_M;
                ulTmpReg |= WDCLKSEL_WDSEL_APB;
                xHWREG(WDT_BASE + WDT_CLKSEL) = ulTmpReg; 
                break;
            }
        case WDT_CFG_CLKSRC_RTC:                          // RTC Clock source.
            {
                ulTmpReg = xHWREG(WDT_BASE + WDT_CLKSEL);
                ulTmpReg &= ~WDCLKSEL_WDSEL_M;
                ulTmpReg |= WDCLKSEL_WDSEL_RTC;
                xHWREG(WDT_BASE + WDT_CLKSEL) = ulTmpReg; 
                break;
            }
    }


/************* Configure Watchdog const value register **************/
    if(ulValue < (unsigned long)0xFF)
    {
        ulValue = 0xFF;
    }

    xHWREG(WDT_BASE + WDT_TC) = ulValue;

    // Feed watchdog.
    xHWREG(WDT_BASE + WDT_FEED) = (unsigned long) 0xAA;
    xHWREG(WDT_BASE + WDT_FEED) = (unsigned long) 0x55;
}


void WDTFeed(void)
{
    // Feed watchdog.
    xHWREG(WDT_BASE + WDT_FEED) = (unsigned long) 0xAA;
    xHWREG(WDT_BASE + WDT_FEED) = (unsigned long) 0x55;
}

void WDTEnable(void)
{
    // Enable Watchdog bit.
    xHWREG(WDT_BASE + WDT_MOD) |= WDMOD_EN;

    // Feed dog to clock the watchdog.
    xHWREG(WDT_BASE + WDT_FEED) = (unsigned long) 0xAA;
    xHWREG(WDT_BASE + WDT_FEED) = (unsigned long) 0x55;
}

unsigned long WDTStatusFlagGet(void)
{
    return xHWREG(WDT_BASE + WDT_MOD);
}



xtBoolean WDTStatusFlagCheck(unsigned long ulFlags)
{
    xASSERT((ulFlags  == WDT_FLAG_TIMEOUT) ||
            (ulFlags  == WDT_FLAG_INT    ) ||
            (ulFlags  == (WDT_FLAG_TIMEOUT |  WDT_FLAG_INT))
           );

    // Check flag.
    if( xHWREG(WDT_BASE + WDT_MOD) & ulFlags )
    {
        return (xtrue);
    }
    else
    {
        return (xfalse);
    }
}

//! \note This function only can be used to clear WDT_FLAG_TIMEOUT flag.
void WDTStatusFlagClear(unsigned long ulFlags)
{

    xASSERT(ulFlags == WDT_FLAG_TIMEOUT);

    // Clear Timeout Flag.
    xHWREG(WDT_BASE + WDT_MOD) &= ~WDMOD_TOF;
    
    // Feed watchdog to action.
    xHWREG(WDT_BASE + WDT_FEED) = (unsigned long) 0xAA;
    xHWREG(WDT_BASE + WDT_FEED) = (unsigned long) 0x55;
}

