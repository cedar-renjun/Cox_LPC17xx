#include "xhw_types.h"
#include "xhw_ints.h"
#include "xcore.h"
#include "xhw_memmap.h"
#include "xhw_nvic.h"
#include "xhw_sysctl.h"
#include "xdebug.h"
#include "xsysctl.h"
#include "xhw_gpio.h"
#include "xgpio.h"
#include "xhw_timer.h"
#include "xtimer.h"



static xtEventCallback g_pfnTimerHandlerCallbacks[4] = {0};


void TIMER0IntHandler(void) 
{
    if( g_pfnTimerHandlerCallbacks[0] != 0)
    {
        g_pfnTimerHandlerCallbacks[0](0, 0, 0, 0);
    }
    else
    {
        while(1);
    }
}

void TIMER1IntHandler(void) 
{
    if( g_pfnTimerHandlerCallbacks[1] != 0)
    {
        g_pfnTimerHandlerCallbacks[1](0, 0, 0, 0);
    }
    else
    {
        while(1);
    }
}

void TIMER2IntHandler(void) 
{
    if( g_pfnTimerHandlerCallbacks[2] != 0)
    {
        g_pfnTimerHandlerCallbacks[2](0, 0, 0, 0);
    }
    else
    {
        while(1);
    }
}

void TIMER3IntHandler(void) 
{
    if( g_pfnTimerHandlerCallbacks[3] != 0)
    {
        g_pfnTimerHandlerCallbacks[3](0, 0, 0, 0);
    }
    else
    {
        while(1);
    }
}

void TimerIntCallbackInit(unsigned long ulBase, xtEventCallback pfnCallback)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    switch(ulBase)
    {
        case TIMER0_BASE:
            {
                g_pfnTimerHandlerCallbacks[0] = pfnCallback;
                break;
            }

        case TIMER1_BASE:
            {
                g_pfnTimerHandlerCallbacks[1] = pfnCallback;
                break;
            }

        case TIMER2_BASE:
            {
                g_pfnTimerHandlerCallbacks[2] = pfnCallback;
                break;
            }

        case TIMER3_BASE:
            {
                g_pfnTimerHandlerCallbacks[3] = pfnCallback;
                break;
            }

        default:
            {
                while(1);
            }
    }
}


/*

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

*/

unsigned long TimerIntStatusGet(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    // Read Interrupt status register 
    return (xHWREG(ulBase + TIMER_IR));
}

xtBoolean TimerIntStatusCheck(unsigned long ulBase, unsigned long ulIntFlags)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    // Check interrupt status register
    if(xHWREG(ulBase + TIMER_IR) & ulIntFlags)
    {
        return (xtrue);
    }
    else
    {
        return (xfalse);
    }

}

void TimerIntStatusClear(unsigned long ulBase, unsigned long ulIntFlags)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    // Clear special interrupt flag by write 1 to correct bit.
    xHWREG(ulBase + TIMER_IR) |= ulIntFlags;
}

void TimerStart(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    // Enable Timer Module.
    xHWREG(ulBase + TIMER_TCR) |= TCR_CNT_EN;
}

void TimerStop(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    // Disable Timer Module.
    xHWREG(ulBase + TIMER_TCR) &= ~TCR_CNT_EN;
}

void TimerReset(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    // Reset Timer Counter.
    xHWREG(ulBase + TIMER_TCR) |=  TCR_CNT_RST;
    xHWREG(ulBase + TIMER_TCR) &= ~TCR_CNT_RST;
}

/*
MODE_TIMER
MODE_COUNTER_CH0_RISING
MODE_COUNTER_CH0_FALLING
MODE_COUNTER_CH0_BOTHEDGE
MODE_COUNTER_CH1_RISING
MODE_COUNTER_CH1_FALLING
MODE_COUNTER_CH1_BOTHEDGE
*/

//TimerCfg

void TimerPrescaleSet(unsigned long ulBase, unsigned long ulValue)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    // Write the prescale value
    xHWREG(ulBase + TIMER_PR) = ulValue;
}

unsigned long TimerPrescaleGet(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    // Read back the prescale value
    return (xHWREG(ulBase + TIMER_PR));
}

void TimerLoadSet(unsigned long ulBase, unsigned long ulValue)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    // Write the prescale value
    xHWREG(ulBase + TIMER_PR) = ulValue;
}

unsigned long TimerLoadGet(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    // Read back the prescale value
    return (xHWREG(ulBase + TIMER_PR));
}


void TimerMatchValueSet(unsigned long ulBase, unsigned long ulChs, unsigned long ulValue)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    switch(ulChs)
    {
        case TIMER_MAT_CH_0:
            {
                xHWREG(ulBase + TIMER_MR0) = ulValue;
                break;
            }

        case TIMER_MAT_CH_1:
            {
                xHWREG(ulBase + TIMER_MR1) = ulValue;
                break;
            }

        case TIMER_MAT_CH_2:
            {
                xHWREG(ulBase + TIMER_MR2) = ulValue;
                break;
            }

        case TIMER_MAT_CH_3:
            {
                xHWREG(ulBase + TIMER_MR3) = ulValue;
                break;
            }

        default:
            {
                while(1);
            }
    }

}

unsigned long  TimerMatchValueGet(unsigned long ulBase, unsigned long ulChs)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    switch(ulChs)
    {
        case TIMER_MAT_CH_0:
            {
                return xHWREG(ulBase + TIMER_MR0);
                break;
            }

        case TIMER_MAT_CH_1:
            {
                return xHWREG(ulBase + TIMER_MR1);
                break;
            }

        case TIMER_MAT_CH_2:
            {
                return xHWREG(ulBase + TIMER_MR2);
                break;
            }

        case TIMER_MAT_CH_3:
            {
                return xHWREG(ulBase + TIMER_MR3);
                break;
            }

        default:
            {
                while(1);
            }
    }

}



unsigned long TimerValueGet(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    // Write the prescale value
    return (xHWREG(ulBase + TIMER_TC));
}


/*
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
*/

void TimerMatchCfg(unsigned long ulBase, unsigned long ulChs, unsigned long ulCfgs)
{
    unsigned long i        = 0;
    unsigned long ulTmpReg = 0;
    unsigned long Tmp      = 0;

    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    xASSERT( (ulChs & ~( TIMER_MAT_CH_0 | TIMER_MAT_CH_1 | 
                TIMER_MAT_CH_2 | TIMER_MAT_CH_3)) == 0 );

    xASSERT( (ulCfgs & ~(TIMER_MAT_MASK | TIMER_MAT_PIN_MASK)) == 0 );

    for(i = 0; i < 4; i++)
    {
        if( ulChs & (1<<i) )                         // Find Match Channel
        {
            /*********** Configure MCR register ******************/
            if(ulCfgs & TIMER_MAT_MASK)
            {
                Tmp = ulCfgs & BIT_MASK(32, 7, 0);
                ulTmpReg = xHWREG(ulBase + TIMER_MCR);
                ulTmpReg &= ~( (MCR_MRxI | MCR_MRxR | MCR_MRxS) << i*3 );
                ulTmpReg |= (Tmp << i*3);
                xHWREG(ulBase + TIMER_MCR) = ulTmpReg;
            }

            /*********** Configure EMR register ******************/
            if(ulCfgs & TIMER_MAT_PIN_MASK)          // Enable Match Pin output
            {
                if(ulCfgs & TIMER_MAT_PIN_NONE)      // Disable Match Pin output
                {
                    xHWREG(ulBase + TIMER_EMR) &= ~( EMR_EM0 << i );
                    xHWREG(ulBase + TIMER_EMR) &= ~( EMR_EMC0_M << (i*2) );
                }
                else
                {
                    Tmp = (ulCfgs >> 8) & BIT_MASK(32, 7, 0);
                    ulTmpReg = xHWREG(ulBase + TIMER_EMR);
                    ulTmpReg |= ( EMR_EM0 << i );
                    ulTmpReg &= ~( EMR_EMC0_M << (2*i) );
                    ulTmpReg |= (Tmp << (2*i));
                    xHWREG(ulBase + TIMER_EMR) = ulTmpReg;
                }
            }
        }
    }
}


/*
#define TIMER_CAP_CH_0          BIT_32_4 
#define TIMER_CAP_CH_1          BIT_32_5 
*/

/*
#define TIMER_CFG_CAP_RISING    BIT_32_0
#define TIMER_CFG_CAP_FALLING   BIT_32_1
#define TIMER_CFG_CAP_INT       BIT_32_2
*/

void TimerCaptureCfg(unsigned long ulBase, unsigned long ulChs, unsigned long ulCfgs)
{
    unsigned long ulTmpReg = 0;
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    xASSERT( (ulChs & ~( TIMER_CAP_CH_0 | TIMER_CAP_CH_1 )) == 0 );

    xASSERT( (ulCfgs & ~( TIMER_CFG_CAP_RISING | TIMER_CFG_CAP_FALLING | TIMER_CFG_CAP_INT )) == 0 );

    // Configure Capture Channel 0
    if(ulChs & TIMER_CAP_CH_0)
    {
        ulTmpReg = xHWREG(ulBase + TIMER_CCR);
        ulTmpReg &= ~(CCR_CAP0RE | CCR_CAP0FE | CCR_CAP0I);
        ulTmpReg |= ulCfgs;
        xHWREG(ulBase + TIMER_CCR) = ulTmpReg;
    }

    // Configure Capture Channel 1
    if(ulChs & TIMER_CAP_CH_1)
    {
        ulTmpReg = xHWREG(ulBase + TIMER_CCR);
        ulTmpReg &= ~(CCR_CAP1RE | CCR_CAP1FE | CCR_CAP1I);
        ulTmpReg |= (ulCfgs << 3);
        xHWREG(ulBase + TIMER_CCR) = ulTmpReg;
    }
}

unsigned long TimerCapValueGet(unsigned long ulBase, unsigned long ulChs)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );

    xASSERT( (ulChs & ~( TIMER_CAP_CH_0 | TIMER_CAP_CH_1 )) == 0 );

    switch(ulChs)
    {
        case TIMER_CAP_CH_0:
            {
                return xHWREG(ulBase + TIMER_CR0);
            }

        case TIMER_CAP_CH_1:
            {
                return xHWREG(ulBase + TIMER_CR1);
            }
        default:
            {
                while(1);
            }
    }
}

/*
#define TIMER_CFG_CNT_CAP0_RISING      (BIT_32_0)
#define TIMER_CFG_CNT_CAP0_FALLING     (BIT_32_1)
#define TIMER_CFG_CNT_CAP0_BOTH        (BIT_32_1 | BIT_32_0)

#define TIMER_CFG_CNT_CAP1_RISING      (BIT_32_2 | BIT_32_0)
#define TIMER_CFG_CNT_CAP1_FALLING     (BIT_32_2 | BIT_32_1)
#define TIMER_CFG_CNT_CAP1_BOTH        (BIT_32_2 | BIT_32_1 | BIT_32_0)
*/

void TimerCounterCfg(unsigned long ulBase, unsigned long ulCfg)
{
    // Check the parameters.
    xASSERT((ulBase == TIMER0_BASE) || (ulBase == TIMER1_BASE) ||
            (ulBase == TIMER2_BASE) || (ulBase == TIMER3_BASE) );
    xASSERT( (ulCfg & ~BIT_MASK(32, 2, 0)) == 0 );

    // Configure Timer Counter/Timer Register.
    xHWREG(ulBase + TIMER_CTCR) = ulCfg;
    xHWREG(ulBase + TIMER_CCR)  = 0x00;

}

