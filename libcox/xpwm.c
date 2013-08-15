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
#include "xhw_pwm.h"
#include "xpwm.h"

/*
#define PWM_CH_0                BIT_32_0           
#define PWM_CH_1                BIT_32_1           
#define PWM_CH_2                BIT_32_2           
#define PWM_CH_3                BIT_32_3           
#define PWM_CH_4                BIT_32_4           
#define PWM_CH_5                BIT_32_5           
#define PWM_CH_6                BIT_32_6
#define PWM_CAP_0               BIT_32_7           
#define PWM_CAP_1               BIT_32_8                 

#define PWM_INT_CH_0                BIT_32_0           
#define PWM_INT_CH_1                BIT_32_1           
#define PWM_INT_CH_2                BIT_32_2           
#define PWM_INT_CH_3                BIT_32_3           
#define PWM_INT_CH_4                BIT_32_8           
#define PWM_INT_CH_5                BIT_32_9           
#define PWM_INT_CH_6                BIT_32_10           
#define PWM_INT_CAP_0               BIT_32_4           
#define PWM_INT_CAP_1               BIT_32_5           
*/

#define PWM_INT_FLAG_MASK                                                            \
                                (PWM_INT_CH_0 | PWM_INT_CH_1  | PWM_INT_CH_2 |       \
                                 PWM_INT_CH_3 | PWM_INT_CH_4  | PWM_INT_CH_5 |       \
                                 PWM_INT_CH_6 | PWM_INT_CAP_0 | PWM_INT_CAP_1)

void PWMIntStatusClear(unsigned long ulBase, unsigned long ulIntFlags)
{

    // Check input parameters.
    xASSERT(ulBase == PWM1_BASE);
    xASSERT((ulIntFlags & ~ PWM_INT_FLAG_MASK) == 0);

    // Check flag.
    xHWREG(PWM1_BASE + PWM_IR) |= ulIntFlags;
}
unsigned long PWMIntStatusGet(unsigned long ulBase)
{

    // Check input parameters.
    xASSERT(ulBase == PWM1_BASE);

    return xHWREG(PWM1_BASE + PWM_IR);

}
xtBoolean PWMIntStatusCheck(unsigned long ulBase, unsigned long ulIntFlags)
{
    // Check input parameters.
    xASSERT(ulBase == PWM1_BASE);
    xASSERT((ulIntFlags & ~ PWM_INT_FLAG_MASK) == 0);

    // Check flag.
    if( xHWREG(PWM1_BASE + PWM_IR) & ulIntFlags )
    {
        return (xtrue);
    }
    else
    {
        return (xfalse);
    }   
}


void PWMCounterEnable(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT(ulBase == PWM1_BASE);

    xHWREG(PWM1_BASE + PWM_TCR) |= TCR_CNT_EN;
}
void PWMCounterDisable(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT(ulBase == PWM1_BASE);

    xHWREG(PWM1_BASE + PWM_TCR) &= ~TCR_CNT_EN;
}
void PWMCounterReset(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT(ulBase == PWM1_BASE);

    xHWREG(PWM1_BASE + PWM_TCR) |=  TCR_CNT_RST;
    xHWREG(PWM1_BASE + PWM_TCR) &= ~TCR_CNT_RST;
}

/*
PWM_CH_0
PWM_CH_1
PWM_CH_2
PWM_CH_3
PWM_CH_4
PWM_CH_5
PWM_CH_6

#define PWM_MATCH_INT_EN        BIT_32_0         
#define PWM_MATCH_INT_DIS       BIT_32_8         
#define PWM_MATCH_RESET_EN      BIT_32_1         
#define PWM_MATCH_RESET_DIS     BIT_32_9         
#define PWM_MATCH_STOP_EN       BIT_32_2         
#define PWM_MATCH_STOP_DIS      BIT_32_10        
*/

// \todo Is there ADN or OR 

void PWMMatchCfg(unsigned long ulBase, unsigned long ulCh, unsigned long ulCfg)
{
    unsigned long i        = 0;
    unsigned long ulTmpReg = 0;

    // Check input parameters.
    xASSERT(ulBase == PWM1_BASE);
    xASSERT( ulCh &~ ( PWM_CH_0 | PWM_CH_1 |
                       PWM_CH_2 | PWM_CH_3 |
                       PWM_CH_4 | PWM_CH_5 |
                       PWM_CH_6)
            );

    for(i = 0; i < 8; i++)
    {
        if((1<<i) & ulCh)
        {
            ulTmpReg = xHWREG(PWM1_BASE + PWM_MCR);
            ulTmpReg &= ~((ulCfg >> 8) & 0xFF);
            ulTmpReg |=  ((ulCfg >> 0) & 0xFF);
            xHWREG(PWM1_BASE + PWM_MCR) = ulTmpReg;
        }
    }
 }

void PWMMatchUpdate(unsigned long ulBase, unsigned long ulCh, unsigned long ulValue)
{
    // Check input parameters.
    xASSERT(ulBase == PWM1_BASE);
    switch(ulCh)
    {
        case PWM_CH_0:
            {
                xHWREG(PWM1_BASE + PWM_MR0) = ulValue;
                break;
            }
        case PWM_CH_1:
            {
                xHWREG(PWM1_BASE + PWM_MR1) = ulValue;
                break;
            }
        case PWM_CH_2:
            {
                xHWREG(PWM1_BASE + PWM_MR2) = ulValue;
                break;
            }
        case PWM_CH_3:
            {
                xHWREG(PWM1_BASE + PWM_MR3) = ulValue;
                break;
            }
        case PWM_CH_4:
            {
                xHWREG(PWM1_BASE + PWM_MR4) = ulValue;
                break;
            }
        case PWM_CH_5:
            {
                xHWREG(PWM1_BASE + PWM_MR5) = ulValue;
                break;
            }
        case PWM_CH_6:
            {
                xHWREG(PWM1_BASE + PWM_MR6) = ulValue;
                break;
            }

        default:                         // Error
            {
                while(1);
            }
    }

    // Write Latch Register.
    xHWREG(PWM1_BASE + PWM_LER) |= ulCh;
}

void PWMOutPutEnable(unsigned long ulBase, unsigned long ulChs)
{
    xHWREG(PWM1_BASE + PWM_PCR) |= (ulChs<<8);
}

void PWMOutPutDisable(unsigned long ulBase, unsigned long ulChs)
{
    xHWREG(PWM1_BASE + PWM_PCR) &= ~(ulChs<<8);
}

/*
//ulCfg
#define PWM_EDGE_DOUBLE         BIT_32_0
#define PWM_EDGE_SINGLE         BIT_32_1
*/

void PWMEdgeCfg(unsigned long ulBase, unsigned long ulChs, unsigned long ulCfg)
{
    switch(ulCfg)
    {
        case PWM_EDGE_DOUBLE:
            {
                xHWREG(PWM1_BASE + PWM_PCR) |= ulChs;
                break;
            }

        case PWM_EDGE_SINGLE:
            {
                xHWREG(PWM1_BASE + PWM_PCR) &= ~ulChs;
                break;
            }
        default:                         // Error
            {
                while(1);
            }
    }
}

/*
#define CH0_FALLING_SAMPLE_EN      BIT_32_0
#define CH0_FALLING_SAMPLE_DIS     BIT_32_8
#define CH0_RISING_SAMPLE_EN       BIT_32_1
#define CH0_RISING_SAMPLE_DIS      BIT_32_9
#define CH0_EDGE_EVENT_INT_EN      BIT_32_2
#define CH0_EDGE_EVENT_INT_DIS     BIT_32_10
#define CH1_FALLING_SAMPLE_EN      BIT_32_3
#define CH1_FALLING_SAMPLE_DIS     BIT_32_11
#define CH1_RISING_SAMPLE_EN       BIT_32_4
#define CH1_RISING_SAMPLE_DIS      BIT_32_12
#define CH1_EDGE_EVENT_INT_EN      BIT_32_5
#define CH1_EDGE_EVENT_INT_DIS     BIT_32_13
*/

void PWMCapCfg(unsigned long ulBase, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    ulTmpReg = xHWREG(PWM1_BASE + PWM_CCR);
    ulTmpReg &= ~((ulCfg >> 8) & 0xFF);
    ulTmpReg |=  ((ulCfg >> 0) & 0xFF);
    xHWREG(PWM1_BASE + PWM_CCR) = ulTmpReg;

}


