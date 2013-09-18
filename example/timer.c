
#include "xhw_types.h"
#include "xhw_ints.h"
#include "xhw_memmap.h"
#include "xhw_nvic.h"
#include "xdebug.h"
#include "xcore.h"
#include "xhw_sysctl.h"
#include "xsysctl.h"
#include "xhw_gpio.h"
#include "xgpio.h"
#include "xhw_timer.h"
#include "xtimer.h"

#define TICK_SLOW              ((unsigned long)0xFFFFF)

static void Delay(unsigned long tick) 
{
    volatile unsigned long _tick  = tick;
    while(_tick--);

}

unsigned long TimerHandler(void *pvCBData, unsigned long ulEvent,
                              unsigned long ulMsgParam, void *pvMsgData)
{
    TimerIntStatusClear(TIMER0_BASE, TIMER_INT_CAP_CH_0 | TIMER_INT_MAT_CH_0);
    return (0);
}

unsigned long Tmp = 0;
unsigned long Status = 0;

void main(void)
{ 
    unsigned long i = 0;

    /********************** Configure System clock *************************/
    SysCtlClockSet(100000000, SYSCTL_OSC_INT | SYSCTL_XTAL_12_MHZ);
    Delay(TICK_SLOW);
    xGPIOSPinTypeGPIOOutput(PA22);
    xGPIOSPinTypeGPIOOutput(PB23);

    /*
    TimerStart(TIMER0_BASE);
    TimerReset(TIMER0_BASE);    
    TimerStop(TIMER0_BASE);
    TimerReset(TIMER0_BASE);
    */

    /*
    /////////////////////////////////////////////////////////////////////////
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_INT);
    TimerMatchCf 
    g(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_RESET);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_STOP);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_INT | TIMER_MAT_STOP);

    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_PIN_LOW);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_PIN_HIGH);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_PIN_TOGGLE);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_PIN_NONE);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_PIN_TOGGLE);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_RESET | TIMER_MAT_PIN_NONE);

    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_INT | TIMER_MAT_STOP |  TIMER_MAT_PIN_TOGGLE);

    /////////////////////////////////////////////////////////////////////////
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0 | TIMER_MAT_CH_2, TIMER_MAT_INT);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0 | TIMER_MAT_CH_2, TIMER_MAT_RESET);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0 | TIMER_MAT_CH_2, TIMER_MAT_STOP);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0 | TIMER_MAT_CH_2, TIMER_MAT_INT | TIMER_MAT_STOP);
                                            
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0 | TIMER_MAT_CH_2, TIMER_MAT_PIN_LOW);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0 | TIMER_MAT_CH_2, TIMER_MAT_PIN_HIGH);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0 | TIMER_MAT_CH_2, TIMER_MAT_PIN_TOGGLE);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0 | TIMER_MAT_CH_2, TIMER_MAT_PIN_NONE);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0 | TIMER_MAT_CH_2, TIMER_MAT_PIN_TOGGLE);
                                           
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0 | TIMER_MAT_CH_2, TIMER_MAT_INT | TIMER_MAT_STOP |  TIMER_MAT_PIN_TOGGLE);

    /////////////////////////////////////////////////////////////////////////
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_INT);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_RESET);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_STOP);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_INT | TIMER_MAT_STOP);

    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_PIN_LOW);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_PIN_HIGH);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_PIN_TOGGLE);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_PIN_NONE);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_PIN_TOGGLE);

    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_INT | TIMER_MAT_STOP |  TIMER_MAT_PIN_TOGGLE);
    */


    /*
    TimerMatchValueSet(TIMER0_BASE, TIMER_MAT_CH_0, 0x00);
    Tmp = TimerMatchValueGet(TIMER0_BASE, TIMER_MAT_CH_0);
    TimerMatchValueSet(TIMER0_BASE, TIMER_MAT_CH_0, 0x55);
    Tmp = TimerMatchValueGet(TIMER0_BASE, TIMER_MAT_CH_0);
    TimerMatchValueSet(TIMER0_BASE, TIMER_MAT_CH_0, 0xF2);
    Tmp = TimerMatchValueGet(TIMER0_BASE, TIMER_MAT_CH_0);

    TimerMatchValueSet(TIMER0_BASE, TIMER_MAT_CH_1, 0x00);
    Tmp = TimerMatchValueGet(TIMER0_BASE, TIMER_MAT_CH_1);
    TimerMatchValueSet(TIMER0_BASE, TIMER_MAT_CH_1, 0x55);
    Tmp = TimerMatchValueGet(TIMER0_BASE, TIMER_MAT_CH_1);
    TimerMatchValueSet(TIMER0_BASE, TIMER_MAT_CH_1, 0xF2);
    Tmp = TimerMatchValueGet(TIMER0_BASE, TIMER_MAT_CH_1);
    */


    //TimerPrescaleSet(TIMER0_BASE, 100000000/10-1);
    //Tmp = TimerPrescaleGet(TIMER0_BASE);
    
    /*
    TimerCounterCfg(TIMER0_BASE, TIMER_CFG_CNT_CAP0_RISING);
    TimerCounterCfg(TIMER0_BASE, TIMER_CFG_CNT_CAP0_FALLING);
    TimerCounterCfg(TIMER0_BASE, TIMER_CFG_CNT_CAP0_BOTH);

    TimerCounterCfg(TIMER0_BASE, TIMER_CFG_CNT_CAP1_RISING);
    TimerCounterCfg(TIMER0_BASE, TIMER_CFG_CNT_CAP1_FALLING);
    TimerCounterCfg(TIMER0_BASE, TIMER_CFG_CNT_CAP1_BOTH);
    */

    /*
    TimerCaptureCfg(TIMER0_BASE, TIMER_CAP_CH_0, TIMER_CFG_CAP_RISING);
    TimerCaptureCfg(TIMER0_BASE, TIMER_CAP_CH_0, TIMER_CFG_CAP_FALLING);
    TimerCaptureCfg(TIMER0_BASE, TIMER_CAP_CH_0, TIMER_CFG_CAP_INT);
    TimerCaptureCfg(TIMER0_BASE, TIMER_CAP_CH_0, TIMER_CFG_CAP_FALLING | TIMER_CFG_CAP_INT);

    TimerCaptureCfg(TIMER0_BASE, TIMER_CAP_CH_1, TIMER_CFG_CAP_RISING);
    TimerCaptureCfg(TIMER0_BASE, TIMER_CAP_CH_1, TIMER_CFG_CAP_FALLING);
    TimerCaptureCfg(TIMER0_BASE, TIMER_CAP_CH_1, TIMER_CFG_CAP_INT);
    TimerCaptureCfg(TIMER0_BASE, TIMER_CAP_CH_1, TIMER_CFG_CAP_FALLING | TIMER_CFG_CAP_INT);
    */

    /************** Timer Interrupt **************************************/
    /*
    GPIOPinFunCfg(GPIOB_BASE, GPIO_PIN_28,  GPIO_PB28_TIM_MAT0_0);
    xIntEnable(xINT_TIMER0);
    TimerIntCallbackInit(TIMER0_BASE, TimerHandler);
    TimerPrescaleSet(TIMER0_BASE, 1000-1);
    TimerMatchCfg(TIMER0_BASE, TIMER_MAT_CH_0, TIMER_MAT_RESET | TIMER_MAT_INT | TIMER_MAT_PIN_TOGGLE);
    TimerMatchValueSet(TIMER0_BASE, TIMER_MAT_CH_0, 5000);
    TimerReset(TIMER0_BASE);
    TimerStart(TIMER0_BASE);
    */

    /************** Timer Capture Interrupt **************************************/

    /*
    // Map GPIO
    GPIOPinFunCfg(GPIOB_BASE, GPIO_PIN_26,  GPIO_PB26_TIM_CAP0_0);

    xIntEnable(xINT_TIMER0);
    TimerIntCallbackInit(TIMER0_BASE, TimerHandler);
    TimerPrescaleSet(TIMER0_BASE, 1);
    TimerCaptureCfg(TIMER0_BASE, TIMER_CAP_CH_0, TIMER_CFG_CAP_RISING | TIMER_CFG_CAP_INT);
    TimerReset(TIMER0_BASE);
    TimerStart(TIMER0_BASE);
    */

    /************** Timer Counter Mode **************************************/

    /*
    GPIOPinFunCfg(GPIOB_BASE, GPIO_PIN_26,  GPIO_PB26_TIM_CAP0_0);

    TimerCounterCfg(TIMER0_BASE, TIMER_CFG_CNT_CAP0_BOTH);
    TimerReset(TIMER0_BASE);
    TimerStart(TIMER0_BASE);

    while (1)
    {
        GPIOPinSet(GPIOB_BASE, GPIO_PIN_23);
        Delay(TICK_SLOW);
        GPIOPinClr(GPIOB_BASE, GPIO_PIN_23);
        Delay(TICK_SLOW);

        if( (TimerValueGet(TIMER0_BASE) % 10) == 0)
        {
            if(Status)
            {
                Status = 0;
                GPIOPinSet(GPIOA_BASE, GPIO_PIN_22);
            }
            else
            {
                Status = 1;
                GPIOPinClr(GPIOA_BASE, GPIO_PIN_22);
            }
        }
    }
    */

    while(1);
}

