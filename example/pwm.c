
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
#include "xhw_pwm.h"
#include "xpwm.h"

#define TICK_SLOW              ((unsigned long)0xFFFFF)

static void Delay(unsigned long tick) 
{
    volatile unsigned long _tick  = tick;
    while(_tick--);

}


void main(void)
{ 

    /********************** Configure System clock *************************/
    SysCtlClockSet(100000000, SYSCTL_OSC_INT | SYSCTL_XTAL_12_MHZ);
    Delay(TICK_SLOW);


    while (1)
    {
    }
}

