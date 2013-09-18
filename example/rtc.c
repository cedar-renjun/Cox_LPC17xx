
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
#include "xhw_rtc.h"
#include "xrtc.h"

#define TICK_SLOW              ((unsigned long)0xFFFFF)

static void Delay(unsigned long tick) 
{
    volatile unsigned long _tick  = tick;
    while(_tick--);

}

unsigned long RTCHandler(void *pvCBData, unsigned long ulEvent,
                              unsigned long ulMsgParam, void *pvMsgData)
{
    if(RTCIntFlagCheck(RTC_INT_INC))
    {
        RTCIntFlagClear(RTC_INT_INC);
        //while(1);
    }

    if(RTCIntFlagCheck(RTC_INT_ALARM))
    {
        RTCIntFlagClear(RTC_INT_ALARM);
        //while(1);
    }
    return (0);
}

void main(void)
{ 
    //unsigned long ulTmp = 0;

    /********************** Configure System clock *************************/
    SysCtlClockSet(100000000, SYSCTL_OSC_INT | SYSCTL_XTAL_12_MHZ);
    Delay(TICK_SLOW);


    /*
    RTCDisable();
    RTCEnable();
    RTCDisable();
    */

    /*
    RTCTimeSet(RTC_TIMETYPE_SECOND, 0);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_SECOND);
    if(ulTmp != 0)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_SECOND, 5);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_SECOND);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_SECOND, 255);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_SECOND);
    if(ulTmp == 255)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_MINUTE, 0);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_MINUTE);
    if(ulTmp != 0)
    {
        while(1);
    } 

    RTCTimeSet(RTC_TIMETYPE_MINUTE, 5);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_MINUTE);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_MINUTE, 255);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_MINUTE);
    if(ulTmp == 255)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_HOUR, 0);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_HOUR);
    if(ulTmp != 0)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_HOUR, 5);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_HOUR);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_HOUR, 255);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_HOUR);
    if(ulTmp == 255)
    {
        while(1);
    }
                  
    RTCTimeSet(RTC_TIMETYPE_DAYOFWEEK, 0);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_DAYOFWEEK);
    if(ulTmp != 0)
    {
        while(1);
    } 

    RTCTimeSet(RTC_TIMETYPE_DAYOFWEEK, 5);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_DAYOFWEEK);
    if(ulTmp != 5)
    {
        while(1);
    } 

    RTCTimeSet(RTC_TIMETYPE_DAYOFWEEK, 255); 
    ulTmp = RTCTimeGet(RTC_TIMETYPE_DAYOFWEEK);
    if(ulTmp == 255)
    {
        while(1);
    }  

    RTCTimeSet(RTC_TIMETYPE_DAYOFMONTH, 0);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_DAYOFMONTH);
    if(ulTmp != 1)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_DAYOFMONTH, 5);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_DAYOFMONTH);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_DAYOFMONTH, 255); 
    ulTmp = RTCTimeGet(RTC_TIMETYPE_DAYOFMONTH);
    if(ulTmp == 255)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_DAYOFYEAR, 0);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_DAYOFYEAR);
    if(ulTmp != 1)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_DAYOFYEAR, 5);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_DAYOFYEAR);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_DAYOFYEAR, 0xFFFF); 
    ulTmp = RTCTimeGet(RTC_TIMETYPE_DAYOFYEAR);
    if(ulTmp == 0xFFFF)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_MONTH, 0);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_MONTH);
    if(ulTmp != 1)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_MONTH, 5);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_MONTH);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_MONTH, 255); 
    ulTmp = RTCTimeGet(RTC_TIMETYPE_MONTH);
    if(ulTmp == 255)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_YEAR, 0);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_YEAR);
    if(ulTmp != 0)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_YEAR, 5);
    ulTmp = RTCTimeGet(RTC_TIMETYPE_YEAR);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCTimeSet(RTC_TIMETYPE_YEAR, 2008); 
    ulTmp = RTCTimeGet(RTC_TIMETYPE_YEAR);
    if(ulTmp != 2008)
    {
        while(1);
    }
    */

//////////////////// Alarm Set/Get ///////////////////////

    /*
    RTCAlarmSet(RTC_TIMETYPE_SECOND, 0);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_SECOND);
    if(ulTmp != 0)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_SECOND, 5);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_SECOND);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_SECOND, 255);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_SECOND);
    if(ulTmp == 255)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_MINUTE, 0);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_MINUTE);
    if(ulTmp != 0)
    {
        while(1);
    } 

    RTCAlarmSet(RTC_TIMETYPE_MINUTE, 5);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_MINUTE);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_MINUTE, 255);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_MINUTE);
    if(ulTmp == 255)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_HOUR, 0);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_HOUR);
    if(ulTmp != 0)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_HOUR, 5);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_HOUR);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_HOUR, 255);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_HOUR);
    if(ulTmp == 255)
    {
        while(1);
    }
                  
    RTCAlarmSet(RTC_TIMETYPE_DAYOFWEEK, 0);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_DAYOFWEEK);
    if(ulTmp != 0)
    {
        while(1);
    } 

    RTCAlarmSet(RTC_TIMETYPE_DAYOFWEEK, 5);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_DAYOFWEEK);
    if(ulTmp != 5)
    {
        while(1);
    } 

    RTCAlarmSet(RTC_TIMETYPE_DAYOFWEEK, 255); 
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_DAYOFWEEK);
    if(ulTmp == 255)
    {
        while(1);
    }  

    RTCAlarmSet(RTC_TIMETYPE_DAYOFMONTH, 0);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_DAYOFMONTH);
    if(ulTmp != 1)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_DAYOFMONTH, 5);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_DAYOFMONTH);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_DAYOFMONTH, 255); 
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_DAYOFMONTH);
    if(ulTmp == 255)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_DAYOFYEAR, 0);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_DAYOFYEAR);
    if(ulTmp != 1)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_DAYOFYEAR, 5);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_DAYOFYEAR);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_DAYOFYEAR, 0xFFFF); 
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_DAYOFYEAR);
    if(ulTmp == 0xFFFF)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_MONTH, 0);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_MONTH);
    if(ulTmp != 1)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_MONTH, 5);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_MONTH);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_MONTH, 255); 
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_MONTH);
    if(ulTmp == 255)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_YEAR, 0);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_YEAR);
    if(ulTmp != 0)
    {
        while(1);D
    }

    RTCAlarmSet(RTC_TIMETYPE_YEAR, 5);
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_YEAR);
    if(ulTmp != 5)
    {
        while(1);
    }

    RTCAlarmSet(RTC_TIMETYPE_YEAR, 2008); 
    ulTmp = RTCAlarmGet(RTC_TIMETYPE_YEAR);
    if(ulTmp != 2008)
    {
        while(1);
    }
    */
//////////////////// General Purpose registers ///////////////////////

    /*
    ulTmp = 0;
    RTCGenRegWrite(RTC_GPREG0, ulTmp);
    if(RTCGenRegRead(RTC_GPREG0) != ulTmp)
    {
        while(1);
    }

    ulTmp = 100;
    RTCGenRegWrite(RTC_GPREG0, ulTmp);
    if(RTCGenRegRead(RTC_GPREG0) != ulTmp)
    {
        while(1);
    }

    ulTmp = 0;
    RTCGenRegWrite(RTC_GPREG1, ulTmp);
    if(RTCGenRegRead(RTC_GPREG1) != ulTmp)
    {
        while(1);
    }

    ulTmp = 100;
    RTCGenRegWrite(RTC_GPREG1, ulTmp);
    if(RTCGenRegRead(RTC_GPREG1) != ulTmp)
    {
        while(1);
    }

    ulTmp = 0;
    RTCGenRegWrite(RTC_GPREG2, ulTmp);
    if(RTCGenRegRead(RTC_GPREG2) != ulTmp)
    {
        while(1);
    }

    ulTmp = 100;
    RTCGenRegWrite(RTC_GPREG2, ulTmp);
    if(RTCGenRegRead(RTC_GPREG2) != ulTmp)
    {
        while(1);
    }

    ulTmp = 0;
    RTCGenRegWrite(RTC_GPREG3, ulTmp);
    if(RTCGenRegRead(RTC_GPREG3) != ulTmp)
    {
        while(1);
    }

    ulTmp = 100;
    RTCGenRegWrite(RTC_GPREG3, ulTmp);
    if(RTCGenRegRead(RTC_GPREG3) != ulTmp)
    {
        while(1);
    }

    ulTmp = 0;
    RTCGenRegWrite(RTC_GPREG4, ulTmp);
    if(RTCGenRegRead(RTC_GPREG4) != ulTmp)
    {
        while(1);
    }

    ulTmp = 100;
    RTCGenRegWrite(RTC_GPREG4, ulTmp);
    if(RTCGenRegRead(RTC_GPREG4) != ulTmp)
    {
        while(1);
    }
    */

//////////// Counter Reset ////////////////////////////////
    /*
    RTCTimeSet(RTC_TIMETYPE_HOUR, 255);
    RTCTimeSet(RTC_TIMETYPE_DAYOFWEEK, 5);
    RTCEnable();
    RTCCounterReset();
    */

//////////// Calib Enable/Disable /////////////////////////
    /*
    RTCCaliEnable();
    RTCCaliDisable();
    RTCCaliEnable();
    RTCCaliDisable();
    */

//////////// Interrupt Configure //////////////////////////
    /*
    RTCIntCfg(INT_SEC_EN         );
    RTCIntCfg(INT_MIN_EN         );
    RTCIntCfg(INT_HOUR_EN        );
    RTCIntCfg(INT_DOM_EN         );
    RTCIntCfg(INT_DOW_EN         );
    RTCIntCfg(INT_DOY_EN         );
    RTCIntCfg(INT_MON_EN         );
    RTCIntCfg(INT_YEAR_EN        );

    RTCIntCfg(INT_SEC_DIS        );
    RTCIntCfg(INT_MIN_DIS        );
    RTCIntCfg(INT_HOUR_DIS       );
    RTCIntCfg(INT_DOM_DIS        );
    RTCIntCfg(INT_DOW_DIS        );
    RTCIntCfg(INT_DOY_DIS        );
    RTCIntCfg(INT_MON_DIS        );
    RTCIntCfg(INT_YEAR_DIS       );

    RTCIntCfg(INT_ALARM_SEC_EN   );
    RTCIntCfg(INT_ALARM_MIN_EN   );
    RTCIntCfg(INT_ALARM_HOUR_EN  );
    RTCIntCfg(INT_ALARM_DOM_EN   );
    RTCIntCfg(INT_ALARM_DOW_EN   );
    RTCIntCfg(INT_ALARM_DOY_EN   );
    RTCIntCfg(INT_ALARM_MON_EN   );
    RTCIntCfg(INT_ALARM_YEAR_EN  );

    RTCIntCfg(INT_ALARM_SEC_DIS  );
    RTCIntCfg(INT_ALARM_MIN_DIS  );
    RTCIntCfg(INT_ALARM_HOUR_DIS );
    RTCIntCfg(INT_ALARM_DOM_DIS  );
    RTCIntCfg(INT_ALARM_DOW_DIS  );
    RTCIntCfg(INT_ALARM_DOY_DIS  );
    RTCIntCfg(INT_ALARM_MON_DIS  );
    RTCIntCfg(INT_ALARM_YEAR_DIS );
    */
///////////////// Interrupt Function test /////////////////

    /*
    xIntDisable(xINT_RTC);
    RTCDisable();
    RTCIntCallbackInit(RTCHandler);
    RTCIntCfg( INT_SEC_DIS         |   INT_MIN_EN          |
               INT_HOUR_DIS        |   INT_DOM_DIS         |
               INT_DOW_DIS         |   INT_DOY_DIS         |
               INT_MON_DIS         |   INT_YEAR_DIS        |
               INT_ALARM_SEC_EN    |   INT_ALARM_MIN_DIS   |
               INT_ALARM_HOUR_DIS  |   INT_ALARM_DOM_DIS   |
               INT_ALARM_DOW_DIS   |   INT_ALARM_DOY_DIS   |
               INT_ALARM_MON_DIS   |   INT_ALARM_YEAR_DIS  );

    RTCAlarmSet(RTC_TIMETYPE_SECOND, 10);
    RTCEnable();
    RTCCounterReset();
    xIntEnable(xINT_RTC);
    */

    while (1)
    {
        ;
    }
}

