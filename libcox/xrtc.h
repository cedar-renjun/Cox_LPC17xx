
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

//! Passed
extern void RTCTimeSet(unsigned long ulType, unsigned long ulValue);
//! Passed
extern unsigned long RTCTimeGet(unsigned long ulType);
//! Passed
extern void RTCAlarmSet(unsigned long ulType, unsigned long ulValue);
//! Passed
extern unsigned long RTCAlarmGet(unsigned long ulType);
//! Passed
extern void RTCGenRegWrite(unsigned long ulID, unsigned long ulValue);
//! Passed
extern unsigned long RTCGenRegRead(unsigned long ulID);

//! Passed
extern unsigned long RTCIntFlagGet(void);
//! Passed
extern xtBoolean RTCIntFlagCheck(unsigned long ulFlags);
//! Passed
extern void RTCIntFlagClear(unsigned long ulFlags);

//! Passed
extern void RTCEnable(void);
//! Passed
extern void RTCDisable(void);
//! Passed
extern void RTCCounterReset(void);
//! Passed
extern void RTCCaliEnable(void);
//! Passed
extern void RTCCaliDisable(void);
//! Passed
extern void RTCIntCfg(unsigned long ulCfg);

extern unsigned long RTCIntCallbackInit(xtEventCallback pfnCallback);
