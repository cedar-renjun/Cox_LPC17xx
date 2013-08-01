


#define WDT_CFG_INT_MODE        BIT_32_0
#define WDT_CFG_RESET_MODE      BIT_32_1 | BIT_32_0

#define WDT_CFG_CLKSRC_IRC      BIT_32_2
#define WDT_CFG_CLKSRC_APB      BIT_32_3
#define WDT_CFG_CLKSRC_RTC      BIT_32_3 | BIT_32_2

//! Watchdog time-out flag.
#define WDT_FLAG_TIMEOUT        BIT_32_2

//! Watchdog interrupt flag.
#define WDT_FLAG_INT            BIT_32_3

extern void WDTCfg(unsigned long ulCfg, unsigned long ulValue);
extern void WDTFeed(void);
extern void WDTEnable(void);
extern unsigned long WDTIntCallbackInit(xtEventCallback pfnCallback);

extern unsigned long WDTStatusFlagGet(void);
extern xtBoolean WDTStatusFlagCheck(unsigned long ulFlags);
extern void WDTStatusFlagClear(unsigned long ulFlags);

