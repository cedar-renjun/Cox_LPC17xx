//! Watchdog mode register.
//! This register contains the basic mode and status of the Watchdog Timer.
#define WDT_MOD ((unsigned long)0x00000000)

//! Watchdog timer constant register.
//! This register determines the time-out value.
#define WDT_TC ((unsigned long)0x00000004)

//! Watchdog feed sequence register.
//! Writing 0xAA followed by 0x55 to this register reloads the Watchdog timer
//! with the value contained in WDTC.
#define WDT_FEED ((unsigned long)0x00000008)

//! Watchdog timer value register.
//! This register reads out the current value of the Watchdog timer.
#define WDT_TV ((unsigned long)0x0000000C)

//! Watchdog clock source selection register.
#define WDT_CLKSEL ((unsigned long)0x00000010)

//! WDT_MOD {{

//! Enable Watchdog
#define WDMOD_EN                BIT_32_0

//! Reset Mode.
#define WDMOD_RESET             BIT_32_1

//! Watchdog time-out flag.
#define WDMOD_TOF               BIT_32_2

//! Watchdog interrupt flag.
#define WDMOD_INT               BIT_32_3

//! WDT_MOD }}

//! WDT_CLKSEL {{

//! Watchdog clock source.
#define WDCLKSEL_WDSEL_M        BIT_MASK(32, 1, 0)

#define WDCLKSEL_WDSEL_IRC      BIT_32_ALL_0
#define WDCLKSEL_WDSEL_APB      BIT_32_0
#define WDCLKSEL_WDSEL_RTC      BIT_32_1

//! Lock watchdog register.
#define WDCLKSEL_WDLOCK         BIT_32_31

//! WDT_CLKSEL }}


