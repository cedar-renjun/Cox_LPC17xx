
//! \addtogroup SysCtlClockSet_ulConfig The input parameter ulConfig
//!  set of SysCtlClockSet function.
//! @{

//! Clock Frequency bit[4:0] mask.
//! \note This macro is used for \ref SysCtlClockSet function.
#define SYSCTL_XTAL_nMHZ_MASK                    BIT_MASK(32, 4, 0)

//! Main Oscillator 1 Mhz.
#define SYSCTL_XTAL_1_MHZ                        1                                

//! Main Oscillator 2 Mhz
#define SYSCTL_XTAL_2_MHZ                        2                                

//! Main Oscillator 3 Mhz
#define SYSCTL_XTAL_3_MHZ                        3                                

//! Main Oscillator 4 Mhz
#define SYSCTL_XTAL_4_MHZ                        4                                

//! Main Oscillator 5 Mhz
#define SYSCTL_XTAL_5_MHZ                        5                                

//! Main Oscillator 6 Mhz
#define SYSCTL_XTAL_6_MHZ                        6                                

//! Main Oscillator 7 Mhz
#define SYSCTL_XTAL_7_MHZ                        7                                

//! Main Oscillator 8 Mhz
#define SYSCTL_XTAL_8_MHZ                        8                                

//! Main Oscillator 9 Mhz
#define SYSCTL_XTAL_9_MHZ                        9                                

//! Main Oscillator 10 Mhz
#define SYSCTL_XTAL_10_MHZ                       10                               

//! Main Oscillator 11 Mhz
#define SYSCTL_XTAL_11_MHZ                       11                               

//! Main Oscillator 12 Mhz
#define SYSCTL_XTAL_12_MHZ                       12                               

//! Main Oscillator 13 Mhz
#define SYSCTL_XTAL_13_MHZ                       13                               

//! Main Oscillator 14 Mhz
#define SYSCTL_XTAL_14_MHZ                       14                               

//! Main Oscillator 15 Mhz
#define SYSCTL_XTAL_15_MHZ                       15                               

//! Main Oscillator 16 Mhz
#define SYSCTL_XTAL_16_MHZ                       16                               

//! Main Oscillator 17 Mhz
#define SYSCTL_XTAL_17_MHZ                       17                               

//! Main Oscillator 18 Mhz
#define SYSCTL_XTAL_18_MHZ                       18                               

//! Main Oscillator 19 Mhz
#define SYSCTL_XTAL_19_MHZ                       19                               

//! Main Oscillator 20 Mhz
#define SYSCTL_XTAL_20_MHZ                       20                               

//! Main Oscillator 21 Mhz
#define SYSCTL_XTAL_21_MHZ                       21                               

//! Main Oscillator 22 Mhz
#define SYSCTL_XTAL_22_MHZ                       22                               

//! Main Oscillator 23 Mhz
#define SYSCTL_XTAL_23_MHZ                       23                               

//! Main Oscillator 24 Mhz
#define SYSCTL_XTAL_24_MHZ                       24                               

//! Main Oscillator 25Mhz
#define SYSCTL_XTAL_25_MHZ                       25                               

//! Device Maximum Clock Speed,120Mhz for LPC1759/LPC1759, 100MHz for others Device.
#define SYSTEM_CLOCK_MAX                         ((unsigned long)120000000)

//! Use Main Oscillator as input clock source.
#define SYSCTL_OSC_MAIN                          BIT_32_5

//! Use Internal Oscillator as input clock source.
#define SYSCTL_OSC_INT                           BIT_32_6

//! Disable Internal Oscillator.
#define SYSCTL_INT_OSC_DIS                       BIT_32_7

//! Disable Main Oscillator.
#define SYSCTL_MAIN_OSC_DIS                      BIT_32_8

//! Power Down PLL Module.
#define SYSCTL_PLL_PWRDN                         BIT_32_9

//! @}

//! MCO {{
void   SysCtlMcoCfg(unsigned long ulCfg);
void   SysCtlMcoEnable(void);
void   SysCtlMcoDisable(void);
xtBoolean SysCtlMcoStatus(void);
//! MCO }}


//! POWER {{

void SysCtlPeripheralReset(unsigned long ulPeripheral);
void SysCtlPeripheralEnable(unsigned long ulPeripheral);
void SysCtlPeripheralDisable(unsigned long ulPeripheral);

unsigned long SysCtlPowerFlagGet(void);
void SysCtlPowerFlagClear(unsigned long ulFlag);


//! POWER }}

//! System Clock Configure {{

//! ulSysClk --> SYSCTL_XTAL_nMHZ
void SysCtlClockSet(unsigned long ulSysClk, unsigned long ulConfig);


//! System Clock Configure }}



//! External Interrupt {{

void SysCtlExtIntCfg(unsigned long ulPin, unsigned long ulCfg);


//! External Interrupt }}

//! Power on reset
#define RESET_FLAG_POR                RSID_POR

//! External reset signal
#define RESET_FLAG_EXTR               RSID_EXTR               

//! Watchdog Timer reset
#define RESET_FLAG_WDTR               RSID_WDTR               

//! Brown-out reset
#define RESET_FLAG_BODR               RSID_BODR               

//! System reset requet reset
#define RESET_FLAG_SYSRESET           RSID_SYSRESET           

//! Lockup reset
#define RESET_FLAG_LOCKUP             RSID_LOCKUP             

unsigned long SysCtlResetFlagGet(void);





















