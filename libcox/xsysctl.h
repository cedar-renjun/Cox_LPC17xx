
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


#if defined(LPC_177x) | defined(LPC_178x)
//! LCD Controller power/clock control bit.
#define SYSCTL_PERIPH_ LCD             PCONP_PCLCD
#endif

//! Timer/Counter 0 power/clock control bit.
#define SYSCTL_PERIPH_TIM0             PCONP_PCTIM0

//! Timer/Counter 1 power/clock control bit.
#define SYSCTL_PERIPH_TIM1             PCONP_PCTIM1

//! UART0 Power/clock control bit.
#define SYSCTL_PERIPH_UART0           PCONP_PCUART0           

//! UART1 Power/clock control bit.
#define SYSCTL_PERIPH_UART1           PCONP_PCUART1           

//! PWM0 Power/Clock control bit.
#define SYSCTL_PERIPH_PWM0            PCONP_PCPWM0            

//! PWM1 Power/Clock control bit.
#define SYSCTL_PERIPH_PWM1            PCONP_PCPWM1            

//! I2C0 Interface Power/Clock control bit.
#define SYSCTL_PERIPH_I2C0            PCONP_PCI2C0            

#if defined(LPC_175x) | defined(LPC_176x)
//! The SPI interface power/clock control bit.
#define SYSCTL_PERIPH_SPI             PCONP_PCSPI             

#elif defined(LPC_177x) | defined(LPC_178x)
//! UART4 power/clock control bit.
#define SYSCTL_PERIPH_UART4           PCONP_PCUART4           

#endif

//! RTC and Event Monitor/Recorder power/clock control bit.
#define SYSCTL_PERIPH_RTC             PCONP_PCRTC             

//! SSP 1 interface power/clock control bit.
#define SYSCTL_PERIPH_SSP1            PCONP_PCSSP1            

#if defined(LPC_177x) | defined(LPC_178x)
//! External Memory Controller power/clock control bit.
#define SYSCTL_PERIPH_EMC             PCONP_PCEMC             
#endif

//! A/D converter (ADC) power/clock control bit.
#define SYSCTL_PERIPH_ADC             PCONP_PCADC             

//! CAN Controller 1 power/clock control bit.
#define SYSCTL_PERIPH_CAN1            PCONP_PCCAN1            

//! CAN Controller 2 power/clock control bit.
#define SYSCTL_PERIPH_CAN2            PCONP_PCCAN2            

//! Power/clock control bit for IOCON, GPIO, and GPIO interrupts.
#define SYSCTL_PERIPH_GPIO            PCONP_PCGPIO            

#if defined(LPC_175x) | defined(LPC_176x)
//! Repetitive Interrupt Timer power/clock control bit.
#define SYSCTL_PERIPH_RIT             PCONP_PCRIT             

#elif defined(LPC_177x) | defined(LPC_178x)
//! SPI Flash Interface power/clock control bit.
#define SYSCTL_PERIPH_SPIFI           PCONP_PCSPIFI           

#endif

//! Motor Control PWM power/clock control bit.
#define SYSCTL_PERIPH_MCPWM           PCONP_PCMCPWM           

//! Quadrature Encoder Interface power/clock control bit.
#define SYSCTL_PERIPH_QEI             PCONP_PCQEI             

//! I2C1 interface power/clock control bit.
#define SYSCTL_PERIPH_I2C1            PCONP_PCI2C1            

#if defined(LPC_177x) | defined(LPC_178x)
//! SSP2 interface power/clock control bit.
#define SYSCTL_PERIPH_SSP2            PCONP_PCSSP2            
#endif

//! SSP0 interface power/clock control bit.
#define SYSCTL_PERIPH_SSP0            PCONP_PCSSP0            

//! Timer 2 power/clock control bit.
#define SYSCTL_PERIPH_TIM2            PCONP_PCTIM2            

//! Timer 3 power/clock control bit.
#define SYSCTL_PERIPH_TIM3            PCONP_PCTIM3            

//! UART 2 power/clock control bit.
#define SYSCTL_PERIPH_UART2           PCONP_PCUART2           

//! UART 3 power/clock control bit.
#define SYSCTL_PERIPH_UART3           PCONP_PCUART3           

//! I2C interface 2 power/clock control bit.
#define SYSCTL_PERIPH_I2C2            PCONP_PCI2C2            

//! I2S interface power/clock control bit.
#define SYSCTL_PERIPH_I2S             PCONP_PCI2S             

#if defined(LPC_177x) | defined(LPC_178x)
//! SD Card interface power/clock control bit.
#define SYSCTL_PERIPH_SDC             PCONP_PCSDC             
#endif

//! GPDMA function power/clock control bit.
#define SYSCTL_PERIPH_GPDMA           PCONP_PCGPDMA           

//! Ethernet block power/clock control bit.
#define SYSCTL_PERIPH_ETH             PCONP_PCENET            

//! USB interface power/clock control bit.
#define SYSCTL_PERIPH_USB             PCONP_PCUSB             

//! Reset control bit for the IOCON registers
#define SYSCTL_PERIPH_IOCON           (RSTCON1_RSTIOCON + 32)

//! D/A converter (DAC) reset control bit
#define SYSCTL_PERIPH_DAC             (RSTCON1_RSTDAC + 32)

//! CAN acceptance filter reset control bit
#define SYSCTL_PERIPH_CANACC          (RSTCON1_RSTCANACC + 32)


extern void SysCtlPeripheralEnable(unsigned long ulPeripheral);
extern void SysCtlPeripheralDisable(unsigned long ulPeripheral);
extern void SysCtlPeripheralReset(unsigned long ulPeripheral);




#define EXT_INT_0                      BIT_32_0
#define EXT_INT_1                      BIT_32_1
#define EXT_INT_2                      BIT_32_2 
#define EXT_INT_3                      BIT_32_3

#define EXT_INT_MASK                   BIT_MASK(32, 3, 0)

#define EXT_INT_LV_H                   BIT_32_0
#define EXT_INT_LV_L                   BIT_32_1
#define EXT_INT_EG_R                   BIT_32_2
#define EXT_INT_EG_F                   BIT_32_3

extern void SysCtlExtIntCfg(unsigned long ulPin, unsigned long ulCfg);





extern void SysCtlDelay(unsigned long ulCount);





#define PWR_MODE_SLEEP               BIT_32_0
#define PWR_MODE_SLEEP_D             BIT_32_1
#define PWR_MODE_PWRDOWN             BIT_32_2
#define PWR_MODE_PWRDOWN_D           BIT_32_3
extern unsigned long SysCtlPwrCfg(unsigned long ulMode);


#define BOD_REDUCE_PWR_EN         BIT_32_18
#define BOD_REDUCE_PWR_DIS        BIT_32_2
#define BOD_GLOBAL_EN             BIT_32_19
#define BOD_GLOBAL_DIS            BIT_32_3
#define BOD_RESET_EN              BIT_32_20
#define BOD_RESET_DIS             BIT_32_4
extern void SysCtlBODCfg(unsigned long ulCfg);

