//! \file xsysctl.c

static unsigned long PLLMNCal(unsigned long Fin,
        unsigned long Fout, unsigned long * pM, unsigned long *pN);






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

//*****************************************************************************
//
//! \brief Configure the system clock of the device.
//! 
//! This function configures the clock of the device.  The input crystal
//! frequency, oscillator to be used, use of the PLL, and the system clock
//! divider are all configured with this function.
//!
//! \param [in] ulSysClk is the target system clock frequency.
//! \param [in] ulConfig is the required configuration of the device clock.
//!
//! The \e ulConfig parameter is the logical OR of several different values,
//! many of which are grouped into sets where only one can be chosen.
//!
//! The external crystal frequency is chosen with one of the following values:
//! - \ref SYSCTL_XTAL_1_MHZ
//! - \ref SYSCTL_XTAL_2_MHZ  
//! - ...
//! - \ref SYSCTL_XTAL_25_MHZ  
//!
//! The oscillator source is chosen with one of the following values:
//! - \ref SYSCTL_OSC_MAIN
//! - \ref SYSCTL_OSC_INT
//!
//! The internal and main oscillators and PLL are disabled with the
//! - \ref SYSCTL_INT_OSC_DIS
//! - \ref SYSCTL_MAIN_OSC_DIS
//! - \ref SYSCTL_PLL_PWRDN 
//! 
//! \return None.
//!
//! \note 
//!      The external oscillator must be enabled in order to use an external clock
//!      source.  Note that attempts to disable the oscillator used to clock the
//!      device will be prevented by the hardware.
//!
//
//*****************************************************************************
void SysCtlClockSet(unsigned long ulSysClk, unsigned long ulConfig)
{

    unsigned long ulTmpReg = 0;      // Temporary register
    unsigned long ulRes    = 0;      // Store Result value
    unsigned long ulM      = 0;      // PLL Multiplier
    unsigned long ulN      = 0;      // PLL Divider

    /************** Check input parameters valid ********************/
    // ulSysClk clock range: 0 --> 12MHz 
    xASSERT( (ulSysClk > 0) && (ulSysClk <= SYSTEM_CLOCK_MAX) );

    // Check ulConfig valid
    xASSERT((ulConfig & ~BIT_MASK(32, 9, 0)) == 0);

    /************** Configure Flash Accelerator  ********************/
    // use 6 CPU clocks.
    // This "safe" setting will work under any conditions
    ulTmpReg = xHWREG(FLASHCFG);
    ulTmpReg &= ~FLASHCFG_FLASHTIM_M;
    ulTmpReg |=  FLASHCFG_FLASHTIM_ANY;
    xHWREG(FLASHCFG) = ulTmpReg;

    /************** Configure Main Oscillator    ********************/
    // Need Enable Main Oscillator ?
    if(ulConfig & SYSCTL_OSC_MAIN)              // Enable Main Osc
    {
        ulTmpReg = xHWREG(SCS);
        if((ulTmpReg & SCS_OSCSTAT) != SCS_OSCSTAT_RDY)
        {
            // Enable Main Osc
            ulTmpReg |= SCS_OSCEN;
            xHWREG(SCS) = ulTmpReg;

            // Waitting for Main Osc stable.
            do
            {
                ulTmpReg =  xHWREG(SCS);
                ulTmpReg &= SCS_OSCSTAT;
            }while(ulTmpReg != SCS_OSCSTAT_RDY);
        }
    }

    /************** Configure Internal oscillator ********************/
    // For LPC 17xx serial MCU, Internal RC is always enable.
    // so, you need not deal with those parameters
    // 1) SYSCTL_OSC_INT
    // 2) SYSCTL_INT_OSC_DIS
    //
    // if(ulConfig & SYSCTL_OSC_INT)             // Enable Internal Osc
    // {
    //     //Do Nothing Here.
    // }
    // 
    // if(ulConfig & SYSCTL_INT_OSC_DIS)        // Disable Internal Osc
    // {
    //     //Do Nothing Here.
    // }

    /************** Configure PLL                 ********************/
    // Need to Turn off PLL ?
    // if(ulConfig & SYSCTL_PLL_PWRDN)
    // {
    //     //Do Nothing Here
    // }

    // Get Input Frequency (unit: Hz)
    ulTmpReg = (ulConfig & SYSCTL_XTAL_nMHZ_MASK) * 1000000;
    ulRes = PLLMNCal(ulTmpReg, ulSysClk, &ulM, &ulN);
    if(!ulRes)                                  // Configure Failure
    {
        while(1);
    }

    // Check PLL Enable/Connect Status
    // if Enable and Connect, then disconnect it.
    // At last, Reconfigure PLL and connect to system clock.

    // PLL Connected ?
    ulTmpReg = xHWREG(PLL0STAT);
    if(ulTmpReg & PLL0STAT_PLLC_STAT)           // Connected to System Clock
    {
        // Disconnect PLL
        ulTmpReg = xHWREG(PLL0CON);
        ulTmpReg &= PLL0CON_PLLC;
        xHWREG(PLL0CON) = ulTmpReg;

        // Write key to PLL Feed register
        xHWREG(PLL0FEED) = (unsigned long)0xAA;
        xHWREG(PLL0FEED) = (unsigned long)0x55;

        // waiting for disconnect.

    }

    // PLL Enable ?
    if()



    // Default: F(system) = Fcco/4
#if defined(LPC_175x) | defined(LPC_176x)
    xHWREG(CCLKCFG) = (unsigned long) 3;
#elif defined(LPC_177x) | defined(LPC_178x)
    xHWREG(CCLKSEL) = ((unsigned long) 4 | CCLKSEL_CCLKSEL);
#endif
    // 
}


//*****************************************************************************
//
//! \brief       Calculate the PLL Multiplier/Diviver
//! \param [in]  Fin is the input clock frequency (1Mhz --> 25Mhz).
//! \param [in]  Fout is the target clock frequency, Maximum value is 120Mhz.
//! \param [out] pM is the Calculate result of multipler.
//! \param [out] pN is the Calculate result of divider.
//! \return      the result of calculate
//!              - 0 Failure
//!              - 1 Success
//! \note
//!              - This function is internal use, user MUST NOT call it.
//!              - pM and pN is the pointer, must be point to a valid variable.
//!              - This function only used for calculate internal RC and
//!                oscillator, NOT supply RTC(32.768KHz).
//
//*****************************************************************************
static unsigned long PLLMNCal(unsigned long Fin,
        unsigned long Fout, unsigned long * pM, unsigned long *pN)
{
    unsigned long M    = 0;   // PLL Multiplier Value
    unsigned long N    = 0;   // PLL Divider Value
    unsigned long Fcco = 0;   // PLL Fcco Output frequency

    /************ Check Input parameters ****************/
    // Input Frequency Range: 1 --> 25 Mhz
    if((Fin < 1000000) || (Fin > 25000000))
    {
        return (0);
    }

    // Target Maximum Frequency Range: 10 --> 120Mhz
    if( (Fout == 0) || (Fout >= 120000000) )
    {
        return (0);
    }

    // Check pN/pM Pointer valid
    if((pN == 0) || (pM == 0))
    {
        return (0);
    }

    /************ Calculate Multiplier/Divider **********/
    for(N = 1; N < 32; N++)
    {

        // The value of PLOCK0 may not be stable when the PLL reference frequency
        // (FREF, the frequency of REFCLK, which is equal to the PLL input
        // frequency divided by the pre-divider value) is less than 100 kHz or
        // greater than 20 MHz
        Fcco = Fin/N;
        if((Fcco < 100000) || (Fcco > 20000000))
        {
            continue;
        }

        for(M = 6; M < 512; M++)
        {
            // Fcco Range: 275 --> 550 MHz
            Fcco = Fin/N;
            Fcco = 2*Fcco*M;
            if( (Fcco < 275000000) || (Fcco > 550000000) )
            {
                continue;
            }

            // Calculate System clock
            // Note: default divider is 4
            Fcco /= 4;

            // Fcco and Fout Maximum different is 10K
            if( ((Fcco - Fout) <= 10000) || ((Fout - Fcco) <= 10000) )
            {
                // Find suitable Multiplier/Divider value.
                *pM = M;
                *pN = N;

                return (1);
            }
        }
    }

    // Can not find suitable Multiplier/Divider value.
    return (0);
}

