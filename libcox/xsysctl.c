#include "xhw_types.h"
#include "xhw_ints.h"
#include "xhw_memmap.h"
#include "xhw_nvic.h"
#include "xdebug.h"
#include "xcore.h"
#include "xhw_sysctl.h"
#include "xsysctl.h"


//! \file xsysctl.c

static unsigned long PLLMNCal(unsigned long Fin,
        unsigned long Fout, unsigned long * pM, unsigned long *pN, unsigned long *pDiv);

//*****************************************************************************
//
//! \brief Provides a small delay.
//!
//! \param ulCount is the number of delay loop iterations to perform.
//!
//! This function provides a means of generating a constant length delay.  It
//! is written in assembly to keep the delay consistent across tool chains,
//! avoiding the need to tune the delay based on the tool chain in use.
//!
//! The loop takes 3 cycles/loop.
//!
//! \return None.
//
//*****************************************************************************
#if defined(gcc) || defined(__GNUC__)
void __attribute__((naked))
SysCtlDelay(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne     SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(ewarm) || defined(__ICCARM__)
void
SysCtlDelay(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(rvmdk) || defined(__CC_ARM)
__asm void
SysCtlDelay(unsigned long ulCount)
{
    subs    r0, #1;
    bne     SysCtlDelay;
    bx      lr;
}
#endif

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
    unsigned long ulDiv    = 0;      // System clock divider
    unsigned long ulFin    = 0;      // Input Clock frequency

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

        // Get Input Frequency (unit: Hz)
        ulTmpReg = (ulConfig & SYSCTL_XTAL_nMHZ_MASK) * 1000000;

        // Configure Main Osc Frequency Range
        if(ulTmpReg > 20000000)
        {
            ulTmpReg = xHWREG(SCS);
            if((ulTmpReg & SCS_OSCRS) == 0)    // Wrong Range: 1 --> 20 MHz
            {
                if(ulTmpReg & SCS_OSCEN)       // Main Osc Have Been enabled
                {
                    //Disable Main Osc then Configure Osc Range
                    ulTmpReg &= ~SCS_OSCEN;
                    xHWREG(SCS) = ulTmpReg;

                    // Waiting for Main Osc Disabled
                    do
                    {
                        ulTmpReg = xHWREG(SCS);
                        ulTmpReg &= SCS_OSCEN;
                    }while(ulTmpReg != 0);

                    // Configure Osc Range
                    ulTmpReg = xHWREG(SCS);
                    ulTmpReg |= SCS_OSCRS;
                    xHWREG(SCS) = ulTmpReg;
                }
            }
        }

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
        
        //Select Main Osc as PLL Input
        ulTmpReg = xHWREG(CLKSRCSEL);
        ulTmpReg &= ~CLKSRCSEL_CLKSRC_M;
        ulTmpReg |= CLKSRCSEL_CLKSRC_OSC;
        xHWREG(CLKSRCSEL) = ulTmpReg;

        // Get Input Frequency (unit: Hz)
        ulFin = (ulConfig & SYSCTL_XTAL_nMHZ_MASK) * 1000000;
    }

    /************** Configure Internal oscillator ********************/
    // For LPC 17xx serial MCU, Internal RC is always enable.
    // so, you need not deal with those parameters
    // 1) SYSCTL_OSC_INT
    // 2) SYSCTL_INT_OSC_DIS
    //
    if(ulConfig & SYSCTL_OSC_INT)             // Enable Internal Osc
    {
        //Need not Enable Internal RC

        //Select Internal RC as PLL Input
        ulTmpReg = xHWREG(CLKSRCSEL);
        ulTmpReg &= ~CLKSRCSEL_CLKSRC_M;
        ulTmpReg |= CLKSRCSEL_CLKSRC_IRC;
        xHWREG(CLKSRCSEL) = ulTmpReg;

        // Get Input Frequency (unit: Hz)
        // For IRC, there is 4MHz
        ulFin = 4 * 1000000;

    }

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

    ulRes = PLLMNCal(ulFin, ulSysClk, &ulM, &ulN, &ulDiv);
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
        ulTmpReg &= ~PLL0CON_PLLC;
        xHWREG(PLL0CON) = ulTmpReg;

        // Write key to PLL Feed register
        xHWREG(PLL0FEED) = (unsigned long)0xAA;
        xHWREG(PLL0FEED) = (unsigned long)0x55;

        // waiting for disconnect
        do
        {
            ulTmpReg = xHWREG(PLL0STAT);
            ulTmpReg &= PLL0STAT_PLLC_STAT;
        }while(ulTmpReg);

    }

    // PLL Enable ?
    ulTmpReg = xHWREG(PLL0STAT);
    if(ulTmpReg & PLL0STAT_PLLE_STAT) // PLL Have Been Enabled, we need to Disable it.
    {
        // Disable PLL
        ulTmpReg =  xHWREG(PLL0CON);
        ulTmpReg &= ~PLL0CON_PLLE;
        xHWREG(PLL0CON) = ulTmpReg;

        // Write key to PLL Feed register
        xHWREG(PLL0FEED) = (unsigned long)0xAA;
        xHWREG(PLL0FEED) = (unsigned long)0x55;

        // Waitting for Disable
        do
        {
            ulTmpReg = xHWREG(PLL0STAT);
            ulTmpReg &= PLL0STAT_PLLE_STAT;
        }while(ulTmpReg);
    }

#if defined(LPC_175x) | defined(LPC_176x)
    xHWREG(CCLKCFG) = (ulDiv - 1);
#elif defined(LPC_177x) | defined(LPC_178x)
    xHWREG(CCLKSEL) = (ulDiv | CCLKSEL_CCLKSEL);
#endif

    // Configure PLL Multiplier/Divider
    ulM -= 1;
    ulN -= 1;
    ulTmpReg = (ulN << PLL0CFG_PSEL_S) | ulM;
    xHWREG(PLL0CFG) = ulTmpReg;

    // Write key to PLL Feed register
    xHWREG(PLL0FEED) = (unsigned long)0xAA;
    xHWREG(PLL0FEED) = (unsigned long)0x55;

    //ReEnable PLL and Wait Locked
    ulTmpReg =  xHWREG(PLL0CON);
    ulTmpReg |= PLL0CON_PLLE;
    xHWREG(PLL0CON) = ulTmpReg;

    // Write key to PLL Feed register
    xHWREG(PLL0FEED) = (unsigned long)0xAA;
    xHWREG(PLL0FEED) = (unsigned long)0x55;

    // Waitting for Enable
    do
    {
        ulTmpReg = xHWREG(PLL0STAT);
        ulTmpReg &= PLL0STAT_PLLE_STAT;
    }while(ulTmpReg == 0);

    // Waitting for Locked
    do
    {
        ulTmpReg = xHWREG(PLL0STAT);
        ulTmpReg &= PLL0STAT_PLOCK;
    }while(ulTmpReg == 0);

    // Connect It
    ulTmpReg =  xHWREG(PLL0CON);
    ulTmpReg |= PLL0CON_PLLC;
    xHWREG(PLL0CON) = ulTmpReg;

    // Write key to PLL Feed register
    xHWREG(PLL0FEED) = (unsigned long)0xAA;
    xHWREG(PLL0FEED) = (unsigned long)0x55;

    // Waitting for Enable
    do
    {
        ulTmpReg = xHWREG(PLL0STAT);
        ulTmpReg &= PLL0STAT_PLLC_STAT;
    }while(ulTmpReg == 0);
}


//*****************************************************************************
//
//! \brief       Calculate the PLL Multiplier/Diviver
//! \param [in]  Fin is the input clock frequency (1Mhz --> 25Mhz).
//! \param [in]  Fout is the target clock frequency, Maximum value is 120Mhz.
//! \param [out] pM is the Calculate result of multipler.
//! \param [out] pN is the Calculate result of divider.
//! \param [out] pDiv is the system divider.
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
        unsigned long Fout, unsigned long * pM, unsigned long *pN, unsigned long *pDiv)
{
    unsigned long M    = 0;   // PLL Multiplier Value
    unsigned long N    = 0;   // PLL Divider Value
    unsigned long Fcco = 0;   // PLL Fcco Output frequency
    unsigned long Fclk = 0;   // System Clock frequency
    unsigned long i    = 0;   // Counter

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

            for(i = 1; i < 256; i++)
            {
                Fclk = Fcco/i;
#if defined(LPC_175x) | defined(LPC_176x)
                if(Fclk > 120000000)
#elif defined(LPC_177x) | defined(LPC_178x)
                if(Fclk > 100000000)
#endif
                {
                    continue;
                }

                // Fclk and Fout Maximum different is 10K
                if( ((Fclk - Fout) <= 10000) || ((Fout - Fclk) <= 10000) )
                {
                    // Find suitable Multiplier/Divider value.
                    *pM   = M;
                    *pN   = N;
                    *pDiv = i;
                    return (1);
                }
            }

        }
    }

    // Can not find suitable Multiplier/Divider value.
    return (0);
}


#define EXT_INT_0
#define EXT_INT_1
#define EXT_INT_2
#define EXT_INT_3

#define EXT_INT_MASK                   BIT_MASK(32, 3, 0)

#define EXT_INT_LV_H                   BIT_32_0
#define EXT_INT_LV_L                   BIT_32_1
#define EXT_INT_EG_R                   BIT_32_2
#define EXT_INT_EG_F                   BIT_32_3


void SysCtlExtIntCfg(unsigned long ulPin, unsigned long ulCfg)
{
    unsigned long i         = 0;
    unsigned long ulTmpReg1 = 0;
    unsigned long ulTmpReg2 = 0;

    // Check input parameters
    xASSERT( (ulPin & EXT_INT_MASK) == 0 );
    xASSERT( (ulCfg & EXT_INT_MASK) == 0 );
    
    ulTmpReg1 = xHWREG(EXTMODE);              // External Interrupt Mode register
    ulTmpReg2 = xHWREG(EXTPOLAR);             // External Interrupt Polar register

    for(i = 0; i < 4; i++)
    {
        if(ulPin & (0x01 << i))
        {
            switch(ulCfg)
            {
                case EXT_INT_LV_H:            // High Level Triggle
                    {
                        ulTmpReg1 &= ~(0x01 << i);
                        ulTmpReg2 |=  (0x01 << i);
                        break;
                    }
                case EXT_INT_LV_L:           // Low Level Triggle
                    {
                        ulTmpReg1 &= ~(0x01 << i);
                        ulTmpReg2 &= ~(0x01 << i);
                        break;
                    }
                case EXT_INT_EG_R:           // Rising Triggle
                    {
                        ulTmpReg1 |=  (0x01 << i);
                        ulTmpReg2 |=  (0x01 << i);
                        break;
                    }
                case EXT_INT_EG_F:           // Falling Triggle
                    {
                        ulTmpReg1 |=  (0x01 << i);
                        ulTmpReg2 &= ~(0x01 << i);
                        break;
                    }
            }
        }
    }

    // Write back to mode/polar register
    xHWREG(EXTMODE) = ulTmpReg1;             // Mode register
    xHWREG(EXTMODE) = ulTmpReg2;             // Polar register

}

#define EXT_INT_0
#define EXT_INT_1
#define EXT_INT_2
#define EXT_INT_3
unsigned long SysCtlExtIntFlagGet(void)
{
    return xHWREG(EXTINT);
}


xtBoolean SysCtlExtIntFlagCheck(unsigned long ulFlag)
{
    unsigned long ulTmpReg = 0;

    ulTmpReg = xHWREG(EXTINT);

    if(ulTmpReg & ulFlag)
    {
        return (xtrue);
    }
    else
    {
        return (xfalse);
    }
}


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

//! return one of the following value
unsigned long SysCtlResetFlagGet(void)
{
   return xHWREG(RSID); 
}



//! \todo Add PLL0 function

#define PCLKSEL_WDT           PCLKSEL0_WDT_S    
                                                
#define PCLKSEL_TIMER0        PCLKSEL0_TIMER0_S 
                                                
#define PCLKSEL_TIMER1        PCLKSEL0_TIMER1_S 
                                                
#define PCLKSEL_UART0         PCLKSEL0_UART0_S  
                                                
#define PCLKSEL_UART1         PCLKSEL0_UART1_S  
                                                
#define PCLKSEL_PWM1          PCLKSEL0_PWM1_S   
                                                
#define PCLKSEL_I2C0          PCLKSEL0_I2C0_S   
                                                
#define PCLKSEL_SPI           PCLKSEL0_SPI_S    
                                                
#define PCLKSEL_SSP1          PCLKSEL0_SSP1_S   
                                                
#define PCLKSEL_DAC           PCLKSEL0_DAC_S    
                                                
#define PCLKSEL_ADC           PCLKSEL0_ADC_S    
                                                
#define PCLKSEL_CAN1          PCLKSEL0_CAN1_S   
                                                
#define PCLKSEL_CAN2          PCLKSEL0_CAN2_S   
                                                
#define PCLKSEL_ACF           PCLKSEL0_ACF_S    
                                                 
#define PCLKSEL_QEI           (PCLKSEL1_QEI_S     + 32)                                
                                                       
#define PCLKSEL_GPIOINT       (PCLKSEL1_GPIOINT_S + 32)                                
                                                       
#define PCLKSEL_PCB           (PCLKSEL1_PCB_S     + 32)                                
                                                       
#define PCLKSEL_I2C1          (PCLKSEL1_I2C1_S    + 32)                                

#define PCLKSEL_SSP0          (PCLKSEL1_SSP0_S    + 32)                                

#define PCLKSEL_TIMER2        (PCLKSEL1_TIMER2_S  + 32)                                

#define PCLKSEL_TIMER3        (PCLKSEL1_TIMER3_S  + 32)                                

#define PCLKSEL_UART2         (PCLKSEL1_UART2_S   + 32)                                

#define PCLKSEL_UART3         (PCLKSEL1_UART3_S   + 32)                                

#define PCLKSEL_I2C2          (PCLKSEL1_I2C2_S    + 32)                                

#define PCLKSEL_I2S           (PCLKSEL1_I2S_S     + 32)                                

#define PCLKSEL_RIT           (PCLKSEL1_RIT_S     + 32)                                

#define PCLKSEL_SYSCON        (PCLKSEL1_SYSCON_S  + 32)                                

#define PCLKSEL_MC            (PCLKSEL1_MC_S      + 32)                                
                                                                           

#define PCLK_CCLK_DIV_1       BIT_32_0
#define PCLK_CCLK_DIV_2       BIT_32_1
#define PCLK_CCLK_DIV_4       BIT_32_ALL_0
#define PCLK_CCLK_DIV_6       (BIT_32_1 | BIT_32_0)
#define PCLK_CCLK_DIV_8       (BIT_32_1 | BIT_32_0)


void SysCtlPeripheralClockCfg(unsigned long ulPeri, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    if(ulCfg < 32)
    {
        ulTmpReg = xHWREG(PCLKSEL0);
        ulTmpReg &= ~(PCLKSEL_PPP_M << ulPeri);
        ulTmpReg |= ulCfg << ulPeri;
        xHWREG(PCLKSEL0) = ulTmpReg;
    }
    else
    {
        ulPeri -= 32;
        ulTmpReg = xHWREG(PCLKSEL1);
        ulTmpReg &= ~(PCLKSEL_PPP_M << ulPeri);
        ulTmpReg |= ulCfg << ulPeri;
        xHWREG(PCLKSEL1) = ulTmpReg;
    }


}

#if defined(LPC_177x) | defined(LPC_178x)
//! LCD Controller power/clock control bit.
#define SYSCTL_PERIPH_ LCD             PCONP_PCLCD
#endif

//! Timer/Counter 0 power/clock control bit.
#define SYSCTL_PERIPH_TIM0             PCONP_PCTIM0            BIT_32_1

//! Timer/Counter 1 power/clock control bit.
#define SYSCTL_PERIPH_TIM1             PCONP_PCTIM1            BIT_32_2

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


void SysCtlPeripheralReset(unsigned long ulPeripheral);
void SysCtlPeripheralEnable(unsigned long ulPeripheral)
{
    xHWREG(PCONP) |= ulPeripheral;
}
void SysCtlPeripheralDisable(unsigned long ulPeripheral);
{
    xHWREG(PCONP) &= ~ulPeripheral;
}

