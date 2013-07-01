

static const 

//SYSCTL_XTAL_nMHZ (n = 4/5/../25)

#define SYSCTL_XTAL_nMHZ_MASK   BIT_MASK(32, 4, 0)

#define SYSCTL_XTAL_4MHZ        0 
#define SYSCTL_XTAL_5MHZ        1 
#define SYSCTL_XTAL_6MHZ        2 
#define SYSCTL_XTAL_7MHZ        3 
#define SYSCTL_XTAL_8MHZ        4 
#define SYSCTL_XTAL_9MHZ        5 
#define SYSCTL_XTAL_10MHZ       6 
#define SYSCTL_XTAL_11MHZ       7 
#define SYSCTL_XTAL_12MHZ       8 
#define SYSCTL_XTAL_13MHZ       9 
#define SYSCTL_XTAL_14MHZ       10
#define SYSCTL_XTAL_15MHZ       11
#define SYSCTL_XTAL_16MHZ       12
#define SYSCTL_XTAL_17MHZ       13
#define SYSCTL_XTAL_18MHZ       14
#define SYSCTL_XTAL_19MHZ       15
#define SYSCTL_XTAL_20MHZ       16
#define SYSCTL_XTAL_21MHZ       17
#define SYSCTL_XTAL_22MHZ       18
#define SYSCTL_XTAL_23MHZ       19
#define SYSCTL_XTAL_24MHZ       20
#define SYSCTL_XTAL_25MHZ       21

void SysCtlClockSet(unsigned long ulSysClk, unsigned long ulConfig)
{

}

unsigned long __N = 0;
unsigned long __N = 0;

// Check input clock range.
if((Fin > 50000000) || (Fin > 50000000))
{
    return (-1);
}

for(__N = 0; __N < 32; __N++)
{
    unsigned long Tmp = 0;

    // The value of PLOCK0 may not be stable when the PLL reference frequency
    // (FREF, the frequency of REFCLK, which is equal to the PLL input
    // frequency divided by the pre-divider value) is less than 100 kHz or
    // greater than 20 MHz

    Tmp = Fin/__N;
    if((Tmp < 100000) || (Tmp > 20000000))
    {
        continue;
    }

    for(__M = 6; __M < 512; __M++)
    {

    }
}


