

//! Slave abort
#define SPI_ABRT            BIT_32_3

//! Mode Fault
#define SPI_MODF            BIT_32_4

//! Read overrun
#define SPI_ROVR            BIT_32_5

//! write collision
#define SPI_WCOL            BIT_32_6

//! SPI transfer finish
#define SPI_SPIF            BIT_32_7

xtBoolean SPIStatCheck(unsigned long ulBase, unsigned long ulFlags)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);
    xASSERT( (ulCfg & ~(
                           SPI_ABRT |
                           SPI_MODF |
                           SPI_ROVR |
                           SPI_WCOL |
                           SPI_SPIF 
                        )
              ) == 0);

    // Check Status Bit.
    if(xHWREG(ulBase + S0SPSR) & ulFlags)        // Set
    {
        return (xtrue);
    }
    else                                         // Unset
    {
        return (xfalse);
    }    
}


unsigned long SPIStatGet(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);

    return xHWREG(ulBase + S0SPSR);
}   

// \note ulFlags can be one of the following value:
// - \ref SPI_ABRT 
// - \ref SPI_MODF 
// - \ref SPI_ROVR  
void SPIStatFlagClear(unsigned long ulBase, unsigned long ulFlags)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);
    xASSERT( (ulCfg & ~(
                           SPI_ABRT |
                           SPI_MODF |
                           SPI_ROVR  
                        )
              ) == 0);

    // Check Status Bit.
    (void) xHWREG(ulBase + S0SPSR);

    // Clear WCOL Write collision by reading SPI status register(S0SPSR), then accessing the
    // SPI control register(S0SPCR).
    if(ulCfg & SPI_MODF)
    {
        unsigned long ulTmpReg = 0;

        ulTmpReg = xHWREG(ulBase + S0SPCR);
        xHWREG(ulBase + S0SPCR) = ulTmpReg;
    }
}  


unsigned long SPIDataReadWrite(unsigned long ulBase, unsigned long ulVal)
{
    xHWREG(ulBase + S0SPDR) = ulVal & S0SPDR_DATA_M;
    return xHWREG(ulBase + S0SPDR);
}

//! \note Those parameters can be used in SPICfg Function.

//! SPI Data Length 8-bit.
#define SPI_DATA_LEN_8             BIT_32_18

//! SPI Data Length 9-bit.
#define SPI_DATA_LEN_9             BIT_32_2 | BIT_32_11 | BIT_32_26 | BIT_32_25 | BIT_32_8

//! SPI Data Length 10-bit.
#define SPI_DATA_LEN_10            BIT_32_2 | BIT_32_11 | BIT_32_26 | BIT_32_9  | BIT_32_24

//! SPI Data Length 11-bit.
#define SPI_DATA_LEN_11            BIT_32_2 | BIT_32_11 | BIT_32_26 | BIT_32_9  | BIT_32_8

//! SPI Data Length 12-bit.
#define SPI_DATA_LEN_12            BIT_32_2 | BIT_32_11 | BIT_32_10 | BIT_32_25 | BIT_32_24

//! SPI Data Length 13-bit.
#define SPI_DATA_LEN_13            BIT_32_2 | BIT_32_11 | BIT_32_10 | BIT_32_25 | BIT_32_8

//! SPI Data Length 14-bit.
#define SPI_DATA_LEN_14            BIT_32_2 | BIT_32_11 | BIT_32_10 | BIT_32_9  | BIT_32_24

//! SPI Data Length 15-bit.
#define SPI_DATA_LEN_15            BIT_32_2 | BIT_32_11 | BIT_32_10 | BIT_32_9  | BIT_32_8 

//! SPI Data Length 16-bit.
#define SPI_DATA_LEN_16            BIT_32_2 | BIT_32_27 | BIT_32_26 | BIT_32_25 | BIT_32_24

//! SPI Master Mode.
#define SPI_MODE_MASTER            BIT_32_5

//! SPI Slave Mode.
#define SPI_MODE_SLAVE             BIT_32_21

//! Data is sampled on the first clock edge of SCK.A transfer starts
//! and ends with activation and deactivation of the SSEL signal.
#define SPI_CPHA_FIRST             BIT_32_19

//! Data is sampled on the second clock edge of the SCK.A transfer starts with the first
//! clock edge, and ends with the last sampling edge when the SSEL signal is active.
#define SPI_CPHA_SECOND            BIT_32_3

//! SCK is active high.
#define SPI_CPOL_HIGH              BIT_32_20

//! SCK is active low.
#define SPI_CPOL_LOW               BIT_32_4

//! SPI data is transferred LSB (bit 1) first.
#define SPI_LSB_FIRST              BIT_32_6

//! SPI data is transferred MSB (bit 7) first.
#define SPI_MSB_FIRST              BIT_32_22

void SPICfg(unsigned long ulBase, unsigned long ulClk, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;
    unsigned long ulActClk = 0;

    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);
    xASSERT( (ulCfg & ~(
                            SPI_DATA_LEN_8  |
                            SPI_DATA_LEN_9  |
                            SPI_DATA_LEN_10 |
                            SPI_DATA_LEN_11 |
                            SPI_DATA_LEN_12 |
                            SPI_DATA_LEN_13 |
                            SPI_DATA_LEN_14 |
                            SPI_DATA_LEN_15 |
                            SPI_DATA_LEN_16 |
                            SPI_MODE_MASTER |
                            SPI_MODE_SLAVE  |
                            SPI_CPHA_FIRST  |
                            SPI_CPHA_SECOND |
                            SPI_CPOL_HIGH   |
                            SPI_CPOL_LOW    |
                            SPI_LSB_FIRST   |
                            SPI_MSB_FIRST
                        )
              ) == 0);

    /********************* Configure SPI Clock frequency ***********************/
    ulActClk = SysCtlPeripheralClockGet(PCLKSEL_SPI0);
    ulActClk /= ulClk;

    // ulActClk/ulClk must be an even number greater than or equal to 8.
    // Violations of this can result in unpredictable behavior.
    if( ((ulActClk/ulClk) < 8) || ((ulActClk/ulClk)%2 != 0))
    {
        while(1); //Error
    }
    xHWREG(ulBase + S0SPCCR) = ulActClk/ulClk;    
    

    /********************* Configure SPI Mode, PHA, POL, DataLen ***************/
    ulTmpReg = xHWREG(ulBase + S0SPCR);
    ulTmpReg &= ((~ulCfg) >> 16);
    ulTmpReg |= (ulCfg & 0xFFFF);
    xHWREG(ulBase + S0SPCR) = ulTmpReg;    

}

void SPIIntEnable(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);

    // Avoid Compiler warning.
    (void) ulBase;

    xHWREG(ulBase + S0SPCR) |= S0SPCR_SPIE;    
}

void SPIIntDisable(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);

    // Avoid Compiler warning.
    (void) ulBase;

    xHWREG(ulBase + S0SPCR) &= ~S0SPCR_SPIE;    
}

//! SPI Interrupt flag.
#define SPI_INT_SPIF      S0SPINT_SPIF

unsigned long SPIIntFlagGet(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);

    // Avoid Compiler warning.
    (void) ulBase;

    return xHWREG(ulBase + S0SPINT);
}

xtBoolean SPIIntFlagCheck(unsigned long ulBase, unsigned long ulFlags)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);
    xASSERT(ulFlags == SPI_INT_SPIF);

    if(xHWREG(ulBase + S0SPINT) & ulFlags) // Interrupt has occures.
    {
        return (xtrue);
    }
    else                                   // Interrupt has not occures.
    {
        return (xfalse);
    }
}

void SPIIntFlagClear(unsigned long ulBase, unsigned long ulFlags)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);
    xASSERT(ulFlags == SPI_INT_SPIF);

    // Clear interrupt flag.
    xHWREG(ulBase + S0SPINT) |= S0SPINT_SPIF;

}

