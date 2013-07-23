



















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

SPI_DATA_LEN_8 
SPI_DATA_LEN_9 
SPI_DATA_LEN_10
SPI_DATA_LEN_11
SPI_DATA_LEN_12
SPI_DATA_LEN_13
SPI_DATA_LEN_14
SPI_DATA_LEN_15
SPI_DATA_LEN_16

SPI_MODE_MASTER
SPI_MODE_SLAVE

SPI_CPHA_FIRST
SPI_CPHA_SECOND

SPI_CPOL_HIGH
SPI_CPOL_LOW

SPI_LSB_FIRST
SPI_MSB_FIRST

void SPICfg(unsigned long ulBase, unsigned long ulCfg)
{

}

void SPIIntEnable();
void SPIIntDisable();
void SPIIntFlagGet();
void SPIIntFlagCheck();
void SPIIntFlagClear();
