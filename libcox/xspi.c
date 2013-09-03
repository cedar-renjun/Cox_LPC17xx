
//*****************************************************************************
//
//! \brief  Check SPI status.
//!         This function check whether the spi status flag has been set.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \param  [in] ulFlags is the SPI status flag, this value can be OR of the
//!              following value:
//!              \ref SPI_ABRT Slave abort
//!              \ref SPI_MODF Mode Fault
//!              \ref SPI_ROVR Read overrun
//!              \ref SPI_WCOL write collision
//!              \ref SPI_SPIF SPI transfer finish
//!
//! \return The status of checked flag.
//!         \ref xtrue status flag has been set.
//!         \ref xflase status flag has not been set.
//!
//
//*****************************************************************************
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

//*****************************************************************************
//
//! \brief  Get SPI bus status.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return The status of SPI, which is the OR of the following value:
//!              \ref SPI_ABRT Slave abort
//!              \ref SPI_MODF Mode Fault
//!              \ref SPI_ROVR Read overrun
//!              \ref SPI_WCOL write collision
//!              \ref SPI_SPIF SPI transfer finish
//!
//
//*****************************************************************************
unsigned long SPIStatGet(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);

    return xHWREG(ulBase + S0SPSR);
}

//*****************************************************************************
//
//! \brief  Clear SPI status flag.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \param  [in] ulFlags is the flag ready to be clear.
//!              can be one of the following value:
//!              \ref SPI_ABRT    Slave abort
//!              \ref SPI_MODF    Mode fault
//!              \ref SPI_ROVR    Read overrun
//
//*****************************************************************************
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

    // Clear WCOL Write collision by reading SPI status register(S0SPSR),
    // then accessing the SPI control register(S0SPCR).
    if(ulCfg & SPI_MODF)
    {
        unsigned long ulTmpReg = 0;

        ulTmpReg = xHWREG(ulBase + S0SPCR);
        xHWREG(ulBase + S0SPCR) = ulTmpReg;
    }
}


//*****************************************************************************
//
//! \brief  SPI single write read data.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \param  [in] ulVal is the data ready to send via spi bus.
//!
//! \return The receive data from spi slave.
//
//*****************************************************************************
unsigned long SPIDataReadWrite(unsigned long ulBase, unsigned long ulVal)
{
    xHWREG(ulBase + S0SPDR) = ulVal & S0SPDR_DATA_M;
    return xHWREG(ulBase + S0SPDR);
}

//*****************************************************************************
//
//! \brief  Enable SPI interrupt.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \param  [in] ulClk is SPI bus clock frequency.
//!
//! \param  [in] ulCfgs is SPI configure parameters.
//!              which can be OR of the following value:
//!              \ref SPI_DATA_LEN_8   
//!              \ref SPI_DATA_LEN_9   
//!              \ref SPI_DATA_LEN_10  
//!              \ref SPI_DATA_LEN_11  
//!              \ref SPI_DATA_LEN_12  
//!              \ref SPI_DATA_LEN_13  
//!              \ref SPI_DATA_LEN_14  
//!              \ref SPI_DATA_LEN_15  
//!              \ref SPI_DATA_LEN_16  
//!              \ref SPI_MODE_MASTER  
//!              \ref SPI_MODE_SLAVE   
//!              \ref SPI_CPHA_FIRST   
//!              \ref SPI_CPHA_SECOND  
//!              \ref SPI_CPOL_HIGH    
//!              \ref SPI_CPOL_LOW     
//!              \ref SPI_LSB_FIRST    
//!              \ref SPI_MSB_FIRST
//!
//! \return None.
//
//*****************************************************************************
void SPICfg(unsigned long ulBase, unsigned long ulClk, unsigned long ulCfgs)
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

//*****************************************************************************
//
//! \brief  Enable SPI interrupt.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return None.
//
//*****************************************************************************
void SPIIntEnable(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);

    // Avoid Compiler warning.
    (void) ulBase;

    xHWREG(ulBase + S0SPCR) |= S0SPCR_SPIE;
}

//*****************************************************************************
//
//! \brief  Disable SPI interrupt.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return None.
//
//*****************************************************************************
void SPIIntDisable(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);

    // Avoid Compiler warning.
    (void) ulBase;

    xHWREG(ulBase + S0SPCR) &= ~S0SPCR_SPIE;
}

//*****************************************************************************
//
//! \brief  Get SPI Interrupt flag.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return The interrupt status of SPI, this value can be
//!         \ref SPI_INT_SPIF SPI event has been occurs.
//!         0.
//
//*****************************************************************************
unsigned long SPIIntFlagGet(unsigned long ulBase)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);

    // Avoid Compiler warning.
    (void) ulBase;

    return xHWREG(ulBase + S0SPINT);
}

//*****************************************************************************
//
//! \brief  Check ADC status flag.
//!         This function is used to check whether special flag is set or not.
//!
//! \param  [in] ulFlags is the flag you want to check
//!         This value is the one of the following value:
//!              \ref SPI_INT_SPIF
//!
//! \return The status of special flag.
//!         - xtrue The check flag has been set. 
//!         - xflase The check flag has not been set. 
//
//*****************************************************************************
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

//*****************************************************************************
//
//! \brief  Clear SPI interrupt status flag.
//!         This function can be used to clear special SPI interrupt status flag.
//!
//! \param  [in] ulFlags is SPI interrupt status flag.
//!              This parameter can be OR of the following value:
//!              \ref SPI_INT_SPIF
//!
//! \return None.
//
//*****************************************************************************       
void SPIIntFlagClear(unsigned long ulBase, unsigned long ulFlags)
{
    // Check the parameters.
    xASSERT(ulBase == SPI0_BASE);
    xASSERT(ulFlags == SPI_INT_SPIF);

    // Clear interrupt flag.
    xHWREG(ulBase + S0SPINT) |= S0SPINT_SPIF;

}

