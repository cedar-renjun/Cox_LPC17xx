#include "xhw_types.h"
#include "xhw_ints.h"
#include "xcore.h"
#include "xhw_memmap.h"
#include "xhw_nvic.h"
#include "xhw_sysctl.h"
#include "xdebug.h"
#include "xsysctl.h"
#include "xhw_gpio.h"
#include "xgpio.h"
#include "xhw_i2c.h"
#include "xi2c.h"

void I2CCfg(unsigned long ulBase, unsigned long TargetClk)
{
	unsigned long clk = 0;

	// Get PCLK of I2C controller
	if (ulBase == I2C0_BASE)
	{
        clk = SysCtlPeripheralClockGet(PCLKSEL_I2C0)/TargetClk;
	}
	else if (ulBase == I2C1_BASE)
	{
        clk = SysCtlPeripheralClockGet(PCLKSEL_I2C1)/TargetClk;
	}
	else if (ulBase == I2C2_BASE)
	{
        clk = SysCtlPeripheralClockGet(PCLKSEL_I2C2)/TargetClk;
	}

    // Set the I2C clock value to register
    xHWREG(ulBase + I2C_SCLH) = (unsigned long)(clk / 2);
    xHWREG(ulBase + I2C_SCLL) = (unsigned long)(clk - xHWREG(ulBase + I2C_SCLH));

    // Set I2C operation to default 
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_AAC | CONCLR_SIC | CONCLR_STAC | CONCLR_I2CENC;
}

void I2CEnable(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT(
            (ulBase == I2C0_BASE) ||
            (ulBase == I2C1_BASE) ||
            (ulBase == I2C2_BASE)   
            );

    // Enable I2C Module.
    xHWREG(ulBase + I2C_CONSET) = CONSET_I2CEN;
}

void I2CDisable(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT(
            (ulBase == I2C0_BASE) ||
            (ulBase == I2C1_BASE) ||
            (ulBase == I2C2_BASE)   
            );

    // Disable I2C Module.
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_I2CENC;
}

void I2CStartSend(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT(
            (ulBase == I2C0_BASE) ||
            (ulBase == I2C1_BASE) ||
            (ulBase == I2C2_BASE)   
            );

	// Reset STA, STO, SI
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC | CONCLR_STAC;

	// Enter to Master Transmitter mode
    xHWREG(ulBase + I2C_CONSET) = CONSET_STA;

	// Wait for complete
	while (!(xHWREG(ulBase + I2C_CONSET) & CONSET_SI));
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_STAC;
}

void I2CStopSend(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT(
            (ulBase == I2C0_BASE) ||
            (ulBase == I2C1_BASE) ||
            (ulBase == I2C2_BASE)   
            );

    // Make sure start bit is not active
    if(xHWREG(ulBase + I2C_CONSET) & CONSET_STA)
    {
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_STAC;
    }

    xHWREG(ulBase + I2C_CONSET) = CONSET_AA | CONSET_STO;
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;
}

void I2CGeneralCallEnable(unsigned long ulBase, unsigned long ulID)
{
    switch(ulID)
    {
        case I2C_SLAVE_ADD0:
            {
                xHWREG(ulBase + I2C_ADR0) |= ADR_GC;
                break;
            }
        case I2C_SLAVE_ADD1:
            {
                xHWREG(ulBase + I2C_ADR1) |= ADR_GC;
                break;
            }
        case I2C_SLAVE_ADD2:
            {
                xHWREG(ulBase + I2C_ADR2) |= ADR_GC;
                break;
            }
        case I2C_SLAVE_ADD3:
            {
                xHWREG(ulBase + I2C_ADR3) |= ADR_GC;
                break;
            }
        default:
            {
                while(1);
            }
    }
}


void I2CGeneralCallDisable(unsigned long ulBase, unsigned long ulID)
{
    switch(ulID)
    {
        case I2C_SLAVE_ADD0:
            {
                xHWREG(ulBase + I2C_ADR0) &= ~ADR_GC;
                break;
            }
        case I2C_SLAVE_ADD1:
            {
                xHWREG(ulBase + I2C_ADR1) &= ~ADR_GC;
                break;
            }
        case I2C_SLAVE_ADD2:
            {
                xHWREG(ulBase + I2C_ADR2) &= ~ADR_GC;
                break;
            }
        case I2C_SLAVE_ADD3:
            {
                xHWREG(ulBase + I2C_ADR3) &= ~ADR_GC;
                break;
            }
        default:
            {
                while(1);
            }
    }
}

void I2CSlaveAddrSet(unsigned long ulBase, unsigned long ulID, unsigned long ulVal)
{
    unsigned long ulTmpReg = 0;

    ulVal &= (unsigned long)0xFE;

    switch(ulID)
    {
        case I2C_SLAVE_ADD0:
            {
                ulTmpReg = xHWREG(ulBase + I2C_ADR0);
                ulTmpReg &= ~ADR_ADDR_M;
                ulTmpReg |= ulVal;
                xHWREG(ulBase + I2C_ADR0)  = ulTmpReg;
                xHWREG(ulBase + I2C_MASK0) = (unsigned long)0xFE;

                break;
            }
        case I2C_SLAVE_ADD1:
            {
                ulTmpReg = xHWREG(ulBase + I2C_ADR1);
                ulTmpReg &= ~ADR_ADDR_M;
                ulTmpReg |= ulVal;
                xHWREG(ulBase + I2C_ADR1)  = ulTmpReg;
                xHWREG(ulBase + I2C_MASK1) = (unsigned long)0xFE;

                break;
            }
        case I2C_SLAVE_ADD2:
            {
                ulTmpReg = xHWREG(ulBase + I2C_ADR2);
                ulTmpReg &= ~ADR_ADDR_M;
                ulTmpReg |= ulVal;
                xHWREG(ulBase + I2C_ADR2)  = ulTmpReg;
                xHWREG(ulBase + I2C_MASK2) = (unsigned long)0xFE;
                break;
            }
        case I2C_SLAVE_ADD3:
            {
                ulTmpReg = xHWREG(ulBase + I2C_ADR3);
                ulTmpReg &= ~ADR_ADDR_M;
                ulTmpReg |= ulVal;
                xHWREG(ulBase + I2C_ADR3)  = ulTmpReg;
                xHWREG(ulBase + I2C_MASK3) = (unsigned long)0xFE;
                break;
            }
        default:
            {
                while(1);                  // Error
            }
    }
}

unsigned long I2CDataRead(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT(
            (ulBase == I2C0_BASE) ||
            (ulBase == I2C1_BASE) ||
            (ulBase == I2C2_BASE)   
            );

    // Read Data Register
    return xHWREG(ulBase + I2C_DAT);
}

void I2CDataWrite(unsigned long ulBase, unsigned long ulValue)
{
    // Check input parameters.
    xASSERT(
            (ulBase == I2C0_BASE) ||
            (ulBase == I2C1_BASE) ||
            (ulBase == I2C2_BASE)   
            );

    // Read Data Register
    xHWREG(ulBase + I2C_DAT) = ulValue;
}

unsigned long I2CStatusGet(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT(
            (ulBase == I2C0_BASE) ||
            (ulBase == I2C1_BASE) ||
            (ulBase == I2C2_BASE)   
            );

    // Get I2C status.
    return xHWREG(ulBase + I2C_STAT);
}





















