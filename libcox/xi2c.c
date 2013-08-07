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



























