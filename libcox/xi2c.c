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

//*****************************************************************************
//
// An array is I2C callback function point
//
//*****************************************************************************
static xtEventCallback g_pfnI2CHandlerCallbacks[3] = {0};

void I2C0IntHandler(void)
{
    if(g_pfnI2CHandlerCallbacks[0] != 0)
    {
        g_pfnI2CHandlerCallbacks[0](0, 0, 0, 0);
    }
    else
    {
        while(1);
    }
}

void I2C1IntHandler(void)
{
    if(g_pfnI2CHandlerCallbacks[1] != 0)
    {
        g_pfnI2CHandlerCallbacks[1](0, 0, 0, 0);
    }
    else
    {
        while(1);
    }
}

void I2C2IntHandler(void)
{
    if(g_pfnI2CHandlerCallbacks[2] != 0)
    {
        g_pfnI2CHandlerCallbacks[2](0, 0, 0, 0);
    }
    else                               // Infinite loop
    {
        while(1);
    }
}

unsigned long I2CIntCallbackInit(unsigned long ulBase, xtEventCallback pfnCallback)
{

    // Check input parameters.
    xASSERT(
            (ulBase == I2C0_BASE) ||
            (ulBase == I2C1_BASE) ||
            (ulBase == I2C2_BASE)
            );
    // Function pointer must be a valid pointer!
    xASSERT(pfnCallback != 0);

    if(pfnCallback == 0)
    {
        while(1);
    }

    switch(ulBase)
    {
        case I2C0_BASE:
            {
                g_pfnI2CHandlerCallbacks[0] = pfnCallback;
                break;
            }
        case I2C1_BASE:
            {
                g_pfnI2CHandlerCallbacks[1] = pfnCallback;
                break;
            }
        case I2C2_BASE:
            {
                g_pfnI2CHandlerCallbacks[2] = pfnCallback;
                break;
            }
        default:                        // Error
            {
                while(1);
            }
    }

    return (0);

}




void I2CMasterInit(unsigned long ulBase, unsigned long TargetClk)
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

void I2CSlaveInit(unsigned long ulBase, unsigned long ulSlaveAddr,
                         unsigned long ulGeneralCall)
{
    // Configure I2C Address
    if(ulGeneralCall == I2C_GENERAL_CALL_EN)
    {
        ulSlaveAddr |= ADR_GC;
        xHWREG(ulBase + I2C_ADR0) = ulSlaveAddr;
    }
    else if(ulGeneralCall == I2C_GENERAL_CALL_DIS)
    {
        ulSlaveAddr &= ~ADR_GC;
        xHWREG(ulBase + I2C_ADR0) = ulSlaveAddr;
    }
    else
    {
        while(1); // Error
    }

    // Enable I2C Auto AckKnowledge
    xHWREG(ulBase + I2C_CONSET) = CONSET_AA;
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC | CONCLR_STAC | CONCLR_I2CENC;
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

void I2CIntEnable(unsigned long ulBase)
{
    switch(ulBase)
    {
        case I2C0_BASE:
            {
                xIntEnable(xINT_I2C0);
                break;
            }
        case I2C1_BASE:
            {
                xIntEnable(xINT_I2C1);
                break;
            }
        case I2C2_BASE:
            {
                xIntEnable(xINT_I2C2);
                break;
            }
        default:                        // Error
            {
                while(1);
            }
    }

}
void I2CIntDisable(unsigned long ulBase)
{
    switch(ulBase)
    {
        case I2C0_BASE:
            {
                xIntDisable(xINT_I2C0);
                break;
            }
        case I2C1_BASE:
            {
                xIntDisable(xINT_I2C1);
                break;
            }
        case I2C2_BASE:
            {
                xIntDisable(xINT_I2C2);
                break;
            }
        default:                        // Error
            {
                while(1);
            }
    }
}

void I2CMasterWriteRequestS1(unsigned long ulBase, unsigned long ucSlaveAddr,
                              unsigned long ucData, xtBoolean bEndTransmition)
{
    unsigned long ulTmpReg = 0;

    // Send start signal
    xHWREG(ulBase + I2C_CONSET) = CONSET_STA;
    while(1)
    {
        ulTmpReg = xHWREG(ulBase + I2C_STAT);
        // A Start/Restart signal has been transmitted.
        if( (ulTmpReg == I2C_STAT_M_TX_START) || (ulTmpReg == I2C_STAT_M_TX_RESTART) )
        {
            break;
        }
    }

    // Send slave address + W
    ucSlaveAddr &= ~BIT_32_0;
    xHWREG(ulBase + I2C_DAT) = ucSlaveAddr;

    // Enable Autuo Ackknowledge 
    xHWREG(ulBase + I2C_CONSET) = CONSET_AA;
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC | CONCLR_STAC;

    // Address+W Transfer OK ?
    while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_TX_SLAW_ACK);
    xHWREG(ulBase + I2C_DAT) = ucData;
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

    // First Data Transfer OK ?
    while(1)
    {
        ulTmpReg = xHWREG(ulBase + I2C_STAT);
        // A Start/Restart signal has been transmitted.
        if( (ulTmpReg == I2C_STAT_M_TX_DAT_ACK) || (ulTmpReg == I2C_STAT_M_TX_DAT_NACK) )
        {
            break;
        }
    }

    // Need to Send Stop signal ?
    if(bEndTransmition == xtrue)
    {
        xHWREG(ulBase + I2C_CONSET) = CONSET_STO | CONSET_AA;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;
    }
}

void I2CMasterWriteRequestS2(unsigned long ulBase, unsigned long ucData,
        xtBoolean bEndTransmition)
{
    unsigned long ulTmpReg = 0;
    
    // Previous Data Transfer OK ?
    while(1)
    {
        ulTmpReg = xHWREG(ulBase + I2C_STAT);
        // A Start/Restart signal has been transmitted.
        if( (ulTmpReg == I2C_STAT_M_TX_DAT_ACK) || (ulTmpReg == I2C_STAT_M_TX_DAT_NACK) )
        {
            break;
        }
    }

    // Feed New Data
    xHWREG(ulBase + I2C_DAT) = ucData;
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

    // Previous Data Transfer OK ?
    while(1)
    {
        ulTmpReg = xHWREG(ulBase + I2C_STAT);
        // A Start/Restart signal has been transmitted.
        if( (ulTmpReg == I2C_STAT_M_TX_DAT_ACK) || (ulTmpReg == I2C_STAT_M_TX_DAT_NACK) )
        {
            break;
        }
    }

    // Need to Send Stop signal ?
    if(bEndTransmition == xtrue)
    {
        xHWREG(ulBase + I2C_CONSET) = CONSET_STO | CONSET_AA;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;
    }
}

void I2CMasterWriteS1(unsigned long ulBase, unsigned long ulSlaveAddr,
                       unsigned long ulData, xtBoolean bEndTransmition)
{
    I2CMasterWriteRequestS1(ulBase, ulSlaveAddr, ulData, bEndTransmition);
}

void I2CMasterWriteS2(unsigned long ulBase, unsigned long ulData, xtBoolean bEndTransmition)
{
    I2CMasterWriteRequestS2(ulBase, ulData, bEndTransmition);
}

void I2CMasterWriteBufS1(unsigned long ulBase, unsigned long ucSlaveAddr,
        unsigned char *pucDataBuf, unsigned long ulLen, xtBoolean bEndTransmition)
{
    unsigned long i        = 0;
    unsigned long ulTmpReg = 0;

    // Send start signal
    xHWREG(ulBase + I2C_CONSET) = CONSET_STA;
    while(1)
    {
        ulTmpReg = xHWREG(ulBase + I2C_STAT);
        // A Start/Restart signal has been transmitted.
        if( (ulTmpReg == I2C_STAT_M_TX_START) || (ulTmpReg == I2C_STAT_M_TX_RESTART) )
        {
            break;
        }
    }

    // Send slave address + W
    ucSlaveAddr &= ~BIT_32_0;
    xHWREG(ulBase + I2C_DAT) = ucSlaveAddr;

    // Enable Autuo Ackknowledge 
    xHWREG(ulBase + I2C_CONSET) = CONSET_AA;
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_STAC | CONCLR_SIC;

    // Address+W Transfer OK ?
    while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_TX_SLAW_ACK);

    // Transfer Data to I2C Bus
    for(i = 0; i < ulLen; i++)
    {
        xHWREG(ulBase + I2C_DAT) = pucDataBuf[i];
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // First Data Transfer OK ?
        while(1)
        {
            ulTmpReg = xHWREG(ulBase + I2C_STAT);
            // A Start/Restart signal has been transmitted.
            if( (ulTmpReg == I2C_STAT_M_TX_DAT_ACK) || (ulTmpReg == I2C_STAT_M_TX_DAT_NACK) )
            {
                break;
            }
        }
    }

    // Need to Send Stop signal ?
    if(bEndTransmition == xtrue)
    {
        xHWREG(ulBase + I2C_CONSET) = CONSET_STO | CONSET_AA;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;
    }
}

void I2CMasterWriteBufS2(unsigned long ulBase, unsigned char *pucDataBuf,
                          unsigned long ulLen, xtBoolean bEndTransmition)
{
    unsigned long ulTmpReg = 0;
    unsigned long i = 0;

    // Previous Data Transfer OK ?
    while(1)
    {
        ulTmpReg = xHWREG(ulBase + I2C_STAT);
        // A Start/Restart signal has been transmitted.
        if( (ulTmpReg == I2C_STAT_M_TX_DAT_ACK) || (ulTmpReg == I2C_STAT_M_TX_DAT_NACK) )
        {
            break;
        }
    }

    // Transfer Data to I2C Bus
    for(i = 0; i < ulLen; i++)
    {
        // Feed New Data
        xHWREG(ulBase + I2C_DAT) = pucDataBuf[i];
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // Previous Data Transfer OK ?
        while(1)
        {
            ulTmpReg = xHWREG(ulBase + I2C_STAT);
            // A Start/Restart signal has been transmitted.
            if( (ulTmpReg == I2C_STAT_M_TX_DAT_ACK) || (ulTmpReg == I2C_STAT_M_TX_DAT_NACK) )
            {
                break;
            }
        }
    }

    // Need to Send Stop signal ?
    if(bEndTransmition == xtrue)
    {
        xHWREG(ulBase + I2C_CONSET) = CONSET_STO | CONSET_AA;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;
    }
}

unsigned long I2CMasterReadRequestS1(unsigned long ulBase,
        unsigned long ucSlaveAddr, xtBoolean bEndTransmition)
{
    unsigned long ulTmpReg = 0;
    unsigned long ulTmp    = 0;

    // Send start signal
    xHWREG(ulBase + I2C_CONSET) = CONSET_STA;
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;
    
    while(1)
    {
        ulTmpReg = xHWREG(ulBase + I2C_STAT);
        // A Start/Restart signal has been transmitted.
        if( (ulTmpReg == I2C_STAT_M_RX_START) || (ulTmpReg == I2C_STAT_M_RX_RESTART) )
        {
            break;
        }
    }

    // Send slave address + W
    ucSlaveAddr |= BIT_32_0;
    xHWREG(ulBase + I2C_DAT) = ucSlaveAddr;
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC | CONCLR_STAC;
    while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_SLAR_ACK);

    // Need to Send Stop signal ?
    if(bEndTransmition == xtrue)                                     // Yes
    {
        // Disable Auto Ackknowledge
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_AAC;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // Wait Receive Data
        while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_DAT_ACK);
        ulTmp = xHWREG(ulBase + I2C_DAT);
        
        // Release Stop Signal
        xHWREG(ulBase + I2C_CONSET) = CONSET_STO;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;
    }
    else                                                             // No
    {
        // Enable Auto Ackknowledge
        xHWREG(ulBase + I2C_CONSET) = CONSET_AA;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // Wait Receive Data
        while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_DAT_ACK);
        ulTmp = xHWREG(ulBase + I2C_DAT);
    }

    return (ulTmp);
}

unsigned long I2CMasterReadRequestS2(unsigned long ulBase, xtBoolean bEndTransmition)
{
    unsigned long ulTmp    = 0;

    // Need to Send Stop signal ?
    if(bEndTransmition == xtrue)                                     // Yes
    {
        // Disable Auto Ackknowledge
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_AAC;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // Wait Receive Data
        while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_DAT_ACK);
        ulTmp = xHWREG(ulBase + I2C_DAT);

        // Release Stop Signal
        xHWREG(ulBase + I2C_CONSET) = CONSET_STO;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;
    }
    else                                                             // No
    {
        // Enable Auto Ackknowledge
        xHWREG(ulBase + I2C_CONSET) = CONSET_AA;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // Wait Receive Data
        while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_DAT_ACK);
        ulTmp = xHWREG(ulBase + I2C_DAT);
    }

    return (ulTmp);
}

unsigned long I2CMasterReadLastRequestS2(unsigned long ulBase)
{    
    // Disable Auto Ackknowledge
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_AAC;
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

    // Wait Receive Data
    while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_DAT_ACK);
    return ( xHWREG(ulBase + I2C_DAT) );
}

unsigned long I2CMasterReadS1(unsigned long ulBase,
        unsigned long ucSlaveAddr,
        unsigned char *pucData, xtBoolean bEndTransmition)
{
    *pucData = I2CMasterReadRequestS1(ulBase, ucSlaveAddr, bEndTransmition);
    return (0);
}

unsigned long I2CMasterReadS2(unsigned long ulBase, unsigned char *pucData, xtBoolean bEndTransmition)
{
    *pucData = I2CMasterReadRequestS2(ulBase, bEndTransmition);
    return (0);
}

unsigned long  I2CMasterReadBufS1(unsigned long ulBase,
        unsigned long ucSlaveAddr,
        unsigned char *pucDataBuf,
        unsigned long ulLen,
        xtBoolean bEndTransmition)
{
    unsigned long ulTmpReg = 0;    
    unsigned long i        = 0;

    // Send start signal
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_AAC;
    xHWREG(ulBase + I2C_CONSET) = CONSET_STA;
    while(1)
    {
        ulTmpReg = xHWREG(ulBase + I2C_STAT);
        // A Start/Restart signal has been transmitted.
        if( (ulTmpReg == I2C_STAT_M_RX_START) || (ulTmpReg == I2C_STAT_M_RX_RESTART) )
        {
            break;
        }
    }

    // Send slave address + W
    ucSlaveAddr |= BIT_32_0;
    xHWREG(ulBase + I2C_DAT) = ucSlaveAddr;
    xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC | CONCLR_STAC;
    while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_SLAR_ACK);

    for(i = 0; i < ulLen; i++)
    {
        // Disable Auto Ackknowledge
        xHWREG(ulBase + I2C_CONSET) = CONSET_AA;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // Wait Receive Data
        while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_DAT_ACK);
        pucDataBuf[i] = xHWREG(ulBase + I2C_DAT);
    }

    // Need to Send Stop signal ?
    if(bEndTransmition == xtrue)                                     // Yes
    {
        // Disable Auto Ackknowledge
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_AAC;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // Wait Receive Data
        while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_DAT_ACK);
        pucDataBuf[i] = xHWREG(ulBase + I2C_DAT);
        
        // Release Stop Signal
        xHWREG(ulBase + I2C_CONSET) = CONSET_STO;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;
    }
    else                                                             // No
    {
        // Enable Auto Ackknowledge
        xHWREG(ulBase + I2C_CONSET) = CONSET_AA;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // Wait Receive Data
        while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_DAT_ACK);
        pucDataBuf[i] = xHWREG(ulBase + I2C_DAT);
    }

    return (ulLen);
}

unsigned long I2CMasterReadBufS2(unsigned long ulBase,
        unsigned char *pucDataBuf,
        unsigned long ulLen,
        xtBoolean bEndTransmition)
{    
    unsigned long i        = 0;

    for(i = 0; i < ulLen; i++)
    {
        // Disable Auto Ackknowledge
        xHWREG(ulBase + I2C_CONSET) = CONSET_AA;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // Wait Receive Data
        while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_DAT_ACK);
        pucDataBuf[i] = xHWREG(ulBase + I2C_DAT);
    }

    // Need to Send Stop signal ?
    if(bEndTransmition == xtrue)                                     // Yes
    {
        // Disable Auto Ackknowledge
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_AAC;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // Wait Receive Data
        while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_DAT_ACK);
        pucDataBuf[i] = xHWREG(ulBase + I2C_DAT);

        // Release Stop Signal
        xHWREG(ulBase + I2C_CONSET) = CONSET_STO;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;
    }
    else                                                             // No
    {
        // Enable Auto Ackknowledge
        xHWREG(ulBase + I2C_CONSET) = CONSET_AA;
        xHWREG(ulBase + I2C_CONCLR) = CONCLR_SIC;

        // Wait Receive Data
        while(xHWREG(ulBase + I2C_STAT) != I2C_STAT_M_RX_DAT_ACK);
        pucDataBuf[i] = xHWREG(ulBase + I2C_DAT);
    }

    return (ulLen);
}

xtBoolean I2CBusBusyStatus(unsigned long ulBase)
{
    if(xHWREG(ulBase + I2C_STAT) != I2C_STAT_NO_INF)           // I2C Bus is busy ?
    {
        return (xtrue);                                        // Yes
    }
    else
    {
        return (xfalse);                                       // No
    }
}

