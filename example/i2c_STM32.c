
#include "xhw_types.h"
#include "xhw_ints.h"
#include "xhw_memmap.h"
#include "xhw_nvic.h"
#include "xdebug.h"
#include "xcore.h"
#include "xhw_sysctl.h"
#include "xsysctl.h"
#include "xhw_gpio.h"
#include "xgpio.h"
#include "xhw_i2c.h"
#include "xi2c.h"

#define TICK_SLOW              ((unsigned long)0xFFFFF)

static void Delay(unsigned long tick) 
{
    volatile unsigned long _tick  = tick;
    while(_tick--);
}

/*
unsigned long WDTHandler(void *pvCBData, unsigned long ulEvent,
                              unsigned long ulMsgParam, void *pvMsgData)
{
    if (WDTStatusFlagCheck(WDT_FLAG_TIMEOUT))
    {
        WDTStatusFlagClear(WDT_FLAG_TIMEOUT);
        while(1);
    }

    if (WDTStatusFlagCheck(WDT_FLAG_INT))
    {
        while(1);
    }

    while(1);
    return (0);
}
*/

/*
void I2CAddrSet(unsigned long ulBase, I2C_OWNSLAVEADDR_CFG_Type *OwnSlaveAddrConfigStruct)
{
    uint32_t tmp;
    CHECK_PARAM(PARAM_I2Cx(I2Cx));
    CHECK_PARAM(PARAM_I2C_SLAVEADDR_CH(OwnSlaveAddrConfigStruct->SlaveAddrChannel));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(OwnSlaveAddrConfigStruct->GeneralCallState));

    tmp = (((uint32_t)(OwnSlaveAddrConfigStruct->SlaveAddr_7bit << 1)) \
            | ((OwnSlaveAddrConfigStruct->GeneralCallState == ENABLE) ? 0x01 : 0x00))& I2C_I2ADR_BITMASK;
    switch (OwnSlaveAddrConfigStruct->SlaveAddrChannel)
    {
    case 0:
        I2Cx->I2ADR0 = tmp;
        I2Cx->I2MASK0 = I2C_I2MASK_MASK((uint32_t) \
                (OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
        break;
    case 1:
        I2Cx->I2ADR1 = tmp;
        I2Cx->I2MASK1 = I2C_I2MASK_MASK((uint32_t) \
                (OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
        break;
    case 2:
        I2Cx->I2ADR2 = tmp;
        I2Cx->I2MASK2 = I2C_I2MASK_MASK((uint32_t) \
                (OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
        break;
    case 3:
        I2Cx->I2ADR3 = tmp;
        I2Cx->I2MASK3 = I2C_I2MASK_MASK((uint32_t) \
                (OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
        break;
    }
}
*/


unsigned long Tmp = 0;
unsigned char I2CData[] = {0x00, 0x00, 0x12, 0x34, 0x56};
unsigned char I2CTmp[]  = {0x00, 0x00, 0x00, 0x00, 0x00};

#define I2C_SLAVE_ADDR  0xF4

void main(void)
{ 
    //unsigned long ulTmp = 0;

    Delay(0xFFFFF);
    
    /********************** Configure System clock *************************/
    SysCtlClockSet(100000000, SYSCTL_OSC_INT | SYSCTL_XTAL_12_MHZ);
    Delay(TICK_SLOW);

    /********************** Configure I2C Port *****************************/
    GPIOPinFunCfg(GPIOA_BASE, GPIO_PIN_19, GPIO_PA19_I2C1SDA);
    GPIOPinFunCfg(GPIOA_BASE, GPIO_PIN_20, GPIO_PA20_I2C1SCL);

    GPIOPinModeCfg(GPIOA_BASE, GPIO_PIN_19, PIN_MODE_OD_EN | PIN_MODE_PULL_UP);
    GPIOPinModeCfg(GPIOA_BASE, GPIO_PIN_20, PIN_MODE_OD_EN | PIN_MODE_PULL_UP);

    /*
    I2CMasterInit(I2C1_BASE, 100000);
    I2CSlaveInit(I2C1_BASE, 0xFF, I2C_GENERAL_CALL_DIS);
    I2CSlaveInit(I2C1_BASE, 0xF0, I2C_GENERAL_CALL_EN);
    */

    // General Call Enable/Disable
    /*
    I2CGeneralCallEnable(I2C0_BASE, I2C_SLAVE_ADD0);
    I2CGeneralCallEnable(I2C0_BASE, I2C_SLAVE_ADD1);
    I2CGeneralCallEnable(I2C0_BASE, I2C_SLAVE_ADD2);
    I2CGeneralCallEnable(I2C0_BASE, I2C_SLAVE_ADD3);

    I2CGeneralCallDisable(I2C0_BASE, I2C_SLAVE_ADD0);
    I2CGeneralCallDisable(I2C0_BASE, I2C_SLAVE_ADD1);
    I2CGeneralCallDisable(I2C0_BASE, I2C_SLAVE_ADD2);
    I2CGeneralCallDisable(I2C0_BASE, I2C_SLAVE_ADD3);

    I2CGeneralCallEnable(I2C1_BASE, I2C_SLAVE_ADD0);
    I2CGeneralCallEnable(I2C1_BASE, I2C_SLAVE_ADD1);
    I2CGeneralCallEnable(I2C1_BASE, I2C_SLAVE_ADD2);
    I2CGeneralCallEnable(I2C1_BASE, I2C_SLAVE_ADD3);

    I2CGeneralCallDisable(I2C1_BASE, I2C_SLAVE_ADD0);
    I2CGeneralCallDisable(I2C1_BASE, I2C_SLAVE_ADD1);
    I2CGeneralCallDisable(I2C1_BASE, I2C_SLAVE_ADD2);
    I2CGeneralCallDisable(I2C1_BASE, I2C_SLAVE_ADD3);

    I2CGeneralCallEnable(I2C2_BASE, I2C_SLAVE_ADD0);
    I2CGeneralCallEnable(I2C2_BASE, I2C_SLAVE_ADD1);
    I2CGeneralCallEnable(I2C2_BASE, I2C_SLAVE_ADD2);
    I2CGeneralCallEnable(I2C2_BASE, I2C_SLAVE_ADD3);

    I2CGeneralCallDisable(I2C2_BASE, I2C_SLAVE_ADD0);
    I2CGeneralCallDisable(I2C2_BASE, I2C_SLAVE_ADD1);
    I2CGeneralCallDisable(I2C2_BASE, I2C_SLAVE_ADD2);
    I2CGeneralCallDisable(I2C2_BASE, I2C_SLAVE_ADD3);
    */

    // Address configure.
    /*
    I2CSlaveAddrSet(I2C0_BASE, I2C_SLAVE_ADD0, 0xA5);
    I2CSlaveAddrSet(I2C0_BASE, I2C_SLAVE_ADD1, 0xA5);
    I2CSlaveAddrSet(I2C0_BASE, I2C_SLAVE_ADD2, 0xA5);
    I2CSlaveAddrSet(I2C0_BASE, I2C_SLAVE_ADD3, 0xA5);

    I2CSlaveAddrSet(I2C0_BASE, I2C_SLAVE_ADD0, 0x5A);
    I2CSlaveAddrSet(I2C0_BASE, I2C_SLAVE_ADD1, 0x5A);
    I2CSlaveAddrSet(I2C0_BASE, I2C_SLAVE_ADD2, 0x5A);
    I2CSlaveAddrSet(I2C0_BASE, I2C_SLAVE_ADD3, 0x5A);

    I2CSlaveAddrSet(I2C1_BASE, I2C_SLAVE_ADD0, 0xA5);
    I2CSlaveAddrSet(I2C1_BASE, I2C_SLAVE_ADD1, 0xA5);
    I2CSlaveAddrSet(I2C1_BASE, I2C_SLAVE_ADD2, 0xA5);
    I2CSlaveAddrSet(I2C1_BASE, I2C_SLAVE_ADD3, 0xA5);

    I2CSlaveAddrSet(I2C1_BASE, I2C_SLAVE_ADD0, 0x5A);
    I2CSlaveAddrSet(I2C1_BASE, I2C_SLAVE_ADD1, 0x5A);
    I2CSlaveAddrSet(I2C1_BASE, I2C_SLAVE_ADD2, 0x5A);
    I2CSlaveAddrSet(I2C1_BASE, I2C_SLAVE_ADD3, 0x5A);

    I2CSlaveAddrSet(I2C2_BASE, I2C_SLAVE_ADD0, 0xAF);
    I2CSlaveAddrSet(I2C2_BASE, I2C_SLAVE_ADD1, 0xAF);
    I2CSlaveAddrSet(I2C2_BASE, I2C_SLAVE_ADD2, 0xAF);
    I2CSlaveAddrSet(I2C2_BASE, I2C_SLAVE_ADD3, 0xAF);

    I2CSlaveAddrSet(I2C2_BASE, I2C_SLAVE_ADD0, 0x50);
    I2CSlaveAddrSet(I2C2_BASE, I2C_SLAVE_ADD1, 0x50);
    I2CSlaveAddrSet(I2C2_BASE, I2C_SLAVE_ADD2, 0x50);
    I2CSlaveAddrSet(I2C2_BASE, I2C_SLAVE_ADD3, 0x50);
    */

    // Module Enable/Disable Function.
    /*
    I2CEnable(I2C0_BASE);
    I2CEnable(I2C1_BASE);
    I2CEnable(I2C2_BASE);

    I2CDisable(I2C0_BASE);
    I2CDisable(I2C1_BASE);
    I2CDisable(I2C2_BASE);

    I2CEnable(I2C0_BASE);
    I2CEnable(I2C1_BASE);
    I2CEnable(I2C2_BASE);

    I2CDisable(I2C0_BASE);
    I2CDisable(I2C1_BASE);
    I2CDisable(I2C2_BASE);
    */

    I2CMasterInit(I2C1_BASE, 100000);
    I2CEnable(I2C1_BASE);
    //I2CIntEnable(I2C1_BASE);

    I2CMasterWriteS1(I2C1_BASE, I2C_SLAVE_ADDR, 0x00, xfalse);
    I2CMasterWriteS2(I2C1_BASE, 0x00, xfalse);
    I2CMasterWriteS2(I2C1_BASE, 0x01, xfalse);
    I2CMasterWriteS2(I2C1_BASE, 0x02, xfalse);
    I2CMasterWriteS2(I2C1_BASE, 0x03, xtrue);

    /*
    I2CMasterWriteBufS1(I2C1_BASE, 0xA0, I2CData, sizeof(I2CData), xtrue);
    
    //Slave need time to deal data.
    SysCtlDelay(0xFFFFF);
    I2CMasterWriteS1(I2C1_BASE, 0xA0, 0x00, xfalse);
    I2CMasterWriteS2(I2C1_BASE, 0x00, xfalse);

    I2CTmp[0] = I2CMasterReadRequestS1(I2C1_BASE, 0xA0, xfalse);
    I2CTmp[1] = I2CMasterReadRequestS2(I2C1_BASE, xfalse);
    I2CTmp[2] = I2CMasterReadRequestS2(I2C1_BASE, xtrue);
    */

    while (1)
    {
        
    }
}

