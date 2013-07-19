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


// GPIO Pin Interrupt
// PA 0  --> 11   12
// PA 15 --> 30   15
// PC 0  --> 13   14

static xtEventCallback g_psGPIOPinIntAssignTable[64] = 
{
    0,
};

static unsigned long PinIDToPos(unsigned long ulPin)
{ 
    unsigned long i      = 0;
    unsigned long ulFlag = 0;

    for(i = 0; i < 32; i++)
    {
        ulFlag = 0x01 << i;
        if(ulPin & ulFlag)
        {
            return (i);
        }
    }

    return (-1);
}  


void xGPIODirModeSet(unsigned long ulPort, unsigned long ulPins, unsigned long ulPinIO)
{
    unsigned long i = 0;

    for(i = 0; i < 32; i++)
    {
        if(ulPins & (0x01 << i))
        {
            if(0 != ulPinIO)
            {
                GPIOPinModeCfg(ulPort, 0x01<<i, ulPinIO);
            }
        }
    }
}

unsigned long GPIOPinToPeripheralId(unsigned long ulPort, unsigned long ulPin)
{

    switch (ulPort)
    {
        case GPIOA_BASE:
            {
                return (SYSCTL_PERIPH_GPIOA);
                break;
            }
        case GPIOB_BASE:
            {
                return (SYSCTL_PERIPH_GPIOB);
                break;
            }
        case GPIOC_BASE:
            {
                return (SYSCTL_PERIPH_GPIOC);
                break;
            }
        case GPIOD_BASE:
            {
                return (SYSCTL_PERIPH_GPIOD);
                break;
            }
        case GPIOE_BASE:
            {
                return (SYSCTL_PERIPH_GPIOE);
                break;
            }
    }

    return (0);                      // Error
}

unsigned long  GPIOPinToPort(unsigned long ulPort, unsigned long ulPin)
{

    return (ulPort);
}

unsigned long  GPIOPinToPin(unsigned long ulPort, unsigned long ulPin)
{

    return (ulPin);
}

unsigned long xGPIODirModeGet(unsigned long ulPort, unsigned long ulPin)
{
    //! \todo Finish this function.
    return (0);
}

void xGPIOPinIntCallbackInit(unsigned long ulPort, unsigned long ulPin, 
                                   xtEventCallback xtPortCallback) 
{
    unsigned long PinNum = PinIDToPos(ulPin);
    
    if(GPIOA_BASE == ulPort)
    {
        g_psGPIOPinIntAssignTable[PinNum] = xtPortCallback;
    }
    else if(GPIOC_BASE == ulPort)
    {
        PinNum += 32;
        g_psGPIOPinIntAssignTable[PinNum] = xtPortCallback;
    }
    else           // Error
    {
        while(1);
    }

}

void xGPIOPinIntEnable(unsigned long ulPort, unsigned long ulPins, unsigned long ulIntType)
{
    unsigned long i = 0;

    for(i = 0; i < 32; i++)
    {
        if(ulPins & (0x01 << i))
        {
            GPIOPinIntCfg(ulPort, (0x01 << i), ulIntType);
            GPIOPinIntEnable(ulPort, (0x01 << i));
        }
    }
}

void xGPIOPinIntDisable(unsigned long ulPort, unsigned long ulPins)
{
    unsigned long i = 0;

    for(i = 0; i < 32; i++)
    {
        if(ulPins & (0x01 << i))
        {
            GPIOPinIntDisable(ulPort, (0x01 << i));
        }
    }
}

void xGPIOPinIntClear(unsigned long ulPort, unsigned long ulPins)
{
    unsigned long i = 0;

    for(i = 0; i < 32; i++)
    {
        if(ulPins & (0x01 << i))
        {
            GPIOPinIntFlagClear(ulPort, (0x01 << i));
        }
    } 
}

unsigned long xGPIOPinRead(unsigned long ulPort, unsigned long ulPins)
{
    xHWREG(ulPort + FIOMASK) = ~ulPins;
    return xHWREG(ulPort + FIOPIN);
}

void xGPIOPinWrite(unsigned long ulPort, unsigned long ulPins,
        unsigned long ulVal)
{
    xHWREG(ulPort + FIOMASK) = ~ulPins;
    xHWREG(ulPort + FIOPIN)  =   ulVal;
}

////////////////////////////////////////////////////////////////////



//! ulPort GPIOA_BASE/GPIOB_BASE/GPIOC_BASE/GPIOD_BASE/GPIOE_BASE
//! ulPin
//! \note When you Configure I2C0, you just need to configure
//!       ulCfg is I2C StandMode/FastMode/FastPlusMode.
void GPIOPinFunCfg(unsigned long ulPort, unsigned long ulPin, unsigned long ulCfg)
{
    unsigned long ulRegAddr = 0;
    unsigned long ulTmpReg  = 0;    

    switch(ulPort)
    {
        case GPIOA_BASE:               // Port 0
            {
                ulRegAddr = PINSEL0;
                break;
            }
        case GPIOB_BASE:               // Port 1
            {
                ulRegAddr = PINSEL2;
                break;
            }
        case GPIOC_BASE:               // Port 2
            {
                ulRegAddr = PINSEL4;
                break;
            }
        case GPIOD_BASE:               // Port 3
            {
                ulRegAddr = PINSEL6;
                break;
            }
        case GPIOE_BASE:               // Port 4
            {
                ulRegAddr = PINSEL8;
                break;
            }
        default:
            {
                //while(1);            // Error
            }
    }

    ulPin = PinIDToPos(ulPin);
    // Select Pin ID
    if(ulPin >= 16)
    {
        ulPin     -= 16; 
        ulRegAddr += 4;
    }

    // Read --> Modify --> WriteBack
    ulTmpReg =  xHWREG(PIN_CON_BASE + ulRegAddr);
    ulTmpReg &= ~(0x03 << (2*ulPin));
    ulTmpReg |= ulCfg;
    xHWREG(PIN_CON_BASE + ulRegAddr) = ulTmpReg;

}

// Up, Down, Both, Neither
// INPUT / OUTPUT

void GPIOPinModeCfg(unsigned long ulPort, unsigned long ulPin, unsigned long ulCfg)
{
    unsigned long ulRegAddr = 0;
    unsigned long ulTmp     = 0;
    unsigned long ulTmpReg  = 0;
    unsigned long ulTmpMode = 0;

    /******************* Configure Input/Output Mode ****************/
    if(ulCfg & BIT_32_1)                         // Need to configure Input/Output Mode.
    {
        if(ulCfg & BIT_32_0)                     // OutPut Mode.
        {
            xHWREG(ulPort + FIODIR) |= ulPin;
        }
        else                                     // Input Mode
        {
            xHWREG(ulPort + FIODIR) &= ~ulPin;
        }
    }

    /***************** Configure pull-up/pull-down Resister ****************/
    if(ulCfg & BIT_32_4)                         // Need to configure Pull-up/Pull-down
    {
        switch(ulPort)                           // 
        {
            case GPIOA_BASE:
                {
                    ulRegAddr = PINMODE0;
                    break;
                }
            case GPIOB_BASE:
                {
                    ulRegAddr = PINMODE2;
                    break;
                }
            case GPIOC_BASE:
                {
                    ulRegAddr = PINMODE4;
                    break;
                }
            case GPIOD_BASE:
                {
                    ulRegAddr = PINMODE6;
                    break;
                }
            case GPIOE_BASE:
                {
                    ulRegAddr = PINMODE8;
                    break;
                }
        }

        ulTmp = PinIDToPos(ulPin);
        if(ulTmp >= 16)
        {
            ulRegAddr += 4;
            ulTmp     -= 16;
        }
        
        ulTmpReg  = xHWREG(PIN_CON_BASE + ulRegAddr);
        ulTmpReg &= ~((unsigned long)0x03<< (2*ulTmp));
        ulTmpMode = ulCfg & BIT_MASK(32, 3, 2);
        ulTmpMode = (ulTmpMode >> 2) << (2 * ulTmp);
        ulTmpReg |= ulTmpMode;
        xHWREG(PIN_CON_BASE + ulRegAddr)  = ulTmpReg;

    }

    /************************** Configure OD Mode **************************/
    if(ulCfg & BIT_32_6)                         // Configure OD
    {
        switch(ulPort)                 
        {
            case GPIOA_BASE:
                {
                    ulRegAddr = PINMODE_OD0;
                    break;
                }
            case GPIOB_BASE:
                {
                    ulRegAddr = PINMODE_OD1;
                    break;
                }
            case GPIOC_BASE:
                {
                    ulRegAddr = PINMODE_OD2;
                    break;
                }
            case GPIOD_BASE:
                {
                    ulRegAddr = PINMODE_OD3;
                    break;
                }
            case GPIOE_BASE:
                {
                    ulRegAddr = PINMODE_OD4;
                    break;
                }
        }

        if(ulCfg & BIT_32_5)                     // OD Enable
        {
            xHWREG(PIN_CON_BASE + ulRegAddr)  |= ulPin;
        }
        else                                     // OD Disable
        {
            xHWREG(PIN_CON_BASE + ulRegAddr)  &= ~ulPin;
        }
    }
}


// ulPin GPIO_PIN_n (n = 0/1/../31)
void GPIOPinSet(unsigned long ulPort, unsigned long ulPins)
{
    xHWREG(ulPort + FIOMASK) =~ulPins;
    xHWREG(ulPort + FIOSET)  = ulPins;
}

// ulPin GPIO_PIN_n (n = 0/1/../31)
void GPIOPinClr(unsigned long ulPort, unsigned long ulPins)
{
    xHWREG(ulPort + FIOMASK) =~ulPins;
    xHWREG(ulPort + FIOCLR)  = ulPins;
}

void GPIOPinWrite(unsigned long ulPort, unsigned long ulPins, unsigned long ulVal)
{
    xHWREG(ulPort + FIOMASK) = ~ulPins;
    if(0 != ulVal)
    {
        xHWREG(ulPort + FIOSET)   =  ulPins;
    }
    else
    {
        xHWREG(ulPort + FIOCLR)   =  ulPins;
    }
}

unsigned long GPIOPinRead(unsigned long ulPort, unsigned long ulPin)
{
    xHWREG(ulPort + FIOMASK) = ~ulPin;
    return xHWREG(ulPort + FIOPIN);
}

unsigned long GPIOPortRead(unsigned long ulPort)
{
    xHWREG(ulPort + FIOMASK) = (unsigned long) 0x00;
    return xHWREG(ulPort + FIOPIN);
}

void GPIOPortWrite(unsigned long ulPort, unsigned long ulVal)
{
    xHWREG(ulPort + FIOMASK) = (unsigned long) 0x00;
    xHWREG(ulPort + FIOPIN)  = ulVal;
}


// ulPort GPIOA_BASE/GPIOA_BASE
// ulPin Special Pin for Port0 Port2
// ulCfg INT_TYPE_RISING INT_TYPE_FALLING


void GPIOPinIntCfg(unsigned long ulPort, unsigned long ulPin, unsigned long ulCfg)
{
    switch(ulPort)
    {
        case GPIOA_BASE:                        // Port 0
            {
                if(ulCfg & INT_TYPE_RISING)     // GPIO Rising Int Type
                {
                    xHWREG(GPIO_INT_BASE + IO0IntEnR) |= ulPin;
                }

                if(ulCfg & INT_TYPE_FALLING)    // GPIO Falling Int Type
                {
                    xHWREG(GPIO_INT_BASE + IO0IntEnF) |= ulPin;
                }

                break;
            }

        case GPIOC_BASE:                        // Port 2
            {
                if(ulCfg & INT_TYPE_RISING)     // GPIO Rising Int Type
                {
                    xHWREG(GPIO_INT_BASE + IO2IntEnR) |= ulPin;
                }

                if(ulCfg & INT_TYPE_FALLING)    // GPIO Falling Int Type
                {
                    xHWREG(GPIO_INT_BASE + IO2IntEnF) |= ulPin;
                }
                break;
            }
        default:
            {
                while(1);                       // Error
            }
    }
}

void GPIOPinIntEnable(unsigned long ulPort, unsigned long ulPin)
{
    // Avoid Compiler warning
    (void) ulPort;
    (void) ulPin;

    xIntEnable(INT_GPIO);
}

void GPIOPinIntDisable(unsigned long ulPort, unsigned long ulPin)
{
    // Avoid Compiler warning
    (void) ulPort;
    (void) ulPin;

    xIntDisable(INT_GPIO);
}

unsigned long GPIOPinIntFlagGet(unsigned long ulPort, unsigned long ulPin)
{
    unsigned long ulResult = 0;

    switch(ulPort)
    {
        case GPIOA_BASE:                           // Port 0
            {
                
                if(xHWREG(GPIO_INT_BASE + IO0IntEnR) & ulPin)      // GPIO Rising Int Type
                {
                    ulResult |= INT_TYPE_RISING;
                }

                if(xHWREG(GPIO_INT_BASE + IO0IntEnF) & ulPin)      // GPIO Falling Int Type
                {
                    ulResult |= INT_TYPE_FALLING;
                }

                break;
            }

        case GPIOC_BASE:                           // Port 2
            {
                if(xHWREG(GPIO_INT_BASE + IO2IntEnR) & ulPin)      // GPIO Rising Int Type
                {
                    ulResult |= INT_TYPE_RISING;
                }

                if(xHWREG(GPIO_INT_BASE + IO2IntEnF) & ulPin)      // GPIO Falling Int Type
                {
                    ulResult |= INT_TYPE_FALLING;
                }
                break;
            }
        default:
            {
                ulResult  = 0;                     // Error
            }
    }

    return (ulResult);
}

void GPIOPinIntFlagClear(unsigned long ulPort, unsigned long ulPin)
{

    switch(ulPort)
    {
        case GPIOA_BASE:                           // Port 0
            {
                xHWREG(GPIO_INT_BASE + IO0IntClr) |= ulPin;
                break;
            }

        case GPIOC_BASE:                           // Port 2
            {
                xHWREG(GPIO_INT_BASE + IO2IntClr) |= ulPin;
                break;
            }
        default:
            {
                while(1);
            }
    }
}

