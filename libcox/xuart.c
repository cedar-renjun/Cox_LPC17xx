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
#include "xhw_uart.h"
#include "xuart.h"

static volatile unsigned long g_DataStatus = 0;

static xtBoolean UartSetDivisors(unsigned long ulBase, unsigned long ulBaudrate)
{
    unsigned long      ulPeriClk     = 0;
    unsigned long      d             = 0;
    unsigned long      m             = 0;
    unsigned long      bestd         = 0;
    unsigned long      bestm         = 0;
    unsigned long      tmp           = 0;
    unsigned long      current_error = 0;
    unsigned long      best_error    = 0;
    unsigned long      max_error     = 3;
    unsigned long      recalcbaud    = 0;
    unsigned long long best_divisor  = 0;
    unsigned long long divisor       = 0;

    switch(ulBase)
    {
        case UART0_BASE:
            {
                ulPeriClk = SysCtlPeripheralClockGet(PCLKSEL_UART0);
                break;
            }

        case UART1_BASE:
            {
                ulPeriClk = SysCtlPeripheralClockGet(PCLKSEL_UART1);
                break;
            }

        case UART2_BASE:
            {
                ulPeriClk = SysCtlPeripheralClockGet(PCLKSEL_UART2);
                break;
            }

        case UART3_BASE:
            {
                ulPeriClk = SysCtlPeripheralClockGet(PCLKSEL_UART3);
                break;
            }
        default:                         // Error
            {
                while(1);
            }
    }


    // In the Uart IP block, baud rate is calculated using FDR and DLL-DLM registers
    // The formula is :
    // BaudRate= uPeriClk * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * (DLL)
    // It involves floating point calculations. That's the reason the formulae are adjusted with
    // Multiply and divide method.
    // The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
    // 0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15

    // Worst case
    best_error = 0xFFFFFFFF;
    bestd = 0;
    bestm = 0;
    best_divisor = 0;

    for (m = 1 ; m <= 15 ;m++)
    {
        for (d = 0 ; d < m ; d++)
        {
            divisor       = ((unsigned long long)ulPeriClk<<28)*m/(ulBaudrate*(m+d));
            current_error = divisor & 0xFFFFFFFF;

            tmp = divisor>>32;

            // Adjust error
            if(current_error > ((unsigned long)1<<31))
            {
                current_error = -current_error;
                tmp++;
            }

            if(tmp<1 || tmp>65536)
            {
                continue;
            }

            if( current_error < best_error)
            {
                best_error   = current_error;
                best_divisor = tmp;
                bestd        = d;
                bestm        = m;
                if(best_error == 0)
                {
                    break;
                }
            }
        }

        if (best_error == 0)
        {
            break;
        }
    }

    // Can not find best match
    if(best_divisor == 0)
    {
        return xfalse;
    }

    recalcbaud = (ulPeriClk>>4) * bestm/(best_divisor * (bestm + bestd));

    // Reuse best_error to evaluate baud error
    if(ulBaudrate > recalcbaud)
    {
        best_error = ulBaudrate - recalcbaud;
    }
    else
    {
        best_error = recalcbaud -ulBaudrate;
    }

    best_error = best_error * 100 / ulBaudrate;

    if (best_error < max_error)
    {
        if(ulBase == UART1_BASE)
        {
            // Nothing to do Now.
        }
        else
        {
            //Configure DLM/DLL
            xHWREG(ulBase + LCR) |= LCR_DLAB;
            xHWREG(ulBase + DLM) = (best_divisor >> 8) & 0xFF;
            xHWREG(ulBase + DLL) = (best_divisor >> 0) & 0xFF;

            // Configure FDR
            xHWREG(ulBase + LCR) &= ~LCR_DLAB;
            bestd &= (unsigned long)0x0F;
            bestm = (bestm << 4) & (unsigned long)0xF0;
            xHWREG(ulBase + FDR) = bestd | bestm;
        }

        return (xtrue);                                    // Success configure Baud registers.
    }

    return (xfalse);                                       // Failure to configure Baud registers.
}


unsigned char UARTByteRead(unsigned long ulBase)
{
    unsigned long ulTmpReg = 0;

    if(0 != g_DataStatus)              // DLAB has been set
    {
        xHWREG(ulBase + LCR) &= ~LCR_DLAB;
    }

    // Waiting UART receive one byte.
    do
    {
        ulTmpReg = xHWREG(ulBase + LSR);
        ulTmpReg &= LSR_RDR;
    }while(0 == ulTmpReg);

    // Read the byte.
    ulTmpReg = xHWREG(ulBase + RBR);

    return (ulTmpReg);
}

void UARTByteWrite(unsigned long ulBase, unsigned char ucData)
{
    unsigned long ulTmpReg = 0;
    
    if(0 != g_DataStatus)                             // DLAB has been set
    {
        xHWREG(ulBase + LCR) &= ~LCR_DLAB;
    }

    // Waiting UART receive one byte.
    do
    {
        ulTmpReg = xHWREG(ulBase + LSR);
        ulTmpReg &= LSR_THRE;
    }while(0 == ulTmpReg);

    // Write Byte into FIFO.
    xHWREG(ulBase + THR) = (unsigned long) ucData;    // UART Data register.
}



xtBoolean UARTByteReadNoBlocking(unsigned long ulBase, unsigned char * ucpData)
{
    unsigned long ulTmpReg = 0;

    if(0 != g_DataStatus)              // DLAB has been set
    {
        xHWREG(ulBase + LCR) &= ~LCR_DLAB;
    }

    // Receive one byte ?
    ulTmpReg = xHWREG(ulBase + LSR);
    if(ulTmpReg & LSR_RDR)            // Yes
    {
        // Read the byte.
        *ucpData = (unsigned char) xHWREG(ulBase + RBR);
        return (xtrue);
    }
    else                             // No
    {
        return (xfalse);
    }

}

xtBoolean UARTByteWriteNoBlocking(unsigned long ulBase, unsigned char ucData)
{
    unsigned long ulTmpReg = 0;
        
    if(0 != g_DataStatus)              //DLAB has been set
    {
        xHWREG(ulBase + LCR) &= ~LCR_DLAB;
    }

    // Transmitter FIFO is empty ?
    ulTmpReg = xHWREG(ulBase + LSR);
    if(ulTmpReg & LSR_THRE)            // Yes
    {
        // Write Byte into FIFO.
        xHWREG(ulBase + THR) = (unsigned long)ucData;
        return (xtrue);
    }
    else                               // No
    {
        return (xfalse);
    }

}

void UARTStrSend(unsigned long ulBase, unsigned char * pStr)
{
    while(NULL != *pStr)
    {
        UARTByteWrite(ulBase, *pStr++);
    }
}

void UARTBufWrite(unsigned long ulBase, unsigned char * pBuf, unsigned long ulLen)
{
    unsigned long i = 0;

    for(i = 0; i < ulLen; i++)
    {
        UARTByteWrite(ulBase, pBuf[i]);
    }
}

void UARTBufRead(unsigned long ulBase, unsigned char * pBuf, unsigned long ulLen)
{
    unsigned long i = 0;

    for(i = 0; i < ulLen; i++)
    {
        pBuf[i] = UARTByteRead(ulBase);
    }
}

void UARTIntEnable(unsigned long ulBase, unsigned long ulIntFlags)
{
    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    xASSERT( (ulIntFlags & ~(
                            INT_RDA     |
                            INT_THRE    |
                            INT_RX_LINE |
                            INT_MODEM   |
                            INT_CTS     |
                            INT_ABEO    |
                            INT_ABTO
                    )
             ) == 0);

    // Set interrupt control bit.
    xHWREG(ulBase + IER) |= ulIntFlags;
}

void UARTIntDisable(unsigned long ulBase, unsigned long ulIntFlags)
{
    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    xASSERT( (ulIntFlags & ~(
                            INT_RDA     |
                            INT_THRE    |
                            INT_RX_LINE |
                            INT_MODEM   |
                            INT_CTS     |
                            INT_ABEO    |
                            INT_ABTO
                    )
             ) == 0);

    // Clear interrupt control bit.
    xHWREG(ulBase + IER) &= ~ulIntFlags;
}

unsigned long UARTIntGet(unsigned long ulBase)
{

    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    // Read status register.
    return xHWREG(ulBase + IIR);
}

xtBoolean UARTIntCheck(unsigned long ulBase, unsigned long ulIntFlags)
{
    // Interrupt Idenetification register.
    unsigned long ulTmpReg = xHWREG(ulBase + IIR) & IIR_INT_ID_M;

    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    xASSERT( (ulIntFlags & ~(
                            IIR_INT_ID_RLS   |
                            IIR_INT_ID_RDA   |
                            IIR_INT_ID_CTI   |
                            IIR_INT_ID_THRE  |
                            IIR_INT_ID_MODEM
                    )
             ) == 0);

    if(ulTmpReg == ulIntFlags)
    {
        return (xtrue);
    }
    else
    {
        return (xfalse);
    }
}



void UARTFIFOCfg(unsigned long ulBase, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    xASSERT( (ulCfg & ~(
                             FIFO_CFG_FIFO_EN       |        
                             FIFO_CFG_FIFO_DIS      |        
                             FIFO_CFG_RX_FIFO_RESET |        
                             FIFO_CFG_TX_FIFO_RESET |        
                             FIFO_CFG_DMA_EN        |        
                             FIFO_CFG_DMA_DIS       |        
                             FIFO_CFG_RX_TRI_LVL_0  |        
                             FIFO_CFG_RX_TRI_LVL_1  |        
                             FIFO_CFG_RX_TRI_LVL_2  |        
                             FIFO_CFG_RX_TRI_LVL_3          
                         )
             ) == 0);

    // Configure FIFO
    ulTmpReg = xHWREG(ulBase + FCR);
    ulTmpReg &= ((~ulCfg) >> 16);
    ulTmpReg |= (ulCfg & 0xFFFF);
    xHWREG(ulBase + FCR) = ulTmpReg;
}

void UARTTransStart(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    xHWREG(ulBase + TER) |= TER_TX_EN;
}

void UARTTransStop(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    xHWREG(ulBase + TER) &= ~TER_TX_EN;
}


unsigned long UARTStatGet(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    return( xHWREG(ulBase + LSR) );
}

xtBoolean UARTStatCheck(unsigned long ulBase, unsigned long ulFlags)
{

    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );
    xASSERT( (ulFlags & ~(
                              RX_FIFO_NOT_EMPTY |             
                              OVERRUN_ERR       |             
                              PARITY_ERR        |             
                              FRAMING_ERR       |             
                              BREAK_INT         |             
                              TX_FIFO_EMPTY     |             
                              TX_EMPTY          |             
                              RX_FIFO_ERR                    
                         )
             ) == 0);

    if(xHWREG(ulBase + LSR) & ulFlags)
    {
        return (xtrue);
    }
    else
    {
        return (xfalse);
    }
}

//! \note This function is only suit for UART0/2/3.
void UARTIrDACfg(unsigned long ulBase, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) ||
            (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) );
    xASSERT( (ulCfg & ~(
                           IRDA_INV_EN        |           
                           IRDA_INV_DIS       |           
                           IRDA_FIX_PULSE_DIS |           
                           IRDA_FIX_PULSE_2   |           
                           IRDA_FIX_PULSE_4   |           
                           IRDA_FIX_PULSE_8   |           
                           IRDA_FIX_PULSE_16  |           
                           IRDA_FIX_PULSE_32  |           
                           IRDA_FIX_PULSE_64  |           
                           IRDA_FIX_PULSE_128 |           
                           IRDA_FIX_PULSE_256             
                         )
             ) == 0);


    // Configure IrDA Invert, Fixed Pulse width.
    ulTmpReg = xHWREG(ulBase + ICR);
    ulTmpReg &= ((~ulCfg) >> 16);
    ulTmpReg |= (ulCfg & 0xFFFF);
    xHWREG(ulBase + ICR) = ulTmpReg;
    
    ulTmpReg = xHWREG(ulBase + ICR);
}

void UARTIrDAEnable(unsigned long ulBase)
{
    xHWREG(ulBase + ICR) |= ICR_IRDA_EN;
}

void UARTIrDADisable(unsigned long ulBase)
{
    xHWREG(ulBase + ICR) &= ~ICR_IRDA_EN;
}


void UARTCfg(unsigned long ulBase, unsigned long ulBaud, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );
    xASSERT( (ulCfg & ~(
                           UART_CFG_LEN_5_BIT   |
                           UART_CFG_LEN_6_BIT   |
                           UART_CFG_LEN_7_BIT   |
                           UART_CFG_LEN_8_BIT   |
                           UART_CFG_STOP_1_BIT  |
                           UART_CFG_STOP_2_BIT  |
                           UART_CFG_PARITY_NONE |
                           UART_CFG_PARITY_ODD  |
                           UART_CFG_PARITY_EVEN |
                           UART_CFG_PARITY_1    |
                           UART_CFG_PARITY_0    |
                           UART_CFG_BREAK_EN    |
                           UART_CFG_BREAK_DIS
                       )
             ) == 0);

    // Configure UART Data length, Parity, stop bit, break.
    ulTmpReg = xHWREG(ulBase + LCR);
    ulTmpReg &= ((~ulCfg) >> 16);
    ulTmpReg |= (ulCfg & 0xFFFF);
    xHWREG(ulBase + LCR) = ulTmpReg;

    // Configure UART baud
    UartSetDivisors(ulBase, ulBaud);
}



void UARTModemCfg(unsigned long ulBase, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    // Check input parameters.
    xASSERT(ulBase == UART1_BASE);
    xASSERT( (ulCfg & ~(
                       LOOPBACK_MODE_EN  | LOOPBACK_MODE_DIS |
                       AUTO_RTS_EN       | AUTO_RTS_DIS      |
                       AUTO_CTS_EN       | AUTO_CTS_DIS
                       )
             ) == 0);

    // Configure UART Modem.
    ulTmpReg = xHWREG(ulBase + MCR);
    ulTmpReg &= ((~ulCfg) >> 16);
    ulTmpReg |= (ulCfg & 0xFFFF);
    xHWREG(ulBase + MCR) = ulTmpReg;

}



void UARTRS485Cfg(unsigned long ulBase, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    // Check input parameters.
    xASSERT(ulBase == UART1_BASE);
    xASSERT( (ulCfg & RS485_PARA_M) == 0 );

    // Configure RS485
    ulTmpReg = xHWREG(ulBase + RS485CTRL);
    ulTmpReg &= ((~ulCfg) >> 16);
    ulTmpReg |= (ulCfg & 0xFFFF);
    xHWREG(ulBase + RS485CTRL) = ulTmpReg;
}

//!  \note This function is only suit for UART1.
void UARTRS485AddrSet(unsigned long ulBase, unsigned long ulVal)
{
    // Check input parameters.
    xASSERT(ulBase == UART1_BASE);
    xASSERT((ulVal & ~RS485ADRMATCH_ADRMATCH_M) == 0);

    xHWREG(ulBase + ADRMATCH) = (ulVal & RS485ADRMATCH_ADRMATCH_M);
}

//! \note This function is only suit for UART1.
void UARTRS485DlyTimeSet(unsigned long ulBase, unsigned long ulVal)
{
    // Check input parameters.
    xASSERT(ulBase == UART1_BASE);
    xASSERT((ulVal & ~RS485DLY_DLY_M) == 0);

    xHWREG(ulBase + RS485DLY) = (ulVal & RS485DLY_DLY_M);
}

//! \note This function is only suit for UART1.
unsigned long UARTModemStatGet(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT(ulBase == UART1_BASE);

    return (xHWREG(ulBase + MSR));
}



//! \note This function is only suit for UART1.
xtBoolean UARTModemStatCheck(unsigned long ulBase, unsigned long ulFlags)
{

    // Check input parameters.
    xASSERT(ulBase == UART1_BASE);

    // Check Status Bit.
    if(xHWREG(ulBase + MSR) & ulFlags)                       // Set
    {
        return (xtrue);
    }
    else                                         // Unset
    {
        return (xfalse);
    }
}
