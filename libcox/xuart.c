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
            divisor       = ((unsigned long long)uPeriClk<<28)*m/(baudrate*(m+d));
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

    recalcbaud = (uPeriClk>>4) * bestm/(best_divisor * (bestm + bestd));

    // Reuse best_error to evaluate baud error
    if(baudrate > recalcbaud)
    {
        best_error = baudrate - recalcbaud;
    }
    else
    {
        best_error = recalcbaud -baudrate;
    }

    best_error = best_error * 100 / baudrate;

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
        *ucpDdata = (unsigned char) xHWREG(ulBase + RBR);
        return (xtrue);
    }
    else                             // No
    {
        return (xfalse);
    }

}

xtBoolean UARTByteWriteNoBlocking(unsigned long ulBase, unsigned char ucData)
{
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
        *pBuf[i] = UARTByteRead(ulBase);
    }
}

#define INT_RDA                        IER_RBR_INT_EN

#define INT_THRE                       IER_THRE_INT_EN

#define INT_RX_LINE                    IER_RX_LINE_STAT_INT_EN

#define INT_ABEO                       IER_ABEO_INT_EN

#define INT_ABTO                       IER_ABTO_INT_EN

void UARTIntEnable(unsigned long ulBase, unsigned long ulIntFlags)
{

    xHWREG(ulBase + IER) |= ulIntFlags;
}

void UARTIntDisable(unsigned long ulBase, unsigned long ulIntFlags)
{

    xHWREG(ulBase + IER) &= ~ulIntFlags;
}

unsigned long UARTIntGet(unsigned long ulBase)
{
    return xHWREG(ulBase + IIR);
}

xtBoolean UARTIntCheck(unsigned long ulBase, unsigned long ulIntFlag)
{
    if(xHWREG(ulBase + IIR) & ulIntFlag)          // Interrupt occurs
    {
        return (xtrue);
    }
    else                                          // Interrupt not occurs.
    {
        return (xfalse);
    }
}

#define FIFO_CFG_FIFO_EN          BIT_32_0
#define FIFO_CFG_FIFO_DIS         BIT_32_16
#define FIFO_CFG_RX_FIFO_RESET    BIT_32_1
#define FIFO_CFG_TX_FIFO_RESET    BIT_32_2
#define FIFO_CFG_DMA_EN           BIT_32_3
#define FIFO_CFG_DMA_DIS          BIT_32_19
#define FIFO_CFG_RX_TRI_LVL_0     (BIT_32_23 | BIT_32_22)
#define FIFO_CFG_RX_TRI_LVL_1     (BIT_32_23 | BIT_32_6 )
#define FIFO_CFG_RX_TRI_LVL_2     (BIT_32_22 | BIT_32_7 )
#define FIFO_CFG_RX_TRI_LVL_3     (BIT_32_7  | BIT_32_6 )

void UARTFIFOCfg(unsigned long ulBase, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    ulTmpReg = xHWREG(ulBase + FCR);
    ulTmpReg &= ((~ulCfg) >> 8);
    ulTmpReg |= (ulCfg & 0xFF);
    xHWREG(ulBase + FCR) = ulTmpReg;
}

void UARTFlowCtlEnable(unsigned long ulBase)
{
    xHWREG(ulBase + TER) |= TER_TX_EN;
}

void UARTFlowCtlDisable(unsigned long ulBase)
{
    xHWREG(ulBase + TER) &= ~TER_TX_EN;
}


#define IRDA_INV_EN                    (BIT_32_1                                    )
#define IRDA_INV_DIS                   (BIT_32_17                                   )
#define IRDA_FIX_PULSE_DIS             (BIT_32_18                                   )
#define IRDA_FIX_PULSE_2               (BIT_32_2 | BIT_32_21 | BIT_32_20 | BIT_32_19)
#define IRDA_FIX_PULSE_4               (BIT_32_2 | BIT_32_21 | BIT_32_20 | BIT_32_3 )
#define IRDA_FIX_PULSE_8               (BIT_32_2 | BIT_32_21 | BIT_32_4  | BIT_32_19)
#define IRDA_FIX_PULSE_16              (BIT_32_2 | BIT_32_21 | BIT_32_4  | BIT_32_3 )
#define IRDA_FIX_PULSE_32              (BIT_32_2 | BIT_32_5  | BIT_32_20 | BIT_32_19)
#define IRDA_FIX_PULSE_64              (BIT_32_2 | BIT_32_5  | BIT_32_20 | BIT_32_3 )
#define IRDA_FIX_PULSE_128             (BIT_32_2 | BIT_32_5  | BIT_32_4  | BIT_32_19)
#define IRDA_FIX_PULSE_256             (BIT_32_2 | BIT_32_5  | BIT_32_4  | BIT_32_3 )

void UARTIrDACfg(unsigned long ulBase, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    ulTmpReg = xHWREG(ulBase + ICR);
    ulTmpReg &= ((~ulCfg) >> 8);
    ulTmpReg |= (ulCfg & 0xFF);
    xHWREG(ulBase + ICR) = ulTmpReg;
}

void UARTIrDAEnable(unsigned long ulBase)
{
    xHWREG(ulBase + ICR) |= ICR_IRDA_EN;
}

void UARTIrDADisable(unsigned long ulBase)
{
    xHWREG(ulBase + ICR) &= ~ICR_IRDA_EN;
}

#define UART_CFG_LEN_5_BIT             (BIT_32_17 | BIT_32_16)
#define UART_CFG_LEN_6_BIT             (BIT_32_17 | BIT_32_0 )
#define UART_CFG_LEN_7_BIT             (BIT_32_1  | BIT_32_16)
#define UART_CFG_LEN_8_BIT             (BIT_32_1  | BIT_32_0 )

#define UART_CFG_STOP_1_BIT            (BIT_32_18            )
#define UART_CFG_STOP_2_BIT            (BIT_32_2             )

#define UART_CFG_PARITY_NONE           (BIT_32_21 | BIT_32_20 | BIT_32_19)
#define UART_CFG_PARITY_ODD            (BIT_32_21 | BIT_32_20 | BIT_32_3 )
#define UART_CFG_PARITY_EVEN           (BIT_32_21 | BIT_32_4  | BIT_32_3 )
#define UART_CFG_PARITY_1              (BIT_32_5  | BIT_32_20 | BIT_32_3 )
#define UART_CFG_PARITY_0              (BIT_32_5  | BIT_32_4  | BIT_32_3 )

#define UART_CFG_BREAK_EN              (BIT_32_6                         )
#define UART_CFG_BREAK_DIS             (BIT_32_22                        )


void UARTCfg(unsigned long ulBase, unsigned long ulBaud, unsigned long ulCfg)
{
     unsigned long ulTmpReg = 0;

    // Configure UART Data length, Parity, stop bit, break.
    ulTmpReg = xHWREG(ulBase + LCR);
    ulTmpReg &= ((~ulCfg) >> 8);
    ulTmpReg |= (ulCfg & 0xFF);
    xHWREG(ulBase + LCR) = ulTmpReg;

    // Configure UART baud
    UartSetDivisors(ulBase, ulBaud);
}






