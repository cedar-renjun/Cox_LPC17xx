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

//! Enables the Receive Data Available interrupt.
#define INT_RDA                        BIT_32_0

//! Enables the THRE interrupt.
#define INT_THRE                       BIT_32_1

//! Enables Rx Line status interrupt.
#define INT_RX_LINE                    BIT_32_2

//! Enables the modem interrupt.
#define INT_MODEM                      BIT_32_3

//! Enable the CTS interrupt.
#define INT_CTS                        BIT_32_7

//! Enables the end of auto-baud interrupt.
#define INT_ABEO                       BIT_32_8

//! Enables the end of auto-baud time-out interrupt.
#define INT_ABTO                       BIT_32_9

void UARTIntEnable(unsigned long ulBase, unsigned long ulIntFlags)
{
    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    xASSERT( (ulCfg & ~(
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

    xASSERT( (ulCfg & ~(
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

xtBoolean UARTIntCheck(unsigned long ulBase, unsigned long ulIntFlag)
{
    // Interrupt Idenetification register.
    unsigned long ulTmpReg = xHWREG(ulBase + IIR) & IIR_INT_ID_M;

    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    xASSERT( (ulCfg & ~(
                            IIR_INT_ID_RLS   |
                            IIR_INT_ID_RDA   |
                            IIR_INT_ID_CTI   |
                            IIR_INT_ID_THRE  |
                            IIR_INT_ID_MODEM
                    )
             ) == 0);

    if(ulTmpReg == ulIntFlag)
    {
        return (xtrue);
    }
    else
    {
        return (xfalse);
    }
}

//! Enable FIFO.
#define FIFO_CFG_FIFO_EN          BIT_32_0

//! Disable FIFO.
#define FIFO_CFG_FIFO_DIS         BIT_32_16

//! Flush Rx FIFO.
#define FIFO_CFG_RX_FIFO_RESET    BIT_32_1

//! Flush Tx FIFO.
#define FIFO_CFG_TX_FIFO_RESET    BIT_32_2

//! Enable DMA Mode.
#define FIFO_CFG_DMA_EN           BIT_32_3

//! Disable DMA Mode.
#define FIFO_CFG_DMA_DIS          BIT_32_19

//! Trigger level 0 (1 character)
#define FIFO_CFG_RX_TRI_LVL_0     (BIT_32_23 | BIT_32_22)

//! Trigger level 1 (4 characters)
#define FIFO_CFG_RX_TRI_LVL_1     (BIT_32_23 | BIT_32_6 )

//! Trigger level 2 (8 characters)
#define FIFO_CFG_RX_TRI_LVL_2     (BIT_32_22 | BIT_32_7 )

//! Trigger level 3 (14 characters)
#define FIFO_CFG_RX_TRI_LVL_3     (BIT_32_7  | BIT_32_6 )

void UARTFIFOCfg(unsigned long ulBase, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    // Configure FIFO
    ulTmpReg = xHWREG(ulBase + FCR);
    ulTmpReg &= ((~ulCfg) >> 8);
    ulTmpReg |= (ulCfg & 0xFF);
    xHWREG(ulBase + FCR) = ulTmpReg;
}

void UARTFlowCtlEnable(unsigned long ulBase)
{
    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    xHWREG(ulBase + TER) |= TER_TX_EN;
}

void UARTFlowCtlDisable(unsigned long ulBase)
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

//! The UART receiver FIFO is not empty.
#define RX_FIFO_NOT_EMPTY       BIT_32_0

//! Overrun error.
#define OVERRUN_ERR             BIT_32_1

//! Parity error.
#define PARITY_ERR              BIT_32_2

//! Framing error.
#define FRAMING_ERR             BIT_32_3

//! Break interrupt.
#define BREAK_INT               BIT_32_4

//! Transmitter holding register empty.
//! \note THRE is set immediately upon detection of an empty UART THR and
//!       is cleared on a THR write.
#define TX_FIFO_EMPTY           BIT_32_5

//! Transmitter empty.
//! \note TEMT is set when both THR and TSR are empty; TEMT is cleared when
//!       either the TSR or the THR contain valid data.
#define TX_EMPTY                BIT_32_6

//! Error in Rx FIFO
#define RX_FIFO_ERR             BIT_32_7

xtBoolean UARTStatCheck(unsigned long ulBase, unsigned long ulFlags)
{
    unsigned long ulTmpReg = 0;

    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) || (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) || (ulBase == UART3_BASE) );

    ulTmpReg = xHWREG(ulBase + LSR);
    if(ulTmpReg & ulFlags)
    {
        return (xtrue);
    }
    else
    {
        return (xfalse);
    }
}

//! Invert input serial.
#define IRDA_INV_EN                    (BIT_32_1                                    )

//! Not Invert input serial.
#define IRDA_INV_DIS                   (BIT_32_17                                   )

//! Disable fixed pulse width mode.
#define IRDA_FIX_PULSE_DIS             (BIT_32_18                                   )

//! Fixed pulse width: 2*Tpclk.
#define IRDA_FIX_PULSE_2               (BIT_32_2 | BIT_32_21 | BIT_32_20 | BIT_32_19)

//! Fixed pulse width: 4*Tpclk.
#define IRDA_FIX_PULSE_4               (BIT_32_2 | BIT_32_21 | BIT_32_20 | BIT_32_3 )

//! Fixed pulse width: 8*Tpclk.
#define IRDA_FIX_PULSE_8               (BIT_32_2 | BIT_32_21 | BIT_32_4  | BIT_32_19)

//! Fixed pulse width: 16*Tpclk.
#define IRDA_FIX_PULSE_16              (BIT_32_2 | BIT_32_21 | BIT_32_4  | BIT_32_3 )

//! Fixed pulse width: 32*Tpclk.
#define IRDA_FIX_PULSE_32              (BIT_32_2 | BIT_32_5  | BIT_32_20 | BIT_32_19)

//! Fixed pulse width: 64*Tpclk.
#define IRDA_FIX_PULSE_64              (BIT_32_2 | BIT_32_5  | BIT_32_20 | BIT_32_3 )

//! Fixed pulse width: 128*Tpclk.
#define IRDA_FIX_PULSE_128             (BIT_32_2 | BIT_32_5  | BIT_32_4  | BIT_32_19)

//! Fixed pulse width: 256*Tpclk.
#define IRDA_FIX_PULSE_256             (BIT_32_2 | BIT_32_5  | BIT_32_4  | BIT_32_3 )

//! \note This function is only suit for UART0/2/3.
void UARTIrDACfg(unsigned long ulBase, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    // Check input parameters.
    xASSERT((ulBase == UART0_BASE) ||
            (ulBase == UART1_BASE) ||
            (ulBase == UART2_BASE) |);

    // Configure IrDA Invert, Fixed Pulse width.
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

//! UART Data Length 5-bit.
#define UART_CFG_LEN_5_BIT             (BIT_32_17 | BIT_32_16)

//! UART Data Length 6-bit.
#define UART_CFG_LEN_6_BIT             (BIT_32_17 | BIT_32_0 )

//! UART Data Length 7-bit.
#define UART_CFG_LEN_7_BIT             (BIT_32_1  | BIT_32_16)

//! UART Data Length 8-bit.
#define UART_CFG_LEN_8_BIT             (BIT_32_1  | BIT_32_0 )

//! UART Stop 1-bit.
#define UART_CFG_STOP_1_BIT            (BIT_32_18            )

//! UART Stop 2-bit.
#define UART_CFG_STOP_2_BIT            (BIT_32_2             )

//! UART None Parity.
#define UART_CFG_PARITY_NONE           (BIT_32_21 | BIT_32_20 | BIT_32_19)

//! UART odd parity.
#define UART_CFG_PARITY_ODD            (BIT_32_21 | BIT_32_20 | BIT_32_3 )

//! UART even parity.
#define UART_CFG_PARITY_EVEN           (BIT_32_21 | BIT_32_4  | BIT_32_3 )

//! UART forced 1 stick parity.
#define UART_CFG_PARITY_1              (BIT_32_5  | BIT_32_20 | BIT_32_3 )

//! UART forced 0 stick parity.
#define UART_CFG_PARITY_0              (BIT_32_5  | BIT_32_4  | BIT_32_3 )

//! Enable break transmission.
#define UART_CFG_BREAK_EN              (BIT_32_6                         )

//! Disable break transmission.
#define UART_CFG_BREAK_DIS             (BIT_32_22                        )


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
    ulTmpReg &= ((~ulCfg) >> 8);
    ulTmpReg |= (ulCfg & 0xFF);
    xHWREG(ulBase + LCR) = ulTmpReg;

    // Configure UART baud
    UartSetDivisors(ulBase, ulBaud);
}

//! Enable Modem loopback mode.
#define LOOPBACK_MODE_EN        BIT_32_4

//! Disable Modem loopback mode.
#define LOOPBACK_MODE_DIS       BIT_32_20

//! Enable Auto-RTS Flow control.
#define AUTO_RTS_EN             BIT_32_6

//! Disable Auto-RTS Flow control.
#define AUTO_RTS_DIS            BIT_32_22

//! Enable Auto-CTS Flow control.
#define AUTO_CTS_EN             BIT_32_7

//! Disable Auto-CTS Flow control.
#define AUTO_CTS_DIS            BIT_32_23

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
    ulTmpReg &= ((~ulCfg) >> 8);
    ulTmpReg |= (ulCfg & 0xFF);
    xHWREG(ulBase + MCR) = ulTmpReg;

}

//! \addtogroup RS485Cfg Parameters of RS485 Configure functions.
//! @{

//! \internal
//! Parameters mask.
#define RS485_PARA_M                   ((unsigned long)0xFFFFC0C0)

//! RS-485/EIA-485 Normal Multidrop Mode (NMM) is disabled
#define RS485_NMM_DIS                  BIT_32_16

//! RS-485/EIA-485 Normal Multidrop Mode (NMM) is enabled.In this mode,
//! an address is detected when a received byte causes the UART to set
//! the parity error and generate an interrupt
#define RS485_NMM_EN                   BIT_32_0

//! Enable receiver.
#define RS485_RX_EN                    BIT_32_17

//! Disable receiver.
#define RS485_RX_DIS                   BIT_32_1

//! Enable Auto Address detect.
#define RS485_AUTO_ADDR_EN             BIT_32_2

//! Disable Auto Address detect.
#define RS485_AUTO_ADDR_DIS            BIT_32_18

//! Disable Auto Direction Control.
#define RS485_AUTO_DIR_DIS             BIT_32_20

//! Enable Auto Direction Control, use pin RTS as direction control.
#define RS485_AUTO_DIR_RTS             (BIT_32_4 | BIT_32_19)

//! Enable Auto Direction Control, use pin DTR as direction control.
#define RS485_AUTO_DIR_DTR             (BIT_32_4 | BIT_32_3 )

//! The direction control pin will be driven to logic '1' when the
//! transmitter has data to be sent.It will be driven to logic '0'
//! after the last bit of data has been transmitted
#define RS485_AUTO_DIR_INV_EN          BIT_32_5

//! The direction control pin will be driven to logic '0' when the
//! transmitter has data to be sent.It will be driven to logic '1'
//! after the last bit of data has been transmitted.
#define RS485_AUTO_DIR_INV_DIS         BIT_32_21

//! @}

void UARTRS485Cfg(unsigned long ulBase, unsigned long ulCfg)
{
    unsigned long ulTmpReg = 0;

    // Check input parameters.
    xASSERT(ulBase == UART1_BASE);
    xASSERT( (ulCfg & RS485_PARA_M) == 0 );

    // Configure RS485
    ulTmpReg = xHWREG(ulBase + RS485CTRL);
    ulTmpReg &= ((~ulCfg) >> 8);
    ulTmpReg |= (ulCfg & 0xFF);
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

//! State change detected on modem input CTS.
#define MODEM_DELTA_CTS         BIT_32_0

//! State change detected on modem input DSR.
#define MODEM_DELTA_DSR         BIT_32_1

//! State change detected on modem input RI.
#define MODEM_TRIL_EDGE_RI      BIT_32_2

//! State change detected on modem input DCD.
#define MODEM_DELTA_DCD         BIT_32_3

//! Clear To Send State.
#define MODEM_CTS               BIT_32_4

//! Data Set Ready State.
#define MODEM_DSR               BIT_32_5

//! Ring indicator state.
#define MODEM_RI                BIT_32_6

//! Data carrier detect state.
#define MODEM_DCD               BIT_32_7

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
