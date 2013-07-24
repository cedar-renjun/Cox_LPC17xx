#include "xhw_types.h"
#include "xhw_ints.h"
#include "xhw_memmap.h"
#include "xdebug.h"

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




//! Enable FIFO.
#define FIFO_CFG_FIFO_EN               BIT_32_0

//! Disable FIFO.
#define FIFO_CFG_FIFO_DIS              BIT_32_16

//! Flush Rx FIFO.
#define FIFO_CFG_RX_FIFO_RESET         BIT_32_1

//! Flush Tx FIFO.
#define FIFO_CFG_TX_FIFO_RESET         BIT_32_2

//! Enable DMA Mode.
#define FIFO_CFG_DMA_EN                BIT_32_3

//! Disable DMA Mode.
#define FIFO_CFG_DMA_DIS               BIT_32_19

//! Trigger level 0 (1 character)
#define FIFO_CFG_RX_TRI_LVL_0          (BIT_32_23 | BIT_32_22)

//! Trigger level 1 (4 characters)     
#define FIFO_CFG_RX_TRI_LVL_1          (BIT_32_23 | BIT_32_6)

//! Trigger level 2 (8 characters)     
#define FIFO_CFG_RX_TRI_LVL_2          (BIT_32_22 | BIT_32_7)

//! Trigger level 3 (14 characters)
#define FIFO_CFG_RX_TRI_LVL_3          (BIT_32_7  | BIT_32_6)



//! The UART receiver FIFO is not empty.
#define RX_FIFO_NOT_EMPTY              BIT_32_0

//! Overrun error.
#define OVERRUN_ERR                    BIT_32_1

//! Parity error.
#define PARITY_ERR                     BIT_32_2

//! Framing error.
#define FRAMING_ERR                    BIT_32_3

//! Break interrupt.
#define BREAK_INT                      BIT_32_4

//! Transmitter holding register empty.
//! \note THRE is set immediately upon detection of an empty UART THR and
//!       is cleared on a THR write.
#define TX_FIFO_EMPTY                  BIT_32_5

//! Transmitter empty.
//! \note TEMT is set when both THR and TSR are empty; TEMT is cleared when
//!       either the TSR or the THR contain valid data.
#define TX_EMPTY                       BIT_32_6

//! Error in Rx FIFO
#define RX_FIFO_ERR                    BIT_32_7




//! Invert input serial.
#define IRDA_INV_EN                    BIT_32_1

//! Not Invert input serial.
#define IRDA_INV_DIS                   BIT_32_17

//! Disable fixed pulse width mode.
#define IRDA_FIX_PULSE_DIS             BIT_32_18

//! Fixed pulse width: 2*Tpclk.
#define IRDA_FIX_PULSE_2               (BIT_32_2 | BIT_32_21 | BIT_32_20 | BIT_32_19)

//! Fixed pulse width: 4*Tpclk.
#define IRDA_FIX_PULSE_4               (BIT_32_2 | BIT_32_21 | BIT_32_20 | BIT_32_3)

//! Fixed pulse width: 8*Tpclk.
#define IRDA_FIX_PULSE_8               (BIT_32_2 | BIT_32_21 | BIT_32_4  | BIT_32_19)

//! Fixed pulse width: 16*Tpclk.
#define IRDA_FIX_PULSE_16              (BIT_32_2 | BIT_32_21 | BIT_32_4  | BIT_32_3)

//! Fixed pulse width: 32*Tpclk.
#define IRDA_FIX_PULSE_32              (BIT_32_2 | BIT_32_5  | BIT_32_20 | BIT_32_19)

//! Fixed pulse width: 64*Tpclk.
#define IRDA_FIX_PULSE_64              (BIT_32_2 | BIT_32_5  | BIT_32_20 | BIT_32_3)

//! Fixed pulse width: 128*Tpclk.
#define IRDA_FIX_PULSE_128             (BIT_32_2 | BIT_32_5  | BIT_32_4  | BIT_32_19)

//! Fixed pulse width: 256*Tpclk.
#define IRDA_FIX_PULSE_256             (BIT_32_2 | BIT_32_5  | BIT_32_4  | BIT_32_3)


//! UART Data Length 5-bit.
#define UART_CFG_LEN_5_BIT             (BIT_32_17 | BIT_32_16)

//! UART Data Length 6-bit.
#define UART_CFG_LEN_6_BIT             (BIT_32_17 | BIT_32_0)

//! UART Data Length 7-bit.
#define UART_CFG_LEN_7_BIT             (BIT_32_1  | BIT_32_16)

//! UART Data Length 8-bit.
#define UART_CFG_LEN_8_BIT             (BIT_32_1  | BIT_32_0)

//! UART Stop 1-bit.
#define UART_CFG_STOP_1_BIT            BIT_32_18

//! UART Stop 2-bit.
#define UART_CFG_STOP_2_BIT            BIT_32_2

//! UART None Parity.
#define UART_CFG_PARITY_NONE           (BIT_32_21 | BIT_32_20 | BIT_32_19)

//! UART odd parity.
#define UART_CFG_PARITY_ODD            (BIT_32_21 | BIT_32_20 | BIT_32_3)

//! UART even parity.
#define UART_CFG_PARITY_EVEN           (BIT_32_21 | BIT_32_4  | BIT_32_3)

//! UART forced 1 stick parity.
#define UART_CFG_PARITY_1              (BIT_32_5  | BIT_32_20 | BIT_32_3)

//! UART forced 0 stick parity.
#define UART_CFG_PARITY_0              (BIT_32_5  | BIT_32_4  | BIT_32_3)

//! Enable break transmission.
#define UART_CFG_BREAK_EN              BIT_32_6

//! Disable break transmission.
#define UART_CFG_BREAK_DIS             BIT_32_22



//! Enable Modem loopback mode.
#define LOOPBACK_MODE_EN               BIT_32_4

//! Disable Modem loopback mode.       
#define LOOPBACK_MODE_DIS              BIT_32_20

//! Enable Auto-RTS Flow control.
#define AUTO_RTS_EN                    BIT_32_6

//! Disable Auto-RTS Flow control.
#define AUTO_RTS_DIS                   BIT_32_22

//! Enable Auto-CTS Flow control.
#define AUTO_CTS_EN                    BIT_32_7

//! Disable Auto-CTS Flow control.
#define AUTO_CTS_DIS                   BIT_32_23



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
#define RS485_AUTO_DIR_DTR             (BIT_32_4 | BIT_32_3)

//! The direction control pin will be driven to logic '1' when the
//! transmitter has data to be sent.It will be driven to logic '0'
//! after the last bit of data has been transmitted
#define RS485_AUTO_DIR_INV_EN          BIT_32_5

//! The direction control pin will be driven to logic '0' when the
//! transmitter has data to be sent.It will be driven to logic '1'
//! after the last bit of data has been transmitted.
#define RS485_AUTO_DIR_INV_DIS         BIT_32_21

//! @}

//! State change detected on modem input CTS.
#define MODEM_DELTA_CTS                BIT_32_0

//! State change detected on modem input DSR.
#define MODEM_DELTA_DSR                BIT_32_1

//! State change detected on modem input RI.
#define MODEM_TRIL_EDGE_RI             BIT_32_2

//! State change detected on modem input DCD.
#define MODEM_DELTA_DCD                BIT_32_3

//! Clear To Send State.
#define MODEM_CTS                      BIT_32_4

//! Data Set Ready State.
#define MODEM_DSR                      BIT_32_5

//! Ring indicator state.
#define MODEM_RI                       BIT_32_6

//! Data carrier detect state.
#define MODEM_DCD                      BIT_32_7


extern void UARTCfg(unsigned long ulBase, unsigned long ulBaud, unsigned long ulCfg);

//! \todo Still need to test.
extern unsigned char UARTByteRead(unsigned long ulBase);
extern void UARTByteWrite(unsigned long ulBase, unsigned char ucData);
extern xtBoolean UARTByteReadNoBlocking(unsigned long ulBase, unsigned char * ucpData);
extern xtBoolean UARTByteWriteNoBlocking(unsigned long ulBase, unsigned char ucData);
extern void UARTStrSend(unsigned long ulBase, unsigned char * pStr);
extern void UARTBufWrite(unsigned long ulBase, unsigned char * pBuf, unsigned long ulLen);
extern void UARTBufRead(unsigned long ulBase, unsigned char * pBuf, unsigned long ulLen);

extern void UARTIntEnable(unsigned long ulBase, unsigned long ulIntFlags);
extern void UARTIntDisable(unsigned long ulBase, unsigned long ulIntFlags);

//! \todo Still need to test.
extern unsigned long UARTIntGet(unsigned long ulBase);
extern xtBoolean UARTIntCheck(unsigned long ulBase, unsigned long ulIntFlags);

extern void UARTFIFOCfg(unsigned long ulBase, unsigned long ulCfg);
extern void UARTTransStop(unsigned long ulBase);
extern void UARTTransStart(unsigned long ulBase);

// manual check
extern unsigned long UARTStatGet(unsigned long ulBase);

// manual check
extern xtBoolean UARTStatCheck(unsigned long ulBase, unsigned long ulFlags);
extern void UARTIrDACfg(unsigned long ulBase, unsigned long ulCfg);
extern void UARTIrDAEnable(unsigned long ulBase);
extern void UARTIrDADisable(unsigned long ulBase);
extern void UARTModemCfg(unsigned long ulBase, unsigned long ulCfg);
extern void UARTRS485Cfg(unsigned long ulBase, unsigned long ulCfg);
extern void UARTRS485AddrSet(unsigned long ulBase, unsigned long ulVal);
extern void UARTRS485DlyTimeSet(unsigned long ulBase, unsigned long ulVal);
extern unsigned long UARTModemStatGet(unsigned long ulBase);
extern xtBoolean UARTModemStatCheck(unsigned long ulBase, unsigned long ulFlags);
