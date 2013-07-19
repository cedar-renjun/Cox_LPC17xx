//! RBR (DLAB =0) Receiver Buffer Register. Contains the next received
//! character to be read.
#define RBR                     ((unsigned long)0x00000000)

//! THR (DLAB =0) Transmit Holding Register. The next character to be
//! transmitted is written here.
#define THR                     ((unsigned long)0x00000000)

//! DLL (DLAB =1) Divisor Latch LSB. Least significant byte of the baud rate
//! divisor value. The full divisor is used to generate a baud rate from the
//! fractional rate divider.
#define DLL                     ((unsigned long)0x00000000)

//! DLM (DLAB =1) Divisor Latch MSB. Most significant byte of the baud rate
//! divisor value. The full divisor is used to generate a baud rate from the
//! fractional rate divider.
#define DLM                     ((unsigned long)0x00000004)

//! IER (DLAB =0) Interrupt Enable Register. Contains individual interrupt
//! enable bits for the 7 potential UART interrupts.
#define IER                     ((unsigned long)0x00000004)

//! IIR Interrupt ID Register. Identifies which interrupt(s) are pending.
#define IIR                     ((unsigned long)0x00000008)

//! FCR FIFO Control Register. Controls UART FIFO usage and modes.
#define FCR                     ((unsigned long)0x00000008)

//! LCR Line Control Register. Contains controls for frame formatting and break
//! generation.
#define LCR                     ((unsigned long)0x0000000C)

//! LSR Line Status Register. Contains flags for transmit and receive status,
//! including line errors.
#define LSR                     ((unsigned long)0x00000014)

//! SCR Scratch Pad Register. 8-bit temporary storage for software.
#define SCR                     ((unsigned long)0x0000001C)

//! ACR Auto-baud Control Register. Contains controls for the auto-baud feature.
#define ACR                     ((unsigned long)0x00000020)

//! ICR IrDA Control Register. Enables and configures the IrDA mode.
#define ICR                     ((unsigned long)0x00000024)

//! FDR Fractional Divider Register. Generates a clock input for the baud rate
//! divider.
#define FDR                     ((unsigned long)0x00000028)

//! TER Transmit Enable Register. Turns off UART transmitter for use with
//! software flow control.
#define TER                     ((unsigned long)0x00000030)

//! RBR {{
//! RBR }}

//! THR {{
//! THR }}

//! DLL {{
//! The UARTn Divisor Latch LSB Register, along with the UnDLM
//! register, determines the baud rate of the UARTn.
#define DLL_SB_M                BIT_MASK(32, 7, 0)
//! DLL }}

//! DLM {{
//! The UARTn Divisor Latch MSB Register, along with the U0DLL register,
//! determines the baud rate of the UARTn.
#define DLM_SB_M                BIT_MASK(32, 7, 0)
//! DLM }}

//! IER {{
//! Enables the Receive Data Available interrupt for UARTn. It also controls
//! the Character Receive Time-out interrupt
#define IER_RBR_INT_EN          BIT_32_0

//! Enables the THRE interrupt for UARTn. The status of this can be read
//! from UnLSR[5].
#define IER_THRE_INT_EN         BIT_32_1

//! Enables the UARTn RX line status interrupts. The status of this interrupt
//! can be read from UnLSR[4:1].
#define IER_RX_LINE_STAT_INT_EN BIT_32_2

//! Enables the end of auto-baud interrupt.
#define IER_ABEO_INT_EN         BIT_32_8

//! Enables the auto-baud time-out interrupt.
#define IER_ABTO_INT_EN         BIT_32_9

//! IER }}


//! IIR {{

//! Interrupt status. Note that UnIIR[0] is active low. The pending interrupt can
//! be determined by evaluating UnIIR[3:1].
//! 0 At least one interrupt is pending.
//! 1 No interrupt is pending.
#define IIR_INT_STAT            BIT_32_0

//! Interrupt identification. UnIER[3:1] identifies an interrupt corresponding to the
//! UARTn Rx or TX FIFO. All other combinations of UnIER[3:1] not listed below
//! are reserved (000,100,101,111).
//! 011 1 - Receive Line Status (RLS).
//! 010 2a - Receive Data Available (RDA).
//! 110 2b - Character Time-out Indicator (CTI).
//! 001 3 - THRE Interrupt
#define IIR_INT_ID_M            BIT_MASK(32, 3, 1)

//! Receive Line Status
#define IIR_INT_ID_RLS          (BIT_32_2 | BIT_32_1)

//! Receive Data Available
#define IIR_INT_ID_RDA          BIT_32_2

//! Character Time-Out Indicator
#define IIR_INT_ID_CTI          BIT_32_3 | BIT_32_2

//! THRE Interrupt 
#define IIR_INT_ID_THRE         BIT_32_1

//! Copies of UnFCR
#define IIR_FIFO_EN_M           BIT_MASK(32, 7, 6)

//! End of auto-baud interrupt.
#define IIR_ABEO_INT            BIT_32_8

//! Auto-baud time-out interrupt.
#define IIR_ABTO_INT            BIT_32_9

//! IIR }}

//! FCR {{

//! FIFO Enable
#define FCR_FIFO_EN            BIT_32_0

//! Reset RX FIFO
#define FCR_RX_FIFO_RESET      BIT_32_1

//! Reset TX FIFO
#define FCR_TX_FIFO_RESET      BIT_32_2

//! DMA Mode Select
#define FCR_DMA_MODE           BIT_32_3

//! RX Trigger Level.
#define FCR_RX_TRI_LEVEL_M     BIT_MASK(32, 7, 6)

//! Trigger level 0 (1 character)
#define FCR_RX_TRI_LEVEL_0     BIT_32_ALL_0

//! Trigger level 1 (4 character)
#define FCR_RX_TRI_LEVEL_1     BIT_32_6

//! Trigger level 2 (8 character)
#define FCR_RX_TRI_LEVEL_2     BIT_32_7

//! Trigger level 3 (14 character)
#define FCR_RX_TRI_LEVEL_3     (BIT_32_7 | BIT_32_6)

//! FCR }}

//! LCR {{

//! Character Length
#define LCR_WORD_LEN_M         BIT_MASK(32, 1, 0)
#define LCR_WORD_LEN_5_BIT     BIT_32_ALL_0
#define LCR_WORD_LEN_6_BIT     BIT_32_0
#define LCR_WORD_LEN_7_BIT     BIT_32_1
#define LCR_WORD_LEN_8_BIT     (BIT_32_1 | BIT_32_0)

//! Stop Bit
#define LCR_STOP_BIT           BIT_32_2

//! Parity Enable
#define LCR_PARITY_EN          BIT_32_3

//! Parity Select
#define LCR_PARITY_SEL_M       BIT_MASK(32, 5, 4)
#define LCR_PARITY_ODD         BIT_32_ALL_0
#define LCR_PARITY_EVEN        BIT_32_4
#define LCR_PARITY_1           BIT_32_5
#define LCR_PARITY_0           (BIT_32_5 | BIT_32_4)

//! Break Control
#define LCR_BREAK_CTL          BIT_32_6

//! Divisor Latch Acess Bit(DLAB)
#define LCR_DLAB               BIT_32_7

//! LCR }}


//! LSR {{

//! Receiver Data Ready
#define LSR_RDR                BIT_32_0

//! Overrun Error
#define LSR_OE                 BIT_32_1

//! Parity Error
#define LSR_PE                 BIT_32_2

//! Framing Error
#define LSR_FE                 BIT_32_3

//! Break Interrupt
#define LSR_BI                 BIT_32_4

//! Transmitter Holding Register Empty
#define LSR_THRE               BIT_32_5

//! Transmitter Empty
#define LSR_TEMT               BIT_32_6

//! Error in RX FIFO
#define LSR_RXFE               BIT_32_7

//! LSR }}

//! SCR {{

//! UART Scratch Pad Mask
#define SCR_PAD_M              BIT_MASK(32, 7, 0)

//! SCR }}

//! ACR {{

//! Start Auto-Baud
#define ACR_START               BIT_32_0

//! Auto-Baud Mode Select bit.
#define ACR_MODE                BIT_32_1

//! Auto ReStart
#define ACR_AUTO_RESTART        BIT_32_2

//! End of auto-baud interrupt clear bit.
#define ACR_ABEO_INT_CLR        BIT_32_8

//! Auto-baud time-out interrupt clear bit.
#define ACR_ABTO_INT_CLR        BIT_32_9

//! ACR }}


//! ICR {{

//! IrDA Enable
#define ICR_IRDA_EN             BIT_32_0

//! IrDA Invert
#define ICR_IRDA_INV            BIT_32_1

//! Enable IrDA Fixed Pulse width mode.
#define ICR_FIX_PULSE_EN        BIT_32_2

//! Pulse Divider.
#define ICR_PULSE_DIV_M         BIT_MASK(32, 5, 3)
#define ICR_PULSE_DIV_S         3

//! ICR }}

//! FDR {{

//! Baud-rate generation pre-scaler divisor value.
#define FDR_DIVADDVAL_M         BIT_MASK(32, 3, 0)
#define FDR_DIVADDVAL_S         0

//! Baud-rate pre-scaler multiplier value.
#define FDR_MULVAL_M            BIT_MASK(32, 7, 4)
#define FDR_MULVAL_S            4

//! FDR }}

//! TER {{

//! Transmitter Enable
#define TER_TX_EN               BIT_32_7

//! TER }}

