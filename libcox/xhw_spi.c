







//! \addtogroup SPI_Register_Offset SPI Register Offsets.
//! @{

//! SPI Control Register.
//! This register controls the operation of the SPI.
#define S0SPCR                  0x00000000

//! SPI Status Register.
//! This register shows the status of the SPI.
#define S0SPSR                  0x00000004

//! SPI Data Register.
//! This bi-directional register provides the transmit and receive data for the SPI.
//! Transmit data is provided to the SPI0 by writing to this register.
//! Data received by the SPI0 can be read from this register.
#define S0SPDR                  0x00000008

//! SPI Clock Counter Register.
//! This register controls the frequency of a master¡¯s SCK0.
#define S0SPCCR                 0x0000000C

//! SPI Interrupt Flag.
//! This register contains the interrupt flag for the SPI interface.
#define S0SPINT                 0x0000001C

//! @}





//! S0SPCR {{

//! SPI BIt control.
//! When set, SPI controller send 8-->16 bit data according to other register bits.
//! When Clear, SPI data length is fixed to 8-bit.
#define S0SPCR_BIT_EN           BIT_32_2

//! Clock Phase control
#define S0SPCR_CPHA             BIT_32_3

//! Clock Polarity control
#define S0SPCR_CPOL             BIT_32_4

//! Master Mode Select
#define S0SPCR_MSTR             BIT_32_5

//! LSB First controls which direction each byte is shifted when transferred.
#define S0SPCR_LSBF             BIT_32_6

//! Enable serial peripheral interrupt
#define S0SPCR_SPIE             BIT_32_7

//! SPI data length mask
#define S0SPCR_BITS_M           BIT_MASK(32, 11, 8)

//! SPI 8-bit per transfer
#define S0SPCR_BITS_8           (BIT_32_11)

//! SPI 9-bit per transfer
#define S0SPCR_BITS_9           (BIT_32_11 | BIT_32_8)

//! SPI 10-bit per transfer
#define S0SPCR_BITS_10          (BIT_32_11 | BIT_32_9)

//! SPI 11-bit per transfer)
#define S0SPCR_BITS_11          (BIT_32_11 | BIT_32_9 | BIT_32_8)

//! SPI 12-bit per transfer)
#define S0SPCR_BITS_12          (BIT_32_11 | BIT_32_10)

//! SPI 13-bit per transfer)
#define S0SPCR_BITS_13          (BIT_32_11 | BIT_32_10 | BIT_32_8)

//! SPI 14-bit per transfer)
#define S0SPCR_BITS_14          (BIT_32_11 | BIT_32_10 | BIT_32_9)

//! SPI 15-bit per transfer)
#define S0SPCR_BITS_15          (BIT_32_11 | BIT_32_10 | BIT_32_9 | BIT_32_8)

//! SPI 16-bit per transfer)
#define S0SPCR_BITS_16          (BIT_32_ALL_0)

//! S0SPCR }}


//! S0SPSR {{

//! Slave abort
#define S0SPSR_ABRT             BIT_32_3

//! Mode Fault
#define S0SPSR_MODF             BIT_32_4

//! Read overrun
#define S0SPSR_ROVR             BIT_32_5

//! Write collision
#define S0SPSR_WCOL             BIT_32_6

//! Transfer complete
#define S0SPSR_SPIF             BIT_32_7

//! S0SPSR }}


//! S0SPDR {{

//! SPI data mask
#define S0SPDR_DATA_M           BIT_MASK(32, 15, 0)

//! S0SPDR }}

//! S0SPCCR {{

//! SPI0 Clock counter setting
#define S0SPCCR_CNT_M           BIT_MASK(32, 7, 0)

//! S0SPCCR }}


//! S0SPINT {{

//! SPI interrupt flag.
#define S0SPINT                 BIT_32_0

//! S0SPINT }}

//


