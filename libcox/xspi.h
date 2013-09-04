//*****************************************************************************
//
//! \file xspi.h
//! \brief Prototypes for the SPI Driver.
//! \version V2.2.1.0
//! \date 11/14/2011
//! \author CooCox
//! \copyright
//!
//! Copyright (c)  2011, CooCox
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!     * Redistributions of source code must retain the above copyright
//! notice, this list of conditions and the following disclaimer.
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution.
//!     * Neither the name of the <ORGANIZATION> nor the names of its
//! contributors may be used to endorse or promote products derived
//! from this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//! THE POSPIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#ifndef __xSPI_H__
#define __xSPI_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup CoX_Peripheral_Lib
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup SPI
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSPI
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSPI_Ints xSPI Interrupts
//! \brief      Values that show xSPI interrupts
//!
//! \section    xSPI_Ints_Section 1. Where to use this group
//!             Values that can be passed to SPIIntEnable, SPIIntDisable, and
//!             SPIIntClear as the ulIntFlags parameter, and returned from
//!             SPIIntStatus.
//!
//! \section    xSPI_Ints_CoX 2. CoX Port Details
//! +-------------------------+-----------------+------------------------+
//! |   xSPI Interrupts       |       CoX       |          LPC17xx       |
//! |-------------------------|-----------------|------------------------|
//! |   xSPI_INT_EOT          |    Mandatory    |            Y           |
//! |-------------------------|-----------------|------------------------|
//! |   xSPI_INT_TX           |  Non-Mandatory  |            N           |
//! |-------------------------|-----------------|------------------------|
//! |   xSPI_INT_RX           |  Non-Mandatory  |            N           |
//! |-------------------------|-----------------|------------------------|
//! |   xSPI_INT_ERROR        |  Non-Mandatory  |            N           |
//! +-------------------------+-----------------+------------------------+
//! @{
//
//*****************************************************************************

//
//! End of transfer
//
#define xSPI_INT_EOT

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSPI_Ints_Event xSPI Interrupt Event
//! \brief      Values that show xSPI interrupt events.
//!
//! \section    xSPI_Ints_Event_Section 1. Where to use this group
//!             Values that can be passed to SPIIntEnable, SPIIntDisable,
//!             and SPIIntClear as the ulIntFlags parameter, and returned from
//!             SPIIntStatus.
//!
//! \section    xSPI_Ints_Event_CoX     2. CoX Port Details
//! +-------------------------+----------------+------------------------+
//! |   xSPI Interrupts       |       CoX      |          LPC17xx       |
//! |-------------------------|----------------|------------------------|
//! |   xSPI_EVENT_EOT        |    Mandatory   |            Y           |
//! |-------------------------|----------------|------------------------|
//! |   xSPI_EVENT_TX         |  Non-Mandatory |            N           |
//! |-------------------------|----------------|------------------------|
//! |   xSPI_EVENT_RX         |  Non-Mandatory |            N           |
//! |-------------------------|----------------|------------------------|
//! |   xSPI_EVENT_ERROR      |  Non-Mandatory |            N           |
//! +-------------------------+----------------+------------------------+
//! @{
//
//*****************************************************************************

//
//! End of transfer
//
#define xSPI_EVENT_EOT

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSPI_Config xSPI Configure
//! \brief      Values that show xSPI Configure
//!
//! \section    xSPI_Config_Section 1. Where to use this group
//!             Values that can be passed to xSPIConfig() as the ulConfig
//!             parameter.
//!
//! \section    xSPI_Config_CoX     2. CoX Port Details
//! +---------------------------+-----------------+---------------------+
//! |  xSPI Configs             |       CoX       |        LPC17xx      |
//! |---------------------------|-----------------|---------------------|
//! |  xSPI_MOTO_FORMAT_MODE_0  |    Mandatory    |          Y          |
//! |---------------------------|-----------------|---------------------|
//! |  xSPI_MOTO_FORMAT_MODE_1  |    Mandatory    |          Y          |
//! |---------------------------|-----------------|---------------------|
//! |  xSPI_MOTO_FORMAT_MODE_2  |    Mandatory    |          Y          |
//! |---------------------------|-----------------|---------------------|
//! |  xSPI_MOTO_FORMAT_MODE_3  |    Mandatory    |          Y          |
//! |---------------------------|-----------------|---------------------|
//! |  xSPI_TI_FORMAT_MODE      |  Non-Mandatory  |          N          |
//! |---------------------------|-----------------|---------------------|
//! |  xSPI_NMW_FORMAT_MODE     |  Non-Mandatory  |          N          |
//! |---------------------------|-----------------|---------------------|
//! |  xSPI_MODE_MASTER         |    Mandatory    |          Y          |
//! |---------------------------|-----------------|---------------------|
//! |  xSPI_MODE_SLAVE          |    Mandatory    |          Y          |
//! |---------------------------|-----------------|---------------------|
//! |  xSPI_MSB_FIRST           |    Mandatory    |          Y          |
//! |---------------------------|-----------------|---------------------|
//! |  xSPI_LSB_FIRST           |  Non-Mandatory  |          Y          |
//! |---------------------------|-----------------|---------------------|
//! |  xSPI_DATA_WIDTHn         |  Non-Mandatory  |  xSPI_DATA_WIDTH8   |
//! +---------------------------+-----------------+---------------------+
//! @{
//
//*****************************************************************************

//
//! Moto Format, polarity 0, phase 0
//
#define xSPI_MOTO_FORMAT_MODE_0

//
//! Moto Format, polarity 0, phase 1
//
#define xSPI_MOTO_FORMAT_MODE_1

//
//! Moto Format, polarity 1, phase 0
//
#define xSPI_MOTO_FORMAT_MODE_2

//
//! Moto Format, polarity 1, phase 1
//
#define xSPI_MOTO_FORMAT_MODE_3

//
//! SPI Master Mode.
//
#define xSPI_MODE_MASTER

//
//! SPI Slave Mode.
//
#define xSPI_MODE_SLAVE

//
//! MSB send first
//
#define xSPI_MSB_FIRST

//
//! LSB send first
//
#define xSPI_LSB_FIRST

//
//! 8-bit data length mode.
//
#define xSPI_DATA_WIDTH8

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSPI_DMA xSPI DMA
//! \brief      Values that show xSPI DMA
//!
//! \section    xSPI_DMA_Section 1. Where to use this group
//!             Values that can be passed to
//!             \ref xSPIDMAEnable() and \ref xSPIDMADisable()
//!             as the ulDmaMode parameter.
//!
//! \section    xSPI_DMA_CoX     2. CoX Port Details
//! +----------------------+----------------+------------------------+
//! |    xSPI DMA          |       CoX      |          LPC17xx       |
//! |----------------------|----------------|------------------------|
//! |    xSPI_DMA_TX       |    Mandatory   |            Y           |
//! |----------------------|----------------|------------------------|
//! |    SPI_DMA_RX        |    Mandatory   |            Y           |
//! |----------------------|----------------|------------------------|
//! |    SPI_DMA_BOTH      |  Non-Mandatory |            Y           |
//! +----------------------+----------------+------------------------+
//! @{
//
//*****************************************************************************

//
//! Enable DMA for transmit
//
#define xSPI_DMA_TX

//
//! Enable DMA for receive
//
#define xSPI_DMA_RX

//
//! Enable DMA for transfer and receive
//
#define xSPI_DMA_BOTH

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSPI_SlaveSelMode xSPI Slave Select Mode
//! \brief      Values show xSPI Slave Select Mode
//!
//! \section    xSPI_SlaveSelMode_Section 1. Where to use this group
//!             Values that can be passed to xSPISSSet() as the ulSSMode parameter.
//!
//! \section    xSPI_SlaveSelMode_CoX 2. CoX Port Details
//! +--------------------------+---------------+-----------------------+
//! |  xSPI Slave Select Mode  |      CoX      |         LPC17xx       |
//! |--------------------------|---------------|-----------------------|
//! |  xSPI_SS_HARDWARE        |   Mandatory   |           Y           |
//! |--------------------------|---------------|-----------------------|
//! |  xSPI_SS_SOFTWARE        |   Mandatory   |           Y           |
//! +--------------------------+---------------+-----------------------+
//! @{
//
//*****************************************************************************

#define xSPI_SS_HARDWARE
#define xSPI_SS_SOFTWARE

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSPI_SlaveSel xSPI Slave Select
//! \brief      Values that show xSPI Slave Select
//!
//! \section    xSPI_SlaveSel_Section 1. Where to use this group
//!             Values that can be passed to xSPISSSet() as the ulSlaveSel parameter.
//!
//! \section    xSPI_SlaveSel_CoX     2. CoX Port Details
//! +------------------------+----------------+------------------------+
//! |   xSPI Slave Select    |       CoX      |          LPC17xx       |
//! |------------------------|----------------|------------------------|
//! |   xSPI_SS_NONE         |    Mandatory   |            Y           |
//! |------------------------|----------------|------------------------|
//! |   xSPI_SSn             |    Mandatory   |        xSPI_SS0        |
//! |                        |                |------------------------|
//! |                        |                |        xSPI_SS1        |
//! |                        |                |------------------------|
//! |                        |                |        xSPI_SS01       |
//! +------------------------+----------------+------------------------+
//! @{
//
//*****************************************************************************

#define xSPI_SS0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xSPI_Exported_APIs xSPI API
//! \brief      xSPI API Reference.
//!
//! \section xSPI_Exported_APIs_Port CoX Port Details
//! +--------------------------+----------------+------------------------+
//! |  xSPI API                |       CoX      |          LPC17xx       |
//! |--------------------------|----------------|------------------------|
//! |  xSPIConfigSet           |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPISingleDataReadWrite |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIBitLengthGet        |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIDataRead            |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIDataWrite           |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIDataPut             |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIDataPutNonBlocking  |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIDataGet             |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIDataGetNonBlocking  |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIIntEnable           |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIIntCallbackInit     |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIIntDisable          |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIStatusGet           |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIIsBusy              |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIIsRxEmpty           |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIIsTxEmpty           |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIIsRxFull            |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIIsTxFull            |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIDMAEnable           |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIDMADisable          |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIEnable              |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPIDisable             |    Mandatory   |            Y           |
//! |--------------------------|----------------|------------------------|
//! |  xSPISSSet               |    Mandatory   |            Y           |
//! +--------------------------+----------------+------------------------+
//! @{
//
//*****************************************************************************


//*****************************************************************************
//
//! \brief  Configures the synchronous serial interface.
//!         This function configures the synchronous serial interface.  It sets
//!         the SPI protocol, mode of operation, bit rate, and data width and
//!         the first bit.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//! \param  [in] ulBitRate specifies the clock rate.
//!         The \e ulBitRate parameter defines the bit rate for the SPI.
//!         This bit rate must satisfy the following clock ratio criteria:
//!         - 0 !=  bit rate (master mode)
//!
//! \param  [in] ulConfig is the required configuration of the SPI.
//!         The \e ulConfig parameter is the logical OR of several different
//!         values, many of which are grouped into sets where only one can be
//!         chosen.
//!
//!         The Protocol of SPI can be one of the following values:
//!         \ref xSPI_MOTO_FORMAT_MODE_0
//!         \ref xSPI_MOTO_FORMAT_MODE_1
//!         \ref xSPI_MOTO_FORMAT_MODE_2
//!         \ref xSPI_MOTO_FORMAT_MODE_3
//!
//!         The SPI module can operate as a master or slave; The Mode of SPI
//!         can be one of the following values:
//!         \ref xSPI_MODE_MASTER
//!         \ref xSPI_MODE_SLAVE
//!
//!         The width of the data transfers can be a value between 1 and 32,
//!         inclusive. for LPC17xx, this parameter must be the following value:
//!         \ref xSPI_DATA_WIDTH8
//!
//!         The first bit of the data transfers, can be one of the following values:
//!         \ref xSPI_MSB_FIRST
//!         \ref xSPI_LSB_FIRST
//!
//! \return None.
//
//*****************************************************************************
#define xSPIConfigSet(ulBase, ulBitRate, ulConfig)


//*****************************************************************************
//
//! \brief  Read and write a data element from and to the SPI interface.
//!         This function send transmitted data to the SPI interface of the
//!         specified SPI module and gets received data from the SPI interface
//!         of the specified SPI module and return that data.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//! \param  [in] ulWData is the data that was transmitted over the SPI interface.
//!
//! \return The data that was received over the SPI interface.
//!
//! \note   Only the lower N bits of the value written to pulData contain valid
//!         data, where N is the data width as configured by SPIConfig().
//!         For example, if the interface is configured for 8-bit data width,
//!         only the lower 8 bits of the value written to pulData contain valid
//!         data.
//
//*****************************************************************************
#define xSPISingleDataReadWrite(ulBase, ulWData)

//*****************************************************************************
//
//! \brief  Gets the number of bits transferred per frame.
//!         This function gets the number of bits transferred per frame.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return The number of bits transferred per frame.
//
//*****************************************************************************
#define xSPIBitLengthGet(ulBase)

//*****************************************************************************
//
//! \brief  Gets a data element from the SPI interface.
//!         This function gets received data from the interface of the specified
//!         SPI module and places that data into the location specified by the
//!         pulData parameter.
//!
//! \param  [in]  ulBase specifies the SPI module base address.
//! \param  [out] pulData is a pointer to a storage location for data that was
//!               received over the SPI interface.
//! \param  [in]  ulLen specifies the length of data will be read.
//!
//! \return None.
//!
//! \note   Only the lower N bits of the value written to pulData contain
//!         valid data, where N is the data width as configured by SPIConfig().
//!         For example, if the interface is configured for 8-bit data width,
//!         only the lower 8 bits of the value written to pulData contain valid
//!         data.
//
//*****************************************************************************
#define xSPIDataRead(ulBase, pulRData, ulLen)

//*****************************************************************************
//
//! \brief  Write datas element to the SPI interface.
//!         This function transmitted data to the interface of the specified
//!         SPI module .
//!
//! \param  [in] ulBase specifies the SPI module base address.
//! \param  [in] pulWData is a pointer to a storage location for data that was
//!              transmitted over the SPI interface.
//! \param  [in] ulLen specifies the length of data will be write.
//!
//! \return None.
//!
//! \note   Only the lower N bits of the value written to pulData contain
//!         valid data, where N is the data width as configured by SPIConfig().
//!         For example, if the interface is configured for 8-bit data width,
//!         only the lower 8 bits of the value written to pulData contain valid
//!         data.
//
//*****************************************************************************
#define xSPIDataWrite(ulBase, pulWData, ulLen)

//*****************************************************************************
//
//! \brief  Write data element to the SPI interface with block.
//!         This function transmitted data to the interface of the specified SPI
//!         module with block. when the TX and TX shift are both empty or in FIFO
//!         mode the TX FIFO depth is equal to or less than the trigger level,
//!         the data element can be transmitted, otherwise the data element will
//!         be blocked until can be transmitted.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//! \param  [in] ulData is data that was transmitted over the SPI interface.
//!
//! \return None.
//!
//! \note   Only the lower N bits of the value written to pulData contain
//!         valid data, where N is the data width as configured by SPIConfig().
//!         For example, if the interface is configured for 8-bit data width,
//!         only the lower 8 bits of the value written to pulData contain valid
//!         data.
//
//*****************************************************************************
#define xSPIDataPut(ulBase, ulData)

//*****************************************************************************
//
//! \brief  Write data element to the SPI interface with Noblock.
//!         This function transmitted data to the interface of the specified
//!         SPI module with Noblock.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//! \param  [in] ulData is data that was transmitted over the SPI interface.
//!
//! \return The number of data that has been transfered..
//!
//! \note   Only the lower N bits of the value written to pulData contain
//!         valid data, where N is the data width as configured by SPIConfig().
//!         For example, if the interface is configured for 8-bit data width,
//!         only the lower 8 bits of the value written to pulData contain valid
//!         data.
//!
//
//*****************************************************************************
#define xSPIDataPutNonBlocking(ulBase, ulData)

//*****************************************************************************
//
//! \brief  Gets a data element from the SPI interface with block.
//!         This function gets received data from the interface of the specified
//!         SPI module with block. when the RX not empty flag is set, the data
//!         element can be transmitted, otherwise the data element will be
//!         blocked until can be transmitted.
//!
//! \param  [in]  ulBase specifies the SPI module base address.
//! \param  [out] pulData is a pointer to a storage location for data that was
//!               received over the SPI interface.
//!
//! \note   Only the lower N bits of the value written to pulData contain
//!         valid data, where N is the data width as configured by SPIConfig().
//!         For example, if the interface is configured for 8-bit data width,
//!         only the lower 8 bits of the value written to pulData contain valid
//!         data.
//!
//! \return None.
//
//*****************************************************************************
#define xSPIDataGet(ulBase, pulData)

//*****************************************************************************
//
//! \brief  Gets a data element from the SPI interface with Noblock.
//!         This function gets received data from the interface of the specified
//!         SPI module with Noblock.
//!
//! \param  [in]  ulBase specifies the SPI module base address.
//! \param  [out] pulData is a pointer to a storage location for data that was
//!               received over the SPI interface.
//!
//! \note   Only the lower N bits of the value written to pulData contain
//!         valid data, where N is the data width as configured by SPIConfig().
//!         For example, if the interface is configured for 8-bit data width,
//!         only the lower 8 bits of the value written to pulData contain valid
//!         data.
//!
//! \return The number of data that has been received.
//
//*****************************************************************************
#define xSPIDataGetNonBlocking(ulBase, pulData)

//*****************************************************************************
//
//! \brief  Enable the SPI interrupt of the specified SPI port.
//!         This function is to enable the SPI interrupt of the specified SPI port.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//! \param  [in] ulIntFlags specifies the type of SPI interrupt.
//!
//! \return None.
//
//*****************************************************************************
#define xSPIIntEnable(ulBase, ulIntFlags)

//*****************************************************************************
//
//! \brief  Register user's interrupts callback function for specified SPI Port.
//!
//! \param  [in] ulPort is the base address of the SPI port.
//! \param  [in] xtI2CCallback is callback for the specified SPI Port.
//!
//! \return None.
//
//*****************************************************************************
#define xSPIIntCallbackInit(ulBase, xtSPICallback)

//*****************************************************************************
//
//! \brief  Disable the SPI interrupt of the specified SPI port.
//!         This function is to disable the SPI interrupt of the specified
//!         SPI port.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return None.
//
//*****************************************************************************
#define xSPIIntDisable(ulBase, ulIntFlags)

//*****************************************************************************
//
//! \brief  Get the SPI interrupt flag of the specified SPI port.
//!         This function returns the interrupt status for the SPI module.
//!         Either the raw interrupt status or the status of interrupts that
//!         are allowed to reflect to the processor can be returned.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//! \param  [in] xbMasked is \b false if the raw interrupt status is required or
//!              \b true if the masked interrupt status is required.
//!
//! \return The SPI interrupt flag.TODO
//
//*****************************************************************************
#define xSPIStatusGet(ulBase, xbMasked)

//*****************************************************************************
//
//! \brief  Check the busy status of the specified SPI port.
//!         This function Check the busy status of the specified SPI module.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return Returns the busy status of the specified SPI port.
//!         \ref xtrue The SPI port is in busy
//!         \ref xfalse The SPI port is not in busy.
//
//***************************e*************************************************
#define xSPIIsBusy(ulBase)

//*****************************************************************************
//
//! \brief  Check the status of the Rx buffer of the specified SPI port.
//!         This function Check the Rx buffer status of the specified SPI module.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return Returns the Rx buffer status of the specified SPI port.
//!         \ref xtrue The Rx buffer is empty
//          \ref xfalse The Rx buffer is not empty
//
//*****************************************************************************
#define xSPIIsRxEmpty(ulBase)

//*****************************************************************************
//
//! \brief  Check the status of the Tx buffer of the specified SPI port.
//!         This function Check the Tx buffer status of the specified SPI module.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return Returns the Tx buffer status of the specified SPI port.
//!         \ref xtrue The Tx buffer is in empty
//!         \ref xfalse The Tx buffer is not empty
//
//*****************************************************************************
#define xSPIIsTxEmpty(ulBase)

//*****************************************************************************
//
//! \brief  Check the status of the Rx buffer of the specified SPI port.
//!         This function Check the Rx buffer status of the specified SPI module.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return Returns the Rx buffer status of the specified SPI port.
//!         \ref xtrue  The Rx buffer is in full
//!         \ref xfalse The Rx buffer is not full
//
//*****************************************************************************
#define xSPIIsRxFull(ulBase)

//*****************************************************************************
//
//! \brief  Check the status of the Tx buffer of the specified SPI port.
//!         This function Check the Tx buffer status of the specified SPI module.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return Returns the Tx buffer status of the specified SPI port.
//!         \ref xtrue  The Tx buffer is in full
//!         \ref xfalse The Tx buffer is not full
//
//*****************************************************************************
#define xSPIIsTxFull(ulBase)

//*****************************************************************************
//
//! \brief  Enable the DMA of the specified SPI port.
//!         This function enable the DMA of the specified SPI port.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \param  [in] ulDmaMode specifies the SPI module base address.
//!
//! \return None.
//
//*****************************************************************************
#define xSPIDMAEnable(ulBase, ulDmaMode)

//*****************************************************************************
//
//! \brief  Disable the DMA of the specified SPI port.
//!         This function disable the DMA of the specified SPI port.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \param  [in] ulDmaMode specifies the SPI module base address.
//!
//! \return None.
//
//*****************************************************************************
#define xSPIDMADisable(ulBase, ulDmaMode)

//*****************************************************************************
//
//! \brief  Enable the specified SPI port.
//!         This function Enable the specified SPI port.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return None.
//
//***************************************************************************
#define xSPIEnable(ulBase)

//*****************************************************************************
//
//! \brief  Disable the specified SPI port.
//!         This function disable the specified SPI port.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \return None.
//
//***************************************************************************
#define xSPIDisable(ulBase)

//*****************************************************************************
//
//! \brief  Set the slave select pins of the specified SPI port.
//!         This function is to Set the slave select pins of the specified SPI port.
//!
//! \param  [in] ulBase specifies the SPI module base address.
//!
//! \param  [in] ulSSMode specifies the SS is hardware control or software control.
//!
//! \param  [in] ulSlaveSel specifies the slave select pins which will be used.
//!
//! \return None.
//!
//! \note   This is only for master.
//
//*****************************************************************************
#define xSPISSSet(ulBase, ulSSMode, ulSlaveSel)

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup  LPC17xx _SPI
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup  LPC17xx _Ints  LPC17xx  SPI Interrupts
//! \brief Values show that xSPI Interrupts
//! Values that can be passed to SPIIntEnable, SPIIntDisable, and SPIIntClear
//! as the ulIntFlags parameter, and returned from SPIIntStatus.
//! @{
//
//*****************************************************************************

//! Slave abort
#define SPI_ABRT                   BIT_32_3

//! Mode Fault
#define SPI_MODF                   BIT_32_4

//! Read overrun
#define SPI_ROVR                   BIT_32_5

//! write collision
#define SPI_WCOL                   BIT_32_6

//! SPI transfer finish
#define SPI_SPIF                   BIT_32_7


//! \note Those parameters can be used in SPICfg Function.

//! SPI Data Length 8-bit.
#define SPI_DATA_LEN_8             BIT_32_18

//! SPI Data Length 9-bit.
#define SPI_DATA_LEN_9             BIT_32_2 | BIT_32_11 | BIT_32_26 | BIT_32_25 | BIT_32_8

//! SPI Data Length 10-bit.
#define SPI_DATA_LEN_10            BIT_32_2 | BIT_32_11 | BIT_32_26 | BIT_32_9  | BIT_32_24

//! SPI Data Length 11-bit.
#define SPI_DATA_LEN_11            BIT_32_2 | BIT_32_11 | BIT_32_26 | BIT_32_9  | BIT_32_8

//! SPI Data Length 12-bit.
#define SPI_DATA_LEN_12            BIT_32_2 | BIT_32_11 | BIT_32_10 | BIT_32_25 | BIT_32_24

//! SPI Data Length 13-bit.
#define SPI_DATA_LEN_13            BIT_32_2 | BIT_32_11 | BIT_32_10 | BIT_32_25 | BIT_32_8

//! SPI Data Length 14-bit.
#define SPI_DATA_LEN_14            BIT_32_2 | BIT_32_11 | BIT_32_10 | BIT_32_9  | BIT_32_24

//! SPI Data Length 15-bit.
#define SPI_DATA_LEN_15            BIT_32_2 | BIT_32_11 | BIT_32_10 | BIT_32_9  | BIT_32_8

//! SPI Data Length 16-bit.
#define SPI_DATA_LEN_16            BIT_32_2 | BIT_32_27 | BIT_32_26 | BIT_32_25 | BIT_32_24

//! SPI Master Mode.
#define SPI_MODE_MASTER            BIT_32_5

//! SPI Slave Mode.
#define SPI_MODE_SLAVE             BIT_32_21

//! Data is sampled on the first clock edge of SCK.A transfer starts
//! and ends with activation and deactivation of the SSEL signal.
#define SPI_CPHA_FIRST             BIT_32_19

//! Data is sampled on the second clock edge of the SCK.A transfer starts with the first
//! clock edge, and ends with the last sampling edge when the SSEL signal is active.
#define SPI_CPHA_SECOND            BIT_32_3

//! SCK is active high.
#define SPI_CPOL_HIGH              BIT_32_20

//! SCK is active low.
#define SPI_CPOL_LOW               BIT_32_4

//! SPI data is transferred LSB (bit 1) first.
#define SPI_LSB_FIRST              BIT_32_6

//! SPI data is transferred MSB (bit 7) first.
#define SPI_MSB_FIRST              BIT_32_22

//!
#define SPI_INT_SPIF               S0SPINT_SPIF


//*****************************************************************************
//
//! @}
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup  LPC17xx _SPI_Exported_APIs   LPC17xx  SPI API
//! \brief  LPC17xx  SPI API Reference
//! @{
//
//*****************************************************************************

extern xtBoolean SPIStatCheck(unsigned long ulBase, unsigned long ulFlags);
extern unsigned long SPIStatGet(unsigned long ulBase);
extern void SPIStatFlagClear(unsigned long ulBase, unsigned long ulFlags);
extern unsigned long SPIDataReadWrite(unsigned long ulBase, unsigned long ulVal);
extern void SPICfg(unsigned long ulBase, unsigned long ulClk, unsigned long ulCfgs);
extern void SPIIntEnable(unsigned long ulBase);
extern void SPIIntDisable(unsigned long ulBase);
extern unsigned long SPIIntFlagGet(unsigned long ulBase);
extern xtBoolean SPIIntFlagCheck(unsigned long ulBase, unsigned long ulFlags);
extern void SPIIntFlagClear(unsigned long ulBase, unsigned long ulFlags);

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __xSPI_H__


