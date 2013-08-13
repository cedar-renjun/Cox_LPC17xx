//*****************************************************************************
//
//! \file xi2c.h
//! \brief Prototypes for the I2C Driver.
//! \version V2.2.1.0
//! \date 02/06/2011
//! \author CooCox
//! \copy
//!
//! Copyright (c)  2011, CooCox
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!
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

#ifndef __xI2C_H__
#define __xI2C_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
//extern "C"
//{
#endif

//*****************************************************************************
//
//! \addtogroup CoX_Peripheral_Lib
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup I2C
//! @{
//
//*****************************************************************************













//*****************************************************************************
//
//! \addtogroup xI2C
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xI2C_INT_Master xI2C Master Interrupt
//!
//! \verbatim
//! +------------------------+----------------+------------------------+
//! |xI2C Master Interrupts  |       CoX      |        STM32F1xx       |
//! |------------------------|----------------|------------------------|
//! |xI2C_MASTER_INT_DATA    |    Mandatory   |            Y           |
//! +------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
// Transmit / Receive a data, or an error occurs
//
#define xI2C_MASTER_INT_DATA

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xI2C_Event_Master xI2C Master Event
//!
//! \verbatim
//! +------------------------+----------------+------------------------+
//! |xI2C Master Event       |       CoX      |        STM32F1xx       |
//! |------------------------|----------------|------------------------|
//! |xI2C_MASTER_EVENT_TX    |    Mandatory   |            Y           |
//! |xI2C_MASTER_EVENT_RX    |    Mandatory   |            Y           |
//! +------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
// Transmit address/data, or some error occurs
//
#define xI2C_MASTER_EVENT_TX

//
// Receive a data, or some error occurs
//
#define xI2C_MASTER_EVENT_RX

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xI2C_INT_Slave xI2C Slave Interrupt
//!
//!
//! \verbatim
//! +------------------------+----------------+------------------------+
//! |xI2C Slave Interrupts   |       CoX      |        STM32F1xx       |
//! |------------------------|----------------|------------------------|
//! |xI2C_SLAVE_INT_START    |    Optional    |            N           |
//! |xI2C_SLAVE_INT_STOP     |    Optional    |            N           |
//! |xI2C_SLAVE_INT_DATA     |    Mandatory   |            Y           |
//! +------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Start Condition
//
#define xI2C_SLAVE_INT_START

//
//! Stop Condition
//
#define xI2C_SLAVE_INT_STOP

//
//! A data received or data requested or error occurs
//
#define xI2C_SLAVE_INT_DATA


//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xI2C_Event_Slave xI2C Slave Event
//!
//! \verbatim
//! +------------------------+----------------+------------------------+
//! |xI2C Slave Event        |       CoX      |        STM32F1xx       |
//! |------------------------|----------------|------------------------|
//! |xI2C_SLAVE_EVENT_START  |    Optional    |            N           |
//! |xI2C_SLAVE_EVENT_STOP   |    Optional    |            N           |
//! |xI2C_SLAVE_EVENT_TREQ   |    Mandatory   |            Y           |
//! |xI2C_SLAVE_EVENT_RREQ   |    Mandatory   |            Y           |
//! +------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Start Condition
//
#define xI2C_SLAVE_EVENT_START

//
//! Stop Condition
//
#define xI2C_SLAVE_EVENT_STOP

//
//! Transmit Request( or address matched in slave transmit mode)
//
#define xI2C_SLAVE_EVENT_TREQ

//
//! Receive Request
//
#define xI2C_SLAVE_EVENT_RREQ

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xI2C_Master_Error xI2C Master Error
//!
//! \verbatim
//! +------------------------+----------------+------------------------+
//! |xI2C Master Error       |       CoX      |        STM32F1xx       |
//! |------------------------|----------------|------------------------|
//! |xI2C_MASTER_ERR_NONE    |    Mandatory   |            Y           |
//! |xI2C_MASTER_ERR_ADDR_ACK|    Mandatory   |            Y           |
//! |xI2C_MASTER_ERR_DATA_ACK|    Mandatory   |            Y           |
//! |xI2C_MASTER_ERR_ARB_LOST|    Mandatory   |            Y           |
//! +------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Every thing is OK
//
#define xI2C_MASTER_ERR_NONE

//
//! The transmitted address was not acknowledged
//
#define xI2C_MASTER_ERR_ADDR_ACK


//
//! The transmitted data was not acknowledged.
//
#define xI2C_MASTER_ERR_DATA_ACK


//
//! The I2C controller lost arbitration
//
#define xI2C_MASTER_ERR_ARB_LOST


//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xI2C_General_Call xI2C General Call
//! \brief Values that show xI2C General Call
//! \n
//! \section xI2C_Transfer_Type_Section 1. Where to use this group
//! Values that can be passed to xI2CSlaveInit()
//! as the ulGeneralCall parameter.
//! \n
//! \section xI2C_Transfer_Type_CoX 2.CoX Mandatory and CoX Non-mandatory
//! \verbatim
//! +------------------------+----------------+---------------------------+
//! |xI2C General Call       |       CoX      |         STM32F1xx         |
//! |------------------------|----------------|---------------------------|
//! |xI2C_GENERAL_CALL_DIS   |    Mandatory   |             Y             |
//! |------------------------|----------------|---------------------------|
//! |xI2C_GENERAL_CALL_EN    |  Non-Mandatory |             Y             |
//! |------------------------|----------------|---------------------------|
//! \endverbatim
//! @{
//
//*****************************************************************************
#define xI2C_GENERAL_CALL_EN                  I2C_GENERAL_CALL_EN
#define xI2C_GENERAL_CALL_DIS                 I2C_GENERAL_CALL_DIS

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xI2C_Exported_APIs xI2C API
//! \brief xI2C API Reference.
//!
//! \section xI2C_Exported_APIs_Port CoX Port Details
//! \verbatim
//! +-------------------------------+----------------+------------------------+
//! |  xI2C API                     |       CoX      |       STM32F1xx        |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterInit               |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterEnable             |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterDisable            |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterBusBusy            |    Mandatory   |            N           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterBusy               |    Mandatory   |            N           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterError              |    Mandatory   |            N           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterDataPut            |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterDataGet            |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterStop               |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterWriteRequestS1     |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterWriteRequestS2     |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterWriteS1            |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterWriteS2            |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterWriteBufS1         |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterWriteBufS2         |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterReadRequestS1      |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterReadRequestS2      |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterReadLastRequestS2  |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterReadS1             |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterReadS2             |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterReadBufS1          |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterReadBufS2          |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterIntEnable          |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CMasterIntDisable         |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CSlaveIntEnable           |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CSlaveIntDisable          |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CSlaveInit                |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CSlaveEnable              |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CSlaveDisable             |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CIntCallbackInit          |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CSlaveDataPut             |    Mandatory   |            Y           |
//! |-------------------------------|----------------|------------------------|
//! |  xI2CSlaveDataGet             |    Mandatory   |            Y           |
//! +-------------------------------+----------------+------------------------+
//! \endverbatim
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief Initialize the I2C controller.
//!
//! \param ulBase is the I2C module base address.
//! \param ulI2CClk is the I2C clock bit rate.
//!
//! This function initializes operation of the I2C Master block.  Upon
//! successful initialization of the I2C block, this function will have set the
//! bus speed for the master, and will have enabled the I2C Master block.
//!
//! The parameter \e ulBase can be:
//! - \ref xI2C0_BASE
//! - \ref xI2C1_BASE
//!
//! The parameter \e ulI2CClk can only be:
//! - \b 100000 - I2C works under standard-mode (Sm), with a bit rate up to
//!               100 kbit/s
//! - \b 400000 - I2C works under fast-mode (Fm), with a bit rate up to
//!               400 kbit/s
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterInit(ulBase, ulI2CClk)       I2CMasterInit(ulBase, ulI2CClk)

//*****************************************************************************
//
//! \brief Enables the I2C Master block.
//!
//! \param ulBase is the base address of the I2C module.
//!
//! This will enable operation of the I2C Master block.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterEnable(ulBase)               I2CEnable(ulBase)

//*****************************************************************************
//
//! \brief Disables the I2C master block.
//!
//! \param ulBase is the base address of the I2C module.
//!
//! This will disable operation of the I2C master block.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterDisable(ulBase)              I2CDisable(ulBase)

//*****************************************************************************
//
//! \brief Enables the I2C Slave block.
//!
//! \param ulBase is the base address of the I2C module.
//!
//! This will enable operation of the I2C Slave block.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CSlaveEnable(ulBase)                I2CEnable(ulBase)

//*****************************************************************************
//
//! \brief Disables the I2C slave block.
//!
//! \param ulBase is the base address of the I2C module.
//!
//! This will disable operation of the I2C slave block.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CSlaveDisable(ulBase)               I2CDisable(ulBase)

//*****************************************************************************
//
//! \brief Indicates whether or not the I2C bus is busy.
//!
//! \param ulBase is the base address of the I2C module.
//!
//! This function returns an indication of whether or not the I2C bus is busy.
//! This function can be used in a multi-master environment to determine if
//! another master is currently using the bus.
//!
//! \return Returns \b xtrue if the I2C bus is busy; otherwise, returns
//! \b xfalse.
//
//*****************************************************************************
#define xI2CMasterBusBusy(ulBase)             I2CBusBusyStatus(ulBase)

//*****************************************************************************
//
//! \brief Indicates whether or not the I2C Master is busy.
//!
//! \param ulBase is the base address of the I2C Master module.
//!
//! This function returns an indication of whether or not the I2C Master is
//! busy transmitting or receiving data.
//!
//! \return Returns \b xtrue if the I2C Master is busy; otherwise, returns
//! \b xfalse.
//
//*****************************************************************************
#define xI2CMasterBusy(ulBase)               I2CBusBusyStatus(ulBase)

//*****************************************************************************
//
//! \brief Init interrupts callback for the specified I2C Port.
//!
//! \param ulPort is the base address of the I2C port.
//! \param xtI2CCallback is callback for the specified I2C Port.
//!
//! Init interrupts callback for the specified I2C Port.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CIntCallbackInit(ulBase, xtI2CCallback)                            \
        I2CIntCallbackInit(ulBase, xtI2CCallback)

//*****************************************************************************
//! \brief Slave Send a byte to I2C bus.
//!
//! \param ulBase specifies the I2C module base address.
//! \param ucData specifies the data which will send to I2C BUS.
//!
//! This function is to send a byte on specified I2C BUS.
//!
//! The \e ulBase can be one of the following values:
//! \b xI2C1_BASE, \b xI2C2_BASE.
//!
//! \note This is only for slave
//!
//! \return None.
//
//*****************************************************************************
#define xI2CSlaveDataPut(ulBase, ucData)                                      \
        I2CDataWrite(ulBase, ucData)


//*****************************************************************************
//! \brief Slave receive a byte to I2C bus.
//!
//! \param ulBase specifies the I2C module base address.
//!
//! This function is to receive a byte on specified I2C BUS.
//!
//! The \e ulBase can be one of the following values:
//! \b xI2C1_BASE, \b xI2C2_BASE.
//!
//! \note This is only for slave
//!
//! \return None.
//
//*****************************************************************************
#define xI2CSlaveDataGet(ulBase)                                              \
        I2CDataRead(ulBase)

//*****************************************************************************
//
//! Enables the I2C Master interrupt.
//!
//! \param ulBase is the base address of the I2C module.
//! \param ulIntType is the interrupt type of the I2C module.
//!
//! The \e ulIntType is the interrupt type of the I2C module.
//!
//! Enables the I2C Master interrupt source.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterIntEnable(ulBase, ulIntType)                                \
        I2CIntEnable(ulBase)

//*****************************************************************************
//
//! Disables the I2C Master interrupt.
//!
//! \param ulBase is the base address of the I2C module.
//! \param ulIntType is the interrupt type of the I2C module.
//!
//! The \e ulIntType is the interrupt type of the I2C module.
//!
//! Disables the I2C Master interrupt source.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterIntDisable(ulBase, ulIntType)                               \
        I2CIntDisable(ulBase)

//*****************************************************************************
//
//! \brief Set the clock rate of the specified I2C port.
//!
//! \param ulBase specifies the I2C module base address.
//! \param ucSlaveAddr specifies the slave address.
//! \param ulGeneralCall specifies enable General Call function or not.
//! Details please refer to \ref xI2C_General_Call.
//!
//! This function is to Set 4 7-bit slave addresses and enable General Call
//! function of specified I2C port.
//!
//! The \e ulBase must be:\b I2C0_BASE.
//!
//! The \e ucSlaveAddr is the I2C slave address,There are 4 slave address.
//! The ucSlaveAddr can be a 7-bit value.
//!
//! The \e ulGeneralCall is to enable the General Call function or not.
//! The ulGeneralCall can be one of the following values:
//! \b I2C_GENERAL_CALL_EN,\b I2C_GENERAL_CALL_DIS.
//! Details please refer to \ref xI2C_General_Call_CoX.
//!
//! \note this is only for slave
//!
//! \return None.
//
//*****************************************************************************
#define xI2CSlaveInit(ulBase, ucSlaveAddr, ulGeneralCall)                      \
        I2CSlaveInit(ulBase, ucSlaveAddr, ulGeneralCall)

//*****************************************************************************
//
//! Enables the I2C Slave interrupt.
//!
//! \param ulBase is the base address of the I2C module.
//! \param ulIntType is the interrupt type of the I2C module.
//!
//! The \e ulIntType is the interrupt type of the I2C module.
//!
//! Enables the I2C Slave interrupt source.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CSlaveIntEnable(ulBase, ulIntType)                                 \
        I2CIntEnable(ulBase)

//*****************************************************************************
//
//! Disables the I2C Slave interrupt.
//!
//! \param ulBase is the base address of the I2C module.
//! \param ulIntType is the interrupt type of the I2C module.
//!
//! The \e ulIntType is the interrupt type of the I2C module.
//!
//! Disables the I2C slave interrupt source.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CSlaveIntDisable(ulBase, ulIntType)                               \
        I2CIntDisable(ulBase)

//*****************************************************************************
//
//! Gets the error status of the I2C Master module.
//!
//! \param ulBase is the base address of the I2C Master module.
//!
//! This function is used to obtain the error status of the Master module send
//! and receive operations.
//!
//! \return Returns the error status, as one of \b I2C_MASTER_ERR_NONE,
//! \b I2C_MASTER_ERR_ADDR_ACK, \b I2C_MASTER_ERR_DATA_ACK, or
//! \b I2C_MASTER_ERR_ARB_LOST.
//
//*****************************************************************************
#define xI2CMasterError(ulBase)

//*****************************************************************************
//
//! Transmits a byte from the I2C Master.
//!
//! \param ulBase is the base address of the I2C module.
//! \param ucData data to be transmitted from the I2C Master
//!
//! This function will place the supplied data into I2C Master Data Register.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterDataPut(ulBase, ucData)                                     \
        I2CDataWrite(ulBase, ucData)

//*****************************************************************************
//
//! Receives a byte that has been sent to the I2C Master.
//!
//! \param ulBase is the base address of the I2C module.
//!
//! This function reads a byte of data from the I2C Master Data Register.
//!
//! \return Returns the byte(unsigned char) received from by the I2C Master.
//
//*****************************************************************************
#define xI2CMasterDataGet(ulBase)                                             \
        I2CDataRead(ulBase)

//*****************************************************************************
//
//! \brief Transmite the STOP condition, master goes to idle state.
//!
//! \param ulBase is the base address of the I2C Master module.
//!
//! This function free the I2C bus. When the master no longer need send or
//! receive any more data, or need to terminate this transmition after getting
//! some errors, call this function.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterStop(ulBase)                                                \
        I2CStopSend(ulBase)

//*****************************************************************************
//
//! \brief Send a master transmit request when the bus is idle.(Write Step1)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param ucSlaveAddr is the 7-bit slave address.
//! \param ucData is the byte to transmit.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! This function init a new write transmition. When the master have not obtained
//! control of the bus, This function send request to transmit the START
//! condition, the slave address and the data, Then it returns immediately, no
//! waiting any bus transmition to complete.
//!
//! Users can call xI2CMasterBusy() to check if all the bus  transmition
//! complete, the call xI2CMasterError() to check if any error occurs.
//!
//! After the master obtained control of the bus, and haven't release it, users
//! can call xI2CMasterWriteRequestS2() to continue transmit data to slave.
//! Users can also call xI2CMasterStop() to terminate this transmition and
//! release the I2C bus.
//!
//! For this function returns immediately, it is always using in the interrupt
//! handler.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterWriteRequestS1(ulBase, ucSlaveAddr, ucData, bEndTransmition) \
        I2CMasterWriteRequestS1(ulBase, ucSlaveAddr, ucData, bEndTransmition)

//*****************************************************************************
//
//! \brief Send a master data transmit request when the master have obtained
//! control of the bus.(Write Step2)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param ucData is the byte to transmit.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! After the master obtained control of the bus(have called
//! xI2CMasterWriteRequestS1() without any error), and haven't release it, users
//! can call this function to continue transmit data to slave.
//!
//! This function just send request to transmit the data, and it returns
//! immediately, no waiting any bus transmition to complete.
//!
//! Users can call xI2CMasterBusy() to check if all the bus transmition
//! complete, the call xI2CMasterError() to check if any error occurs. Users call
//! also can xI2CMasterStop() to terminate this transmition and release the
//! I2C bus.
//!
//! For this function returns immediately, it is always using in the interrupt
//! handler.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterWriteRequestS2(ulBase, ucData, bEndTransmition)             \
        I2CMasterWriteRequestS2(ulBase, ucData, bEndTransmition)

//*****************************************************************************
//
//! \brief Write a data to the slave when the bus is idle, and waiting for all
//! bus transmiton complete.(Write Step1)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param ucSlaveAddr is the 7-bit slave address.
//! \param ucData is the byte to transmit.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! This function init a new write transmition. When the master have not obtained
//! control of the bus, This function transmit the START condition, the slave
//! address and the data, then waiting for all bus transmition complete.
//!
//! Users can then check the return value to see if any error occurs:
//! - \ref xI2C_MASTER_ERR_NONE     - \b 0, no error
//! - \ref xI2C_MASTER_ERR_ADDR_ACK - The transmitted address was not acknowledged
//! - \ref xI2C_MASTER_ERR_DATA_ACK - The transmitted data was not acknowledged
//! - \ref xI2C_MASTER_ERR_ARB_LOST - The I2C controller lost arbitration.
//!
//! After the master obtained control of the bus, and haven't release it, users
//! can call xI2CMasterWriteS2() to continue transmit data to slave.
//! Users call also can xI2CMasterStop() to terminate this transmition and
//! release the I2C bus.
//!
//! This function is always used in thread mode.
//!
//! \return Returns the master error status.
//
//*****************************************************************************
#define xI2CMasterWriteS1(ulBase, ucSlaveAddr, ucData, bEndTransmition)       \
        I2CMasterWriteS1(ulBase, ucSlaveAddr, ucData, bEndTransmition)

//*****************************************************************************
//
//! \brief Write a data to the slave, when the master have obtained control of
//! the bus, and waiting for all bus transmiton complete.(Write Step2)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param ucData is the byte to transmit.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! After the master obtained control of the bus(have called
//! xI2CMasterWriteS1() without any error), and haven't release it, users
//! can call this function to continue transmit data to slave.
//!
//! This function transmit the data to the slave, and waiting for all bus
//! transmition complete.
//!
//! Users can then check the return value to see if any error occurs:
//! - \ref xI2C_MASTER_ERR_NONE     - \b 0, no error
//! - \ref xI2C_MASTER_ERR_ADDR_ACK - The transmitted address was not acknowledged
//! - \ref xI2C_MASTER_ERR_DATA_ACK - The transmitted data was not acknowledged
//! - \ref xI2C_MASTER_ERR_ARB_LOST - The I2C controller lost arbitration.
//!
//! Then users can call this function to continue transmit data to slave.
//! Users call also call xI2CMasterStop() to terminate this transmition and
//! release the I2C bus.
//!
//! This function is always used in thread mode.
//!
//! \return Returns the master error status.
//
//*****************************************************************************
#define xI2CMasterWriteS2(ulBase, ucData, bEndTransmition)                    \
        I2CMasterWriteS2(ulBase, ucData, bEndTransmition)

//*****************************************************************************
//
//! \brief Write a data buffer to the slave when the bus is idle, and waiting
//! for all bus transmiton complete.(Write Buffer Step1)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param ucSlaveAddr is the 7-bit slave address.
//! \param pucDataBuf is the data buffer to transmit.
//! \param ulLen is the data buffer byte size.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! This function init a new data buffer write transmition. When the master have
//! not obtained control of the bus, This function transmit the START condition,
//! the slave address and the data, then waiting for the data transmition
//! complete, and continue next data transmition, until all complete. If there
//! is any error occurs, the remain data will be canceled.
//!
//! Users can then check the return value to see how many datas have been
//! successfully transmited. if the number != ulLen, user can call
//! xI2CMasterError() to see what error occurs.
//!
//! After the master obtained control of the bus, and haven't release it, users
//! can call xI2CMasterWriteS2() / xI2CMasterWriteBufS2() to continue transmit data
//! to slave. Users call also call xI2CMasterStop() to terminate this transmition
//! and release the I2C bus.
//!
//! This function is always used in thread mode.
//!
//! \return Returns the data number that have been successully tranmited.
//
//*****************************************************************************
#define xI2CMasterWriteBufS1(ulBase, ucSlaveAddr, pucDataBuf, ulLen, bEndTransmition) \
        I2CMasterWriteBufS1(ulBase, ucSlaveAddr, pucDataBuf, ulLen, bEndTransmition)


//*****************************************************************************
//
//! \brief Write a data buffer to the slave, when the master have obtained
//! control of the bus, and waiting for all bus transmiton complete.(Write
//! Buffer Step2)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param pucDataBuf is the data buffer to transmit.
//! \param ulLen is the data buffer byte size.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! After the master obtained control of the bus(have called
//! xI2CMasterWriteS1() or xI2CMasterWriteBufS1() without any error), and haven't
//! release it, users can call this function to continue transmit data to slave.
//!
//! This function transmit the data one by one to the slave, waiting for every
//! data transmition complete, and continue next data transmition, until all
//! complete. If there is any error occurs, the remain data will be canceled.
//!
//! Users can then check the return value to see how many datas have been
//! successfully transmited. if the number != ulLen, user can call
//! xI2CMasterError() to see what error occurs.
//!
//! Then users can call xI2CMasterWriteS2() or this function to continue
//! transmit data to slave. Users call also call xI2CMasterStop() to terminate
//! this transmition and release the I2C bus.
//!
//! This function is always used in thread mode.
//!
//! \return Returns the data number that have been successully tranmited.
//
//*****************************************************************************
#define xI2CMasterWriteBufS2(ulBase, pucDataBuf, ulLen, bEndTransmition)      \
        I2CMasterWriteBufS2(ulBase, pucDataBuf, ulLen, bEndTransmition)

//*****************************************************************************
//
//! \brief Send a master receive request when the bus is idle.(Read Step1)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param ucSlaveAddr is the 7-bit slave address.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! This function init a new receive transmition. When the master have not obtained
//! control of the bus, This function send request to transmit the START
//! condition, the slave address and the data request, Then it returns
//! immediately, no waiting any bus transmition to complete.
//!
//! If bEndTransmition is xtrue, the receive operation will followed by an
//! negative ACK and STOP condition.
//!
//! Users can call xI2CMasterBusy() to check if all the bus transmition
//! complete, then call xI2CMasterError() to check if any error occurs. Then user
//! can get the data by calling xI2CMasterDataGet() if there is no error occurs.
//!
//! After the master obtained control of the bus, and haven't release it, users
//! can call xI2CMasterReadRequestS2() to continue receive data from slave.
//! Users call also can xI2CMasterStop() to terminate this transmition and
//! release the I2C bus.
//!
//! For this function returns immediately, it is always using in the interrupt
//! hander.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterReadRequestS1(ulBase, ucSlaveAddr, bEndTransmition)         \
        I2CMasterReadRequestS1(ulBase, ucSlaveAddr, bEndTransmition)

//*****************************************************************************
//
//! \brief Send a master data receive request when the master have obtained
//! control of the bus.(Write Step2)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! After the master obtained control of the bus(have called
//! xI2CMasterReadRequestS1() without any error), and haven't release it, users
//! can call this function to continue receive data from slave.
//!
//! If bEndTransmition is xtrue, the receive operation will followed by an
//! negative ACK and STOP condition.
//!
//! Users can call xI2CMasterBusy() to check if all the bus transmition
//! complete, then call xI2CMasterError() to check if any error occurs. Then user
//! can get the data by calling xI2CMasterDataGet() if there is no error occurs.
//!
//! Then users can call this function to continue receive data from slave.
//! Users call also can xI2CMasterStop() to terminate this transmition and
//! release the I2C bus.
//!
//! For this function returns immediately, it is always using in the interrupt
//! hander.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterReadRequestS2(ulBase, bEndTransmition)                      \
        I2CMasterReadRequestS2(ulBase, bEndTransmition)

//*****************************************************************************
//
//! \brief Send a master data receive request with an NACK when the master have
//! obtained control of the bus(Write Step2).
//!
//! \param ulBase is the base address of the I2C Master module.
//!
//! This function is used to request the last data to receive, and signal the
//! end of the transfer to the slave transmitter. Then the master can repeat
//! START condition, switch to transmit or other slaves without lost control
//! of the bus.
//!
//! Users can call xI2CMasterBusy() to check if all the bus transmition
//! complete, then call xI2CMasterError() to check if any error occurs. Then user
//! can get the data by calling xI2CMasterDataGet() if there is no error occurs.
//!
//! Users call also can xI2CMasterStop() to terminate this transmition and
//! release the I2C bus.
//!
//! For this function returns immediately, it is always using in the interrupt
//! handler.
//!
//! \return None.
//
//*****************************************************************************
#define xI2CMasterReadLastRequestS2(ulBase)                                   \
        I2CMasterReadLastRequestS2(ulBase)


//*****************************************************************************
//
//! \brief Read a data from a slave when the bus is idle, and waiting for all
//! bus transmiton complete.(Read Step1)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param ucSlaveAddr is the 7-bit slave address.
//! \param pucData is the buffer where to save the data.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! This function init a new receive transmition. When the master have not obtained
//! control of the bus, This function send request to transmit the START
//! condition, the slave address and the data request, then waiting for all bus
//! transmition complete.
//!
//! If bEndTransmition is xtrue, the receive operation will followed by an
//! negative ACK and STOP condition.
//!
//! Users can then check the return value to see if any error occurs:
//! - \ref xI2C_MASTER_ERR_NONE     - \b 0, no error
//! - \ref xI2C_MASTER_ERR_ADDR_ACK - The transmitted address was not acknowledged
//! - \ref xI2C_MASTER_ERR_DATA_ACK - The transmitted data was not acknowledged
//! - \ref xI2C_MASTER_ERR_ARB_LOST - The I2C controller lost arbitration.
//!
//! After the master obtained control of the bus, and haven't release it, users
//! can call xI2CMasterReadS2() to continue receive data from slave.
//! Users call also can xI2CMasterStop() to terminate this transmition and
//! release the I2C bus.
//!
//! This function is usually used in thread mode.
//!
//! \return Returns the master error status.
//
//*****************************************************************************
#define xI2CMasterReadS1(ulBase, ucSlaveAddr, pucData, bEndTransmition)       \
        I2CMasterReadS1(ulBase, ucSlaveAddr, pucData, bEndTransmition)

//*****************************************************************************
//
//! \brief Read a data from a slave when the master have obtained control of
//! the bus, and waiting for all bus transmiton complete.(Read Step2)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param pucData is the buffer where to save the data.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! After the master obtained control of the bus(have called
//! xI2CMasterReadS1() without any error), and haven't release it, users can
//! call this function to continue receive data from the slave.
//!
//! If bEndTransmition is xtrue, the receive operation will followed by an
//! negative ACK and STOP condition.
//!
//! It will be waiting for all bus transmition complete before return.
//! Users can then check the return value to see if any error occurs:
//! - \ref xI2C_MASTER_ERR_NONE     - \b 0, no error
//! - \ref xI2C_MASTER_ERR_ADDR_ACK - The transmitted address was not acknowledged
//! - \ref xI2C_MASTER_ERR_DATA_ACK - The transmitted data was not acknowledged
//! - \ref xI2C_MASTER_ERR_ARB_LOST - The I2C controller lost arbitration.
//!
//! Then useres can call this function to continue receive data from slave.
//! Users call also can xI2CMasterStop() to terminate this transmition and
//! release the I2C bus.
//!
//! This function is usually used in thread mode.
//!
//! \return Returns the master error status.
//
//*****************************************************************************
#define xI2CMasterReadS2(ulBase, pucData, bEndTransmition)                    \
        I2CMasterReadS2(ulBase, pucData, bEndTransmition)

//*****************************************************************************
//
//! \brief Read some data from a slave when the bus is idle, and waiting for all
//! bus transmiton complete.(Read Buffer Step1)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param ucSlaveAddr is the 7-bit slave address.
//! \param pucDataBuf is the buffer where to save the data.
//! \param ulLen is the data number to receive.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! This function init a new data buffer receive transmition. When the master
//! have not obtained control of the bus, This function send request to transmit
//! the START condition, the slave address and the data request, then waiting for
//! the data transmition complete, and continue next data transmition, until all
//! complete. If there is any error occurs, the remain data will be canceled.
//!
//! If bEndTransmition is xtrue, the receive operation will followed by an
//! negative ACK and STOP condition.
//!
//! Users can then check the return value to see how many datas have been
//! successfully received. if the number != ulLen, user can call
//! xI2CMasterError() to see what error occurs.
//!
//! After the master obtained control of the bus, and haven't release it, users
//! can call xI2CMasterReadS2() or xI2CMasterReadBufS2() to continue receive data .
//! from slave .Users call also can xI2CMasterStop() to terminate this transmition
//! and release the I2C bus.
//!
//! This function is usually used in thread mode.
//!
//! \return Returns the data number that have been successully received.
//
//*****************************************************************************
#define xI2CMasterReadBufS1(ulBase, ucSlaveAddr, pucDataBuf, ulLen, bEndTransmition) \
        I2CMasterReadBufS1(ulBase, ucSlaveAddr, pucDataBuf, ulLen, bEndTransmition)


//*****************************************************************************
//
//! \brief Read some data from a slave when the master have obtained control of
//! the bus, and waiting for all bus transmiton complete.(Write Buffer Step2)
//!
//! \param ulBase is the base address of the I2C Master module.
//! \param ucSlaveAddr is the 7-bit slave address.
//! \param pucDataBuf is the buffer where to save the data.
//! \param ulLen is the data number to receive.
//! \param bEndTransmition is flag to control if transmit the STOP condition and
//! terminate this transmition.
//!
//! After the master obtained control of the bus(have called
//! xI2CMasterReadS1() or xI2CMasterReadBufS1() without any error), and haven't
//! release it, users can call this function to continue receive data from slave.
//!
//! This function receive data one by one from the slave, waiting for every
//! data transmition complete, and continue next data transmition, until all
//! complete. If there is any error occurs, the remain data will be canceled.
//!
//! If bEndTransmition is xtrue, the receive operation will followed by an
//! negative ACK and STOP condition.
//!
//! Users can then check the return value to see how many datas have been
//! successfully received. if the number != ulLen, user can call
//! xI2CMasterError() to see what error occurs.
//!
//! After the master obtained control of the bus, and haven't release it, users
//! can call xI2CMasterReadS2() or xI2CMasterReadBufS2() to continue receive data
//! from slave. Users call also can xI2CMasterStop() to terminate this transmition
//! and release the I2C bus.
//!
//! This function is usually used in thread mode.
//!
//! \return Returns the data number that have been successully received.
//
//*****************************************************************************
#define xI2CMasterReadBufS2(ulBase, pucDataBuf, ulLen, bEndTransmition)       \
        I2CMasterReadBufS2(ulBase, pucDataBuf, ulLen, bEndTransmition)


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




            
#define I2C_GENERAL_CALL_EN     BIT_32_0
#define I2C_GENERAL_CALL_DIS    BIT_32_1
            

#define I2C_SLAVE_ADD0          BIT_32_0
#define I2C_SLAVE_ADD1          BIT_32_1
#define I2C_SLAVE_ADD2          BIT_32_2
#define I2C_SLAVE_ADD3          BIT_32_3


/*------------- I2C return status code definitions ------------------*/
//! No relevant information
#define I2C_STAT_NO_INF                       ((unsigned long) 0xF8)

//! Bus Error
#define I2C_STAT_BUS_ERROR                    ((unsigned long) 0x00)

/*-------------------- Master transmit mode -------------------------*/
//! A start condition has been transmitted
#define I2C_STAT_M_TX_START                   ((unsigned long) 0x08)

//! A repeat start condition has been transmitted
#define I2C_STAT_M_TX_RESTART                 ((unsigned long) 0x10)

//! SLA+W has been transmitted, ACK has been received
#define I2C_STAT_M_TX_SLAW_ACK                ((unsigned long) 0x18)

//! SLA+W has been transmitted, NACK has been received
#define I2C_STAT_M_TX_SLAW_NACK               ((unsigned long) 0x20)

//! Data has been transmitted, ACK has been received
#define I2C_STAT_M_TX_DAT_ACK                 ((unsigned long) 0x28)

//! Data has been transmitted, NACK has been received
#define I2C_STAT_M_TX_DAT_NACK                ((unsigned long) 0x30)

//! Arbitration lost in SLA+R/W or Data bytes
#define I2C_STAT_M_TX_ARB_LOST                ((unsigned long) 0x38)

/*------------------------Master receive mode----------------------*/
//! A start condition has been transmitted
#define I2C_STAT_M_RX_START                   ((unsigned long) 0x08)

//! A repeat start condition has been transmitted
#define I2C_STAT_M_RX_RESTART                 ((unsigned long) 0x10)

//! Arbitration lost
#define I2C_STAT_M_RX_ARB_LOST                ((unsigned long) 0x38)

//! SLA+R has been transmitted, ACK has been received
#define I2C_STAT_M_RX_SLAR_ACK                ((unsigned long) 0x40)

//! SLA+R has been transmitted, NACK has been received
#define I2C_STAT_M_RX_SLAR_NACK               ((unsigned long) 0x48)

//! Data has been received, ACK has been returned
#define I2C_STAT_M_RX_DAT_ACK                 ((unsigned long) 0x50)

//! Data has been received, NACK has been return
#define I2C_STAT_M_RX_DAT_NACK                ((unsigned long) 0x58)

/*-----------------------Slave receive mode------------------------*/
//! Own slave address has been received, ACK has been returned
#define I2C_STAT_S_RX_SLAW_ACK                ((unsigned long) 0x60)

//! Arbitration lost in SLA+R/W as master
#define I2C_STAT_S_RX_ARB_LOST_M_SLA          ((unsigned long) 0x68)

//! General call address has been received, ACK has been returned
#define I2C_STAT_S_RX_GENCALL_ACK             ((unsigned long) 0x70)

//! Arbitration lost in SLA+R/W (GENERAL CALL) as master
#define I2C_STAT_S_RX_ARB_LOST_M_GENCALL      ((unsigned long) 0x78)

//! Previously addressed with own SLV address;
//! Data has been received, ACK has been return
#define I2C_STAT_S_RX_PRE_SLA_DAT_ACK         ((unsigned long) 0x80)

//! Previously addressed with own SLA;
//! Data has been received and NOT ACK has been return
#define I2C_STAT_S_RX_PRE_SLA_DAT_NACK        ((unsigned long) 0x88)

//! Previously addressed with General Call;
//! Data has been received and ACK has been return
#define I2C_STAT_S_RX_PRE_GENCALL_DAT_ACK     ((unsigned long) 0x90)

//! Previously addressed with General Call;
//! Data has been received and NOT ACK has been return
#define I2C_STAT_S_RX_PRE_GENCALL_DAT_NACK    ((unsigned long) 0x98)

//! A STOP condition or repeated START condition has been received while still
//! addressed as SLV/REC (Slave Receive) or SLV/TRX (Slave Transmit)
#define I2C_STAT_S_RX_STA_STO_SLVREC_SLVTRX   ((unsigned long) 0xA0)

/*------------------------Slave transmit mode-------------------------*/
//! Own SLA+R has been received, ACK has been returned
#define I2C_STAT_S_TX_SLAR_ACK                ((unsigned long) 0xA8)

//! Arbitration lost in SLA+R/W as master
#define I2C_STAT_S_TX_ARB_LOST_M_SLA          ((unsigned long) 0xB0)

//! Data has been transmitted, ACK has been received
#define I2C_STAT_S_TX_DAT_ACK                 ((unsigned long) 0xB8)

//! Data has been transmitted, NACK has been received
#define I2C_STAT_S_TX_DAT_NACK                ((unsigned long) 0xC0)

//! Last data byte in I2DAT has been transmitted (AA = 0); ACK has been received
#define I2C_STAT_S_TX_LAST_DAT_ACK            ((unsigned long) 0xC8)


extern unsigned long I2CIntCallbackInit(unsigned long ulBase, xtEventCallback pfnCallback);
extern void I2CMasterInit(unsigned long ulBase, unsigned long TargetClk);
extern void I2CSlaveInit(unsigned long ulBase, unsigned long ulSlaveAddr, unsigned long ulGeneralCall);;
extern void I2CEnable(unsigned long ulBase);
extern void I2CDisable(unsigned long ulBase);
extern void I2CStartSend(unsigned long ulBase);
extern void I2CStopSend(unsigned long ulBase);
extern void I2CGeneralCallEnable(unsigned long ulBase, unsigned long ulID);
extern void I2CGeneralCallDisable(unsigned long ulBase, unsigned long ulID);
extern void I2CSlaveAddrSet(unsigned long ulBase, unsigned long ulID, unsigned long ulVal);
extern unsigned long I2CDataRead(unsigned long ulBase);
extern void I2CDataWrite(unsigned long ulBase, unsigned long ulValue);
extern unsigned long I2CStatusGet(unsigned long ulBase);
extern void I2CIntEnable(unsigned long ulBase);
extern void I2CIntDisable(unsigned long ulBase);
extern void I2CMasterWriteRequestS1(unsigned long ulBase, unsigned long ucSlaveAddr, unsigned long ucData, xtBoolean bEndTransmition);
extern void I2CMasterWriteRequestS2(unsigned long ulBase, unsigned long ucData, xtBoolean bEndTransmition);
extern void I2CMasterWriteS1(unsigned long ulBase, unsigned long ucSlaveAddr, unsigned long ucData, xtBoolean bEndTransmition);
extern void I2CMasterWriteS2(unsigned long ulBase, unsigned long ucData, xtBoolean bEndTransmition);
extern void I2CMasterWriteBufS1(unsigned long ulBase, unsigned long ucSlaveAddr, unsigned char *pucDataBuf, unsigned long ulLen, xtBoolean bEndTransmition);
extern void I2CMasterWriteBufS2(unsigned long ulBase, unsigned char *pucDataBuf, unsigned long ulLen, xtBoolean bEndTransmition);
externunsigned long I2CMasterReadRequestS1(unsigned long ulBase, unsigned long ucSlaveAddr, xtBoolean bEndTransmition);
extern unsigned long I2CMasterReadRequestS2(unsigned long ulBase, xtBoolean bEndTransmition);
extern unsigned long I2CMasterReadLastRequestS2(unsigned long ulBase);
extern unsigned long I2CMasterReadS1(unsigned long ulBase, unsigned long ucSlaveAddr, unsigned char *pucData, xtBoolean bEndTransmition);
extern unsigned long I2CMasterReadS2(unsigned long ulBase, unsigned char *pucData, xtBoolean bEndTransmition);
extern unsigned long  I2CMasterReadBufS1(unsigned long ulBase, unsigned long ucSlaveAddr, unsigned char *pucDataBuf, unsigned long ulLen, xtBoolean bEndTransmition);
extern unsigned long I2CMasterReadBufS2(unsigned long ulBase, unsigned char *pucDataBuf, unsigned long ulLen, xtBoolean bEndTransmition);
extern xtBoolean I2CBusBusyStatus(unsigned long ulBase);

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
//}
#endif

#endif // __xI2C_H__
