//*****************************************************************************
//
//! \file xadc.h
//! \brief Defines and Macros for ADC API.
//! \version V2.2.1.0
//! \date 6/1/2012
//! \author CooCox
//! \copy
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
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#ifndef __xADC_H__
#define __xADC_H__

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
//! \addtogroup ADC
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xADC
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xADC_Data_Resolution xADC Data Resolution
//!
//! \brief      Show the ADC Data Resolution (Data Mask, Data Length)
//!
//! \note       For LPC17xx ADC Module is 12 bit successive approximation analog
//!             to digital converter.
//! @{
//
//*****************************************************************************

//
//! Channel n Data Mask
//
#define xADC_DATA_MASK          0x00000FFF

//
//! Channel n Data Bit Length
//
#define xADC_DATA_BIT_SIZE      12

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xADC_Ints xADC Interrupt Source
//!
//!             They can be passed to xADCIntEnable(), xADCIntDisable() as the
//!             ulIntFlags parameter to enable/disable the corresponding interrupt.
//!
//! \section    xADC_Ints_Sec_Port CoX Port Details
//!
//! \verbatim
//!     +-------------------------+----------------+------------------+
//!     | xADC Interrupt Source   |       CoX      |     LPC17xx      |
//!     |-------------------------|----------------|------------------|
//!     | xADC_INT_END_CONVERSION |    Mandatory   |         Y        |
//!     |-------------------------|----------------|------------------|
//!     | xADC_INT_COMP           |  Non-Mandatory |         N        |
//!     |-------------------------|----------------|------------------|
//! \endverbatim
//!    
//! @{
//
//*****************************************************************************

//
//! Interrupt occurs when convert finish.
//
#define xADC_INT_END_CONVERSION (ADC_CH_0 | ADC_CH_1 | ADC_CH_2               \
                                 ADC_CH_3 | ADC_CH_4 | ADC_CH_5 | ADC_CH_6)

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xADC_Events xADC Interrupt Event
//!
//!             Users use the event ID in the interrupt callback function
//!             (\ref xtEventCallback) as \b ulEvent parameter to determine
//!             which interrupt event occurs. 
//!
//! \verbatim
//!     +---------------------------+---------------+---------------------+
//!     | xADC Event                |      CoX      |     LPC17xx         |
//!     |---------------------------|---------------|---------------------|
//!     | xADC_EVENT_END_CONVERSION |   Mandatory   |         Y           |
//!     |---------------------------|---------------|---------------------|
//!     | xADC_EVENT_COMPx          | Non-Mandatory |         N           |
//!     |---------------------------|---------------|---------------------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! The adc conversion end event
//
#define xADC_EVENT_END_CONVERSION      BIT_32_0
                               

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xADC_Mode xADC Operation Mode
//!
//! \brief      Values that show the ADC Operation Mode.
//!
//!             They can be passed to xADCConfigure() as the ulMode parameter
//!             to configure the ADC.
//!
//! \section    xADC_Mode_Sec_Port CoX Port Details
//! \verbatim
//! +-----------------------------+---------------+-------------------+
//! | xADC Mode                   |      CoX      |      LPC17xx      |
//! |-----------------------------|---------------|-------------------|
//! | xADC_MODE_SCAN_SINGLE_CYCLE |   Mandatory   |         Y         |
//! |-----------------------------|---------------|-------------------|
//! | xADC_MODE_SCAN_CONTINUOUS   | Non-Mandatory |         Y         |
//! |-----------------------------|---------------|-------------------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Single sample mode.
//
#define xADC_MODE_SCAN_SINGLE_CYCLE    BIT_32_0

//
//! Continuous scan mode.
//
#define xADC_MODE_SCAN_CONTINUOUS      BIT_32_1
                               
//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xADC_Trigger_Source xADC Trigger Source
//!
//! \brief      values that show the ADC trigger source.
//!
//!             Can be passed to xADCConfigure() as the ulTrigger parameter to
//!             configure the ADC.
//!
//! \verbatim
//! +-----------------------------+----------------+---------------------------+
//! | xADC Trigger Source         |       CoX      |        LPC17xx            |
//! |-----------------------------|----------------|---------------------------|
//! | xADC_TRIGGER_PROCESSOR      |    Mandatory   |            Y              |
//! |-----------------------------|----------------|---------------------------|
//! | xADC_TRIGGER_EXT_$ShortPin$ |  Non-Mandatory |  xADC_TRIGGER_EXT_n       |
//! |-----------------------------|----------------|---------------------------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Processor trigger (Software)
//
#define xADC_TRIGGER_PROCESSOR         BIT_32_0

//
//! External int0 pin trigger.
//
#define xADC_TRIGGER_EXT_EINT0         BIT_32_1

//
//! External cap01 pin trigger.
//
#define xADC_TRIGGER_EXT_CAP01         BIT_32_2

//
//! External mat01 pin trigger.
//
#define xADC_TRIGGER_EXT_MAT01         BIT_32_3

//
//! External mat03 pin trigger.
//
#define xADC_TRIGGER_EXT_MAT03         BIT_32_4

//
//! External mat10 pin trigger.
//
#define xADC_TRIGGER_EXT_MAT10         BIT_32_5

//
//! External mat11 pin trigger.
//
#define xADC_TRIGGER_EXT_MAT11         BIT_32_6

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xADC_EXT_Trigger_Mode xADC External Tigger Source Mode
//!
//!             Can be passed to the ADCConfigure() as the ulTrigger
//!             parameter, when the trigger source set as xADC_TRIGGER_EXT_$ShortPin$.
//!             The ulTrigger parameter is an OR vaule with the \ref xADC_Trigger_Source.
//!             such as  \ref xADC_TRIGGER_EXT_PD2 | \ref xADC_TRIGGER_EXT_RISING_EDGE.
//!
//! \section    xADC_EXT_Trigger_Mode_Sec_Port CoX Port Details
//!
//! \verbatim
//! +-------------------------------+----------------+------------------+
//! | xADC Ext Trigger Mode         |       CoX      |     LPC17xx      |
//! |-------------------------------|----------------|------------------|
//! | xADC_TRIGGER_EXT_LOW_LEVEL    |  Non-Mandatory |         N        |
//! |-------------------------------|----------------|------------------|
//! | xADC_TRIGGER_EXT_HIGH_LEVEL   |  Non-Mandatory |         N        |
//! |-------------------------------|----------------|------------------|
//! | xADC_TRIGGER_EXT_RISING_EDGE  |  Non-Mandatory |         Y        |
//! |-------------------------------|----------------|------------------|
//! | xADC_TRIGGER_EXT_FALLING_EDGE |  Non-Mandatory |         Y        |
//! |-------------------------------|----------------|------------------|
//! | xADC_TRIGGER_EXT_BOTH_EDGE    |  Non-Mandatory |         N        |
//! |-------------------------------|----------------|------------------|
//! \endverbatim 
//! @{
//
//*****************************************************************************

//
//! EXT Tigger is rising edge.
//
#define xADC_TRIGGER_EXT_RISING_EDGE   BIT_32_0
                            
//
//! EXT Tigger is falling edge.
//
#define xADC_TRIGGER_EXT_FALLING_EDGE  BIT_32_1

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xADC_Step_Config xADC Step Config
//! \brief      ADC step config values, such as single-end / difference,
//!             channel source.
//!
//!             They are can be passed to the xADCStepConfigure() as the ulConfig
//!             parameter.
//! 
//! \section    xADC_Step_Config_Sec_Divide xADC Step Config Divide
//!             The Step Config value is an logic OR type of the following config:
//!
//!             - Channel source select
//!               like \b xADC_CTL_$CHx$, such as \ref xADC_CTL_CH0.  
//!               Or \ref xADC_CTL_TS to select the temperature sensor.
//!
//!             - Comparator select to monitor the convertion value
//!               like \b xADC_CTL_$CMPx$, such as \ref xADC_CTL_CMP0.
//!
//!             - single-end/difference input mode
//!               \b xADC_CTL_D shows difference input mode, and 0 shows single-end mode.
//!
//!             - \ref xADC_CTL_END shows this is the end step.
//! .
//! \section xADC_Step_Config_Sec_Port CoX Port Details.
//! \verbatim
//! +-------------------+-----------------+-----------------------+
//! |  xADC Step Config |       CoX       |        LPC17xx        |
//! |-------------------|-----------------|-----------------------|
//! |  xADC_CTL_CH$x$   |  Non-Mandatory  |     xADC_CTL_CH0      |
//! |                   |                 |     xADC_CTL_CH1      |
//! |                   |                 |     xADC_CTL_CH2      |
//! |                   |                 |     xADC_CTL_CH3      |
//! |                   |                 |     xADC_CTL_CH4      |
//! |                   |                 |     xADC_CTL_CH5      |
//! |                   |                 |     xADC_CTL_CH6      |
//! |                   |                 |     xADC_CTL_CH7      |
//! |-------------------|-----------------|-----------------------|
//! |  xADC_CTL_TS      |  Non-Mandatory  |           N           |
//! |-------------------|-----------------|-----------------------|
//! |  xADC_CTL_CMP$x$  |  Non-Mandatory  |           N           |
//! |-------------------|-----------------|-----------------------|
//! |  xADC_CTL_D       |  Non-Mandatory  |           N           |
//! |-------------------|-----------------|-----------------------|
//! |  xADC_CTL_END     |  Non-Mandatory  |           N           |
//! |-------------------|-----------------|-----------------------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//
//! Input channel 0
//
#define xADC_CTL_CH0                   ADC_CH_0

//
//! Input channel 1
//
#define xADC_CTL_CH1                   ADC_CH_0        

//
//! Input channel 2
//
#define xADC_CTL_CH2                   ADC_CH_2

//
//! Input channel 3
//
#define xADC_CTL_CH3                   ADC_CH_3

//
//! Input channel 4
//
#define xADC_CTL_CH4                   ADC_CH_4

//
//! Input channel 5
//
#define xADC_CTL_CH5                   ADC_CH_5

//
//! Input channel 6
//
#define xADC_CTL_CH6                   ADC_CH_6

//
//! Input channel 7
//
#define xADC_CTL_CH7                   ADC_CH_7

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup xADC_Exported_APIs xADC API
//! \brief      xADC API Reference
//!
//! \section xADC_Exported_APIs_Port CoX Port Details
//! \verbatim
//! +--------------------------+----------------+------------------+
//! |  xADC API                |       CoX      |     LPC17xx      |
//! |--------------------------|----------------|------------------|
//! |  xADCConfigure           |    Mandatory   |         Y        |
//! |--------------------------|----------------|------------------|
//! |  xADCStepConfigure       |    Mandatory   |         Y        |
//! |--------------------------|----------------|------------------|
//! |  xADCIntCallbackInit     |    Mandatory   |         Y        |
//! |--------------------------|----------------|------------------|
//! |  xADCIntEnable           |    Mandatory   |         Y        |
//! |--------------------------|----------------|------------------|
//! |  xADCIntDisable          |    Mandatory   |         Y        |
//! |--------------------------|----------------|------------------|
//! |  xADCEnable              |    Mandatory   |         Y        |
//! |--------------------------|----------------|------------------|
//! |  xADCDisable             |    Mandatory   |         Y        |
//! |--------------------------|----------------|------------------|
//! |  xADCProcessorTrigger    |    Mandatory   |         Y        |
//! |--------------------------|----------------|------------------|
//! |  xADCDataGet             |    Mandatory   |         Y        |
//! |--------------------------|----------------|------------------|
//! |  xADCOverflow            |    Mandatory   |         Y        |
//! |--------------------------|----------------|------------------|
//! |  xADCOverflowClear       |    Mandatory   |         Y        |
//! |--------------------------|----------------|------------------|
//! \endverbatim
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief  Configure ADC Module.
//!         
//!         This function configures ADC convert mode(single/continue scan mode)
//!         and triggle mode(soft/hardware triggler).
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be
//!              \ref xADC0_BASE.
//!
//! \param  [in] ulMode is ADC scan mode, can be one of the following value:
//!              \ref xADC_MODE_SCAN_SINGLE_CYCLE
//!              \ref xADC_MODE_SCAN_CONTINUOUS
//!
//! \param  [in] ulTrigger is ADC triggle source select. For LPC17xx, it can be
//!              one of the following value:
//!              \ref xADC_TRIGGER_PROCESSOR         
//!              \ref xADC_TRIGGER_EXT_EINT0        
//!              \ref xADC_TRIGGER_EXT_CAP01       
//!              \ref xADC_TRIGGER_EXT_MAT01      
//!              \ref xADC_TRIGGER_EXT_MAT03     
//!              \ref xADC_TRIGGER_EXT_MAT10    
//!              \ref xADC_TRIGGER_EXT_MAT11   
//!
//! \return None.
//!
//! \note   This function must be used with \ref xADCStepConfigure.
//
//*****************************************************************************
#define xADCConfigure(ulBase, ulMode, ulTrigger)                              \
         ADCConfigure(ulBase, ulMode, ulTrigger)

//*****************************************************************************
//
//! \brief  Configure ADC convert array.
//!
//!         This function is used to fill 
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be \ref xADC0_BASE.
//!
//! \param  [in] ulStep is the ID of ADC converter channel array, which can be
//!              used in continue convert mode.
//!              0 <= ulStep <=7.
//!
//! \param  [in] ulConfig is ADC channerl source, which can be one of the
//!              following value:
//!              \ref xADC_CTL_CH0
//!              \ref xADC_CTL_CH1
//!              \ref xADC_CTL_CH2
//!              \ref xADC_CTL_CH3
//!              \ref xADC_CTL_CH4
//!              \ref xADC_CTL_CH5
//!              \ref xADC_CTL_CH6
//!              \ref xADC_CTL_CH7
//!
//! \return None.
//!
//! \note   For LPC17xx, The relationship between ADC Convert array is fixed.
//!             Array[0] --> xADC_CTL_CH0
//!             Array[1] --> xADC_CTL_CH1
//!             ...
//!             Array[7] --> xADC_CTL_CH7
//!         so, you can use this function to configure ADC continue scan mode
//!         or single sample mode.
//!         for example:
//!         ---------------------------------------------------------------
//!         | Correct --> xADCStepConfigure(xADC0_BASE, 0, xADC_CTL_CH0); |
//!         | Wrong   --> xADCStepConfigure(xADC0_BASE, 1, xADC_CTL_CH0); |
//!         ---------------------------------------------------------------
//
//*****************************************************************************
#define xADCStepConfigure(ulBase, ulStep, ulConfig)                           \
         ADCStepConfigure(ulBase, ulStep, ulConfig)

//*****************************************************************************
//
//! \brief  Register user interrupt callback function for the ADC timer.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be \ref xADC0_BASE.
//!
//! \param  [in] xtPortCallback is callback for the Analog digital converter.
//!
//! \return None.
//
//*****************************************************************************     
#define xADCIntCallbackInit(ulBase, xtPortCallback)                           \
         ADCIntCallbackInit(ulBase, xtPortCallback)

//*****************************************************************************
//
//! \brief  Enable individual ADC interrupt sources.
//!         Enables the indicated ADC interrupt sources.  Only the sources that
//!         are enabled can be reflected to the processor interrupt; disabled
//!         sources have no effect on the processor.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be \ref xADC0_BASE.
//!
//! \param  [in] ulIntFlags is the bit mask of the interrupt sources to be enabled.
//!              This \ref xADC_INT_END_CONVERSION
//!
//! \return None.
//
//*****************************************************************************
#define xADCIntEnable(ulBase, ulIntFlags)                                     \
         ADCIntEnable(ulBase, ulIntFlags)

//*****************************************************************************
//
//! \brief  Disable ADC sample sequence interrupts.
//!         This function disables the requested ADC interrupts.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be \ref xADC0_BASE.
//!
//! \param  ulIntFlags is the interrupt flags.
//!
//! \return None.
//
//*****************************************************************************
#define xADCIntDisable(ulBase, ulIntFlags)                                    \
         ADCIntDisable(ulBase, ulIntFlags)

//*****************************************************************************
//
//! \brief  Enable ADC sample sequence.
//!         Allows the specified sample sequence to be captured when its trigger
//!         is detected. A sample sequence must be configured before it is enabled.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be \ref xADC0_BASE.
//!
//! \return None.
//
//*****************************************************************************
extern void xADCEnable(unsigned long ulBase);

//*****************************************************************************
//
//! \brief  Disable the ADC sample sequence.
//!         Prevents the sample sequence from being captured when its trigger is
//!         detected. The sample sequence should be disabled before it is configured.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be \ref xADC0_BASE.
//!
//! \return None.
//
//*****************************************************************************
#define xADCDisable(ulBase)                                                   \
         ADCStop(ulBase)

//*****************************************************************************
//
//! \brief  Cause a processor trigger for a sample sequence.
//!         This function triggers a processor-initiated sample sequence if the
//!         sample sequence trigger is configured to \b xADC_TRIGGER_PROCESSOR. 
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be \ref xADC0_BASE.
//!
//! \return None.
//
//*****************************************************************************
#define xADCProcessorTrigger(ulBase)                                          \
        xADCEnable(ulBase)

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief  Get the captured data from a sample sequence.
//!
//!         This function copies data from the specified sample sequence
//!         FIFO to a memory resident buffer.  The number of samples
//!         available in the FIFO are copied into the buffer, which is
//!         assumed to be large enough to hold that many samples.
//!         This will only return the samples that are presently available,
//!         which may not be the entire sample sequence if it is in the
//!         process of being executed.
//!
//! \param  [in] ulBase is the base address of the ADC module.
//!              This value must be \ref xADC0_BASE.
//!
//! \param  [in] pulBuffer is the address where the data is stored.
//!
//! \return The number of sample channel, if return 0, indicate that some channel
//!         is converting.
//!
//
//*****************************************************************************
extern unsigned long xADCDataGet(unsigned long ulBase, unsigned long * pulBuffer);

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup  LPC17xx _ADC
//! @{
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup  LPC17xx _ADC_Dual_Type  LPC17xx  Dual ADC Mode
//! \brief Values that show the Dual ADC Mode.
//! @{
//
//*****************************************************************************


//! \todo wait to comment this macros
//! Note by cedar

#define ADC_CH_M                       BIT_MASK(32, 6, 0)

#define ADC_CH_0                       BIT_32_0
#define ADC_CH_1                       BIT_32_1
#define ADC_CH_2                       BIT_32_2
#define ADC_CH_3                       BIT_32_3
#define ADC_CH_4                       BIT_32_4
#define ADC_CH_5                       BIT_32_5
#define ADC_CH_6                       BIT_32_6

//! Start burst mode
#define ADC_START_MODE_BURST           CR_BURST

//! Start conversion now
#define ADC_START_MODE_NOW             CR_START_NOW

//! Start conversion when the edge selected by bit 27 occurs on P2.10/EINT0
#define ADC_START_MODE_EINT0           CR_START_EINT0             

//! Start conversion when the edge selected by bit 27 occurs on P1.27/CAP0.1
#define ADC_START_MODE_CAP01           CR_START_CAP01             

//! Start conversion when the edge selected by bit 27 occurs on MAT0.1
#define ADC_START_MODE_MAT01           CR_START_MAT01             

//! Start conversion when the edge selected by bit 27 occurs on MAT0.3
#define ADC_START_MODE_MAT03           CR_START_MAT03             

//! Start conversion when the edge selected by bit 27 occurs on MAT1.0
#define ADC_START_MODE_MAT10           CR_START_MAT10             

//! Start conversion when the edge selected by bit 27 occurs on MAT1.1
#define ADC_START_MODE_MAT11           CR_START_MAT11             

//! ADC channel convert done
#define ADC_DONE                       BIT_32_0

//! ADC channel convert overrun
#define ADC_OVERRUN                    BIT_32_1


//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup  LPC17xx _ADC_Exported_APIs  LPC17xx  ADC API
//! \brief  LPC17xx  ADC API Reference
//! @{
//
//*****************************************************************************

extern void ADCInit(unsigned long ulBase, unsigned long ulRate);

extern void ADCStart(unsigned long ulBase, unsigned long ulChs, unsigned long ulMode);
extern void ADCStop(unsigned long ulBase);
//pass
extern void ADCIntEnable(unsigned long ulBase, unsigned long ulChs);
//pass
extern void ADCIntDisable(unsigned long ulBase, unsigned long ulChs);

extern unsigned long ADCStatusGet(unsigned long ulBase, unsigned long ulChs);
extern xtBoolean ADCStatusCheck(unsigned long ulBase, unsigned long ulChs, unsigned long ulFlags);
extern unsigned long ADCDataRead(unsigned long ulBase, unsigned long ulCh);

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

#endif // __xADC_H__

