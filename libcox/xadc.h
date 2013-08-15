
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




extern void ADCInit(unsigned long ulBase, unsigned long ulRate)
;

extern void ADCStart(unsigned long ulBase, unsigned long ulChs, unsigned long ulMode);
extern void ADCStop(unsigned long ulBase);
//pass
extern void ADCIntEnable(unsigned long ulBase, unsigned long ulChs);
//pass
extern void ADCIntDisable(unsigned long ulBase, unsigned long ulChs);

extern unsigned long ADCStatusGet(unsigned long ulBase, unsigned long ulChs);
extern xtBoolean ADCStatusCheck(unsigned long ulBase, unsigned long ulChs, unsigned long ulFlags);
extern unsigned long ADCDataRead(unsigned long ulBase, unsigned long ulCh);


