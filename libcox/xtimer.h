
#define TIMER_MAT_CH_0          BIT_32_0 
#define TIMER_MAT_CH_1          BIT_32_1 
#define TIMER_MAT_CH_2          BIT_32_2 
#define TIMER_MAT_CH_3          BIT_32_3 

#define TIMER_CAP_CH_0          BIT_32_4 
#define TIMER_CAP_CH_1          BIT_32_5 

#define TIMER_INT_MAT_CH_0      BIT_32_0 
#define TIMER_INT_MAT_CH_1      BIT_32_1 
#define TIMER_INT_MAT_CH_2      BIT_32_2 
#define TIMER_INT_MAT_CH_3      BIT_32_3 

#define TIMER_INT_CAP_CH_0      BIT_32_4 
#define TIMER_INT_CAP_CH_1      BIT_32_5 

#define TIMER_MAT_MASK          BIT_MASK(32, 2, 0)
//! An interrupt is generated when MR matches the value in the TC. 
#define TIMER_MAT_INT           BIT_32_0

//! The TC will be reset if MR matches it.
#define TIMER_MAT_RESET         BIT_32_1

//! The TC and PC will be stopped and TCR will be set to 0 if MR matches the TC.
#define TIMER_MAT_STOP          BIT_32_2

#define TIMER_MAT_PIN_MASK      BIT_MASK(32, 14, 12)
#define TIMER_MAT_PIN_NONE      BIT_32_14
#define TIMER_MAT_PIN_LOW       BIT_32_12
#define TIMER_MAT_PIN_HIGH      BIT_32_13
#define TIMER_MAT_PIN_TOGGLE    (BIT_32_13 | BIT_32_12)


extern unsigned long TimerStatusGet(unsigned long ulBase);
extern xtBoolean TimerStatusCheck(unsigned long ulBase, unsigned long ulIntFlags);
extern void TimerStatusClear(unsigned long ulBase, unsigned long ulIntFlags);
extern void TimerStart(unsigned long ulBase);
extern void TimerStop(unsigned long ulBase);
extern void TimerReset(unsigned long ulBase);
extern void TimerPrescaleSet(unsigned long ulBase, unsigned long ulValue);
extern unsigned long TimerPrescaleGet(unsigned long ulBase);
extern void TimerLoadSet(unsigned long ulBase, unsigned long ulValue);
extern unsigned long TimerLoadGet(unsigned long ulBase);
extern unsigned long TimerValueGet(unsigned long ulBase);
extern void TimerMatchCfg(unsigned long ulBase, unsigned long ulChs, unsigned long ulCfgs);
