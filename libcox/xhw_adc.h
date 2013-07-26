//! A/D Control Register.The ADCR register must be written to select the
//! operating mode before A/D conversion can occur.
#define AD_CR                           ((unsigned long)0x00000000)

//! A/D Global Data Register.This register contains the ADC��s DONE bit and the
//! result of the most recent A/D conversion.
#define AD_GDR                          ((unsigned long)0x00000004)

//! A/D Interrupt Enable Register.This register contains enable bits that allow
//! the DONE flag of each A/D channel to be included or excluded from
//! contributing to the generation of an A/D interrupt.
#define AD_INTEN                        ((unsigned long)0x0000000C)

//! A/D Channel 0 Data Register.This register contains the result of the most
//! recent conversion completed on channel 0.
#define AD_DR0                          ((unsigned long)0x00000010)

//! A/D Channel 1 Data Register.This register contains the result of the most
//! recent conversion completed on channel 1.
#define AD_DR1                          ((unsigned long)0x00000014)

//! A/D Channel 2 Data Register.This register contains the result of the most
//! recent conversion completed on channel 2.
#define AD_DR2                          ((unsigned long)0x00000018)

//! A/D Channel 3 Data Register.This register contains the result of the most
//! recent conversion completed on channel 3.
#define AD_DR3                          ((unsigned long)0x0000001C)

//! A/D Channel 4 Data Register.This register contains the result of the most
//! recent conversion completed on channel 4.
#define AD_DR4                          ((unsigned long)0x00000020)

//! A/D Channel 5 Data Register.This register contains the result of the most
//! recent conversion completed on channel 5.
#define AD_DR5                          ((unsigned long)0x00000024)

//! A/D Channel 6 Data Register.This register contains the result of the most
//! recent conversion completed on channel 6.
#define AD_DR6                          ((unsigned long)0x00000028)

//! A/D Channel 7 Data Register.This register contains the result of the most
//! recent conversion completed on channel 7.
#define AD_DR7                          ((unsigned long)0x0000002C)

//! A/D Status Register.This register contains DONE and OVERRUN flags for all of
//! the A/D channels, as well as the A/D interrupt/DMA flag.
#define AD_STAT                         ((unsigned long)0x00000030)

//! ADC trim register.
#define AD_TRM                          ((unsigned long)0x00000034)



//! AD_CR {{

//! Selects which of the AD0.7:0 pins is (are) to be sampled and converted.
#define AD_CR_SEL_M                    BIT_MASK(32, 7, 0)
#define AD_CR_SEL_S                    0

//! APB Clock divider.
#define AD_CR_CLKDIV_M                 BIT_MASK(32, 15, 8)
#define AD_CR_CLKDIV_S                 8

//! ADC Burst mode.
#define AD_CR_BURST                    BIT_32_16

//! ADC Power-down mode.
#define AD_CR_PDN                      BIT_32_21

//! Start ADC conversion.
#define AD_CR_START_M                  BIT_MASK(32, 26, 24)
#define AD_CR_START_S                  24

//! Start conversion edge.
#define AD_CR_EDGE                     BIT_32_27

//! AD_CR }}


//! AD_GDR {{

//! 
#define AD_GDR_RESULT_M                BIT_MASK(32, 15, 4)
#define AD_GDR_RESULT_S                4

//! 
#define AD_GDR_CHN_M                   BIT_MASK(32, 26, 24)
#define AD_GDR_CHN_S                   24

//! 
#define AD_GDR_OVERRUN                 BIT_32_30

//! 
#define AD_GDR_DONE                    BIT_32_31

//! AD_GDR }}

//! AD_INTEN {{

//! Enable ADC channel 0.
#define AD_INTEN_EN0                   BIT_32_0

//! Enable ADC channel 1.
#define AD_INTEN_EN1                   BIT_32_1

//! Enable ADC channel 2.
#define AD_INTEN_EN2                   BIT_32_2

//! Enable ADC channel 3.
#define AD_INTEN_EN3                   BIT_32_3

//! Enable ADC channel 4.
#define AD_INTEN_EN4                   BIT_32_4

//! Enable ADC channel 5.
#define AD_INTEN_EN5                   BIT_32_5

//! Enable ADC channel 6.
#define AD_INTEN_EN6                   BIT_32_6

//! Enable ADC channel 7.
#define AD_INTEN_EN7                   BIT_32_7

//! Enable global interrupt.
#define AD_INTEN_GEN                   BIT_32_8

//! AD_INTEN }}

//! AD_DR {{

//!
#define AD_DR_RESULT_M                 BIT_MASK(32, 15, 4)
#define AD_DR_RESULT_S                 4

//!
#define AD_DR_OVERRUN                  BIT_32_30

//!
#define AD_DR_DONE                     BIT_32_31

//! AD_DR }}

//! AD_STAT {{

//!
#define AD_STAT_DONE_0                 BIT_32_0

//!
#define AD_STAT_DONE_1                 BIT_32_1

//!
#define AD_STAT_DONE_2                 BIT_32_2

//!
#define AD_STAT_DONE_3                 BIT_32_3

//!
#define AD_STAT_DONE_4                 BIT_32_4

//!
#define AD_STAT_DONE_5                 BIT_32_5

//!
#define AD_STAT_DONE_6                 BIT_32_6

//!
#define AD_STAT_DONE_7                 BIT_32_7

//! 
#define AD_STAT_OVERRUN_0              BIT_32_8 

//! 
#define AD_STAT_OVERRUN_1              BIT_32_9 

//! 
#define AD_STAT_OVERRUN_2              BIT_32_10

//!
#define AD_STAT_OVERRUN_3              BIT_32_11

//! 
#define AD_STAT_OVERRUN_4              BIT_32_12

//!
#define AD_STAT_OVERRUN_5              BIT_32_13

//! 
#define AD_STAT_OVERRUN_6              BIT_32_14

//!
#define AD_STAT_OVERRUN_7              BIT_32_15

//! 
#define AD_STAT_ADINT                  BIT_32_16

//! AD_STAT }}

//! AD_TRIM {{

//! Offset trim bits for ADC operation.
#define AD_TRIM_COFFS_M                BIT_MASK(32, 7, 4)
#define AD_TRIM_COFFS_S                4

//! written-to by boot code.Can not be overwritten by the user.
//! These bits are locked after boot code write.
#define AD_TRIM_TRIM_M                 BIT_MASK(32, 11, 8)
#define AD_TRIM_TRIM_S                 8

//! AD_TRIM }}

