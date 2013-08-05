//! Interrupt Register.
//! The IR can be written to clear interrupts.The IR can be
//! read to identify which of eight possible interrupt sources are pending.
#define PWM_IR 0x00000000

//! Timer Control Register.
//! The TCR is used to control the Timer Counter functions.
//! The Timer Counter can be disabled or reset through the TCR.
#define PWM_TCR 0x00000004

//! Timer Counter.
//! The 32-bit TC is incremented every PR+1 cycles of PCLK.
//! The TC is controlled through the TCR.
#define PWM_TC 0x00000008

//! Prescale Register.
//! The TC is incremented every PR+1 cycles of PCLK.
#define PWM_PR 0x0000000C

//! Prescale Counter.
//! The 32-bit PC is a counter which is incremented to the value stored in PR.
//! When the value in PR is reached, the TC is incremented.
//! The PC is observable and controllable through the bus interface.
#define PWM_PC 0x00000010

//! Match Control Register.
//! The MCR is used to control if an interrupt is generated and if the TC is
//! reset when a Match occurs.
#define PWM_MCR 0x00000014

//! Match Register 0.
//! MR0 can be enabled in the MCR to reset the TC, stop
//! both the TC and PC, and/or generate an interrupt when it matches the TC.
//! In addition, a match between this value and the TC sets any PWM output
//! that is in single-edge mode, and sets PWM1 if it¡¯s in double-edge mode.
#define PWM_MR0 0x00000018

//! Match Register 1.
//! MR1 can be enabled in the MCR to reset the TC, stop both the TC and PC,
//! and/or generate an interrupt when it matches the TC.In addition, a match
//! between this value and the TC clears PWM1 in either edge mode, and sets
//! PWM2 if it¡¯s in double-edge mode.
#define PWM_MR1 0x0000001C

//! Match Register 2.
//! MR2 can be enabled in the MCR to reset the TC, stop both the TC and PC,
//! and/or generate an interrupt when it matches the TC.
//! In addition, a match between this value and the TC clears PWM2 in either
//! edge mode, and sets PWM3 if it¡¯s in double-edge mode.
#define PWM_MR2 0x00000020

//! Match Register 3.
//! MR3 can be enabled in the MCR to reset the TC, stop both the TC and PC,
//! and/or generate an interrupt when it matches the TC.
//! In addition, a match between this value and the TC clears PWM3 in either
//! edge mode, and sets PWM4 if it¡¯s in double-edge mode.
#define PWM_MR3 0x00000024

//! Capture Control Register.
//! The CCR controls which edges of the capture inputs are used to load the Capture
//! Registers and whether or not an interrupt is generated when a capture takes place.
#define PWM_CCR 0x00000028

//! CR0 Capture Register 0.
//! CR0 is loaded with the value of the TC when there is an event on the CAPn.0 input.
#define PWM_CR0 0x0000002C

//! Capture Register 1.
#define PWM_CR1 0x00000030

//! Capture Register 2.
#define PWM_CR2 0x00000034

//! Capture Register 3.
#define PWM_CR3 0x00000038

//! Match Register 4.
//! MR4 can be enabled in the MCR to reset the TC, stop both the TC and PC,
//! and/or generate an interrupt when it matches the TC.In addition, a match
//! between this value and the TC clears PWM4 in either edge mode, and sets
//! PWM5 if it¡¯s in double-edge mode.
#define PWM_MR4 0x00000040

//! Match Register 5.
//! MR5 can be enabled in the MCR to reset the TC, stop both the TC and PC,
//! and/or generate an interrupt when it matches the TC.In addition, a match
//! between this value and the TC clears PWM5 in either edge mode, and sets
//! PWM6 if it¡¯s in double-edge mode.
#define PWM_MR5 0x00000044

//! Match Register 6.
//! MR6 can be enabled in the MCR to reset the TC, stop both the TC and PC,
//! and/or generate an interrupt when it matches the TC.In addition, a match
//! between this value and the TC clears PWM6 in either edge mode.
#define PWM_MR6 0x00000048

//! PWM Control Register.
//! Enables PWM outputs and selects PWM channel types as either single edge or
//! double edge controlled.
#define PWM_PCR 0x0000004C

//! Load Enable Register.
//! Enables use of new PWM match values.
#define PWM_LER 0x00000050

//! Count Control Register.
//! The CTCR selects between Timer and Counter mode, and in Counter mode selects
//! the signal and edge(s) for counting.
#define PWM_CTCR 0x00000070


//! PWM_IR {{

//! PWM Match interrupt channel 0.
#define IR_MR_0                 BIT_32_0

//! PWM Match interrupt channel 1.
#define IR_MR_1                 BIT_32_1

//! PWM Match interrupt channel 2.
#define IR_MR_2                 BIT_32_2

//! PWM Match interrupt channel 3.
#define IR_MR_3                 BIT_32_3

//! PWM Capture input interrupt 0.
#define IR_CAP0                 BIT_32_4

//! PWM Capture input interrupt 1.
#define IR_CAP1                 BIT_32_5

//! PWM Match interrupt channel 4.
#define IR_MR_4                 BIT_32_8

//! PWM Match interrupt channel 5.
#define IR_MR_5                 BIT_32_9

//! PWM Match interrupt channel 6.
#define IR_MR_6                 BIT_32_10

//! PWM_IR }}

//! PWM_TCR {{

//! Enable PWM Timer counter and prescale counter.
#define TCR_CNT_EN              BIT_32_0

//! Reset PWM Timer counter and prescale counter.
#define TCR_CNT_RST             BIT_32_1

//! Enable PWM Mode.
#define TCR_PWM_EN              BIT_32_3

//! PWM_TCR }}


//! PWM_CTCR {{

//! Mode Configure.
#define CTCR_MODE_M             BIT_MASK(32, 1, 0)
#define CTCR_MODE_S             0

//! Count input select.
#define CTCR_CIS_M              BIT_MASK(32, 3, 2)
#define CTCR_CIS_S              2


//! PWM_CTCR }}

//! PWM_MCR {{

//! Interrupt on PWMMR0.
#define MCR_PWMMR0I             BIT_32_0

//! Reset on PWMMR0.
#define MCR_PWMMR0R             BIT_32_1

//! Stop on PWMMR0.
#define MCR_PWMMR0S             BIT_32_2

//! Interrupt on PWMMR1.
#define MCR_PWMMR1I             BIT_32_3

//! Reset on PWMMR1.
#define MCR_PWMMR1R             BIT_32_4

//! Stop on PWMMR1.
#define MCR_PWMMR1S             BIT_32_5

//! Interrupt on PWMMR2.
#define MCR_PWMMR2I             BIT_32_6

//! Reset on PWMMR2.
#define MCR_PWMMR2R             BIT_32_7

//! Stop on PWMMR2.
#define MCR_PWMMR2S             BIT_32_8

//! Interrupt on PWMMR3.
#define MCR_PWMMR3I             BIT_32_9

//! Reset on PWMMR3.
#define MCR_PWMMR3R             BIT_32_10

//! Stop on PWMMR3.
#define MCR_PWMMR3S             BIT_32_11

//! Interrupt on PWMMR4.
#define MCR_PWMMR4I             BIT_32_12

//! Reset on PWMMR4.
#define MCR_PWMMR4R             BIT_32_13

//! Stop on PWMMR4.
#define MCR_PWMMR4S             BIT_32_14

//! Interrupt on PWMMR5.
#define MCR_PWMMR5I             BIT_32_15

//! Reset on PWMMR5.
#define MCR_PWMMR5R             BIT_32_16

//! Stop on PWMMR5.
#define MCR_PWMMR5S             BIT_32_17

//! Interrupt on PWMMR6.
#define MCR_PWMMR6I             BIT_32_18

//! Reset on PWMMR6.
#define MCR_PWMMR6R             BIT_32_19

//! Stop on PWMMR6.
#define MCR_PWMMR6S             BIT_32_20

//! PWM_MCR }}

//! PWM_CCR {{

//! Rising edge capture.
#define CCR_CAP0_RISING         BIT_32_0

//! Falling edge capture.
#define CCR_CAP0_FALLING        BIT_32_1

//! Interrupt on CAP 0 Event.
#define CCR_CAP0_INT_EN         BIT_32_2

//! Rising edge capture.
#define CCR_CAP1_RISING         BIT_32_3

//! Falling edge capture.
#define CCR_CAP1_FALLING        BIT_32_4

//! Interrupt on CAP Event.
#define CCR_CAP1_INT_EN         BIT_32_5

//! PWM_CCR }}

//! PWM_PCR {{

//! Edge control mode select for PWM2.
#define PCR_SEL2                BIT_32_2

//! Edge control mode select for PWM3.
#define PCR_SEL3                BIT_32_3

//! Edge control mode select for PWM4.
#define PCR_SEL4                BIT_32_4

//! Edge control mode select for PWM5.
#define PCR_SEL5                BIT_32_5

//! Edge control mode select for PWM6.
#define PCR_SEL6                BIT_32_6

//! PWM1 output Enable
#define PCR_SEL1                BIT_32_9

//! PWM2 output Enable
#define PCR_SEL2                BIT_32_10

//! PWM3 output Enable
#define PCR_SEL3                BIT_32_11

//! PWM4 output Enable
#define PCR_SEL4                BIT_32_12

//! PWM5 output Enable
#define PCR_SEL5                BIT_32_13

//! PWM6 output Enable
#define PCR_SEL6                BIT_32_14

//! PWM_PCR }}

//! PWM_LER {{

//! Enable PWM0 Match Latch.
#define LER_MATCH0_LAT_EN       BIT_32_0

//! Enable PWM1 Match Latch.
#define LER_MATCH1_LAT_EN       BIT_32_1

//! Enable PWM2 Match Latch.
#define LER_MATCH2_LAT_EN       BIT_32_2

//! Enable PWM3 Match Latch.
#define LER_MATCH3_LAT_EN       BIT_32_3

//! Enable PWM4 Match Latch.
#define LER_MATCH4_LAT_EN       BIT_32_4

//! Enable PWM5 Match Latch.
#define LER_MATCH5_LAT_EN       BIT_32_5

//! Enable PWM6 Match Latch.
#define LER_MATCH6_LAT_EN       BIT_32_6

//! PWM_LER }}
