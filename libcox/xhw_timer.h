//! Interrupt Register.
//! The IR can be written to clear interrupts.The IR can be read to identify
//! which of eight possible interrupt sources are pending.
#define TIMER_IR                ((unsigned long)0x00000000)

//! Timer Control Register.
//! The TCR is used to control the Timer Counter functions.
//! The Timer Counter can be disabled or reset through the TCR.
#define TIMER_TCR               ((unsigned long)0x00000004)

//! Timer Counter.
//! The 32-bit TC is incremented every PR+1 cycles of PCLK.The TC is controlled
//! through the TCR.
#define TIMER_TC                ((unsigned long)0x00000008)

//! Prescale Register.
//! When the Prescale Counter (below) is equal to this value,
//! the next clock increments the TC and clears the PC.
#define TIMER_PR                ((unsigned long)0x0000000C)

//! Prescale Counter.
//! The 32-bit PC is a counter which is incremented to the value stored in PR.
//! When the value in PR is reached, the TC is incremented and the PC is cleared.
//! The PC is observable and controllable through the bus interface.
#define TIMER_PC                ((unsigned long)0x00000010)

//! Match Control Register.
//! The MCR is used to control if an interrupt is generated and if the TC is reset
//! when a Match occurs.
#define TIMER_MCR               ((unsigned long)0x00000014)

//! Match Register 0.
//! MR0 can be enabled through the MCR to reset the TC, stop both the TC and PC,
//! and/or generate an interrupt every time MR0 matches the TC.
#define TIMER_MR0               ((unsigned long)0x00000018)

//! Match Register 1.
#define TIMER_MR1               ((unsigned long)0x0000001C)

//! Match Register 2.
#define TIMER_MR2               ((unsigned long)0x00000020)

//! Match Register 3.
#define TIMER_MR3               ((unsigned long)0x00000024)

//! Capture Control Register.
//! The CCR controls which edges of the capture inputs are used to load the Capture
//! Registers and whether or not an interrupt is generated when a capture takes place.
#define TIMER_CCR               ((unsigned long)0x00000028)

//! Capture Register 0.
//! CR0 is loaded with the value of TC when there is an event on the CAPn.0 
//! (CAP0.0 or CAP1.0 respectively) input.
#define TIMER_CR0               ((unsigned long)0x0000002C)

//! Capture Register 1.
#define TIMER_CR1               ((unsigned long)0x00000030)

//! External Match Register.
//! The EMR controls the external match pins MATn.0-3 (MAT0.0-3 and MAT1.0-3 respectively).
#define TIMER_EMR               ((unsigned long)0x0000003C)

//! Count Control Register.
//! The CTCR selects between Timer and Counter mode, and in Counter mode selects the signal
//! and edge(s) for counting.
#define TIMER_CTCR              ((unsigned long)0x00000070)



//! IR {{

//! Interrupt flag for match channel 0.
#define IR_INT_MR0              BIT_32_0

//! Interrupt flag for match channel 1.
#define IR_INT_MR1              BIT_32_1

//! Interrupt flag for match channel 2.
#define IR_INT_MR2              BIT_32_2

//! Interrupt flag for match channel 3.
#define IR_INT_MR3              BIT_32_3

//! Interrupt flag for capture channel 0 event.
#define IR_INT_CR0              BIT_32_4

//! Interrupt flag for capture channel 1 event.
#define IR_INT_CR1              BIT_32_5

//! IR }}


//! TCR {{

//! Enable Counter.
#define TCR_CNT_EN              BIT_32_0

//! Reset Counter.
#define TCR_CNT_RST             BIT_32_1

//! TCR }}


//! CTCR {{

//! Counter/Timer Mode.
#define CTCR_MODE_M             BIT_MASK(32, 1, 0)
#define CTCR_MODE_S             0

//! Count input select.
#define CTCR_CNT_INPUT_SEL_M    BIT_MASK(32, 3, 2)
#define CTCR_CNT_INPUT_SEL_S    2

//! CTCR }}

//! MCR {{

//! Interrupt on MR0.
#define MCR_MR0I                BIT_32_0

//! Reset on MR0.
#define MCR_MR0R                BIT_32_1

//! Stop on MR0.
#define MCR_MR0S                BIT_32_2

//! Interrupt on MR1.
#define MCR_MR1I                BIT_32_3

//! Reset on MR1.
#define MCR_MR1R                BIT_32_4

//! Stop on MR1.
#define MCR_MR1S                BIT_32_5

//! Interrupt on MR2.
#define MCR_MR2I                BIT_32_6

//! Reset on MR2.
#define MCR_MR2R                BIT_32_7

//! Stop on MR2.
#define MCR_MR2S                BIT_32_8

//! Interrupt on MR3.
#define MCR_MR3I                BIT_32_9

//! Reset on MR3.
#define MCR_MR3R                BIT_32_10

//! Stop on MR3.
#define MCR_MR3S                BIT_32_11

//! MCR }}


//! CCR {{

//! Capture on CAPn.0 rising edge.
#define CCR_CAP0RE              BIT_32_0

//! Capture on CAPn.0 falling edge.
#define CCR_CAP0FE              BIT_32_1

//! Interrupt on CAPn.0 event.
#define CCR_CAP0I               BIT_32_2

//! Capture on CAPn.1 rising edge.
#define CCR_CAP1RE              BIT_32_3

//! Capture on CAPn.1 falling edge.
#define CCR_CAP1FE              BIT_32_4

//! Interrupt on CAPn.1 event.
#define CCR_CAP1I               BIT_32_5

//! CCR }}


//! EMR {{

//! External Match 0.
#define EMR_EM0                 BIT_32_0

//! External Match 1.
#define EMR_EM1                 BIT_32_1

//! External Match 2.
#define EMR_EM2                 BIT_32_2

//! External Match 3.
#define EMR_EM3                 BIT_32_3

//! External match control 0.
#define EMR_EMC0_M              BIT_MASK(32, 5, 4)
#define EMR_EMC0_S              4

//! External match control 1.
#define EMR_EMC1_M              BIT_MASK(32, 7, 6)
#define EMR_EMC1_S              6

//! External match control 2.
#define EMR_EMC2_M              BIT_MASK(32, 9, 8)
#define EMR_EMC2_S              8

//! External match control 3.
#define EMR_EMC3_M              BIT_MASK(32, 11, 10)
#define EMR_EMC3_S              10

//! EMR }}
