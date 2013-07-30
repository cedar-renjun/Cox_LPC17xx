//! Interrupt Location Register
#define RTC_ILR                 ((unsigned long)0x00000000)

//! Clock Control Register 
#define RTC_CCR                 ((unsigned long)0x00000008)

//! Counter Increment Interrupt Register 
#define RTC_CIIR                ((unsigned long)0x0000000C)

//! Alarm Mask Register 
#define RTC_AMR                 ((unsigned long)0x00000010)

//! RTC Auxiliary control register 
#define RTC_RTC_AUX             ((unsigned long)0x0000005C)

//! RTC Auxiliary Enable register 
#define RTC_RTC_AUXEN           ((unsigned long)0x00000058)

//! Consolidated Time Register 0 
#define RTC_CTIME0              ((unsigned long)0x00000014)

//! Consolidated Time Register 1 
#define RTC_CTIME1              ((unsigned long)0x00000018)

//! Consolidated Time Register 2 
#define RTC_CTIME2              ((unsigned long)0x0000001C)

//! Seconds Counter 
#define RTC_SEC                 ((unsigned long)0x00000020)

//! Minutes Register 
#define RTC_MIN                 ((unsigned long)0x00000024)

//! Hours Register 
#define RTC_HOUR                ((unsigned long)0x00000028)

//! Day of Month Register 
#define RTC_DOM                 ((unsigned long)0x0000002C)

//! Day of Week Register 
#define RTC_DOW                 ((unsigned long)0x00000030)

//! Day of Year Register 
#define RTC_DOY                 ((unsigned long)0x00000034)

//! Months Register 
#define RTC_MONTH               ((unsigned long)0x00000038)

//! Years Register 
#define RTC_YEAR                ((unsigned long)0x0000003C)

//! Calibration Value Register 
#define RTC_CALIBRATION         ((unsigned long)0x00000040)

//! General Purpose Register 0 
#define RTC_GPREG0              ((unsigned long)0x00000044)

//! General Purpose Register 1 
#define RTC_GPREG1              ((unsigned long)0x00000048)

//! General Purpose Register 2 
#define RTC_GPREG2              ((unsigned long)0x0000004C)

//! General Purpose Register 3 
#define RTC_GPREG3              ((unsigned long)0x00000050)

//! General Purpose Register 4 
#define RTC_GPREG4              ((unsigned long)0x00000054)

//! Alarm value for Seconds 
#define RTC_ALSEC               ((unsigned long)0x00000060)

//! Alarm value for Minutes 
#define RTC_ALMIN               ((unsigned long)0x00000064)

//! Alarm value for Hours 
#define RTC_ALHOUR              ((unsigned long)0x00000068)

//! Alarm value for Day of Month 
#define RTC_ALDOM               ((unsigned long)0x0000006C)

//! Alarm value for Day of Week 
#define RTC_ALDOW               ((unsigned long)0x00000070)

//! Alarm value for Day of Year 
#define RTC_ALDOY               ((unsigned long)0x00000074)

//! Alarm value for Months 
#define RTC_ALMON               ((unsigned long)0x00000078)

//! Alarm value for Year 
#define RTC_ALYEAR              ((unsigned long)0x0000007C)


//! RTC_ILR {{

//! RTC counter increment interrupt flag.
#define ILR_CIF             BIT_32_0

//! RTC alarm registers interrupt flag.
#define ILR_CALF            BIT_32_1

//! RTC_ILR }}


//! RTC_CCR {{

//! RTC Clock Enable.
#define CCR_CLKEN               BIT_32_0

//! CTC reset.
#define CCR_CTCRST              BIT_32_1

//! Calibration counter enable.
#define CCR_CCALEN              BIT_32_4

//! RTC_CCR }}


//! RTC_CIIR {{

//! Second increment interrupt.
#define CIIR_IMSEC              BIT_32_0

//! Minute increment interrupt.
#define CIIR_IMMIN              BIT_32_1

//! Hour increment interrupt.
#define CIIR_IMHOUR             BIT_32_2

//! Day of month increment interrupt.
#define CIIR_IMDOM              BIT_32_3

//! Week of month increment interrupt.
#define CIIR_IMDOW              BIT_32_4

//! Year of month increment interrupt.
#define CIIR_IMDOY              BIT_32_5

//! Month increment interrupt.
#define CIIR_IMMON              BIT_32_6

//! Year increment interrupt.
#define CIIR_IMYEAR             BIT_32_7

//! RTC_CIIR }}



//! RTC_AMR {{

//! Second increment interrupt.
#define AMR_SEC                 BIT_32_0

//! Minute increment interrupt.
#define AMR_MIN                 BIT_32_1

//! Hour increment interrupt.   
#define AMR_HOUR                BIT_32_2

//! Day of month increment interrupt.
#define AMR_DOM                 BIT_32_3

//! Week of month increment interrupt.
#define AMR_DOW                 BIT_32_4

//! Year of month increment interrupt.
#define AMR_DOY                 BIT_32_5

//! Month increment interrupt.
#define AMR_MON                 BIT_32_6

//! Year increment interrupt.   
#define AMR_YEAR                BIT_32_7

//! RTC_AMR }}



//! RTC_AUX {{

//! RTC Oscillator fail detect flag.
#define RTC_AUX_OSCF            BIT_32_4

//! RTC_AUX }}


//! RTC_AUXEN {{

//! Oscillator fail detect interrupt enable.
#define RTC_AUXEN_OSCFEN        BIT_32_4

//! RTC_AUXEN }}


//! RTC_CTIME0 {{


//! Second value.
#define CTIME0_SEC_M            BIT_MASK(32, 5, 0)
#define CTIME0_SEC_S            0

//! Minutes value.
#define CTIME0_MIN_M            BIT_MASK(32, 13, 8)
#define CTIME0_MIN_S            8

//! Hours value.
#define CTIME0_HOUR_M           BIT_MASK(32, 20, 16)
#define CTIME0_HOUR_S           16

//! Day of week value.
#define CTIME0_DOW_M            BIT_MASK(32, 26, 24)
#define CTIME0_DOW_S            24


//! RTC_CTIME0 }}


//! RTC_CTIME1 {{


//! Day of month value.
#define CTIME1_DOM_M            BIT_MASK(32, 4, 0)
#define CTIME1_DOM_S            0

//! Month value.
#define CTIME1_MON_M            BIT_MASK(32, 11, 8)
#define CTIME1_MON_S            8

//! Year value.
#define CTIME1_YEAR_M           BIT_MASK(32, 27, 16)
#define CTIME1_YEAR_S           16

//! RTC_CTIME1 }}



//! RTC_CTIME2 {{

//! Day of year value.
#define CTIME2_DOY_M            BIT_MASK(32, 11, 0)
#define CTIME2_DOY_S            0

//! RTC_CTIME2 }}

//! CALI {{

//! Calibration counter value.
#define CALI_VAL_M              BIT_MASK(32, 16, 0)
#define CALI_VAL_S              0

//! Calibration counter value.
#define CALI_DIR                BIT_32_17

//! CALI }}

