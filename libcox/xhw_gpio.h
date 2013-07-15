
// Bit Assign
// PIN_CON_BASE
#define PINSEL0                        ((unsigned long)0x00000000)
#define PINSEL1                        ((unsigned long)0x00000004)
#define PINSEL2                        ((unsigned long)0x00000008)
#define PINSEL3                        ((unsigned long)0x0000000C)
#define PINSEL4                        ((unsigned long)0x00000010)
#define PINSEL5                        ((unsigned long)0x00000014)
#define PINSEL6                        ((unsigned long)0x00000018)
#define PINSEL7                        ((unsigned long)0x0000001C)
#define PINSEL8                        ((unsigned long)0x00000020)
#define PINSEL9                        ((unsigned long)0x00000024)
#define PINSEL10                       ((unsigned long)0x00000028)
#define PINMODE0                       ((unsigned long)0x00000040)
#define PINMODE1                       ((unsigned long)0x00000044)
#define PINMODE2                       ((unsigned long)0x00000048)
#define PINMODE3                       ((unsigned long)0x0000004C)
#define PINMODE4                       ((unsigned long)0x00000050)
#define PINMODE5                       ((unsigned long)0x00000054)
#define PINMODE6                       ((unsigned long)0x00000058)
#define PINMODE7                       ((unsigned long)0x0000005C)
#define PINMODE8                       ((unsigned long)0x00000060)
#define PINMODE9                       ((unsigned long)0x00000064)
#define PINMODE_OD0                    ((unsigned long)0x00000068)
#define PINMODE_OD1                    ((unsigned long)0x0000006C)
#define PINMODE_OD2                    ((unsigned long)0x00000070)
#define PINMODE_OD3                    ((unsigned long)0x00000074)
#define PINMODE_OD4                    ((unsigned long)0x00000078)
#define I2CPADCFG                      ((unsigned long)0x0000007C)

// GPIO_PORTx_BASE (x = A/B/C/D/E)
#define FIODIR                         ((unsigned long)0x00000000)
#define FIO0DIR                        ((unsigned long)0x00000000)
#define FIO1DIR                        ((unsigned long)0x00000000)
#define FIO2DIR                        ((unsigned long)0x00000000)
#define FIO3DIR                        ((unsigned long)0x00000000)
#define FIO4DIR                        ((unsigned long)0x00000000)

#define FIOMASK                        ((unsigned long)0x00000010)
#define FIO0MASK                       ((unsigned long)0x00000010)
#define FIO1MASK                       ((unsigned long)0x00000010)
#define FIO2MASK                       ((unsigned long)0x00000010)
#define FIO3MASK                       ((unsigned long)0x00000010)
#define FIO4MASK                       ((unsigned long)0x00000010)

#define FIOPIN                         ((unsigned long)0x00000014)
#define FIO1PIN                        ((unsigned long)0x00000014)
#define FIO2PIN                        ((unsigned long)0x00000014)
#define FIO3PIN                        ((unsigned long)0x00000014)
#define FIO4PIN                        ((unsigned long)0x00000014)

#define FIOSET                         ((unsigned long)0x00000018)
#define FIO1SET                        ((unsigned long)0x00000018)
#define FIO2SET                        ((unsigned long)0x00000018)
#define FIO3SET                        ((unsigned long)0x00000018)
#define FIO4SET                        ((unsigned long)0x00000018)

#define FIOCLR                         ((unsigned long)0x0000001C)
#define FIO1CLR                        ((unsigned long)0x0000001C)
#define FIO2CLR                        ((unsigned long)0x0000001C)
#define FIO3CLR                        ((unsigned long)0x0000001C)
#define FIO4CLR                        ((unsigned long)0x0000001C)

// GPIO_INT_BASE 0x40028000
#define IO0IntEnR                      ((unsigned long)0x00000090)
#define IO2IntEnR                      ((unsigned long)0x000000B0)

#define IO0IntEnF                      ((unsigned long)0x00000094)
#define IO2IntEnF                      ((unsigned long)0x000000B4)

#define IO0IntStatR                    ((unsigned long)0x00000084)
#define IO2IntStatR                    ((unsigned long)0x000000A4)

#define IO0IntStatF                    ((unsigned long)0x00000088)
#define IO2IntStatF                    ((unsigned long)0x000000A8)

#define IO0IntClr                      ((unsigned long)0x0000008C)
#define IO2IntClr                      ((unsigned long)0x000000AC)

#define IOIntStatus                    ((unsigned long)0x00000080)





//! \note I2C0 is special pin, this must be pay attention.
