#define I2C_SLAVE_ADD0          BIT_32_0
#define I2C_SLAVE_ADD1          BIT_32_1
#define I2C_SLAVE_ADD2          BIT_32_2
#define I2C_SLAVE_ADD3          BIT_32_3


/* I2C return status code definitions ----------------------------- */

/** No relevant information */
#define I2C_STAT_NO_INF                       ((0xF8))

/** Bus Error */
#define I2C_STAT_BUS_ERROR                    ((0x00))

/* Master transmit mode -------------------------------------------- */
/** A start condition has been transmitted */
#define I2C_STAT_M_TX_START                   ((0x08))

/** A repeat start condition has been transmitted */
#define I2C_STAT_M_TX_RESTART                 ((0x10))

/** SLA+W has been transmitted, ACK has been received */
#define I2C_STAT_M_TX_SLAW_ACK                ((0x18))

/** SLA+W has been transmitted, NACK has been received */
#define I2C_STAT_M_TX_SLAW_NACK               ((0x20))

/** Data has been transmitted, ACK has been received */
#define I2C_STAT_M_TX_DAT_ACK                 ((0x28))

/** Data has been transmitted, NACK has been received */
#define I2C_STAT_M_TX_DAT_NACK                ((0x30))

/** Arbitration lost in SLA+R/W or Data bytes */
#define I2C_STAT_M_TX_ARB_LOST                ((0x38))

/* Master receive mode -------------------------------------------- */
/** A start condition has been transmitted */
#define I2C_STAT_M_RX_START                   ((0x08))

/** A repeat start condition has been transmitted */
#define I2C_STAT_M_RX_RESTART                 ((0x10))

/** Arbitration lost */
#define I2C_STAT_M_RX_ARB_LOST                ((0x38))

/** SLA+R has been transmitted, ACK has been received */
#define I2C_STAT_M_RX_SLAR_ACK                ((0x40))

/** SLA+R has been transmitted, NACK has been received */
#define I2C_STAT_M_RX_SLAR_NACK               ((0x48))

/** Data has been received, ACK has been returned */
#define I2C_STAT_M_RX_DAT_ACK                 ((0x50))

/** Data has been received, NACK has been return */
#define I2C_STAT_M_RX_DAT_NACK                ((0x58))

/* Slave receive mode -------------------------------------------- */
/** Own slave address has been received, ACK has been returned */
#define I2C_STAT_S_RX_SLAW_ACK                ((0x60))

/** Arbitration lost in SLA+R/W as master */
#define I2C_STAT_S_RX_ARB_LOST_M_SLA          ((0x68))

/** General call address has been received, ACK has been returned */
#define I2C_STAT_S_RX_GENCALL_ACK             ((0x70))

/** Arbitration lost in SLA+R/W (GENERAL CALL) as master */
#define I2C_STAT_S_RX_ARB_LOST_M_GENCALL      ((0x78))

/** Previously addressed with own SLV address;
 * Data has been received, ACK has been return */
#define I2C_STAT_S_RX_PRE_SLA_DAT_ACK         ((0x80))

/** Previously addressed with own SLA;
 * Data has been received and NOT ACK has been return */
#define I2C_STAT_S_RX_PRE_SLA_DAT_NACK        ((0x88))

/** Previously addressed with General Call;
 * Data has been received and ACK has been return */
#define I2C_STAT_S_RX_PRE_GENCALL_DAT_ACK     ((0x90))

/** Previously addressed with General Call;
 * Data has been received and NOT ACK has been return */
#define I2C_STAT_S_RX_PRE_GENCALL_DAT_NACK    ((0x98))

/** A STOP condition or repeated START condition has
 * been received while still addressed as SLV/REC
 * (Slave Receive) or SLV/TRX (Slave Transmit) */
#define I2C_STAT_S_RX_STA_STO_SLVREC_SLVTRX   ((0xA0))

/** Slave transmit mode */
/** Own SLA+R has been received, ACK has been returned */
#define I2C_STAT_S_TX_SLAR_ACK                ((0xA8))

/** Arbitration lost in SLA+R/W as master */
#define I2C_STAT_S_TX_ARB_LOST_M_SLA          ((0xB0))

/** Data has been transmitted, ACK has been received */
#define I2C_STAT_S_TX_DAT_ACK                 ((0xB8))

/** Data has been transmitted, NACK has been received */
#define I2C_STAT_S_TX_DAT_NACK                ((0xC0))

/** Last data byte in I2DAT has been transmitted (AA = 0);
 ACK has been received */
#define I2C_STAT_S_TX_LAST_DAT_ACK            ((0xC8))


extern void I2CCfg(unsigned long ulBase, unsigned long TargetClk);

// pass
extern void I2CEnable(unsigned long ulBase);
// pass
extern void I2CDisable(unsigned long ulBase);
// wait
extern void I2CStartSend(unsigned long ulBase);
// wait
extern void I2CStopSend(unsigned long ulBase);

// pass
extern void I2CGeneralCallEnable(unsigned long ulBase, unsigned long ulID);
// pass
extern void I2CGeneralCallDisable(unsigned long ulBase, unsigned long ulID);

extern void I2CSlaveAddrSet(unsigned long ulBase, unsigned long ulID, unsigned long ulVal);

extern unsigned long I2CDataRead(unsigned long ulBase);
extern void I2CDataWrite(unsigned long ulBase, unsigned long ulValue);
