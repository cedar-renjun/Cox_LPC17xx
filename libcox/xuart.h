
extern unsigned long UARTByteRead(unsigned long ulBase);
extern void UARTByteWrite(unsigned long ulBase, unsigned long ulData);

extern xtBoolean UARTByteReadNoBlocking(unsigned long ulBase, unsigned long * ulpData);
extern xtBoolean UARTByteWriteNoBlocking(unsigned long ulBase, unsigned long ulData);

