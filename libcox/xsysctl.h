

//! MCO {{
void   SysCtlMcoCfg(unsigned long ulCfg);
void   SysCtlMcoEnable(void);
void   SysCtlMcoDisable(void);
xtbool SysCtlMcoStatus(void);
//! MCO }}


//! POWER {{

void SysCtlPeripheralReset(unsigned long ulPeripheral);
void SysCtlPeripheralEnable(unsigned long ulPeripheral);
void SysCtlPeripheralDisable(unsigned long ulPeripheral);

unsigned long SysCtlPowerFlagGet(void);
void SysCtlPowerFlagClear(unsigned long ulFlag);


//! POWER }}

//! System Clock Configure {{

//! ulSysClk --> SYSCTL_XTAL_nMHZ
void SysCtlClockSet(unsigned long ulSysClk, unsigned long ulConfig)






//! System Clock Configure }}



























