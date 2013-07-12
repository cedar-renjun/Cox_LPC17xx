
//! Pin Type {{
void GPIOPinType(unsigned long ulPinCfg);
//! Pin Type }}


//! xGPIO Pin ID
#define xGPIO_PIN_0             GPIO_PIN_0 
#define xGPIO_PIN_1             GPIO_PIN_1 
#define xGPIO_PIN_2             GPIO_PIN_2 
#define xGPIO_PIN_3             GPIO_PIN_3 
#define xGPIO_PIN_4             GPIO_PIN_4 
#define xGPIO_PIN_5             GPIO_PIN_5 
#define xGPIO_PIN_6             GPIO_PIN_6 
#define xGPIO_PIN_7             GPIO_PIN_7 
#define xGPIO_PIN_8             GPIO_PIN_8 
#define xGPIO_PIN_9             GPIO_PIN_9 
#define xGPIO_PIN_10            GPIO_PIN_10
#define xGPIO_PIN_11            GPIO_PIN_11
#define xGPIO_PIN_12            GPIO_PIN_12
#define xGPIO_PIN_13            GPIO_PIN_13
#define xGPIO_PIN_14            GPIO_PIN_14
#define xGPIO_PIN_15            GPIO_PIN_15
#define xGPIO_PIN_16            GPIO_PIN_16
#define xGPIO_PIN_17            GPIO_PIN_17
#define xGPIO_PIN_18            GPIO_PIN_18
#define xGPIO_PIN_19            GPIO_PIN_19
#define xGPIO_PIN_20            GPIO_PIN_20
#define xGPIO_PIN_21            GPIO_PIN_21
#define xGPIO_PIN_22            GPIO_PIN_22
#define xGPIO_PIN_23            GPIO_PIN_23
#define xGPIO_PIN_24            GPIO_PIN_24
#define xGPIO_PIN_25            GPIO_PIN_25
#define xGPIO_PIN_26            GPIO_PIN_26
#define xGPIO_PIN_27            GPIO_PIN_27
#define xGPIO_PIN_28            GPIO_PIN_28
#define xGPIO_PIN_29            GPIO_PIN_29
#define xGPIO_PIN_30            GPIO_PIN_30
#define xGPIO_PIN_31            GPIO_PIN_31


//! GPIO Pin
#define GPIO_PIN_0              BIT_32_0
#define GPIO_PIN_1              BIT_32_1
#define GPIO_PIN_2              BIT_32_2
#define GPIO_PIN_3              BIT_32_3
#define GPIO_PIN_4              BIT_32_4
#define GPIO_PIN_5              BIT_32_5
#define GPIO_PIN_6              BIT_32_6
#define GPIO_PIN_7              BIT_32_7
#define GPIO_PIN_8              BIT_32_8
#define GPIO_PIN_9              BIT_32_9
#define GPIO_PIN_10             BIT_32_10
#define GPIO_PIN_11             BIT_32_11
#define GPIO_PIN_12             BIT_32_12
#define GPIO_PIN_13             BIT_32_13
#define GPIO_PIN_14             BIT_32_14
#define GPIO_PIN_15             BIT_32_15
#define GPIO_PIN_16             BIT_32_16
#define GPIO_PIN_17             BIT_32_17
#define GPIO_PIN_18             BIT_32_18
#define GPIO_PIN_19             BIT_32_19
#define GPIO_PIN_20             BIT_32_20
#define GPIO_PIN_21             BIT_32_21
#define GPIO_PIN_22             BIT_32_22
#define GPIO_PIN_23             BIT_32_23
#define GPIO_PIN_24             BIT_32_24
#define GPIO_PIN_25             BIT_32_25
#define GPIO_PIN_26             BIT_32_26
#define GPIO_PIN_27             BIT_32_27
#define GPIO_PIN_28             BIT_32_28
#define GPIO_PIN_29             BIT_32_29
#define GPIO_PIN_30             BIT_32_30
#define GPIO_PIN_31             BIT_32_31


#define PIN_MODE_PULL_UP               0x00
#define PIN_MODE_REPEATER              0x01
#define PIN_MODE_NONE                  0x10
#define PIN_MODE_PULL_DOWN             0x11

#define PIN_MODE_OD                    /\/\    
void GPIOPinCfg(unsigned long ulPort, unsigned long ulPin, unsigned long ulCfg);
void GPIOPinSet(unsigned long ulPort, unsigned long ulPin);
void GPIOPinReset(unsigned long ulPort, unsigned long ulPin);
void GPIOPinWrite(unsigned long ulPort, unsigned long ulPin, unsigned long ulVal);
unsigned long GPIOPinRead(unsigned long ulPort, unsigned long ulPin);

unsigned long GPIOPortRead(unsigned long ulPort);
void GPIOPortWrite(unsigned long ulPort, unsigned long ulVal);

void GPIOPinIntCfg(unsigned long ulPort, unsigned long ulPin, unsigned long ulCfg);
unsigned long GPIOPinIntFlagGet(unsigned long ulPort, unsigned long ulPin);
void GPIOPinIntFlagClear(unsigned long ulPort, unsigned long ulPin);
void GPIOPinIntEnable(unsigned long ulPort, unsigned long ulPin);
void GPIOPinIntDisable(unsigned long ulPort, unsigned long ulPin);


