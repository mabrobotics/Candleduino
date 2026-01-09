# Candleduino

**Arduino library for controlling MD drives** across multiple platforms: AVR(MCP2515), Renesas, and Teensy >4.0.

---

## Features
- Cross-platform support: AVR(MCP2515), Renesas, Teensy >4.0  
- Creating MD assigned to instance with unique ID
- Managing internal registers of MD and properties of PDS
- Provides helper functions for managing MD and PDS, for example: blink(), zero(), setTargetPosition()

---

## Installation

### Using Arduino Library Manager
1. Open Arduino IDE  
2. Go to **Sketch → Include Library → Manage Libraries…**  
3. Search for `Candleduino` and click **Install**  

### Manual Installation
1. Download the library from GitHub  
2. Extract the folder into your Arduino `libraries` folder  
3. Restart the Arduino IDE  

---

## MD
```cpp
#include "MD_arduino.hpp"
uint16_t ID = 100; //ID of the MD drive

MD md(ID); 
// MD md(ID, CS_PIN); for custom CS_PIN (Arduino AVR and MCP2515), default is 9

// FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus; //CANFD
//or
// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus; //CAN2.0
// MD md(ID, &CANbus); for Teensy

md.init(); //initialize MD object
```


For basic communication there are predefined commands like:
```cpp
md.blink();

md.getMosfetTemperature(float &temperature);

md.setPositionPIDparam(float kp, float ki, float kd, float integralMax);

...
```
For other registers it is recommended to use:

```cpp
MD::Error_t MD::readRegister<T>(Message<T> &registerData)
MD::Error_t MD::writeRegister<T>(Message<T> registerData)

MD::Error_t MD::readRegister<T>(uint16_t registerId, T &registerData)
MD::Error_t MD::writeRegister<T>(uint16_t registerId, T registerData)

//Message example:
Message<float> temperature;
temperature.messageID = TEMPERATURE;
temperature.value = 70.f;
```

Or CANFD only use readRegisters or writeRegisters for multiple data frame

```cpp
MD::Error_t MD::readRegisters<T>(Message<T> registerData, ...)
MD::Error_t MD::writeRegisters<T>(Message<T> registerData, ...)
```

For multiple MD drives user can create multiple MD object with unique ID's.

Example:
```cpp
MD md1(100);
MD md2(120);
```

## PDS

PDS arduino library works only on Teensy CANFD compatible boards.

```cpp
#include "PDS_arduino.hpp"
uint16_t ID = 100; //ID of the PDS

FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus; //CANFD

PDS pds(ID, &CANbus);

pds.init(); //initialize PDS object
```
For simplicity, there is a structure called PDSmodule:

Example:
```cpp
PDSmodule IC = {ISOLATED_CONVERTER, 2}; // {NAME, SOCKET_INDEX}
```
For basic communication there are predefined commands like:
```cpp
pds.enable(PDSmodule module);
pds.getTemperature(PDSmodule module, float &temperature);
...
```
And similarly to MD, PDS uses the same Message structure to send data as properties.
```cpp
Error_t readProperty(PDSmodule module, Message<T> &propertyData)
Error_t readProperty(PDSmodule module, uint8_t propertyId, T &propertyData)
Error_t readProperties(PDSmodule module, T &...message)

Error_t writeProperty(PDSmodule module, Message<T> propertyData)
Error_t writeProperty(PDSmodule module, uint8_t propertyId, T propertyData)
Error_t writeProperties(PDSmodule module, T... message)
```

For multiple PDS stacks simply use multiple instances:

Example:
```cpp
PDS pds1(100, &canBus);
PDS pds2(120, &canBus);
```

Other registers can be found in CANdleduino documentation or in MD/Communication [documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/MD/Communication/fdcan.html#) section.