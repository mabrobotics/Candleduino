# Candleduino

**Arduino library for controlling MD drives** across multiple platforms: AVR(MCP2515), Renesas, and Teensy >4.0.

---

## Features
- Cross-platform support: AVR(MCP2515), Renesas, Teensy >4.0  
- Creating MDs assigned to instance with unique IDs
- Managing internal registers of MDs
- Provides helper functions for managing MDs, for example: blink(), zero(), setTargetPosition()

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

## Usage
```cpp
mab::canId_t ID = 100; //ID of the MD drive

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
MD::Error_t MD::readRegister<T>(MD::Message<T> &registerData)
MD::Error_t MD::writeRegister<T>(MD::Message<T> registerData)

MD::Error_t MD::readRegister<T>(uint16_t registerId, T &registerData)
MD::Error_t MD::writeRegister<T>(uint16_t registerId, T registerData)
```

Or CANFD only use readRegisters or writeRegisters for multiple data frame

```cpp
MD::Error_t MD::readRegisters<T>(MD::Message<T> registerData, ...)
MD::Error_t MD::writeRegisters<T>(MD::Message<T> registerData, ...)
```

For multiple MD drives user can create multiple MD object with unique ID's.

Example:
```cpp
MD md1(100);
MD md2(120);
```

Other registers can be found in CANdleduino documentation or in MD/Communication [documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/MD/Communication/fdcan.html#) section.