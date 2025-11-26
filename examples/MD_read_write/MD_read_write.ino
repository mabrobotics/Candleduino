#include "MD_arduino.hpp"

uint16_t ID = 100; // MD ID

// #define FD // Uncomment for CANFD TEENSY ONLY

#if defined(TEENSYDUINO)
//  CAN object defined outside class
#ifdef FD
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;
#else
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;
#endif
MD md(ID, &CANbus);
#else
MD md(ID); // MD md(ID, CS_PIN); CS_PIN - custom pin for SPI, default = 9
#endif

void setup()
{
    Serial.begin(115200);

    delay(500);

    md.init();

    delay(500);
}

void loop()
{
    // Predefined function
    md.blink();

    // Message to read/write
    MD::Message<uint32_t> id;
    id.registerID = 0x001;
    id.value = 0;

    // Raw value
    float temp = 0.0f;

    // Read
    md.readRegister(id);
    md.readRegister(MOSFET_TEMPERATURE, temp);

    Serial.print("ID: ");
    Serial.println(id.value);
    Serial.print("Position: ");
    Serial.println(temp);

    delay(1000);
}