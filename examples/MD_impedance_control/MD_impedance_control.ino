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

    // Zero out the position of the drive
    md.zero();
    delay(100);
}

float stepSize = 0.04f;
float targetPosition = 0.0f;
float position = 0.0f;

void loop()
{
    // Selecting motion mode to be impedance. Important note is that motion mode resets every time
    // MD is disabled or timed out
    md.setMotionMode(motionModeMab_E::IMPEDANCE);
    md.enable();
    delay(100);

    for (uint16_t i = 0; i < 100; i++)
    {
        targetPosition += stepSize;

        md.setTargetPosition(targetPosition);

        if (i % 10 == 0)
        {
            md.getMainEncoderPosition(position);
            Serial.print("Position: ");
            Serial.println(position);
        }

        delay(20);
    }
    stepSize = -stepSize;

    md.disable();
    delay(100);

    // Selecting motion mode to be impedance. Important note is that motion mode resets every time
    // MD is disabled or timed out
    md.setMotionMode(motionModeMab_E::IMPEDANCE);
    md.enable();
    delay(100);

    for (uint16_t i = 0; i < 100; i++)
    {
        targetPosition += stepSize;

        md.setTargetPosition(targetPosition);

        if (i % 10 == 0)
        {
            md.getMainEncoderPosition(position);
            Serial.print("Position: ");
            Serial.println(position);
        }

        delay(20);
    }
    stepSize = -stepSize;

    md.disable();
    delay(100);
}