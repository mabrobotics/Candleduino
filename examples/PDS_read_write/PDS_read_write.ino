#include "PDS_arduino.hpp"

uint16_t ID = 100; // PDS ID

#if defined(TEENSYDUINO)

FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;

PDS pds(ID, &CANbus);

void setup()
{
    Serial.begin(115200);

    delay(500);

    pds.init();

    delay(500);
}

void loop()
{
    PDSmodule CB = {CONTROL_BOARD, 0}; // Control board is set as 0 socket

    // Predefined function
    uint32_t voltage = 0;
    pds.getVoltage(CB, voltage);

    // Message to read/write
    Message<float> temperature;
    temperature.messageID = TEMPERATURE;
    temperature.value = 0.0f;

    pds.readProperty(CB, temperature);
    Serial.print("Temperature: ");
    Serial.println(temperature.value);

    // Read raw value
    uint16_t canID = 0;
    pds.readProperty(CB, CAN_ID_PDS, canID);

    Serial.print("CAN ID: ");
    Serial.println(canID);

    // Similarly writing properties
    pds.setTemperatureLimit(CB, 90.0f);
    // or
    // pds.writeProperty(CB, TEMPERATURE_LIMIT, 90.0f);

    // or many properties in one frame
    Message<uint32_t> battery_level;
    battery_level.messageID = BATTERY_VOLTAGE_L1;
    battery_level.value = 0;
    temperature.messageID = TEMPERATURE_LIMIT;
    pds.readProperties(CB, temperature, battery_level);

    Serial.print("Bus voltage: ");
    Serial.println(voltage);
    Serial.print("Temperature limit: ");
    Serial.println(temperature.value);
    Serial.print("Battery level: ");
    Serial.println(battery_level.value);

    delay(1000);
}

#endif