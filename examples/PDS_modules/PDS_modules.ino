#include "PDS_arduino.hpp"

uint16_t ID = 100; // PDS ID

#if defined(TEENSYDUINO)

FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;

PDS pds(ID, &CANbus);

PDSmodule IC = {ISOLATED_CONVERTER, 1};
PDSmodule PS = {POWER_STAGE, 2};
PDSmodule BR = {BRAKE_RESISTOR, 3};

void setup()
{
    Serial.begin(115200);

    delay(500);

    pds.init();

    delay(500);

    // Configuration PS
    pds.setTemperatureLimit(PS, 90.0f); // 90 Celsius degrees
    pds.setOCDdelay(PS, 25000);         // 25 A OCD level
    pds.setOCDlevel(PS, 1000);          // 1 ms delay

    // Configuration IC
    pds.setTemperatureLimit(IC, 70.0f); // 70 Celsius degrees
    pds.setOCDlevel(IC, 4000);          // 4 A

    // Braking resistor configuration
    pds.bindBrakeResistor(PS, BR.index);           // Bind BR to PS
    pds.setBrakeResistorTriggerVoltage(PS, 30000); // 30V DC

    pds.enable(IC);
    pds.enable(PS);
    delay(100); // delay for enabling Power Stage
}

void loop()
{

    uint32_t outputVoltage_ps = 0;
    uint32_t outputVoltage_ic = 0;
    float temperature_ps = 0.0f;
    float temperature_ic = 0.0f;

    pds.getTemperature(PS, temperature_ps);
    pds.getVoltage(PS, outputVoltage_ps);
    pds.getVoltage(IC, outputVoltage_ic);
    pds.getTemperature(IC, temperature_ic);

    Serial.print("[PS] Temperature: ");
    Serial.println(temperature_ps);
    Serial.print("[IC] Temperature: ");
    Serial.println(temperature_ic);
    Serial.print("[PS] Output voltage: ");
    Serial.println(outputVoltage_ps);
    Serial.print("[IC] Output voltage: ");
    Serial.println(outputVoltage_ic);

    delay(1000);
}

#endif