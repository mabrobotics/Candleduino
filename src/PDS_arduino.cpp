#include "PDS_arduino.hpp"
#if defined(TEENSYDUINO)

Error_t PDS::enable(PDSmodule module)
{
    return writeProperty(module, ENABLE, 1);
}

Error_t PDS::disable(PDSmodule module)
{
    return writeProperty(module, ENABLE, 0);
}

Error_t PDS::getEnable(PDSmodule module, bool &enable)
{
    return readProperty(module, ENABLE, enable);
}

Error_t PDS::getVoltage(PDSmodule module, uint32_t &voltage)
{
    return readProperty(module, BUS_VOLTAGE, voltage);
}

Error_t PDS::getOCDdelay(PDSmodule module, uint32_t &OCDdelay)
{
    return readProperty(module, OCD_DELAY, OCDdelay);
}

Error_t PDS::setOCDdelay(PDSmodule module, uint32_t OCDdelay)
{
    return writeProperty(module, OCD_DELAY, OCDdelay);
}

Error_t PDS::getOCDlevel(PDSmodule module, uint32_t &OCDlevel)
{
    return readProperty(module, OCD_LEVEL, OCDlevel);
}

Error_t PDS::setOCDlevel(PDSmodule module, uint32_t OCDlevel)
{
    return writeProperty(module, OCD_LEVEL, OCDlevel);
}

Error_t PDS::getTemperature(PDSmodule module, float &temperature)
{
    return readProperty(module, TEMPERATURE, temperature);
}

Error_t PDS::getTemperatureLimit(PDSmodule module, float &temperatureLimit)
{
    return readProperty(module, TEMPERATURE_LIMIT, temperatureLimit);
}

Error_t PDS::setTemperatureLimit(PDSmodule module, float temperatureLimit)
{
    return writeProperty(module, TEMPERATURE_LIMIT, temperatureLimit);
}

Error_t PDS::getLoadCurrent(PDSmodule module, int32_t &loadCurrent)
{
    return readProperty(module, LOAD_CURRENT, loadCurrent);
}

Error_t PDS::getLoadPower(PDSmodule module, int32_t &loadPower)
{
    return readProperty(module, LOAD_POWER, loadPower);
}
Error_t PDS::bindBrakeResistor(PDSmodule module, uint8_t brakeResistorIndex)
{
    return writeProperty(module, BR_SOCKET_INDEX, brakeResistorIndex);
}
Error_t PDS::setBrakeResistorTriggerVoltage(PDSmodule module, uint32_t brTriggerVoltage)
{
    return writeProperty(module, BR_TRIGGER_VOLTAGE, brTriggerVoltage);
}

Error_t PDS::getBrakeResistorTriggerVoltage(PDSmodule module, uint32_t &brTriggerVoltage)
{
    return readProperty(module, BR_TRIGGER_VOLTAGE, brTriggerVoltage);
}
#endif
