#include "PDS_arduino.hpp"
#if defined(TEENSYDUINO)
Error_t PDS::enable(PDSmodule module, bool enable)
{
    return writeProperty(module, ENABLE, enable);
}

Error_t PDS::getEnable(PDSmodule module, bool &enable)
{
    return readProperty(module, ENABLE, enable);
}

Error_t PDS::getBusVoltage(PDSmodule module, uint32_t &busVoltage)
{
    return readProperty(module, BUS_VOLTAGE, busVoltage);
}

Error_t PDS::getOCDdelay(PDSmodule module, uint32_t &OCDdelay)
{
    return readProperty(module, OCD_DELAY, OCDdelay);
}

Error_t PDS::getOCDlevel(PDSmodule module, uint32_t &OCDlevel)
{
    return readProperty(module, OCD_LEVEL, OCDlevel);
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
#endif
