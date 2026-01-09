#include "Candleduino.hpp"
#if defined(TEENSYDUINO)

class PDS : public MAB_DEVICE
{
public:
    using MAB_DEVICE::MAB_DEVICE;

    /**
     * @brief Writes data to property CANFD only
     *
     * @param module    PDS module
     * @param propertyId ID of property to write
     * @param propertyData   Data of property to write
     * @return  Error code indicating success or failure.
     */
    template <typename T>
    Error_t writeProperty(PDSmodule module, uint8_t propertyId, T propertyData)
    {

        uint8_t bufferSize = 4 + sizeof(uint8_t) + sizeof(uint32_t);
        uint8_t buffer[bufferSize] = {0};
        uint8_t respBuffer[bufferSize] = {0};
        buffer[0] = FRAME_WRITE_PROPERTY_FD;
        buffer[1] = module.type;
        if (module.index > 0 && module.index <= 6)
            buffer[2] = module.index;
        else
            buffer[2] = 0x00;
        buffer[3] = 0x01;

        memcpy(buffer + 4, &propertyId, sizeof(uint8_t));
        memcpy(buffer + 4 + sizeof(uint8_t), &propertyData, sizeof(propertyData));

        auto result = writeReadFD(buffer, respBuffer, bufferSize);
        if (result != Error_t::OK)
            return Error_t::NOT_CONNECTED;
        return Error_t::OK;
    }

    template <typename T>
    Error_t writeProperty(PDSmodule module, Message<T> propertyData)
    {
        return writeProperty(module, propertyData.messageID, propertyData.value);
    }

    /**
     * @brief Reads data to property CANFD only
     *
     * @param module    PDS module
     * @param propertyId   ID of property
     * @param propertyData Data of property to read
     * @return  Error code indicating success or failure.
     */
    template <typename T>
    Error_t readProperty(PDSmodule module, uint8_t propertyId, T &propertyData)
    {
        uint8_t bufferSize = 4 + sizeof(uint8_t) + sizeof(uint32_t);
        uint8_t buffer[bufferSize] = {0};
        uint8_t respBuffer[bufferSize] = {0};
        respBuffer[0] = 0x01;
        buffer[0] = FRAME_READ_PROPERTY_FD;
        buffer[1] = module.type;
        if (module.index > 0 && module.index <= 6)
            buffer[2] = module.index;
        else
            buffer[2] = 0x00;
        buffer[3] = 0x01;

        memcpy(buffer + 4, &propertyId, sizeof(uint8_t));

        auto result = writeReadFD(buffer, respBuffer, bufferSize);

        if (result != Error_t::OK)
            return Error_t::NOT_CONNECTED;

        if (respBuffer[0] == 0)
        {
            memcpy(&propertyData, respBuffer + 2 + sizeof(uint8_t), sizeof(T));
        }
        else
        {
            return Error_t::REQUEST_INVALID;
        }

        return Error_t::OK;
    }

    template <typename T>
    Error_t readProperty(PDSmodule module, Message<T> &propertyData)
    {
        return readProperty(module, uint8_t(propertyData.messageID), propertyData.value);
    }

    /**
     * @brief Writes data to properties CANFD only
     *
     * @param module    PDS module
     * @param message   Message to write
     * @return  Error code indicating success or failure.
     */
    template <typename... T>
    Error_t writeProperties(PDSmodule module, T... message)
    {
        size_t bufferSize = 4;
        ((bufferSize += sizeof(message.messageID) + sizeof(uint32_t)), ...);
        uint8_t buffer[bufferSize] = {0};
        uint8_t respBuffer[bufferSize] = {0};

        buffer[0] = FRAME_WRITE_PROPERTY_FD;
        buffer[1] = module.type;
        buffer[2] = module.index;
        buffer[3] = (bufferSize - 4) / 5;

        bufferSize = 4;

        ([&]()
         {
        memcpy(buffer + bufferSize, &message.messageID, sizeof(uint8_t));
        memcpy(buffer + sizeof(uint8_t) + bufferSize, &message.value, sizeof(uint32_t));
        bufferSize += 5; }(),
         ...);
        auto result = writeReadFD(buffer, respBuffer, bufferSize);

        if (result != Error_t::OK)
            return Error_t::NOT_CONNECTED;

        return Error_t::OK;
    }

    /**
     * @brief Reads data from properties CANFD only
     *
     * @param module    PDS module
     * @param message   Message to read
     * @return  Error code indicating success or failure.
     */
    template <typename... T>
    Error_t readProperties(PDSmodule module, T &...message)
    {
        size_t bufferSize = 4;
        ((bufferSize += sizeof(uint8_t(message.messageID))), ...);
        size_t responseBufferSize = 2;
        ((responseBufferSize += sizeof(uint8_t(message.messageID)) + sizeof(uint32_t)), ...);
        uint8_t responseBufferDiff = responseBufferSize - bufferSize;
        uint8_t respBuffer[responseBufferSize] = {0};
        uint8_t buffer[bufferSize] = {0};

        buffer[0] = FRAME_READ_PROPERTY_FD;
        buffer[1] = module.type;
        buffer[2] = module.index;
        buffer[3] = (responseBufferSize - 2) / 5;

        bufferSize = 4;

        ([&]()
         {
         memcpy(buffer + bufferSize, &message.messageID, sizeof(uint8_t));
         bufferSize += 1; }(),
         ...);

        auto result = writeReadFD(buffer, respBuffer, responseBufferSize, responseBufferDiff);
        if (result != Error_t::OK)
            return Error_t::NOT_CONNECTED;

        bufferSize = 2;

        if (respBuffer[0] == 0)
        {
            ([&]()
             { 
            memcpy(&message.value, respBuffer + bufferSize + sizeof(uint8_t), sizeof(message.value)); 
            bufferSize += 5; }(),
             ...);
        }

        return Error_t::OK;
    }
    /// @brief Enable module
    /// @return Error code indicating success or failure.
    Error_t enable(PDSmodule module);
    /// @brief Disable module
    /// @return Error code indicating success or failure.
    Error_t disable(PDSmodule module);

    /// @brief Get enable state of module
    /// @return Error code indicating success or failure.
    Error_t getEnable(PDSmodule module, bool &enable);

    /// @brief Get: Control Board - bus voltage, Power Stage - output voltage, Isolated Converter - output voltage
    /// @return Error code indicating success or failure.
    Error_t getVoltage(PDSmodule module, uint32_t &voltage);

    /// @brief Get OCD delay (Power Stage or Isolated Converter)
    /// @return Error code indicating success or failure.
    Error_t getOCDdelay(PDSmodule module, uint32_t &OCDdelay);

    /// @brief Set OCD delay (Power Stage or Isolated Converter)
    /// @return Error code indicating success or failure.
    Error_t setOCDdelay(PDSmodule module, uint32_t OCDdelay);

    /// @brief Get OCD level (Power Stage or Isolated Converter)
    /// @return Error code indicating success or failure.
    Error_t getOCDlevel(PDSmodule module, uint32_t &OCDlevel);

    /// @brief Set OCD level (Power Stage or Isolated Converter)
    /// @return Error code indicating success or failure.
    Error_t setOCDlevel(PDSmodule module, uint32_t OCDlevel);

    /// @brief Get temperature
    /// @return Error code indicating success or failure.
    Error_t getTemperature(PDSmodule module, float &temperature);

    /// @brief Get temperature limit
    /// @return Error code indicating success or failure.
    Error_t getTemperatureLimit(PDSmodule module, float &temperatureLimit);

    /// @brief Set temperature limit
    /// @return Error code indicating success or failure.
    Error_t setTemperatureLimit(PDSmodule module, float temperatureLimit);

    /// @brief Get load current (Power Stage or Isolated Converter)
    /// @return Error code indicating success or failure.
    Error_t getLoadCurrent(PDSmodule module, int32_t &loadCurrent);

    /// @brief Get load power (Power Stage or Isolated Converter)
    /// @return Error code indicating success or failure.
    Error_t getLoadPower(PDSmodule module, int32_t &loadPower);

    /// @brief Bind Brake Resistor to Power stage
    /// @return Error code indicating success or failure.
    Error_t bindBrakeResistor(PDSmodule module, uint8_t brakeResistorIndex);

    /// @brief Set Brake reistor trigger voltage
    /// @return Error code indicating success or failure.
    Error_t setBrakeResistorTriggerVoltage(PDSmodule module, uint32_t brakeResistorIndex);

    /// @brief Read Brake resistor trigger voltage
    /// @return Error code indicating success or failure.
    Error_t getBrakeResistorTriggerVoltage(PDSmodule module, uint32_t &brTriggerVoltage);
};

#endif