#include "Candleduino.hpp"
#if defined(TEENSYDUINO)
class PDS : public MAB_DEVICE
{
public:
    using MAB_DEVICE::MAB_DEVICE;

    /**
     * @brief Writes data to register
     *
     * @param registerId    Register ID
     * @param data    Data to write
     * @return  Error code indicating success or failure.
     */

    template <typename T>
    Error_t writeProperty(PDSmodule module, uint8_t registerId, T registerData)
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

        memcpy(buffer + 4, &registerId, sizeof(uint8_t));
        memcpy(buffer + 4 + sizeof(uint8_t), &registerData, sizeof(registerData));

        auto result = writeReadFD(buffer, respBuffer, bufferSize);
        if (result != Error_t::OK)
            return Error_t::NOT_CONNECTED;
        return Error_t::OK;
    }

    template <typename T>
    Error_t writeProperty(PDSmodule module, Message<T> registerData)
    {
        return writeProperty(module, registerData.registerID, registerData.value);
    }

    /**
     * @brief Reads data from register
     *
     * @param registerId    Register ID
     * @param data    Data to store
     * @return  Error code indicating success or failure.
     */
    template <typename T>
    Error_t readProperty(PDSmodule module, uint8_t registerId, T &registerData)
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

        memcpy(buffer + 4, &registerId, sizeof(uint8_t));

        auto result = writeReadFD(buffer, respBuffer, bufferSize);

        if (result != Error_t::OK)
            return Error_t::NOT_CONNECTED;

        if (respBuffer[0] == 0)
        {
            memcpy(&registerData, respBuffer + 2 + sizeof(uint8_t), sizeof(T));
        }
        else
        {
            return Error_t::REQUEST_INVALID;
        }

        return Error_t::OK;
    }

    template <typename T>
    Error_t readProperty(PDSmodule module, Message<T> &registerData)
    {
        return readProperty(module, uint8_t(registerData.registerID), registerData.value);
    }

    /**
     * @brief Writes data to registers CANFD only
     *
     * @param registerId    Register ID
     * @param data    Data to store
     * @param dataType  Data type
     * @return  Error code indicating success or failure.
     */

    template <typename... T>
    Error_t writeProperties(PDSmodule module, T... message)
    {
        size_t bufferSize = 4;
        ((bufferSize += sizeof(message.registerID) + sizeof(uint32_t)), ...);
        uint8_t buffer[bufferSize] = {0};
        uint8_t respBuffer[bufferSize] = {0};

        buffer[0] = FRAME_WRITE_PROPERTY_FD;
        buffer[1] = module.type;
        buffer[2] = module.index;
        buffer[3] = (bufferSize - 4) / 5;

        bufferSize = 4;

        ([&]()
         {
        memcpy(buffer + bufferSize, &message.registerID, sizeof(uint8_t));
        memcpy(buffer + sizeof(uint8_t) + bufferSize, &message.value, sizeof(uint32_t));
        bufferSize += 5; }(),
         ...);
        auto result = writeReadFD(buffer, respBuffer, bufferSize);

        if (result != Error_t::OK)
            return Error_t::NOT_CONNECTED;

        return Error_t::OK;
    }

    /**
     * @brief Reads data from registers CANFD only
     *
     * @param registerId    Register ID
     * @param data    Data to store
     * @param dataType  Data type
     * @return  Error code indicating success or failure.
     */
    template <typename... T>
    Error_t readProperties(PDSmodule module, T &...message)
    {
        size_t bufferSize = 4;
        ((bufferSize += sizeof(uint8_t(message.registerID))), ...);
        size_t responseBufferSize = 2;
        ((responseBufferSize += sizeof(uint8_t(message.registerID)) + sizeof(uint32_t)), ...);
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
         memcpy(buffer + bufferSize, &message.registerID, sizeof(uint8_t));
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

    Error_t enable(PDSmodule module, bool enable);

    Error_t getEnable(PDSmodule module, bool &enable);

    Error_t getBusVoltage(PDSmodule module, uint32_t &busVoltage);

    Error_t getOCDdelay(PDSmodule module, uint32_t &OCDdelay);

    Error_t getOCDlevel(PDSmodule module, uint32_t &OCDlevel);

    Error_t getTemperature(PDSmodule module, float &temperature);

    Error_t getTemperatureLimit(PDSmodule module, float &temperatureLimit);

    Error_t setTemperatureLimit(PDSmodule module, float temperatureLimit);

    Error_t getLoadCurrent(PDSmodule module, int32_t &loadCurrent);

    Error_t getLoadPower(PDSmodule module, int32_t &loadPower);
};

#endif