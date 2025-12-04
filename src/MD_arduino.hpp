#include "Candleduino.hpp"

#define MAB_CAN_BUFF_SIZE 8

class MD : public MAB_DEVICE
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
    Error_t writeRegister(uint16_t registerId, T registerData)
    {
        if (FD)
        {
#if defined(TEENSYDUINO)
            uint8_t bufferSize = 2 + sizeof(uint16_t) + sizeof(registerData);
            uint8_t buffer[bufferSize] = {0};
            uint8_t respBuffer[bufferSize] = {0};
            buffer[0] = FRAME_WRITE_REGISTER_FD;
            buffer[1] = 0x00;

            memcpy(buffer + 2, &registerId, sizeof(uint16_t));
            memcpy(buffer + 2 + sizeof(uint16_t), &registerData, sizeof(registerData));

            auto result = writeReadFD(buffer, respBuffer, bufferSize);
            if (result != Error_t::OK)
                return Error_t::NOT_CONNECTED;
#endif
        }
        else
        {
            uint8_t buffer[MAB_CAN_BUFF_SIZE] = {0};
            uint8_t respBuffer[MAB_CAN_BUFF_SIZE] = {0};
            buffer[0] = FRAME_WRITE_REGISTER;
            buffer[1] = 0x00;

            memcpy(buffer + 2, &registerId, sizeof(uint16_t));
            memcpy(buffer + 2 + sizeof(uint16_t), &registerData, sizeof(registerData));

            auto result = writeRead(buffer, respBuffer);
            if (result != Error_t::OK)
                return Error_t::NOT_CONNECTED;
        }

        return Error_t::OK;
    }

    template <typename T>
    Error_t writeRegister(Message<T> registerData)
    {
        return writeRegister(registerData.registerID, registerData.value);
    }

    /**
     * @brief Reads data from register
     *
     * @param registerId    Register ID
     * @param data    Data to store
     * @return  Error code indicating success or failure.
     */
    template <typename T>
    Error_t readRegister(uint16_t registerId, T &registerData)
    {
        if (FD)
        {
#if defined(TEENSYDUINO)
            uint8_t bufferSize = 2 + sizeof(uint16_t) + sizeof(registerData);
            uint8_t buffer[bufferSize] = {0};
            uint8_t respBuffer[bufferSize] = {0};
            buffer[0] = FRAME_READ_REGISTER_FD;
            buffer[1] = 0x00;

            memcpy(buffer + 2, &registerId, sizeof(uint16_t));
            memcpy(buffer + 2 + sizeof(uint16_t), &registerData, sizeof(registerData));

            auto result = writeReadFD(buffer, respBuffer, bufferSize);

            if (result != Error_t::OK)
                return Error_t::NOT_CONNECTED;
            memcpy(&registerData, respBuffer + 2 + sizeof(uint16_t), sizeof(T));
#endif
        }
        else
        {
            uint8_t buffer[MAB_CAN_BUFF_SIZE] = {0};
            uint8_t respBuffer[MAB_CAN_BUFF_SIZE] = {0};
            buffer[0] = FRAME_READ_REGISTER;
            buffer[1] = 0x00;

            memcpy(buffer + 2, &registerId, sizeof(uint16_t));
            memcpy(buffer + 2 + sizeof(uint16_t), &registerData, sizeof(registerData));

            auto result = writeRead(buffer, respBuffer);

            if (result != Error_t::OK)
                return Error_t::NOT_CONNECTED;
            memcpy(&registerData, respBuffer + 2 + sizeof(uint16_t), sizeof(T));
        }

        return Error_t::OK;
    }

    template <typename T>
    Error_t readRegister(Message<T> &registerData)
    {
        return readRegister(uint16_t(registerData.registerID), registerData.value);
    }

#if defined(TEENSYDUINO)
    /**
     * @brief Writes data to registers CANFD only
     *
     * @param registerId    Register ID
     * @param data    Data to store
     * @param dataType  Data type
     * @return  Error code indicating success or failure.
     */
    template <typename... T>
    Error_t writeRegisters(T... message)
    {
        if (!FD)
        {
            return Error_t::WRONG_MODE;
        }
        size_t bufferSize = 2;
        ((bufferSize += sizeof(message.registerID) + sizeof(message.value)), ...);
        uint8_t buffer[bufferSize] = {0};
        uint8_t respBuffer[bufferSize] = {0};
        bufferSize = 2;

        buffer[0] = FRAME_WRITE_REGISTER_FD;
        buffer[1] = 0x00;

        ([&]()
         {
         memcpy(buffer + bufferSize, &message.registerID, sizeof(uint16_t));
         memcpy(buffer + sizeof(uint16_t) + bufferSize, &message.value, sizeof(message.value));
         bufferSize += sizeof(message.registerID) + sizeof(message.value); }(),
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
    Error_t readRegisters(T &...message)
    {
        if (!FD)
        {
            return Error_t::WRONG_MODE;
        }
        size_t bufferSize = 2;
        ((bufferSize += sizeof(message.registerID) + sizeof(message.value)), ...);
        uint8_t buffer[bufferSize] = {0};
        uint8_t respBuffer[bufferSize] = {0};
        bufferSize = 2;

        buffer[0] = FRAME_READ_REGISTER_FD;
        buffer[1] = 0x00;

        ([&]()
         {
         memcpy(buffer + bufferSize, &message.registerID, sizeof(uint16_t));
         memcpy(buffer + sizeof(uint16_t) + bufferSize, &message.value, sizeof(message.value));
         bufferSize += sizeof(message.registerID) + sizeof(message.value); }(),
         ...);

        auto result = writeReadFD(buffer, respBuffer, bufferSize);
        if (result != Error_t::OK)
            return Error_t::NOT_CONNECTED;

        bufferSize = 2;

        ([&]()
         { memcpy(&message.value, respBuffer + bufferSize + sizeof(message.registerID), sizeof(message.value)); 
        bufferSize+=sizeof(message.value)+sizeof(message.registerID); }(),
         ...);

        return Error_t::OK;
    }
#endif

    /// @brief Blink the built-in LEDs
    Error_t blink();

    /// @brief Enable PWM output of the drive
    /// @return
    Error_t enable();

    /// @brief Disable PWM output of the drive
    /// @return
    Error_t disable();

    /// @brief Reset the driver
    /// @return
    Error_t reset();

    /// @brief Clear errors present in the driver
    /// @return
    Error_t clearErrors();

    /// @brief Save configuration data to the memory
    /// @return
    Error_t save();

    /// @brief Zero out the position of the encoder
    /// @return
    Error_t zero();

    /// @brief Enable PWM output of the drive
    /// @return
    Error_t setMotionMode(motionModeMab_E motionMode);

    /// @brief Enable PWM output of the drive
    /// @return
    Error_t getMotionMode(motionModeMab_E &motionMode);

    /// @brief Get Pole Pairs value
    /// @return
    Error_t getPolePairs(uint32_t &polePairs);

    /// @brief Get mosfet temperature
    /// @return
    Error_t getMosfetTemperature(float &temperature);

    /// @brief Set position controller PID parameters
    /// @param kp
    /// @param ki
    /// @param kd
    /// @param integralMax
    /// @return
    Error_t setPositionPIDparam(float kp, float ki, float kd, float integralMax);

    /// @brief Set velocity controller PID parameters
    /// @param kp
    /// @param ki
    /// @param kd
    /// @param integralMax
    /// @return
    Error_t setVelocityPIDparam(float kp, float ki, float kd, float integralMax);

    /// @brief Set impedance controller parameters
    /// @param kp
    /// @param kd
    /// @return
    Error_t setImpedanceParams(float kp, float kd);

    /// @brief Set target position
    /// @param position target position in radians
    /// @return
    Error_t setTargetPosition(float position /*rad*/);

    /// @brief Get target position
    /// @param position target position in radians
    /// @return
    Error_t getTargetPosition(float &position /*rad*/);

    /// @brief Set target velocity
    /// @param velocity target velocity in radians per second
    /// @return
    Error_t setTargetVelocity(float velocity /*rad/s*/);

    /// @brief Get target velocity
    /// @param velocity target velocity in radians per second
    /// @return
    Error_t getTargetVelocity(float &velocity /*rad/s*/);

    /// @brief Get Position from main encoder
    Error_t getMainEncoderPosition(float &position);

    /// @brief Get Position from main encoder
    Error_t getMainEncoderVelocity(float &position);

    /// @brief Set target torque
    /// @param torque target torque in Nm
    /// @return
    Error_t getOutputEncoderPos(float &position /*Nm*/);

    /// @brief Set target torque
    /// @param torque target torque in Nm
    /// @return
    Error_t getOutputEncoderVel(float &velocity /*rad*/);
};
