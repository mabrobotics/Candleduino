// MIT License

// Copyright (c) 2025 MAB Robotics

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "mab_types.hpp"
#include "mab_can.hpp"
#include "mab_queue/mab_queue.h"

#if defined(ARDUINO_ARCH_AVR)
#include "mcp_canbus.h"
#include "SPI.h"
#elif defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)

#elif defined(TEENSYDUINO)
#include <FlexCAN_T4.h>
#elif defined(ARDUINO_ARCH_RENESAS)
#include <Arduino_CAN.h>
#else
#include <Arduino_CAN.h>
#endif

#define MAB_CAN_BUFF_SIZE 8
/*
#define REGISTER_LIST                                \
    MD_REG(Nullreg, u8, 0x000, RO)                   \
    MD_REG(CanID, u32, 0x001, RW)                    \
    MD_REG(CanBaudrate, u32, 0x002, RW)              \
    MD_REG(CanWatchdog, u16, 0x003, RW)              \
    MD_REG(CanTermination, u8, 0x004, RW)            \
                                                     \
    MD_REG(MotorPolePairs, u32, 0x011, RW)           \
    MD_REG(MotorKt, float, 0x012, RW)                \
    MD_REG(MotorKtPhaseA, float, 0x013, RW)          \
    MD_REG(MotorKtPhaseB, float, 0x014, RW)          \
    MD_REG(MotorKtPhaseC, float, 0x015, RW)          \
    MD_REG(MotorIMax, float, 0x016, RW)              \
    MD_REG(MotorGearRatio, float, 0x017, RW)         \
    MD_REG(MotorTorqueBandwidth, u16, 0x018, RW)     \
    MD_REG(MotorFriction, float, 0x019, RW)          \
    MD_REG(MotorStiction, float, 0x01A, RW)          \
    MD_REG(MotorResistance, float, 0x01B, RO)        \
    MD_REG(MotorInductance, float, 0x01C, RO)        \
    MD_REG(MotorKV, u16, 0x01D, RW)                  \
    MD_REG(MotorCalibrationMode, u8, 0x01E, RW)      \
    MD_REG(MotorThermistorType, u8, 0x01F, RW)       \
                                                     \
    MD_REG(AuxEncoder, u8, 0x020, RW)                \
    MD_REG(AuxEncoderDir, u8, 0x021, WO)             \
    MD_REG(AuxEncoderDefaultBaud, u32, 0x022, RW)    \
    MD_REG(AuxEncoderVelocity, float, 0x023, RO)     \
    MD_REG(AuxEncoderPosition, float, 0x024, RO)     \
    MD_REG(AuxEncoderMode, u8, 0x025, RW)            \
    MD_REG(AuxEncoderCalibrationMode, u8, 0x026, RW) \
                                                     \
    MD_REG(MotorPosPidKp, float, 0x030, RW)          \
    MD_REG(MotorPosPidKi, float, 0x031, RW)          \
    MD_REG(MotorPosPidKd, float, 0x032, RW)          \
    MD_REG(MotorPosPidWindup, float, 0x034, RW)      \
                                                     \
    MD_REG(MotorVelPidKp, float, 0x040, RW)          \
    MD_REG(MotorVelPidKi, float, 0x041, RW)          \
    MD_REG(MotorVelPidKd, float, 0x042, RW)          \
    MD_REG(MotorVelPidWindup, float, 0x044, RW)      \
                                                     \
    MD_REG(MotorImpPidKp, float, 0x050, RW)          \
    MD_REG(MotorImpPidKd, float, 0x051, RW)          \
                                                     \
    MD_REG(MainEncoderVelocity, float, 0x062, RO)    \
    MD_REG(MainEncoderPosition, float, 0x063, RO)    \
    MD_REG(MotorTorque, float, 0x064, RO)            \
                                                     \
    MD_REG(HomingMode, u8, 0x070, RW)                \
    MD_REG(HomingMaxTravel, float, 0x071, RW)        \
    MD_REG(HomingVelocity, float, 0x072, RW)         \
    MD_REG(HomingTorque, float, 0x073, RW)           \
                                                     \
    MD_REG(RunSaveCmd, u8, 0x080, WO)                \
    MD_REG(RunTestMainEncoderCmd, u8, 0x081, WO)     \
    MD_REG(RunTestAuxEncoderCmd, u8, 0x082, WO)      \
    MD_REG(RunCalibrateCmd, u8, 0x083, WO)           \
    MD_REG(RunCalibrateAuxEncoderCmd, u8, 0x084, WO) \
    MD_REG(RunCalibratePiGains, u8, 0x085, WO)       \
    MD_REG(RunHoming, u8, 0x086, WO)                 \
    MD_REG(RunRestoreFactoryConfig, u8, 0x087, WO)   \
    MD_REG(RunReset, u8, 0x088, WO)                  \
    MD_REG(RunClearWarnings, u8, 0x089, WO)          \
    MD_REG(RunClearErrors, u8, 0x08A, WO)            \
    MD_REG(RunBlink, u8, 0x08B, WO)                  \
    MD_REG(RunZero, u8, 0x08C, WO)                   \
    MD_REG(RunCanReinit, u8, 0x08D, WO)              \
                                                     \
    MD_REG(CalAuxEncoderStdDev, float, 0x100, RO)    \
    MD_REG(CalAuxEncoderMinE, float, 0x101, RO)      \
    MD_REG(CalAuxEncoderMaxE, float, 0x102, RO)      \
    MD_REG(CalMainEncoderStdDev, float, 0x103, RO)   \
    MD_REG(CalMainEncoderMinE, float, 0x104, RO)     \
    MD_REG(CalMainEncoderMaxE, float, 0x105, RO)     \
                                                     \
    MD_REG(PositionLimitMax, float, 0x110, RW)       \
    MD_REG(PositionLimitMin, float, 0x111, RW)       \
    MD_REG(MaxTorque, float, 0x112, RW)              \
    MD_REG(MaxVelocity, float, 0x113, RW)            \
    MD_REG(MaxAcceleration, float, 0x114, RW)        \
    MD_REG(MaxDeceleration, float, 0x115, RW)        \
                                                     \
    MD_REG(ProfileVelocity, f32, 0x120, RW)          \
    MD_REG(ProfileAcceleration, f32, 0x121, RW)      \
    MD_REG(ProfileDeceleration, f32, 0x122, RW)      \
    MD_REG(QuickStopDeceleration, f32, 0x123, RW)    \
    MD_REG(PositionWindow, f32, 0x124, RW)           \
    MD_REG(VelocityWindow, f32, 0x125, RW)           \
                                                     \
    MD_REG(MotionModeCommand, u8, 0x140, WO)         \
    MD_REG(MotionModeStatus, u8, 0x141, RO)          \
    MD_REG(State, u16, 0x142, RW)                    \
                                                     \
    MD_REG(TargetPosition, float, 0x150, RW)         \
    MD_REG(TargetVelocity, float, 0x151, RW)         \
    MD_REG(TargetTorque, float, 0x152, RW)           \
                                                     \
    MD_REG(UserGpioConfiguration, u8, 0x160, RW)     \
    MD_REG(UserGpioState, u16, 0x161, RO)            \
                                                     \
    MD_REG(ReverseDirection, u8, 0x600, RW)          \
                                                     \
    MD_REG(ShuntResistance, float, 0x700, RW)        \
                                                     \
    MD_REG(BuildDate, u32, 0x800, RO)                \
    MD_REG(FirmwareVersion, u32, 0x802, RO)          \
    MD_REG(LegacyHardwareVersion, u8, 0x803, RO)     \
    MD_REG(BridgeType, u8, 0x804, RO)                \
    MD_REG(QuickStatus, u16, 0x805, RO)              \
    MD_REG(MosfetTemperature, f32, 0x806, RO)        \
    MD_REG(MotorTemperature, f32, 0x807, RO)         \
    MD_REG(MotorShutdownTemp, u8, 0x808, RW)         \
    MD_REG(MainEncoderStatus, u32, 0x809, RO)        \
    MD_REG(AuxEncoderStatus, u32, 0x80A, RO)         \
    MD_REG(CalibrationStatus, u32, 0x80B, RO)        \
    MD_REG(BridgeStatus, u32, 0x80C, RO)             \
    MD_REG(HardwareStatus, u32, 0x80D, RO)           \
    MD_REG(CommunicationStatus, u32, 0x80E, RO)      \
    MD_REG(HomingStatus, u32, 0x80F, RO)             \
    MD_REG(MotionStatus, u32, 0x810, RO)             \
    MD_REG(DcBusVoltage, f32, 0x811, RO)             \
    MD_REG(BootloaderFixed, u8, 0x812, RO)           \
    MD_REG(MiscStatus, u32, 0x813, RO)
*/

#pragma once
#ifndef USE_CANFD
#define USE_CANFD 1 // default is classic CAN
#endif

class MD
{

public:
    enum class Error_t : u8
    {
        UNKNOWN_ERROR,
        OK,
        REQUEST_INVALID,
        TRANSFER_FAILED,
        NOT_CONNECTED
    };

    template <typename T>
    struct Message
    {
        uint16_t registerID;
        T value;
    };

protected:
    Error_t send(uint8_t buffer[8], uint8_t respBuffer[8]);
    Error_t sendFD(uint8_t *buffer, uint8_t *respBuffer, uint8_t length);
    FifoFrame_S m_receiveQueue;
    long unsigned int rxId;
    unsigned char rxLen;
    unsigned char rxBuffer[8];
    static MD *instance;

public:
#if defined(ARDUINO_ARCH_AVR)
    MCP_CAN m_CAN;

    int m_SPI_CS_PIN = 9;

    /// @brief Create MD object with deafult SPI CS pin 9
    /// @return
    MD(mab::canId_t canId) : m_CAN(m_SPI_CS_PIN), m_canId(canId) {};

    /// @brief Create MD object with custom SPI CS pin
    /// @return
    MD(mab::canId_t canId, int SPI_CS_PIN) : m_CAN(SPI_CS_PIN), m_canId(canId) {};
#elif defined(TEENSYDUINO)
#if USE_CANFD
    FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> m_Can;
#else
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> m_Can;
#endif
    /// @brief Default constructor
    /// @return
    MD(mab::canId_t canId) : m_canId(canId)
    {
        instance = this;
    };

    void handleMessage(const CAN_message_t &msg);

    /// @brief Static callback required by FlexCAN_T4
    /// @return
    static void canCallback(const CAN_message_t &msg);

    void handleMessageFD(const CANFD_message_t &msg);

    /// @brief Static callback required by FlexCAN_T4
    /// @return
    static void canCallbackFD(const CANFD_message_t &msg);

#else
    /// @brief Default constructor
    /// @return
    MD(mab::canId_t canId) : m_canId(canId) {};
#endif
    /// @brief Initialize MD CAN communication
    /// @return
    Error_t init()
    {
        Serial.println("CAN begin done"); // confirm this runs

#if USE_CANFD
        CANFD_timings_t config;
        config.clock = CLK_24MHz;
        config.baudrate = 1000000;
        config.baudrateFD = 1000000;
        config.propdelay = 190;
        config.bus_length = 1;
        config.sample = 70;
        m_Can.begin();
        m_Can.setBaudRate(config);
        m_Can.setRegions(64);
        m_Can.onReceive(MB0, canCallbackFD);
        m_Can.enableMBInterrupts();
        m_Can.mailboxStatus();

#else
        m_Can.begin();
        m_Can.setBaudRate(1000000);
        m_Can.setMaxMB(16);
        m_Can.enableFIFO();
        m_Can.enableFIFOInterrupt();
        m_Can.onReceive(canCallback);
        m_Can.mailboxStatus();
#endif

        return MD::Error_t::OK;
    }

    /// @brief Update receive queue
    /// @return
    Error_t update();

    /// @brief MD can node ID
    mab::canId_t m_canId;

    void printHexBuffer(const uint8_t *buf, size_t len)
    {
        for (size_t i = 0; i < len; i++)
            Serial.printf("%02X ", buf[i]);
        Serial.println();
    }

    /**
     * @brief Writes data to register
     *
     * @param registerId    Register ID
     * @param data    Data to write
     * @param dataType  Data type
     * @return  Error code indicating success or failure.
     */

    template <typename T>
    Error_t writeRegister(uint16_t registerId, T registerData)
    {
        uint8_t buffer[MAB_CAN_BUFF_SIZE] = {0};
        if (!writeRegisterPrepareDataFrame<T>(
                buffer, MAB_CAN_BUFF_SIZE, registerId, &registerData, false))
            return MD::Error_t::UNKNOWN_ERROR;
        uint8_t respBuffer[MAB_CAN_BUFF_SIZE] = {0};

        auto result = send(buffer, respBuffer);
        if (result != MD::Error_t::OK)
            return MD::Error_t::NOT_CONNECTED;

        T data = 0;

        parseResponse(respBuffer, MAB_CAN_BUFF_SIZE, 0x42, registerId, &data);

        if (data == (T)registerData)
            return MD::Error_t::OK;
        else
            return MD::Error_t::REQUEST_INVALID;
    }

    template <typename... T>
    Error_t writeRegisters(T... message)
    {
        size_t bufferSize = 2;
        ((bufferSize += sizeof(message.registerID) + sizeof(message.value)), ...);
        uint8_t buffer[bufferSize] = {0};

        bufferSize = 2;

        if (false)
            buffer[0] = FRAME_WRITE_REGISTER_DEFAULT_RESPONSE;
        else
            buffer[0] = FRAME_WRITE_REGISTER_FD;
        buffer[1] = 0x00;

        ([&]()
         {
         memcpy(buffer + bufferSize, &message.registerID, sizeof(uint16_t));
         memcpy(buffer + sizeof(uint16_t) + bufferSize, &message.value, sizeof(message.value));
         bufferSize += sizeof(message.registerID) + sizeof(message.value); }(),
         ...);
        printHexBuffer(buffer, bufferSize);
        uint8_t respBuffer[bufferSize] = {0};

        auto result = sendFD(buffer, respBuffer, bufferSize);
        if (result != MD::Error_t::OK)
            return MD::Error_t::NOT_CONNECTED;

        return MD::Error_t::OK;
    }

    template <typename T>
    Error_t writeRegister(MD::Message<T> registerData)
    {
        writeRegister(uint16_t(registerData.registerID), registerData.value);
        return MD::Error_t::OK;
    }
    /**
     * @brief Reads data from register
     *
     * @param registerId    Register ID
     * @param data    Data to store
     * @param dataType  Data type
     * @return  Error code indicating success or failure.
     */
    template <typename T>
    Error_t readRegister(uint16_t registerId, T &registerData)
    {
        uint8_t buffer[MAB_CAN_BUFF_SIZE] = {0};
        uint8_t respBuffer[MAB_CAN_BUFF_SIZE] = {0};

        if (!readRegisterPrepareDataFrame<T>(&buffer, MAB_CAN_BUFF_SIZE, registerId))
            return MD::Error_t::UNKNOWN_ERROR;

        if (send(buffer, respBuffer) != MD::Error_t::OK)
            return MD::Error_t::NOT_CONNECTED;

        parseResponse(
            &respBuffer, MAB_CAN_BUFF_SIZE, FRAME_READ_REGISTER, registerId, &registerData);

        return MD::Error_t::OK;
    }

    template <typename T>
    Error_t readRegister(MD::Message<T> &registerData)
    {
        readRegister(uint16_t(registerData.registerID), registerData.value);
        return MD::Error_t::OK;
    }

    template <typename... T>
    Error_t readRegisters(T &...message)
    {
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
        printHexBuffer(buffer, sizeof(buffer));

        Serial.println(bufferSize);
        auto result = sendFD(buffer, respBuffer, bufferSize);
        if (result != MD::Error_t::OK)
            return MD::Error_t::NOT_CONNECTED;

        bufferSize = 2;

        ([&]()
         { memcpy(&message.value, respBuffer + bufferSize + sizeof(message.registerID), sizeof(message.value)); 
        bufferSize+=sizeof(message.value)+sizeof(message.registerID); }(),
         ...);
        printHexBuffer(respBuffer, sizeof(respBuffer));
        return MD::Error_t::OK;
    }

    /// @brief Blink the built-in LEDs
    Error_t blink();

    /// @brief Enable PWM output of the drive
    /// @return
    Error_t enable();

    /// @brief Enable PWM output of the drive
    /// @return
    Error_t getMotionMode(motionModeMab_E &motionMode);

    /// @brief Get Pole Pairs value
    /// @return
    Error_t getPolePairs(u32 &polePairs);

    /// @brief Get Position
    Error_t getPosition(f32 &position);

    /// @brief Disable PWM output of the drive
    /// @return
    Error_t disable();

    // /// @brief Reset the driver
    // /// @return
    // Error_t reset();

    // /// @brief Clear errors present in the driver
    // /// @return
    // Error_t clearErrors();

    // /// @brief Save configuration data to the memory
    // /// @return
    // Error_t save();

    // /// @brief Zero out the position of the encoder
    // /// @return
    // Error_t zero();

    // /// @brief Set current limit associated with motor that is driven
    // /// @param currentLimit Current limit in Amps
    // /// @return
    // Error_t setCurrentLimit(float currentLimit /*A*/);

    // /// @brief Set update rate for the torque control loop
    // /// @param torqueBandwidth Update rate in Hz
    // /// @return
    // Error_t setTorqueBandwidth(u16 torqueBandwidth /*Hz*/);

    // /// @brief Set controller mode
    // /// @param mode Mode selected
    // /// @return
    // Error_t setMotionMode(mab::MdMode_E mode);

    // /// @brief Set position controller PID parameters
    // /// @param kp
    // /// @param ki
    // /// @param kd
    // /// @param integralMax
    // /// @return
    // Error_t setPositionPIDparam(float kp, float ki, float kd, float integralMax);

    // /// @brief Set velocity controller PID parameters
    // /// @param kp
    // /// @param ki
    // /// @param kd
    // /// @param integralMax
    // /// @return
    // Error_t setVelocityPIDparam(float kp, float ki, float kd, float integralMax);

    // /// @brief Set impedance controller parameters
    // /// @param kp
    // /// @param kd
    // /// @return
    // Error_t setImpedanceParams(float kp, float kd);

    // /// @brief Set max torque to be output by the controller
    // /// @param maxTorque max torque value in Nm
    // /// @return
    // Error_t setMaxTorque(float maxTorque /*Nm*/);

    // /// @brief Set target velocity of the profile movement
    // /// @param profileVelocity
    // /// @return
    // Error_t setProfileVelocity(float profileVelocity /*s^-1*/);

    // /// @brief Set target profile acceleration when performing profile movement
    // /// @param profileAcceleration
    // /// @return
    // Error_t setProfileAcceleration(float profileAcceleration /*s^-2*/);

    // /// @brief Set target position
    // /// @param position target position in radians
    // /// @return
    // Error_t setTargetPosition(float position /*rad*/);

    // /// @brief Set target velocity
    // /// @param velocity target velocity in radians per second
    // /// @return
    // Error_t setTargetVelocity(float velocity /*rad/s*/);

    // /// @brief Set target torque
    // /// @param torque target torque in Nm
    // /// @return
    // Error_t setTargetTorque(float torque /*Nm*/);
};
