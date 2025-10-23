#include "Arduino.h"

#include "mab_types.hpp"

#include "mab_can/mab_can.h"
#include "mab_queue/mab_queue.h"

#if defined(ARDUINO_ARCH_AVR)
#include "mcp_canbus.h"
#include "SPI.h"
#elif defined(ARDUINO_ARCH_SAM)
#include <Arduino_CAN.h>
#endif

#define MAB_CAN_BUFF_SIZE 8

// namespace mab
// {
class MD
{

protected:
    bool send(uint8_t buffer[8], uint8_t respBuffer[8], bool &popped);

    FifoFrame_S m_receiveQueue;
    long unsigned int rxId;
    unsigned char rxLen;
    unsigned char rxBuffer[8];

public:
    enum class Error_t : u8
    {
        UNKNOWN_ERROR,
        OK,
        REQUEST_INVALID,
        TRANSFER_FAILED,
        NOT_CONNECTED
    };
#if defined(ARDUINO_ARCH_AVR)
    /// @brief MCP CAN object
    MCP_CAN m_CAN;
    int m_SPI_CS_PIN = 9;
    MD() : m_CAN(m_SPI_CS_PIN) {};

    MD(int SPI_CS_PIN) : m_CAN(SPI_CS_PIN) {};
#elif defined(ARDUINO_ARCH_SAM)
    MD() {};
#endif

    Error_t init();

    bool update();
    /// @brief MD can node ID
    mab::canId_t m_canId;

    /// @brief Blink the built-in LEDs
    Error_t blink();

    /// @brief Enable PWM output of the drive
    /// @return
    Error_t enable();

    /// @brief Enable PWM output of the drive
    /// @return
    Error_t getMotionMode(motionModeMab_E motionMode);

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

    /// @brief Set current limit associated with motor that is driven
    /// @param currentLimit Current limit in Amps
    /// @return
    Error_t setCurrentLimit(float currentLimit /*A*/);

    /// @brief Set update rate for the torque control loop
    /// @param torqueBandwidth Update rate in Hz
    /// @return
    Error_t setTorqueBandwidth(u16 torqueBandwidth /*Hz*/);

    /// @brief Set controller mode
    /// @param mode Mode selected
    /// @return
    Error_t setMotionMode(mab::MdMode_E mode);

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

    /// @brief Set max torque to be output by the controller
    /// @param maxTorque max torque value in Nm
    /// @return
    Error_t setMaxTorque(float maxTorque /*Nm*/);

    /// @brief Set target velocity of the profile movement
    /// @param profileVelocity
    /// @return
    Error_t setProfileVelocity(float profileVelocity /*s^-1*/);

    /// @brief Set target profile acceleration when performing profile movement
    /// @param profileAcceleration
    /// @return
    Error_t setProfileAcceleration(float profileAcceleration /*s^-2*/);

    /// @brief Set target position
    /// @param position target position in radians
    /// @return
    Error_t setTargetPosition(float position /*rad*/);

    /// @brief Set target velocity
    /// @param velocity target velocity in radians per second
    /// @return
    Error_t setTargetVelocity(float velocity /*rad/s*/);

    /// @brief Set target torque
    /// @param torque target torque in Nm
    /// @return
    Error_t setTargetTorque(float torque /*Nm*/);
};
// } // namespace mab
