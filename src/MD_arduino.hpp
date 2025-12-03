#include "Candleduino.hpp"

#define MAB_CAN_BUFF_SIZE 8

class MD : public MAB_DEVICE
{
public:
    using MAB_DEVICE::MAB_DEVICE;

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
