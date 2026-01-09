#include "MD_arduino.hpp"

Error_t MD::blink()
{
    return writeRegister(BLINK, 0x01);
}

Error_t MD::enable()
{
    return writeRegister(CONTROL_WORD, CONTROL_WORD_ENABLE);
}

Error_t MD::disable()
{
    return writeRegister(CONTROL_WORD, CONTROL_WORD_DISABLE);
}

Error_t MD::reset()
{
    return writeRegister(RESET, 0x01);
}

Error_t MD::clearErrors()
{
    return writeRegister(CLEAR_ERRORS, 0x01);
}

Error_t MD::save()
{
    return writeRegister(SAVE, 0x01);
}

Error_t MD::zero()
{
    return writeRegister(ZERO, 0x01);
}

Error_t MD::setMotionMode(motionModeMab_E motionMode)
{
    return writeRegister(MOTION_MODE_COMMAND, motionMode);
}

Error_t MD::getMotionMode(motionModeMab_E &motionMode)
{
    return readRegister(MOTION_MODE_STATUS, motionMode);
}

Error_t MD::getPolePairs(uint32_t &polePairs)
{
    return readRegister(MOTOR_POLE_PAIRS, polePairs);
}

Error_t MD::getMosfetTemperature(float &temperature)
{
    return readRegister(0x806, temperature);
}

Error_t MD::setPositionPIDparam(float kp, float ki, float kd, float integralMax)
{
    Message<float> m_kp = {POSITION_KP, kp};
    Message<float> m_ki = {POSITION_KI, kp};
    Message<float> m_kd = {POSITION_KD, kd};
    Message<float> m_int = {POSITION_WINDUP, integralMax};
    if (FD)
    {
#if defined(Teensyduino)
        return writeRegisters(m_kp, m_ki, m_kd, m_int);
#endif
    }
    else
    {
        writeRegister(m_kp);
        writeRegister(m_ki);
        writeRegister(m_kd);
        writeRegister(m_int);
    }
    return Error_t::OK;
}

Error_t MD::setVelocityPIDparam(float kp, float ki, float kd, float integralMax)
{
    Message<float> m_kp = {VELOCITY_KP, kp};
    Message<float> m_ki = {VELOCITY_KI, kp};
    Message<float> m_kd = {VELOCITY_KD, kd};
    Message<float> m_int = {VELOCITY_WINDUP, integralMax};
    if (FD)
    {
#if defined(Teensyduino)
        return writeRegisters(m_kp, m_ki, m_kd, m_int);
#endif
    }
    else
    {
        writeRegister(m_kp);
        writeRegister(m_ki);
        writeRegister(m_kd);
        writeRegister(m_int);
    }
    return Error_t::OK;
}

Error_t MD::setImpedanceParams(float kp, float kd)
{
    Message<float> m_kp = {VELOCITY_KP, kp};
    Message<float> m_kd = {IMPEDANCE_KD, kd};
    if (FD)
    {
#if defined(Teensyduino)
        return writeRegisters(m_kp, m_kd);
#endif
    }
    else
    {
        writeRegister(m_kp);
        writeRegister(m_kd);
    }
    return Error_t::OK;
}

Error_t MD::getTargetPosition(float &position)
{
    return readRegister(MAIN_ENCODER_POS, position);
}

Error_t MD::setTargetPosition(float position)
{
    return writeRegister(TARGET_POS, position);
}

Error_t MD::getTargetVelocity(float &velocity)
{
    return readRegister(TARGET_VEL, velocity);
}

Error_t MD::setTargetVelocity(float velocity)
{
    return writeRegister(TARGET_VEL, velocity);
}

Error_t MD::getMainEncoderPosition(float &position)
{
    return readRegister(MAIN_ENCODER_POS, position);
}

Error_t MD::getOutputEncoderPos(float &position)
{
    return readRegister(OUTPUT_ENCODER_POS, position);
}

Error_t MD::getOutputEncoderVel(float &velocity)
{
    return readRegister(OUTPUT_ENCODER_VEL, velocity);
}
