#include "MD_arduino.hpp"

#if defined(TEENSYDUINO)
MD *MD::instance = nullptr;
#endif

#if defined(ARDUINO_ARCH_AVR)

MD::Error_t MD::init()
{
    while (CAN_OK != m_CAN.begin(CAN_1000KBPS))
    {
        Serial.print("Waiting for connection");
        delay(500);
    }
    queueInit(&m_receiveQueue);
    return MD::Error_t::OK;
}
MD::Error_t MD::sendFD(uint8_t *buffer, uint8_t *respBuffer, uint8_t bufferLength) { return MD::Error_t::UNKNOWN_ERROR; }; // empty function
MD::Error_t MD::send(uint8_t buffer[8], uint8_t respBuffer[8])
{
    bool popped = false;
    m_CAN.sendMsgBuf(m_canId, 0, MAB_CAN_BUFF_SIZE, buffer);
    _delay_us(500);
    if (MD::update() != MD::Error_t::OK)
        return MD::Error_t::NOT_CONNECTED;

    for (int i = 0; i < FIFO_CAPACITY; i++)
    {
        if (m_receiveQueue.count > 0)
        {
            queuePop(&m_receiveQueue, respBuffer);
            if (memcmp(respBuffer, buffer, 4))
            {
                // Flushing all pending messages when getting missmatched data
                if (CAN_MSGAVAIL == m_CAN.checkReceive())
                {
                    m_CAN.readMsgBufID(&rxId, &rxLen, rxBuffer);
                }
                memset(respBuffer, 0, MAB_CAN_BUFF_SIZE);
            }
            popped = true;
            break;
        }
    }

    if (!popped)
        return MD::Error_t::TRANSFER_FAILED;

    return MD::Error_t::OK;
}

MD::Error_t MD::update()
{
    if (CAN_MSGAVAIL == m_CAN.checkReceive())
    {
        m_CAN.readMsgBufID(&rxId, &rxLen, rxBuffer);
        if (rxId == (uint32_t)m_canId)
        {
            queuePush(&m_receiveQueue, rxBuffer);
            return MD::Error_t::OK;
        }
    }
    return MD::Error_t ::NOT_CONNECTED;
}
#elif defined(TEENSYDUINO)

MD::Error_t MD::sendFD(uint8_t *buffer, uint8_t *respBuffer, uint8_t bufferLength)
{

    CANFD_message_t tx_msg;
    tx_msg.id = m_canId;
    tx_msg.len = bufferLength;
    tx_msg.flags.extended = 0;
    tx_msg.esi = 1;
    tx_msg.brs = 0;

    for (uint8_t i = 0; i < bufferLength; i++)
    {
        tx_msg.buf[i] = buffer[i];
    }

    m_CanFD.write(tx_msg);

    delayMicroseconds(1000);

    m_CanFD.events();

    if (memcmp(buffer, respBuffer2, 4))
    {
        while (m_CanFD.events())
        {
            // Flushing RX/TX buffer
        }
        memset(respBuffer2, 0, sizeof(respBuffer2));
    }
    else
    {
        memcpy(respBuffer, respBuffer2, bufferLength);

        memset(respBuffer2, 0, sizeof(respBuffer2));
    }

    return MD::Error_t::OK;
}
MD::Error_t MD::send(uint8_t buffer[8], uint8_t respBuffer[8])
{

    CAN_message_t tx_msg;
    tx_msg.id = m_canId;

    tx_msg.len = MAB_CAN_BUFF_SIZE;
    tx_msg.flags.extended = 0;

    for (uint8_t i = 0; i < MAB_CAN_BUFF_SIZE; i++)
    {
        tx_msg.buf[i] = buffer[i];
    }

    if (m_Can->write(tx_msg))
    {
        delayMicroseconds(1000);

        bool popped = false;

        for (int i = 0; i < FIFO_CAPACITY; i++)
        {
            if (m_receiveQueue.count > 0)
            {
                queuePop(&m_receiveQueue, respBuffer);
                if (memcmp(buffer, respBuffer, 4))
                {
                    queueInit(&m_receiveQueue); // Flushing queue
                    memset(respBuffer, 0, MAB_CAN_BUFF_SIZE);
                }
                popped = true;
                break;
            }
        }

        if (!popped)
            return MD::Error_t::TRANSFER_FAILED;
    }
    else
    {
        queueInit(&m_receiveQueue);
        return MD::Error_t::TRANSFER_FAILED;
    }

    return MD::Error_t::OK;
}

void MD::canCallback(const CAN_message_t &msg)
{
    if (instance)
        instance->handleMessage(msg);
}

void MD::handleMessage(const CAN_message_t &msg)
{
    if (msg.id == (uint32_t)m_canId)
        queuePush(&m_receiveQueue, msg.buf);
}

void MD::canCallbackFD(const CANFD_message_t &msg)
{
    if (instance)
        instance->handleMessageFD(msg);
}

void MD::handleMessageFD(const CANFD_message_t &msg)
{

    if (msg.id == (uint32_t)m_canId)
        memcpy(respBuffer2, msg.buf, sizeof(msg.buf));
}

#else
MD::Error_t MD::sendFD(uint8_t *buffer, uint8_t *respBuffer, uint8_t bufferLength) { return MD::Error_t::UNKNOWN_ERROR; }; // empty function
MD::Error_t MD::send(uint8_t buffer[8], uint8_t respBuffer[8])
{
    bool popped = false;
    CanMsg const msg(CanStandardId(m_canId), MAB_CAN_BUFF_SIZE, buffer);

    if (CAN.write(msg) == 1)
    {
        delay(1);
        if (MD::update() != MD::Error_t::OK)
            return MD::Error_t::NOT_CONNECTED;

        if (memcmp(buffer, rxBuffer, 4))
        {
            // Flushing all pending frames when getting missmatched messages
            while (CAN.available())
            {
                CAN.read();
            }
            memset(rxBuffer, 0, MAB_CAN_BUFF_SIZE);
            return MD::Error_t::TRANSFER_FAILED;
        }
        else
        {
            memcpy(respBuffer, rxBuffer, MAB_CAN_BUFF_SIZE);
        }
    }
    else
    {
        return MD::Error_t::NOT_CONNECTED;
    }

    return MD::Error_t::OK;
}

MD::Error_t MD::init()
{
    if (!CAN.begin(CanBitRate::BR_1000k))
    {
        Serial.print("Waiting for connection");
        delay(500);
    }
    return MD::Error_t::OK;
}

MD::Error_t MD::update()
{
    if (CAN.available())
    {
        CanMsg const msg = CAN.read();

        if (CanStandardId(msg.id) == (uint32_t)m_canId)
        {
            memcpy(rxBuffer, msg.data, MAB_CAN_BUFF_SIZE);
            return MD::Error_t::OK;
        }
    }
    return MD::Error_t::NOT_CONNECTED;
}
template <typename... T>
MD::Error_t MD::writeRegisters(T &...message) {};
#endif

MD::Error_t MD::blink()
{
    return writeRegister(REG_BLINK, 0x01);
}

MD::Error_t MD::enable()
{
    return writeRegister(REG_CONTROL_WORD, CONTROL_WORD_ENABLE);
}

MD::Error_t MD::disable()
{
    return writeRegister(REG_CONTROL_WORD, CONTROL_WORD_DISABLE);
}

MD::Error_t MD::reset()
{
    return writeRegister(REG_RESET, 0x01);
}

MD::Error_t MD::clearErrors()
{
    return writeRegister(REG_CLEAR_ERRORS, 0x01);
}

MD::Error_t MD::save()
{
    return writeRegister(REG_SAVE, 0x01);
}

MD::Error_t MD::zero()
{
    return writeRegister(REG_ZERO, 0x01);
}

MD::Error_t MD::setMotionMode(motionModeMab_E motionMode)
{
    return writeRegister(REG_MOTION_MODE_COMMAND, motionMode);
}

MD::Error_t MD::getMotionMode(motionModeMab_E &motionMode)
{
    return readRegister(REG_MOTION_MODE_STATUS, motionMode);
}

MD::Error_t MD::getPolePairs(u32 &polePairs)
{
    return readRegister(REG_MOTOR_POLE_PAIRS, polePairs);
}

MD::Error_t MD::getMosfetTemperature(float &temperature)
{
    return readRegister(0x806, temperature);
}

MD::Error_t MD::setPositionPIDparam(float kp, float ki, float kd, float integralMax)
{
    Message<float> m_kp = {REG_POSITION_KP, kp};
    Message<float> m_ki = {REG_POSITION_KI, kp};
    Message<float> m_kd = {REG_POSITION_KD, kd};
    Message<float> m_int = {REG_POSITION_WINDUP, integralMax};
    if (FD)
    {
        return writeRegisters(m_kp, m_ki, m_kd, m_int);
    }
    else
    {
        writeRegister(m_kp);
        writeRegister(m_ki);
        writeRegister(m_kd);
        writeRegister(m_int);
    }
    return MD::Error_t::OK;
}

MD::Error_t MD::setVelocityPIDparam(float kp, float ki, float kd, float integralMax)
{
    Message<float> m_kp = {REG_VELOCITY_KP, kp};
    Message<float> m_ki = {REG_VELOCITY_KI, kp};
    Message<float> m_kd = {REG_VELOCITY_KD, kd};
    Message<float> m_int = {REG_VELOCITY_WINDUP, integralMax};
    if (FD)
    {
        return writeRegisters(m_kp, m_ki, m_kd, m_int);
    }
    else
    {
        writeRegister(m_kp);
        writeRegister(m_ki);
        writeRegister(m_kd);
        writeRegister(m_int);
    }
    return MD::Error_t::OK;
}

MD::Error_t MD::setImpedanceParams(float kp, float kd)
{
    Message<float> m_kp = {REG_VELOCITY_KP, kp};
    Message<float> m_kd = {REG_IMPEDANCE_KD, kd};
    if (FD)
    {
        return writeRegisters(m_kp, m_kd);
    }
    else
    {
        writeRegister(m_kp);
        writeRegister(m_kd);
    }
    return MD::Error_t::OK;
}

MD::Error_t MD::getTargetPosition(float &position)
{
    return readRegister(REG_MAIN_ENCODER_POS, position);
}

MD::Error_t MD::setTargetPosition(float position)
{
    return writeRegister(REG_TARGET_POS, position);
}

MD::Error_t MD::getTargetVelocity(float &velocity)
{
    return readRegister(REG_TARGET_VEL, velocity);
}

MD::Error_t MD::setTargetVelocity(float velocity)
{
    return writeRegister(REG_TARGET_VEL, velocity);
}

MD::Error_t MD::getMainEncoderPosition(float &position)
{
    return readRegister(REG_MAIN_ENCODER_POS, position);
}

MD::Error_t MD::getOutputEncoderPos(float &position)
{
    return readRegister(REG_OUTPUT_ENCODER_POS, position);
}

MD::Error_t MD::getOutputEncoderVel(float &velocity)
{
    return readRegister(REG_OUTPUT_ENCODER_VEL, velocity);
}
