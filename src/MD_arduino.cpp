#include "MD_arduino.hpp"

#if defined(TEENSYDUINO)
MD *MD::instance = nullptr;
uint8_t MD::respBuffer2[64] = {0};
uint32_t MD::rxFDID = 0;
#endif

#if defined(ARDUINO_ARCH_AVR)

MD::Error_t MD::init()
{
    uint8_t timeoutCount = 5;
    while (CAN_OK != m_CAN.begin(CAN_1000KBPS))
    {
        Serial.print("Waiting for connection");
        delay(500);
        timeoutCount--;
        if (!timeoutCount)
            return MD::Error_t::NOT_CONNECTED;
    }
    queueInit(&m_receiveQueue);
    return MD::Error_t::OK;
}
MD::Error_t MD::writeRead(uint8_t buffer[8], uint8_t respBuffer[8])
{
    bool popped = false;
    m_CAN.sendMsgBuf(m_canId, 0, MAB_CAN_BUFF_SIZE, buffer);
    _delay_us(500);
    if (MD::receive() != MD::Error_t::OK)
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

MD::Error_t MD::receive()
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

    m_CanFD->write(tx_msg);

    delayMicroseconds(1000);

    m_CanFD->events();
    if (memcmp(buffer, respBuffer2, 4))
    {
        while (m_CanFD->events())
        {
            // Flushing RX/TX buffer
        }
        memset(respBuffer2, 0, sizeof(respBuffer2));
    }
    else
    {
        if (m_canId == rxFDID)
            memcpy(respBuffer, respBuffer2, bufferLength);

        memset(respBuffer2, 0, sizeof(respBuffer2));
    }

    return MD::Error_t::OK;
}
MD::Error_t MD::writeRead(uint8_t buffer[8], uint8_t respBuffer[8])
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
        delayMicroseconds(500);

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
    memcpy(respBuffer2, msg.buf, sizeof(msg.buf));
    rxFDID = msg.id;
}

#else
MD::Error_t MD::writeRead(uint8_t buffer[8], uint8_t respBuffer[8])
{
    bool popped = false;
    CanMsg const msg(CanStandardId(m_canId), MAB_CAN_BUFF_SIZE, buffer);

    if (CAN.write(msg) == 1)
    {
        delayMicroseconds(500);
        if (MD::receive() != MD::Error_t::OK)
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
    uint8_t timeoutCount = 5;
    while (!CAN.begin(CanBitRate::BR_1000k))
    {
        Serial.print("Waiting for connection");
        delay(500);
        timeoutCount--;
        if (!timeoutCount)
        {
            Serial.println("CAN begin done not XD"); // confirm this runs
            return MD::Error_t::NOT_CONNECTED;
        }
    }
    Serial.println("CAN begin done"); // confirm this runs
    return MD::Error_t::OK;
}

MD::Error_t MD::receive()
{

    if (CAN.available())
    {
        CanMsg const msg = CAN.read();

        if ((msg.id & 0x7FF) == m_canId)
        {
            memcpy(rxBuffer, msg.data, 8);
            return MD::Error_t::OK;
        }
    }

    return MD::Error_t::NOT_CONNECTED;
}
#endif

MD::Error_t MD::blink()
{
    return writeRegister(BLINK, 0x01);
}

MD::Error_t MD::enable()
{
    return writeRegister(CONTROL_WORD, CONTROL_WORD_ENABLE);
}

MD::Error_t MD::disable()
{
    return writeRegister(CONTROL_WORD, CONTROL_WORD_DISABLE);
}

MD::Error_t MD::reset()
{
    return writeRegister(RESET, 0x01);
}

MD::Error_t MD::clearErrors()
{
    return writeRegister(CLEAR_ERRORS, 0x01);
}

MD::Error_t MD::save()
{
    return writeRegister(SAVE, 0x01);
}

MD::Error_t MD::zero()
{
    return writeRegister(ZERO, 0x01);
}

MD::Error_t MD::setMotionMode(motionModeMab_E motionMode)
{
    return writeRegister(MOTION_MODE_COMMAND, motionMode);
}

MD::Error_t MD::getMotionMode(motionModeMab_E &motionMode)
{
    return readRegister(MOTION_MODE_STATUS, motionMode);
}

MD::Error_t MD::getPolePairs(uint32_t &polePairs)
{
    return readRegister(MOTOR_POLE_PAIRS, polePairs);
}

MD::Error_t MD::getMosfetTemperature(float &temperature)
{
    return readRegister(0x806, temperature);
}

MD::Error_t MD::setPositionPIDparam(float kp, float ki, float kd, float integralMax)
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
    return MD::Error_t::OK;
}

MD::Error_t MD::setVelocityPIDparam(float kp, float ki, float kd, float integralMax)
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
    return MD::Error_t::OK;
}

MD::Error_t MD::setImpedanceParams(float kp, float kd)
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
    return MD::Error_t::OK;
}

MD::Error_t MD::getTargetPosition(float &position)
{
    return readRegister(MAIN_ENCODER_POS, position);
}

MD::Error_t MD::setTargetPosition(float position)
{
    return writeRegister(TARGET_POS, position);
}

MD::Error_t MD::getTargetVelocity(float &velocity)
{
    return readRegister(TARGET_VEL, velocity);
}

MD::Error_t MD::setTargetVelocity(float velocity)
{
    return writeRegister(TARGET_VEL, velocity);
}

MD::Error_t MD::getMainEncoderPosition(float &position)
{
    return readRegister(MAIN_ENCODER_POS, position);
}

MD::Error_t MD::getOutputEncoderPos(float &position)
{
    return readRegister(OUTPUT_ENCODER_POS, position);
}

MD::Error_t MD::getOutputEncoderVel(float &velocity)
{
    return readRegister(OUTPUT_ENCODER_VEL, velocity);
}
