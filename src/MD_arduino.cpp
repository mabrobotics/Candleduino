#include "MD_arduino.hpp"

#if defined(ARDUINO_ARCH_AVR)

MD::Error_t MD::init()
{
    while (CAN_OK != m_CAN.begin(CAN_1000KBPS))
    {
        delay(100);
    }
    queueInit(&m_receiveQueue);
    return MD::Error_t::OK;
}

bool MD::send(uint8_t buffer[8], uint8_t respBuffer[8], bool &popped)
{
    m_CAN.sendMsgBuf((uint32_t)m_canId, 0, MAB_CAN_BUFF_SIZE, buffer);

    delayMicroseconds(500);

    for (int i = 0; i < FIFO_CAPACITY; i++)
    {
        if (m_receiveQueue.count > 0)
        {
            queuePop(&m_receiveQueue, respBuffer);
            popped = true;
            break;
        }
        else
        {
            delayMicroseconds(500);
        }
    }

    if (!popped)
        return false;

    return true;
}

bool MD::update()
{
    if (CAN_MSGAVAIL == m_CAN.checkReceive())
    {
        m_CAN.readMsgBufID(&rxId, &rxLen, rxBuffer);
        if (rxId == (uint32_t)m_canId)
        {
            queuePush(&m_receiveQueue, rxBuffer);
            return true;
        }
    }
    return false;
}
#elif defined(ARDUINO_ARCH_SAM)
bool MD::send(uint8_t buffer[8], uint8_t respBuffer[8], bool &popped)
{
    CanMsg const msg(CanStandardId(m_canId), sizeof(buffer), buffer);

    CAN.write(msg);

    delayMicroseconds(500);

    for (int i = 0; i < FIFO_CAPACITY; i++)
    {
        if (m_receiveQueue.count > 0)
        {
            queuePop(&m_receiveQueue, respBuffer);
            popped = true;
            break;
        }
        else
        {
            delayMicroseconds(500);
        }
    }

    if (!popped)
        return false;

    return true;
}

MD::Error_t MD::init()
{
    if (!CAN.begin(CanBitRate::BR_1000k))
    {
        delay(100);
    }
}

bool MD::update()
{
    if (CAN.available())
    {
        CanMsg const msg = CAN.read();

        if (CanStandardId(msg.id) == (uint32_t)m_canId)
        {
            queuePush(&m_receiveQueue, rxBuffer);
            return true;
        }
    }
    return false;
}
#endif

MD::Error_t MD::getMotionMode(motionModeMab_E motionMode)
{
    uint8_t buffer[MAB_CAN_BUFF_SIZE] = {0};
    uint8_t respBuffer[MAB_CAN_BUFF_SIZE] = {0};
    uint8_t modeMainRead = 0;
    bool popped = false;

    if (getMotionModePrepareDataFrame(&buffer, MAB_CAN_BUFF_SIZE) != 1)
        return MD::Error_t::TRANSFER_FAILED;

    if (!send(buffer, respBuffer, popped))
        return MD::Error_t::TRANSFER_FAILED;

    if (getMotionModeParseResponse(&respBuffer, MAB_CAN_BUFF_SIZE, &modeMainRead) != 1)
        return MD::Error_t::TRANSFER_FAILED;

    motionMode = (motionModeMab_E)modeMainRead;

    return MD::Error_t::OK;
}

MD::Error_t MD::enable()
{
    uint8_t buffer[MAB_CAN_BUFF_SIZE] = {0};
    uint8_t respBuffer[MAB_CAN_BUFF_SIZE] = {0};
    bool popped = false;

    if (setStateEnablePrepareDataFrame(&buffer, MAB_CAN_BUFF_SIZE) != 1)
        return MD::Error_t::UNKNOWN_ERROR;

    if (!send(buffer, respBuffer, popped))
        return MD::Error_t::TRANSFER_FAILED;

    if (setStateEnableParseResponse(&respBuffer, MAB_CAN_BUFF_SIZE) != 1)
        return MD::Error_t::UNKNOWN_ERROR;

    return MD::Error_t::OK;
}

MD::Error_t MD::disable()
{
    uint8_t buffer[MAB_CAN_BUFF_SIZE] = {0};
    uint8_t respBuffer[MAB_CAN_BUFF_SIZE] = {0};
    bool popped = false;

    if (setStateDisablePrepareDataFrame(&buffer, MAB_CAN_BUFF_SIZE) != 1)
        return MD::Error_t::UNKNOWN_ERROR;

    if (!send(buffer, respBuffer, popped))
        return MD::Error_t::TRANSFER_FAILED;

    if (setStateDisableParseResponse(&respBuffer, MAB_CAN_BUFF_SIZE) != 1)
        return MD::Error_t::UNKNOWN_ERROR;

    return MD::Error_t::OK;
}
