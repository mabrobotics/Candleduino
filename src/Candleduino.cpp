#include "Candleduino.hpp"

#if defined(TEENSYDUINO)
MAB_DEVICE *MAB_DEVICE::instance = nullptr;
uint8_t MAB_DEVICE::SHARED_BUFFER[64] = {0};
uint32_t MAB_DEVICE::rxFDID = 0;
FifoCANFD_S MAB_DEVICE::QUEUE_FD;
#endif

#if defined(ARDUINO_ARCH_AVR)

Error_t MAB_DEVICE::init()
{
    uint8_t timeoutCount = 5;
    while (CAN_OK != m_CAN.begin(CAN_1000KBPS))
    {
        Serial.print("Waiting for connection");
        delay(500);
        timeoutCount--;
        if (!timeoutCount)
            return Error_t::NOT_CONNECTED;
    }
    queueInit(&m_receiveQueue);
    return Error_t::OK;
}
Error_t MAB_DEVICE::writeRead(uint8_t buffer[8], uint8_t respBuffer[8])
{
    bool popped = false;
    m_CAN.sendMsgBuf(m_canId, 0, MAB_CAN_BUFF_SIZE, buffer);
    _delay_us(500);
    if (MAB_DEVICE::receive() != Error_t::OK)
        return Error_t::NOT_CONNECTED;

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
        return Error_t::TRANSFER_FAILED;

    return Error_t::OK;
}

Error_t MAB_DEVICE::receive()
{
    if (CAN_MSGAVAIL == m_CAN.checkReceive())
    {
        m_CAN.readMsgBufID(&rxId, &rxLen, rxBuffer);
        if (rxId == (uint32_t)m_canId)
        {
            queuePush(&m_receiveQueue, rxBuffer);
            return Error_t::OK;
        }
    }
    return Error_t ::NOT_CONNECTED;
}
#elif defined(TEENSYDUINO)

Error_t MAB_DEVICE::writeReadFD(uint8_t *buffer, uint8_t *respBuffer, uint8_t bufferLength)
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

    if (!isQueueEmpty(&QUEUE_FD))
    {
        CANFD_message_t message;
        queuePopMessage(&QUEUE_FD, message);

        if (message.id == m_canId)
        {
            memcpy(respBuffer, message.buf, bufferLength);
        }
    }
    else
    {
        return Error_t::TRANSFER_FAILED;
    }

    return Error_t::OK;
}

Error_t MAB_DEVICE::writeRead(uint8_t buffer[8], uint8_t respBuffer[8])
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
            return Error_t::TRANSFER_FAILED;
    }
    else
    {
        queueInit(&m_receiveQueue);
        return Error_t::TRANSFER_FAILED;
    }

    return Error_t::OK;
}

void MAB_DEVICE::canCallback(const CAN_message_t &msg)
{
    if (instance)
        instance->handleMessage(msg);
}

void MAB_DEVICE::handleMessage(const CAN_message_t &msg)
{
    if (msg.id == (uint32_t)m_canId)
        queuePush(&m_receiveQueue, msg.buf);
}

void MAB_DEVICE::canCallbackFD(const CANFD_message_t &msg)
{
    if (instance)
        instance->handleMessageFD(msg);
}

void MAB_DEVICE::handleMessageFD(const CANFD_message_t &msg)
{
    queuePushMessage(&QUEUE_FD, msg);
    memcpy(SHARED_BUFFER, msg.buf, sizeof(msg.buf));
    rxFDID = msg.id;
}

#else
Error_t MAB_DEVICE::writeRead(uint8_t buffer[8], uint8_t respBuffer[8])
{
    bool popped = false;
    CanMsg const msg(CanStandardId(m_canId), MAB_CAN_BUFF_SIZE, buffer);

    if (CAN.write(msg) == 1)
    {
        delayMicroseconds(500);
        if (MAB_DEVICE::receive() != Error_t::OK)
            return Error_t::NOT_CONNECTED;

        if (memcmp(buffer, rxBuffer, 4))
        {
            // Flushing all pending frames when getting missmatched messages
            while (CAN.available())
            {
                CAN.read();
            }
            memset(rxBuffer, 0, MAB_CAN_BUFF_SIZE);
            return Error_t::TRANSFER_FAILED;
        }
        else
        {
            memcpy(respBuffer, rxBuffer, MAB_CAN_BUFF_SIZE);
        }
    }
    else
    {
        return Error_t::NOT_CONNECTED;
    }

    return Error_t::OK;
}

Error_t MAB_DEVICE::init()
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
            return Error_t::NOT_CONNECTED;
        }
    }
    Serial.println("CAN begin done"); // confirm this runs
    return Error_t::OK;
}

Error_t MAB_DEVICE::receive()
{

    if (CAN.available())
    {
        CanMsg const msg = CAN.read();

        if ((msg.id & 0x7FF) == m_canId)
        {
            memcpy(rxBuffer, msg.data, 8);
            return Error_t::OK;
        }
    }

    return Error_t::NOT_CONNECTED;
}
#endif
