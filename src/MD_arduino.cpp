#include "MD_arduino.hpp"
// Define static member once outside the class
MD *MD::instance = nullptr;

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

MD::Error_t MD::send(uint8_t buffer[8], uint8_t respBuffer[8])
{
    bool popped = false;
    m_CAN.sendMsgBuf(m_canId, 0, MAB_CAN_BUFF_SIZE, buffer);
    if (MD::update() != MD::Error_t::OK)
        return MD::Error_t::NOT_CONNECTED;

    for (int i = 0; i < FIFO_CAPACITY; i++)
    {
        if (m_receiveQueue.count > 0)
        {
            queuePop(&m_receiveQueue, respBuffer);
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
    m_Can.write(tx_msg);

    // delayMicroseconds(500);
    m_Can.events();
    bool popped = false;

    for (int i = 0; i < FIFO_CAPACITY; i++)
    {
        if (m_receiveQueue.count > 0)
        {
            queuePop(&m_receiveQueue, respBuffer);
            popped = true;
            break;
        }
    }

    if (!popped)
        return MD::Error_t::TRANSFER_FAILED;

    return MD::Error_t::OK;
}

MD::Error_t MD::send(uint8_t buffer[8], uint8_t respBuffer[8])
{

    CAN_message_t tx_msg;
    tx_msg.id = m_canId;
    memcpy(tx_msg.buf, buffer, MAB_CAN_BUFF_SIZE);
    tx_msg.len = MAB_CAN_BUFF_SIZE;
    tx_msg.flags.extended = 0;
    m_Can.write(tx_msg);

    delayMicroseconds(500);
    m_Can.events();
    bool popped = false;

    for (int i = 0; i < FIFO_CAPACITY; i++)
    {
        if (m_receiveQueue.count > 0)
        {
            queuePop(&m_receiveQueue, respBuffer);
            popped = true;
            break;
        }
    }

    if (!popped)
        return MD::Error_t::TRANSFER_FAILED;

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
        queuePush(&m_receiveQueue, msg.buf);
}

#else
MD::Error_t MD::send(uint8_t buffer[8], uint8_t respBuffer[8])
{
    bool popped = false;
    CanMsg const msg(CanStandardId(m_canId), MAB_CAN_BUFF_SIZE, buffer);

    CAN.write(msg);
    if (MD::update() != MD::Error_t::OK)
        return MD::Error_t::NOT_CONNECTED;

    for (int i = 0; i < FIFO_CAPACITY; i++)
    {
        if (m_receiveQueue.count > 0)
        {
            queuePop(&m_receiveQueue, respBuffer);
            popped = true;
            break;
        }
    }

    if (!popped)
        return MD::Error_t::TRANSFER_FAILED;

    return MD::Error_t::OK;
}

MD::Error_t MD::init()
{
    if (!CAN.begin(CanBitRate::BR_1000k))
    {
        Serial.print("Waiting for connection");
        delay(500);
    }
    queueInit(&m_receiveQueue);
    return MD::Error_t::OK;
}

MD::Error_t MD::update()
{
    if (CAN.available())
    {
        CanMsg const msg = CAN.read();

        if (CanStandardId(msg.id) == (uint32_t)m_canId)
        {
            queuePush(&m_receiveQueue, rxBuffer);
            return MD::Error_t::OK;
        }
    }
    return MD::Error_t::NOT_CONNECTED;
}
#endif

/*
#define MD_REG(name, type, addr, access) _GEN_DECL_##access(name, type)
#define _GEN_DECL_RO(name, type)
#define _GEN_DECL_RW(name, type)
#define _GEN_DECL_WO(name, type)

REGISTER_LIST

#undef MD_REG
#undef _GEN_DECL_RO
#undef _GEN_DECL_RW
#undef _GEN_DECL_WO

#define MD_REG(name, type, addr, access) _GEN_DEF_##access(name, type, addr)

#define _GEN_DEF_RO(name, type, addr) \
    MD::Error_t MD::get##name(type &name) { return readRegister(addr, name); }

#define _GEN_DEF_RW(name, type, addr)                                          \
    MD::Error_t MD::set##name(type name) { return writeRegister(addr, name); } \
    MD::Error_t MD::get##name(type &name) { return readRegister(addr, name); }

#define _GEN_DEF_WO(name, type, addr) \
    MD::Error_t MD::set##name(type name) { return writeRegister(addr, name); }

REGISTER_LIST

#undef _GEN_DEF_RO
#undef _GEN_DEF_RW
#undef _GEN_DEF_WO
#undef MD_REG
*/
MD::Error_t MD::enable()
{
    return writeRegister(REG_CONTROL_WORD, CONTROL_WORD_ENABLE);
}

MD::Error_t MD::disable()
{
    return writeRegister(REG_CONTROL_WORD, CONTROL_WORD_DISABLE);
}

MD::Error_t MD::blink()
{
    return writeRegister(BLINK, 0x01);
}

MD::Error_t MD::getMotionMode(motionModeMab_E &motionMode)
{
    return readRegister(REG_MOTION_MODE_STATUS, motionMode);
}

MD::Error_t MD::getPolePairs(u32 &polePairs)
{
    return readRegister(REG_MOTOR_POLE_PAIRS, polePairs);
}

MD::Error_t MD::getPosition(f32 &position)
{
    return readRegister(REG_MAIN_ENCODER_POS, position);
}
