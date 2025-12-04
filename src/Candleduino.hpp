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
#pragma once
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

class MAB_DEVICE
{

public:
    bool FD = false;

    /// @brief MD can node ID
    uint16_t m_canId;

    CANFD_message_t messages[FIFO_CAPACITY];
    Error_t writeRead(uint8_t buffer[8], uint8_t respBuffer[8]);
    Error_t writeReadFD(uint8_t *buffer, uint8_t *respBuffer, uint8_t bufferLength, uint8_t responseBufferDiff = 0);
    FifoFrame_S m_receiveQueue;
    static FifoCANFD_S QUEUE_FD;
    static MAB_DEVICE *instance;
    uint8_t rxBuffer[MAB_CAN_BUFF_SIZE] = {0};
    uint8_t rxLen = 0;
    unsigned long rxId;
    static uint8_t SHARED_BUFFER[64];
    int m_SPI_CS_PIN = 9;

    static uint32_t rxFDID;

public:
#if defined(ARDUINO_ARCH_AVR)
    MCP_CAN m_CAN;

    /// @brief Create MD object with deafult SPI CS pin 9
    /// @return
    MAB_DEVICE(uint16_t canId) : m_CAN(m_SPI_CS_PIN), m_canId(canId) {};

    /// @brief Create MD object with custom SPI CS pin
    /// @return
    MAB_DEVICE(uint16_t canId, int SPI_CS_PIN) : m_CAN(SPI_CS_PIN), m_canId(canId) {};

    Error_t init();
#elif defined(TEENSYDUINO)
    FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> *m_CanFD = nullptr;
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> *m_Can = nullptr;

    MAB_DEVICE(uint16_t canId, FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> *canBus) : m_canId(canId), m_Can(canBus)
    {
        instance = this;
    };

    MAB_DEVICE(uint16_t canId, FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> *canBusFD) : m_canId(canId), m_CanFD(canBusFD)
    {
        if (m_CanFD)
            FD = true;
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

    /// @brief Initialize MD CAN communication
    /// @return

    Error_t init()
    {
        if (m_CanFD)
        {
            Serial.println("CANFD");
            CANFD_timings_t config;
            config.clock = CLK_24MHz;
            config.baudrate = 1000000;
            config.baudrateFD = 1000000;
            config.propdelay = 190;
            config.bus_length = 1;
            config.sample = 70;
            m_CanFD->begin();
            m_CanFD->setBaudRate(config);
            m_CanFD->setRegions(64);
            m_CanFD->onReceive(MB0, canCallbackFD);
            m_CanFD->enableMBInterrupts();
        }
        else
        {
            Serial.println("CAN2.0");
            m_Can->begin();
            m_Can->setBaudRate(1000000);
            m_Can->setMaxMB(16);
            m_Can->enableFIFO();
            m_Can->enableFIFOInterrupt();
            m_Can->onReceive(canCallback);
        }

        Serial.println("CAN begin done"); // confirm this runs
        return Error_t::OK;
    }
#else
    MD(uint16_t canId) : m_canId(canId) {};
    Error_t init();
#endif

    /// @brief Update receive queue
    /// @return
    Error_t receive();

    void printHexBuffer(const uint8_t *buf, size_t len)
    {
        for (size_t i = 0; i < len; i++)
        {
            Serial.print(int(buf[i]));
            Serial.print(" ");
        }

        Serial.println();
    }
};
