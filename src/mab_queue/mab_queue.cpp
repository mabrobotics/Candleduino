#include <string.h>

#include "mab_queue.h"

void queueInit(FifoFrame_S *fifo)
{
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
}

bool queuePush(FifoFrame_S *fifo, const uint8_t *frame)
{
    if (fifo->count >= FIFO_CAPACITY)
        return false;

    memcpy(fifo->buffer[fifo->tail], frame, MAX_SIZE);
    fifo->tail = (fifo->tail + 1) % FIFO_CAPACITY;
    fifo->count++;

    return true;
}

bool queuePop(FifoFrame_S *fifo, uint8_t *frameOut)
{
    if (fifo->count <= 0)
        return false;

    memcpy(frameOut, fifo->buffer[fifo->head], MAX_SIZE);
    fifo->head = (fifo->head + 1) % FIFO_CAPACITY;
    fifo->count--;

    return true;
}

bool isQueueEmpty(const FifoFrame_S *fifo)
{
    return fifo->count == 0;
}

bool isQueueFull(const FifoFrame_S *fifo)
{
    return fifo->count == FIFO_CAPACITY;
}
#if defined(TEENSYDUINO)
void queueInit(FifoCANFD_S *fifo)
{
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
}

bool queuePushMessage(FifoCANFD_S *fifo, const CANFD_message_t message)
{
    if (fifo->count >= FIFO_CAPACITY)
        return false;

    fifo->messages[fifo->head] = message;
    fifo->tail = (fifo->tail + 1) % FIFO_CAPACITY;
    fifo->count++;
    return true;
}

bool queuePopMessage(FifoCANFD_S *fifo, CANFD_message_t &messageOut)
{
    if (fifo->count <= 0)
        return false;

    messageOut = fifo->messages[fifo->head];
    fifo->head = (fifo->head + 1) % FIFO_CAPACITY;
    fifo->count--;

    return true;
}

uint16_t queueSeeId(FifoCANFD_S *fifo)
{
    return fifo->messages[fifo->head].id;
}

bool isQueueEmpty(const FifoCANFD_S *fifo)
{
    return fifo->count == 0;
}

bool isQueueFull(const FifoCANFD_S *fifo)
{
    return fifo->count == FIFO_CAPACITY;
}
#endif