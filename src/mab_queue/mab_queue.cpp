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
