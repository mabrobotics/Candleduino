// fifo.h
#ifndef MAB_QUEUE_H
#define MAB_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_SIZE 64
#define FIFO_CAPACITY 3

typedef struct
{
    uint8_t buffer[FIFO_CAPACITY][MAX_SIZE];
    int head;
    int tail;
    int count;
} FifoFrame_S;

void queueInit(FifoFrame_S *fifo);
bool queuePush(FifoFrame_S *fifo, const uint8_t *frame);
bool queuePop(FifoFrame_S *fifo, uint8_t *frameOut);
bool isQueueEmpty(const FifoFrame_S *fifo);
bool isQueueFull(const FifoFrame_S *fifo);

#endif // MAB_QUEUE_H
