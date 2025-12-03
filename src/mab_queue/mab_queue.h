// fifo.h
#ifndef MAB_QUEUE_H
#define MAB_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#if defined(TEENSYDUINO)
#include <FlexCAN_T4.h>
#endif

#define MAX_SIZE 8
#define FIFO_CAPACITY 3

typedef struct
{
    uint8_t buffer[FIFO_CAPACITY][MAX_SIZE];
    int head = 0;
    int tail = 0;
    int count = 0;
} FifoFrame_S;

typedef struct
{
    CANFD_message_t messages[FIFO_CAPACITY];
    int head = 0;
    int tail = 0;
    int count = 0;
} FifoCANFD_S;

void queueInit(FifoFrame_S *fifo);
void queueInit(FifoCANFD_S *fifo);
bool queuePush(FifoFrame_S *fifo, const uint8_t *frame);
bool queuePushMessage(FifoCANFD_S *fifo, const CANFD_message_t message);
bool queuePop(FifoFrame_S *fifo, uint8_t *frameOut);
bool queuePopMessage(FifoCANFD_S *fifo, CANFD_message_t &messageOut);
bool isQueueEmpty(const FifoFrame_S *fifo);
bool isQueueFull(const FifoFrame_S *fifo);
bool isQueueEmpty(const FifoCANFD_S *fifo);
bool isQueueFull(const FifoCANFD_S *fifo);

uint16_t queueSeeId(FifoCANFD_S *fifo);

#endif // MAB_QUEUE_H
