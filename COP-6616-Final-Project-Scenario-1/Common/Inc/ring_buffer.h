#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//TODO: Add magic numbers
typedef struct
{
	uint8_t *data;
	uint16_t size;
	uint16_t r;
	uint16_t w;

} ringbuf_t;

//TODO: Check magic numbers in struct validation

// -= Initialization =-
void RingBuffer_Init(volatile ringbuf_t *buffer, void *internalBuffer, uint16_t size);
bool RingBuffer_Validate(volatile ringbuf_t *buffer);
void RingBuffer_Reset(volatile ringbuf_t *buffer);

// -= Main operations =-
uint16_t RingBuffer_Write(volatile ringbuf_t *buffer, void *writeData, uint16_t writeSize);
uint16_t RingBuffer_Read(volatile ringbuf_t *buffer, void *readData, uint16_t readSize);

// -= Size operations =-
// Treating the buffer as a ring
uint16_t RingBuffer_GetReadLength_Ring(volatile ringbuf_t *buffer);
uint16_t RingBuffer_GetWriteLength_Ring(volatile ringbuf_t *buffer);

// Treating the buffer as a 1D array
uint16_t RingBuffer_GetReadLength_Linear(volatile ringbuf_t *buffer);
uint16_t RingBuffer_GetWriteLength_Linear(volatile ringbuf_t *buffer);

// -= Direct buffer access =-
void *RingBuffer_GetReadAddress(volatile ringbuf_t *buffer);
void *RingBuffer_GetWriteAddress(volatile ringbuf_t *buffer);

// -= Manual pointer movement =-
uint16_t RingBuffer_AdvanceReadPointer(volatile ringbuf_t *buffer, uint16_t size);
uint16_t RingBuffer_AdvanceWritePointer(volatile ringbuf_t *buffer, uint16_t size);
