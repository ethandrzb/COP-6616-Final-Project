#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//TODO: Add magic numbers
typedef struct
{
	uint8_t *data;
	uint32_t size;
	uint32_t r;
	uint32_t w;

} ringbuf_t;

//TODO: Check magic numbers in struct validation

// -= Initialization =-
void RingBuffer_Init(volatile ringbuf_t *buffer, void *internalBuffer, uint32_t size);
bool RingBuffer_Validate(volatile ringbuf_t *buffer);
void RingBuffer_Reset(volatile ringbuf_t *buffer);

// -= Main operations =-
uint32_t RingBuffer_Write(volatile ringbuf_t *buffer, void *writeData, uint32_t writeSize);
uint32_t RingBuffer_Read(volatile ringbuf_t *buffer, void *readData, uint32_t readSize);

// -= Size operations =-
// Treating the buffer as a ring
uint32_t RingBuffer_GetReadLength_Ring(volatile ringbuf_t *buffer);
uint32_t RingBuffer_GetWriteLength_Ring(volatile ringbuf_t *buffer);

// Treating the buffer as a 1D array
uint32_t RingBuffer_GetReadLength_Linear(volatile ringbuf_t *buffer);
uint32_t RingBuffer_GetWriteLength_Linear(volatile ringbuf_t *buffer);

// -= Direct buffer access =-
void *RingBuffer_GetReadAddress(volatile ringbuf_t *buffer);
void *RingBuffer_GetWriteAddress(volatile ringbuf_t *buffer);

// -= Manual pointer movement =-
uint32_t RingBuffer_AdvanceReadPointer(volatile ringbuf_t *buffer, uint32_t size);
uint32_t RingBuffer_AdvanceWritePointer(volatile ringbuf_t *buffer, uint32_t size);
