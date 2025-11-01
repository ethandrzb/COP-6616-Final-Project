#include "../Inc/ring_buffer.h"
#include "stm32h7xx_hal.h"

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

#define COPY_WRITE(src, dest, size) memcpy(src, dest, size);
#define COPY_READ(src, dest, size) memcpy(src, dest, size);
//extern DMA_HandleTypeDef hdma_memtomem_dma1_stream0;
//extern DMA_HandleTypeDef hdma_memtomem_dma1_stream1;
//#define COPY_WRITE(src, dest, size) HAL_DMA_Start_IT(&hdma_memtomem_dma1_stream0, (uint32_t) src, (uint32_t) dest, size);
//#define COPY_READ(src, dest, size) HAL_DMA_Start_IT(&hdma_memtomem_dma1_stream1, (uint32_t) src, (uint32_t) dest, size);

// -= Initialization =-
void RingBuffer_Init(volatile ringbuf_t *buffer, void *internalBuffer, uint16_t size)
{
	if((buffer == NULL) || (internalBuffer == NULL) || (size == 0))
	{
		return;
	}

	buffer->size = size;
	buffer->data = internalBuffer;

	// Initialize pointers
	buffer->r = 0;
	buffer->w = 0;
}

bool RingBuffer_Validate(volatile ringbuf_t *buffer)
{
	return (buffer != NULL) && (buffer->data != NULL);
}

void RingBuffer_Reset(volatile ringbuf_t *buffer)
{
	buffer->r = 0;
	buffer->w = 0;
}

// -= Main operations =-
uint16_t RingBuffer_Write(volatile ringbuf_t *buffer, void *writeData, uint16_t writeSize)
{
	if(!RingBuffer_Validate(buffer) || writeData == NULL)
	{
		return 0;
	}

	// Step 1: Clip write size to maximum number of available bytes to write
	uint16_t totalBytesToWrite = MIN(writeSize, RingBuffer_GetWriteLength_Ring(buffer));

	// Step 2: Write until we reach the end of the array and advance write pointer
	uint16_t numBytesToWriteBeforeOverflow = MIN(totalBytesToWrite, RingBuffer_GetWriteLength_Linear(buffer));
	COPY_WRITE(&(buffer->data[buffer->w]), writeData, numBytesToWriteBeforeOverflow * sizeof(uint8_t));
	buffer->w += numBytesToWriteBeforeOverflow;
	totalBytesToWrite -= numBytesToWriteBeforeOverflow;

	// Step 3: Write remaining data, if any, to start of buffer and advance write pointer
	if(totalBytesToWrite > 0)
	{
		COPY_WRITE(buffer->data, &(writeData[numBytesToWriteBeforeOverflow]), totalBytesToWrite);
		buffer->w = totalBytesToWrite;
	}

	// Step 4: Reset write pointer if out of bounds
	if(buffer->w >= buffer->size)
	{
		buffer->w = 0;
	}

	return totalBytesToWrite + numBytesToWriteBeforeOverflow;
}

uint16_t RingBuffer_Read(volatile ringbuf_t *buffer, void *readData, uint16_t readSize)
{
	if(!RingBuffer_Validate(buffer) || readData == NULL)
	{
		return 0;
	}

	// Step 1: Clip read size to maximum number of available bytes to read
	uint16_t totalBytesToRead = MIN(readSize, RingBuffer_GetReadLength_Ring(buffer));

	// Step 2: Read until we reach the end of the array and advance read pointer
	uint16_t numBytesToReadBeforeOverflow = MIN(totalBytesToRead, RingBuffer_GetReadLength_Linear(buffer));
	COPY_READ(readData, &(buffer->data[buffer->r]), numBytesToReadBeforeOverflow * sizeof(uint8_t));
	buffer->r += numBytesToReadBeforeOverflow;
	totalBytesToRead -= numBytesToReadBeforeOverflow;

	// Step 3: Read remaining data, if any, from start of buffer and advance read pointer
	if(totalBytesToRead > 0)
	{
		COPY_READ(&(readData[numBytesToReadBeforeOverflow]), buffer->data, totalBytesToRead);
		buffer->r = totalBytesToRead;
	}

	// Step 4: Reset read pointer if out of bounds
	if(buffer->r >= buffer->size)
	{
		buffer->r = 0;
	}

	return totalBytesToRead + numBytesToReadBeforeOverflow;
}

// -= Size operations =-
// Treating the buffer as a ring
uint16_t RingBuffer_GetReadLength_Ring(volatile ringbuf_t *buffer)
{
	if(!RingBuffer_Validate(buffer))
	{
		return 0;
	}

	// Case 1: Read behind write
	if(buffer->r < buffer->w)
	{
		return buffer->w - buffer->r;
	}
	// Case 2: Read ahead of write
	else if(buffer->r > buffer->w)
	{
		return buffer->size - (buffer->r - buffer->w);
	}
	// Case 3: Read overlaps with write
	else
	{
		// Assume all written data has been read
		// Gives write operations priority over read operations
		return 0;
	}
}
uint16_t RingBuffer_GetWriteLength_Ring(volatile ringbuf_t *buffer)
{
	if(!RingBuffer_Validate(buffer))
	{
		return 0;
	}

	// Case 1: Read behind write
	if(buffer->r < buffer->w)
	{
		return (buffer->size - (buffer->w - buffer->r)) - 1;
	}
	// Case 2: Read ahead of write
	else if(buffer->r > buffer->w)
	{
		return (buffer->r - buffer->w) - 1;
	}
	// Case 3: Read overlaps with write
	else
	{
		// Assume all data has been read
		// Gives write operations priority over read operations
		return buffer->size - 1;
	}
}

// Treating the buffer as a 1D array
uint16_t RingBuffer_GetReadLength_Linear(volatile ringbuf_t *buffer)
{
	if(!RingBuffer_Validate(buffer))
	{
		return 0;
	}

	// Case 1: Read behind write
	if(buffer->r < buffer->w)
	{
		return buffer->w - buffer->r;
	}
	// Case 2: Read ahead of write
	else if(buffer->r > buffer->w)
	{
		return buffer->size - buffer->r;
	}
	// Case 3: Read overlaps with write
	else
	{
		// Assume all data has been read
		// Gives write operations priority over read operations
		return 0;
	}
}
uint16_t RingBuffer_GetWriteLength_Linear(volatile ringbuf_t *buffer)
{
	if(!RingBuffer_Validate(buffer))
	{
		return 0;
	}

	// Case 1: Read behind write
	if(buffer->r < buffer->w)
	{
		return buffer->size - buffer->w;
	}
	// Case 2: Read ahead of write
	else if(buffer->r > buffer->w)
	{
		return (buffer->r - buffer->w) - 1;
	}
	// Case 3: Read overlaps with write
	else
	{
		// Assume all data has been read
		// Gives write operations priority over read operations
		return buffer->size - 1;
	}
}

// -= Direct buffer access =-
void *RingBuffer_GetReadAddress(volatile ringbuf_t *buffer)
{
	if(!RingBuffer_Validate(buffer))
	{
		return NULL;
	}

	return &(buffer->data[buffer->r]);
}
void *RingBuffer_GetWriteAddress(volatile ringbuf_t *buffer)
{
	if(!RingBuffer_Validate(buffer))
	{
		return NULL;
	}

	return &(buffer->data[buffer->w]);
}

// -= Manual pointer movement =-
uint16_t RingBuffer_AdvanceReadPointer(volatile ringbuf_t *buffer, uint16_t size);
uint16_t RingBuffer_AdvanceWritePointer(volatile ringbuf_t *buffer, uint16_t size);
