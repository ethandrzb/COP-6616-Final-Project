#ifndef COMMON_H
#define COMMON_H

#include "ring_buffer.h"

/* Align X to 4 bytes */
#define MEM_ALIGN(x)                        (((x) + 0x00000003) & ~0x00000003)

/* Shared RAM between 2 cores is SRAM4 in D3 domain */
#define SHD_RAM_START_ADDR                  0x38000000
#define SHD_RAM_LEN                         0x0000FFFF

/* Buffer from CM4 to CM7 */
#define BUFF_CM4_TO_CM7_ADDR                MEM_ALIGN(SHD_RAM_START_ADDR)
#define BUFF_CM4_TO_CM7_LEN                 MEM_ALIGN(sizeof(ringbuf_t))
#define BUFFDATA_CM4_TO_CM7_ADDR            MEM_ALIGN(BUFF_CM4_TO_CM7_ADDR + BUFF_CM4_TO_CM7_LEN)
#define BUFFDATA_CM4_TO_CM7_LEN             MEM_ALIGN(0x00007F00)	// Default is  0x00000400

/* Buffer from CM7 to CM4 */
#define BUFF_CM7_TO_CM4_ADDR                MEM_ALIGN(BUFFDATA_CM4_TO_CM7_ADDR + BUFFDATA_CM4_TO_CM7_LEN)
#define BUFF_CM7_TO_CM4_LEN                 MEM_ALIGN(sizeof(ringbuf_t))
#define BUFFDATA_CM7_TO_CM4_ADDR            MEM_ALIGN(BUFF_CM7_TO_CM4_ADDR + BUFF_CM7_TO_CM4_LEN)
#define BUFFDATA_CM7_TO_CM4_LEN             MEM_ALIGN(0x00007F00)	// Default is  0x00000400

#define TEST_BUFFER_SIZE 1024

#endif
