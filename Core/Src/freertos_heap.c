#include <stdint.h>

#include "FreeRTOS.h"

__attribute__((section(".freertos_heap"), aligned(8)))
uint8_t ucHeap[configTOTAL_HEAP_SIZE];
