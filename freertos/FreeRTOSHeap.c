/*
 * FreeRTOSHeap.c
 *
 *  Created on: 2024-03-22
 *      Author: Gintaras
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSHeap.h"

#if configHEAP_ALLOCATION_SCHEME  == HEAP_ALLOCATION_TYPE5
const HeapRegion_t xHeapRegions[] =
{
    { ( uint8_t * ) 0x18000000UL, 0x200000 },
    { NULL, 0 } /* Terminates the array. */
};
#endif

void FreeRTOSHeapInit(void)
{
#if configHEAP_ALLOCATION_SCHEME  == HEAP_ALLOCATION_TYPE5

    /* Pass the array into vPortDefineHeapRegions(). */
    vPortDefineHeapRegions( xHeapRegions );

#endif
    return;
}
