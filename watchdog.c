/*
 * watchdog.c
 *
 *  Created on: 2024-06-18
 *      Author: GDR
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "watchdog.h"

#define WDT_USED
#define WDT_TIME_OUT_MS             4000

/* WDT object */
cyhal_wdt_t wdt_obj;

void watchdog_init(void)
{

#ifdef WDT_USED

	cy_rslt_t result;

    /* Initialize the WDT */
    result = cyhal_wdt_init(&wdt_obj, WDT_TIME_OUT_MS);

    /* WDT initialization failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

#endif

}

void watchdog_feed(void)
{

#ifdef WDT_USED

        /* Reset WDT */
        cyhal_wdt_kick(&wdt_obj);

#endif

}

void watchdog_stop(void)
{

#ifdef WDT_USED

    /* Stop WDT */
    cyhal_wdt_stop(&wdt_obj);

#endif

}

void watchdog_start(void)
{

#ifdef WDT_USED

    /* Start WDT */
    cyhal_wdt_start(&wdt_obj);

#endif

}

void watchdog_deinit(void)
{

#ifdef WDT_USED

    /* Deinitialize the WDT */
    cyhal_wdt_free(&wdt_obj);


#endif

}

