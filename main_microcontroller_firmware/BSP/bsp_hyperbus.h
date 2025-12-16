/**
 * @file      bsp_hyperbus.h
 * @brief     This module is responsible for initializing and de-initializing the Hyperbus interface used by the sytem.
 *
 * This module requires:
 * - Shared use of the hyperbus peripheral
 * - Shared use of hyperbus pins P1.11 - P1.16, and P1.18 - P1.21
 */

#ifndef HYPERBUS_H__
#define HYPERBUS_H__

/* Includes ----------------------------------------------------------------------------------------------------------*/

/* Public function declarations --------------------------------------------------------------------------------------*/

/**
 * @brief `bsp_hyperbus_init()` enables and initializes the GPIO pins for the hyperbus peripheral.
 *
 * @post The hyperbus peripheral is initialized and ready to use.
 *
 * @retval Success/Fail, see MXC_Error_Codes for a list of return codes.
 */
int bsp_hyperbus_init(void);

/**
 * @brief `bsp_hyperbus_deinit()` de-initializes the hyperbus peripheral and sets all the associated pins to high-Z.
 *
 * @post The hyperbus peripheral is de-initialized and all the associated pins are set to high-Z.
 *
 * @retval Success/Fail, see MXC_Error_Codes for a list of return codes.
 */
int bsp_hyperbus_deinit(void);

/**
 * @brief `bsp_hyperbus_flush_cache()` flushes the external memory cache controller (EMCC) cache.
 *
 * This function ensures that any cached data in the EMCC is written back to the hyperbus memory,
 * maintaining data coherency between the cache and external memory.
 *
 * @post The EMCC cache is flushed and all pending writes are committed to hyperbus memory.
 */
void bsp_hyperbus_flush_cache(void);

#endif
