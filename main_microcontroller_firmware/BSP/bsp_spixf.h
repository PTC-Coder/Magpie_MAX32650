/**
 * @file      bsp_spixf.h
 * @brief     This module is responsible for initializing and de-initializing the SPIXF interface used by the sytem.
 *
 * This module requires:
 * - Shared use of SPIXF
 * - Shared use of SPIXF pins P0.0, P0.1, P0.2, and P0.3
 */

#ifndef BSP_SPIXF_H__
#define BSP_SPIXF_H__

#define BSP_SPIXF_BAUD_RATE     8000000  // 8 MHz SPI clock rate

/* Public function declarations --------------------------------------------------------------------------------------*/

/**
 * @brief `bsp_spixf_init()` initializes and starts the SPIXF interface in 4-wire SPI mode.
 *
 * @post SPIXF is initialized and ready to use. The GPIO pins associated with the bus are configured for SPIXF operation.
 *
 * @retval Success/Fail, see MXC_Error_Codes for a list of return codes.
 */
int bsp_spixf_init(void);

/**
 * @brief `bsp_spixf_deinit()` deinitializes SPIXF and sets the associated pins to high-Z.
 *
 * @post SPIXF is deinitialized. The GPIO pins associated with the bus are hig-Z.
 *
 * @retval Success/Fail, see MXC_Error_Codes for a list of return codes.
 */
int bsp_spixf_deinit(void);

int bsp_spixf_config(void);


// int bsp_spifx_flash_module_read(uint8_t *read, unsigned len, unsigned deassert,
//                                 Ext_Flash_DataLine_t width);

// int bsp_spifx_flash_module_write(const uint8_t *write, unsigned len, unsigned deassert,
//                                  Ext_Flash_DataLine_t width);

// int bsp_spifx_flash_clock(unsigned len, unsigned deassert);

// int bsp_setup_spifx_comm(void);

#endif
