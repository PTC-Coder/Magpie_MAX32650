
/* Private includes --------------------------------------------------------------------------------------------------*/

#include <stddef.h>
#include "bsp_spixf.h"
#include "bsp_pins.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "spixf.h"

/* Private defines ---------------------------------------------------------------------------------------------------*/

#define BSP_SPIXF_BAUD_RATE     8000000  // 8 MHz SPI clock rate
#define BSP_SPIXF_READ_CMD      0x0B     // Fast read command

/* Public function definitions ---------------------------------------------------------------------------------------*/

int bsp_spixf_init()
{
    MXC_GPIO_Config(&bsp_pins_spixf_active_cfg);

    //Turn on the clocks for SPIXF and XIP ICACHE
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPIXIPM); // SPIX memory
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPIXIPF); // SPIXF flash controller
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_ICACHEXIP); // XIP ICACHE

    // Initialize SPIXF with fast read command and baud rate
    // err = MXC_SPIXF_Init(BSP_SPIXF_READ_CMD, BSP_SPIXF_BAUD_RATE);
    // if (err != E_NO_ERROR) {
    //     return err;
    // }

    return E_NO_ERROR;
}

int bsp_spixf_config()
{
        // Configure SPIXF settings (SPIXF should be disabled after Init)
    MXC_SPIXF_SetSPIFrequency(BSP_SPIXF_BAUD_RATE);
    MXC_SPIXF_SetMode(MXC_SPIXF_MODE_0);
    MXC_SPIXF_SetSSPolActiveLow();
    MXC_SPIXF_SetSSActiveTime(MXC_SPIXF_SYS_CLOCKS_2);
    MXC_SPIXF_SetSSInactiveTime(MXC_SPIXF_SYS_CLOCKS_3);

    // Configure for single data line mode
    MXC_SPIXF_SetCmdValue(BSP_SPIXF_READ_CMD);
    MXC_SPIXF_SetCmdWidth(MXC_SPIXF_SINGLE_SDIO);
    MXC_SPIXF_SetAddrWidth(MXC_SPIXF_SINGLE_SDIO);
    MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_1);
    MXC_SPIXF_SetModeClk(8); // 8 dummy clocks for fast read

    MXC_SPIXF_Set3ByteAddr();
    MXC_SPIXF_SCKFeedbackEnable();
    MXC_SPIXF_SetSCKNonInverted();

    return E_NO_ERROR;
}

int bsp_spixf_deinit()
{
    MXC_SPIXF_Disable();
    MXC_GPIO_Config(&bsp_pins_spixf_high_z_cfg);
    return E_NO_ERROR;
}

// /******************************************************************************/
// //SPIFX read write setup communication functions (not the actual flash read/write)
// int bsp_spifx_flash_module_read(uint8_t *read, unsigned len, unsigned deassert,
//                                 Ext_Flash_DataLine_t width)
// {
//     mxc_spixf_req_t req = { deassert, 0, NULL, read, (mxc_spixf_width_t)width, len, 0, 0, NULL };

//     if (MXC_SPIXF_Transaction(&req) != len) {
//         return E_COMM_ERR;
//     }
//     return E_NO_ERROR;
// }

// /******************************************************************************/
// int bsp_spifx_flash_module_write(const uint8_t *write, unsigned len, unsigned deassert,
//                                  Ext_Flash_DataLine_t width)
// {
//     mxc_spixf_req_t req = { deassert, 0, write, NULL, (mxc_spixf_width_t)width, len, 0, 0, NULL };

//     if (MXC_SPIXF_Transaction(&req) != len) {
//         return E_COMM_ERR;
//     }
//     return E_NO_ERROR;
// }

// /******************************************************************************/
// int bsp_spifx_flash_clock(unsigned len, unsigned deassert)
// {
//     return MXC_SPIXF_Clocks(len, deassert);
// }

// int bsp_setup_spifx_comm(void)
// {
//     int err;
//     Ext_Flash_Config_t exf_cfg = { .init = bsp_spifx_flash_module_init,
//                                    .read = bsp_spifx_flash_module_read,
//                                    .write = bsp_spifx_flash_module_write,
//                                    .clock = bsp_spifx_flash_clock };

//     if ((err = Ext_Flash_Configure(&exf_cfg)) != E_NO_ERROR) {
//         return err;
//     }
//     return E_NO_ERROR;
// }
// /******************************************************************************/