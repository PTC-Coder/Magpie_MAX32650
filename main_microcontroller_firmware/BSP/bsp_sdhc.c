/**
 * IMPORTANT: as of this writing, the preprocessor definition `MSDK_NO_GPIO_CLK_INIT` has no effect on the SDHC lib,
 * this means that ALL of the SDHC pins will be configured when we call `MXC_SDHC_Init()`. The problem with this is
 * that we use P1.6 for an unrelated task. This pin is the SDHC Write Protect pin, but we don't use it for that.
 *
 * Because of this we need to make sure that initializing an de-initializing the SDHC does not overwrite the pin config
 * for P1.6. We need to read the pin config for P1.6, stash it, and then reconfigure P1.6 when we're done configuring
 * the SDHC peripheral.
 *
 * This will result in a brief glitch on P1.6, but this won't affect any critical functionality, as this is the ADC
 * chip select enable pin, and the ADC should not be converting before the SD card is initialized and ready.
 */

/* Private includes --------------------------------------------------------------------------------------------------*/

#include "board.h"
#include "bsp_pins.h"
#include "bsp_sdhc.h"
#include "sdhc_lib.h"

/* Private defines ---------------------------------------------------------------------------------------------------*/

#define SD_CARD_INIT_NUM_RETRIES (100)

#define SDHC_CONFIG_BLOCK_GAP (0)
#define SDHC_CONFIG_CLK_DIV (0x0b0)

/* Public function definitions ---------------------------------------------------------------------------------------*/

int bsp_sdhc_init()
{   
    MXC_GPIO_Config(&bsp_pins_sdhc_active_cfg);

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SDHC);

    const mxc_sdhc_cfg_t sdhc_cfg = {
        .bus_voltage = MXC_SDHC_Bus_Voltage_3_3,
        .block_gap = SDHC_CONFIG_BLOCK_GAP,
        .clk_div = SDHC_CONFIG_CLK_DIV,
    };

    return MXC_SDHC_Init(&sdhc_cfg);
}

int bsp_sdhc_deinit()
{
    MXC_GPIO_Config(&bsp_pins_sdhc_high_z_cfg);

    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SDHC);

    return MXC_SDHC_Shutdown();
}
