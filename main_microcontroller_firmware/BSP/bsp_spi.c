
/* Private includes --------------------------------------------------------------------------------------------------*/

#include "spi.h"

#include "board.h"
#include "bsp_pins.h"
#include "bsp_spi.h"
#include "mxc_errors.h"

/* Public variables --------------------------------------------------------------------------------------------------*/

mxc_spi_regs_t *bsp_spi_adc_cfg_spi_handle = MXC_SPI2;

// note that channel 0 uses SPI1, and channel 1 uses SPI 0, this is because of the way the PCB is routed
mxc_spi_regs_t *bsp_spi_adc_ch0_data_spi_handle = MXC_SPI1;
const mxc_dma_reqsel_t bsp_spi_adc_ch0_data_spi_dma_req = MXC_DMA_REQUEST_SPI1RX;

mxc_spi_regs_t *bsp_spi_adc_ch1_data_spi_handle = MXC_SPI0;
const mxc_dma_reqsel_t bsp_spi_adc_ch1_data_spi_dma_req = MXC_DMA_REQUEST_SPI0RX;

/* Private function declarations -------------------------------------------------------------------------------------*/

/**
 * `reset_spi_and_en_periph_clk(hspi)` enables the peripheral clock and resets SPI peripheral `hspi`.
 *
 * This function is necessary due to the definition of `MSDK_NO_GPIO_CLK_INIT`, it replicates the code in the MSDK SPI
 * init functions which is not compiled due to the `MSDK_NO_GPIO_CLK_INIT` definition.
 *
 * @retval `E_NO_ERROR` if successful, else a negative error code
 */
static int reset_spi_and_en_periph_clk(const mxc_spi_regs_t *hspi);

/* Public function definitions ---------------------------------------------------------------------------------------*/

int bsp_adc_config_spi_init()
{
    MXC_GPIO_Config(&bsp_pins_adc_cfg_spi_active_cfg); // spi pins

    MXC_GPIO_Config(&bsp_pins_adc_cfg_spi_cs_out_cfg);      // manual chip select pin
    gpio_write_pin(&bsp_pins_adc_cfg_spi_cs_out_cfg, true); // chip select high at startup

    return reset_spi_and_en_periph_clk(bsp_spi_adc_cfg_spi_handle);
}

int bsp_adc_config_spi_deinit()
{
    MXC_GPIO_Config(&bsp_pins_adc_cfg_spi_high_z_cfg);

    MXC_GPIO_Config(&bsp_pins_adc_cfg_spi_cs_high_z_cfg);

    return MXC_SPI_Shutdown(bsp_spi_adc_cfg_spi_handle);
}

int bsp_adc_ch0_data_spi_init()
{
    MXC_GPIO_Config(&bsp_pins_adc_ch0_data_spi_active_cfg);

    return reset_spi_and_en_periph_clk(bsp_spi_adc_ch0_data_spi_handle);
}

int bsp_adc_ch0_data_spi_deinit()
{
    MXC_GPIO_Config(&bsp_pins_adc_ch0_data_spi_high_z_cfg);

    return MXC_SPI_Shutdown(bsp_spi_adc_ch0_data_spi_handle);
}

int bsp_adc_ch1_data_spi_init()
{
    MXC_GPIO_Config(&bsp_pins_adc_ch1_data_spi_active_cfg);

    return reset_spi_and_en_periph_clk(bsp_spi_adc_ch1_data_spi_handle);
}

int bsp_adc_ch1_data_spi_deinit()
{
    MXC_GPIO_Config(&bsp_pins_adc_ch1_data_spi_high_z_cfg);

    return MXC_SPI_Shutdown(bsp_spi_adc_ch1_data_spi_handle);
}

/* Private function definitions --------------------------------------------------------------------------------------*/

int reset_spi_and_en_periph_clk(const mxc_spi_regs_t *hspi)
{
    if (hspi == MXC_SPI0)
    {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_SPI0);
    }
    else if (hspi == MXC_SPI1)
    {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_SPI1);
    }
    else if (hspi == MXC_SPI2)
    {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI2);
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_SPI2);
    }
    else
    {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}
