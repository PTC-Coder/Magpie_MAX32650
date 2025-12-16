
/* Private includes --------------------------------------------------------------------------------------------------*/

#include <stddef.h>

#include "hpb.h"
#include "mxc_sys.h"
#include "mxc_errors.h"
#include "emcc.h"

#include "board.h"
#include "bsp_hyperbus.h"

/* Private defines ---------------------------------------------------------------------------------------------------*/

/**
 * @brief HyperRAM Configuration Register 0 address
 */
#define HYPERRAM_CR0_ADDR (0x01000)

/* Private types -----------------------------------------------------------------------------------------------------*/

/**
 * @brief Deep Power Down Enable options for CR0[15]
 */
typedef enum
{
    HYPERRAM_DPD_ENABLE = 0,          /**< Writing 0 to CR0[15] causes the device to enter Deep Power Dow */
    HYPERRAM_DPD_NORMAL_OPERATION = 1 /**< Normal operation (default) */
} hyperram_dpd_enable_t;

/**
 * @brief Drive Strength options for CR0[14:12]
 */
typedef enum
{
    HYPERRAM_DRIVE_34_OHMS_DEFAULT = 0b000, /**< 34 ohms (default) */
    HYPERRAM_DRIVE_115_OHMS = 0b001,        /**< 115 ohms */
    HYPERRAM_DRIVE_67_OHMS = 0b010,         /**< 67 ohms */
    HYPERRAM_DRIVE_46_OHMS = 0b011,         /**< 46 ohms */
    HYPERRAM_DRIVE_34_OHMS = 0b100,         /**< 34 ohms */
    HYPERRAM_DRIVE_27_OHMS = 0b101,         /**< 27 ohms */
    HYPERRAM_DRIVE_22_OHMS = 0b110,         /**< 22 ohms */
    HYPERRAM_DRIVE_19_OHMS = 0b111          /**< 19 ohms */
} hyperram_drive_strength_t;

/**
 * @brief Initial Latency options for CR0[7:4]
 */
typedef enum
{
    HYPERRAM_LATENCY_5_CLK_133MHZ = 0b0000, /**< 5 Clock Latency @ 133MHz Max */
    HYPERRAM_LATENCY_6_CLK_166MHZ = 0b0001, /**< 6 Clock Latency @ 166MHz Max */
    HYPERRAM_LATENCY_7_CLK_200MHZ = 0b0010, /**< 7 Clock Latency @ 200MHz Max (default) */
    HYPERRAM_LATENCY_7_CLK_250MHZ = 0b0100, /**< 7 Clock Latency @ 250MHz Max */
    HYPERRAM_LATENCY_3_CLK_85MHZ = 0b1110,  /**< 3 Clock Latency @ 85MHz Max */
    HYPERRAM_LATENCY_4_CLK_104MHZ = 0b1111  /**< 4 Clock Latency @ 104MHz Max */
} hyperram_initial_latency_t;

/**
 * @brief Fixed Latency Enable options for CR0[3]
 */
typedef enum
{
    HYPERRAM_VARIABLE_LATENCY = 0b0, /**< Variable Latency - 1 or 2 times Initial Latency */
    HYPERRAM_FIXED_LATENCY = 0b1     /**< Fixed 2 times Initial Latency (default) */
} hyperram_fixed_latency_enable_t;

/**
 * @brief Hybrid Burst Enable options for CR0[2]
 */
typedef enum
{
    HYPERRAM_HYBRID_BURST = 0b0, /**< Wrapped burst sequences follow hybrid burst sequencing */
    HYPERRAM_LEGACY_BURST = 0b1  /**< Wrapped burst sequences in legacy wrapped burst manner (default) */
} hyperram_hybrid_burst_enable_t;

/**
 * @brief Burst Length options for CR0[1:0]
 */
typedef enum
{
    HYPERRAM_BURST_128_BYTES = 0b00, /**< 128 bytes */
    HYPERRAM_BURST_64_BYTES = 0b01,  /**< 64 bytes */
    HYPERRAM_BURST_16_BYTES = 0b10,  /**< 16 bytes */
    HYPERRAM_BURST_32_BYTES = 0b11   /**< 32 bytes (default) */
} hyperram_burst_length_t;

/**
 * @brief HyperRAM Configuration Register 0 structure
 */
typedef struct
{
    hyperram_dpd_enable_t dpd_enable;                     /**< Deep Power Down Enable [15] */
    hyperram_drive_strength_t drive_strength;             /**< Drive Strength [14:12] */
    uint8_t reserved;                                     /**< Reserved bits [11:8] - should be 0x1 */
    hyperram_initial_latency_t initial_latency;           /**< Initial Latency [7:4] */
    hyperram_fixed_latency_enable_t fixed_latency_enable; /**< Fixed Latency Enable [3] */
    hyperram_hybrid_burst_enable_t hybrid_burst_enable;   /**< Hybrid Burst Enable [2] */
    hyperram_burst_length_t burst_length;                 /**< Burst Length [1:0] */
} hyperram_cr0_config_t;

/* Private variables -------------------------------------------------------------------------------------------------*/

extern uint8_t __hpb_cs0_start;

/* Private function definitions --------------------------------------------------------------------------------------*/

/**
 * @brief Convert HyperRAM CR0 configuration structure to register value
 *
 * @param config Pointer to configuration structure
 * @return uint16_t Register value to write to CR0
 */
static uint16_t hyperram_cr0_config_to_value(const hyperram_cr0_config_t *config)
{
    uint16_t value = 0;

    value |= (config->dpd_enable & 0b1) << 15;
    value |= (config->drive_strength & 0b111) << 12;
    value |= (config->reserved & 0b1111) << 8;
    value |= (config->initial_latency & 0b1111) << 4;
    value |= (config->fixed_latency_enable & 0b1) << 3;
    value |= (config->hybrid_burst_enable & 0b1) << 2;
    value |= (config->burst_length & 0b11) << 0;

    return value;
}

/* Public function definitions ---------------------------------------------------------------------------------------*/

int bsp_hyperbus_init()
{
    const mxc_gpio_cfg_t bsp_pins_hyperbus_active_cfg = {
        .port = MXC_GPIO1,
        .mask = (MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 | MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
        .pad = MXC_GPIO_PAD_NONE,
        .func = MXC_GPIO_FUNC_ALT1,
        .vssel = MXC_GPIO_VSSEL_VDDIO,
        .drvstr = MXC_GPIO_DRVSTR_0,
    };
    MXC_GPIO_Config(&bsp_pins_hyperbus_active_cfg);

    const mxc_gpio_cfg_t bsp_pins_hyperbus_reset_pin_active_cfg = {
        .port = MXC_GPIO1,
        .mask = (MXC_GPIO_PIN_17),
        .pad = MXC_GPIO_PAD_NONE,
        .func = MXC_GPIO_FUNC_OUT,
        .vssel = MXC_GPIO_VSSEL_VDDIO,
        .drvstr = MXC_GPIO_DRVSTR_0,
    };
    MXC_GPIO_Config(&bsp_pins_hyperbus_reset_pin_active_cfg);
    gpio_write_pin(&bsp_pins_hyperbus_reset_pin_active_cfg, false);
    bsp_delay_ms(1);
    // set the reset pin high to enable the PSRAM
    gpio_write_pin(&bsp_pins_hyperbus_reset_pin_active_cfg, true);

    // required because we define MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_HBC);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SCACHE);

    // Enable and configure the External Memory Cache Controller (EMCC) for HyperBus
    // Safe to use cache now since DMA writes to SRAM first, then CPU copies to HyperRAM
    MXC_EMCC_Enable();
    MXC_EMCC_Flush();

    mxc_hpb_mem_config_t mem;
    mxc_hpb_cfg_reg_val_t cfg_reg[1];
    int result;

    /* Hyperbus RAM chip configuration */
    const hyperram_cr0_config_t cr0_config = {
        .dpd_enable = HYPERRAM_DPD_NORMAL_OPERATION,
        .drive_strength = HYPERRAM_DRIVE_34_OHMS_DEFAULT,
        .reserved = 0x1,
        .initial_latency = HYPERRAM_LATENCY_3_CLK_85MHZ,   // MAX32650 hyperbus clock is 60MHz
        .fixed_latency_enable = HYPERRAM_VARIABLE_LATENCY, // Variable latency for better sequential performance
        .hybrid_burst_enable = HYPERRAM_LEGACY_BURST,
        .burst_length = HYPERRAM_BURST_128_BYTES, // Larger bursts for better throughput
    };

    cfg_reg[0].addr = HYPERRAM_CR0_ADDR;
    cfg_reg[0].val = hyperram_cr0_config_to_value(&cr0_config);

    mem.base_addr = (unsigned int)&__hpb_cs0_start;

    mem.device_type = MXC_HPB_DEV_HYPER_RAM;
    mem.cfg_reg_val = cfg_reg;
    mem.cfg_reg_val_len = 1;
    mem.read_cs_high = MXC_HPB_CS_HIGH_3_5;         // Aggressive timing for 60MHz
    mem.write_cs_high = MXC_HPB_CS_HIGH_4_5;        // Tighter write timing
    mem.read_cs_setup = MXC_HPB_CS_SETUP_HOLD_3;    // Minimum setup for reads
    mem.write_cs_setup = MXC_HPB_CS_SETUP_HOLD_5;   // Tighter write setup
    mem.read_cs_hold = MXC_HPB_CS_SETUP_HOLD_2;     // Minimum hold for reads
    mem.write_cs_hold = MXC_HPB_CS_SETUP_HOLD_8;    // Reduced from 12, still safe at 60MHz
    mem.latency_cycle = MXC_V_HPB_MTR_LATENCY_3CLK; // Must match chip config in the cfg_reg from above
    mem.fixed_latency = 0;                          // Variable latency enabled, make sure to match cfg_reg setting

    result = MXC_HPB_Init(&mem, NULL);

    if (result == E_NO_ERROR)
    {
        // Wait for EMCC to be ready before proceeding
        while (!MXC_EMCC_Ready())
        {
            // Wait for cache to be ready
        }
    }

    return result;
}

int bsp_hyperbus_deinit()
{
    const mxc_gpio_cfg_t bsp_pins_hyperbus_high_z_cfg = {
        .port = MXC_GPIO1,
        .mask = (MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12 | MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14 | MXC_GPIO_PIN_15 | MXC_GPIO_PIN_16 | MXC_GPIO_PIN_18 | MXC_GPIO_PIN_19 | MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21),
        .pad = MXC_GPIO_PAD_NONE,
        .func = MXC_GPIO_FUNC_IN,
        .vssel = MXC_GPIO_VSSEL_VDDIO,
    };
    MXC_GPIO_Config(&bsp_pins_hyperbus_high_z_cfg);

    const mxc_gpio_cfg_t bsp_pins_hyperbus_reset_pin_high_z_cfg = {
        .port = MXC_GPIO1,
        .mask = (MXC_GPIO_PIN_17),
        .pad = MXC_GPIO_PAD_NONE,
        .func = MXC_GPIO_FUNC_IN,
        .vssel = MXC_GPIO_VSSEL_VDDIO,
    };
    MXC_GPIO_Config(&bsp_pins_hyperbus_reset_pin_high_z_cfg);

    // Disable and flush EMCC cache before deinit
    MXC_EMCC_Flush();
    MXC_EMCC_Disable();

    // TODO, should we do any other de-init stuff? I'm not sure if we'll ever de-init in practice since we need the PSRAM on all the time while recording
    return E_NO_ERROR;
}

void bsp_hyperbus_flush_cache(void)
{
    MXC_EMCC_Flush();
}
