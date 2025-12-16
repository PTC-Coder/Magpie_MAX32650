/**
 * @file    max17261.h
 * @brief   A software module for controlling the max17261 fuel gauge is represented here.
 * @details This mddule is responsible for initializing, configuring and reading the fueal gauge.
            
 *
 * This module requires:
 * - shared I2C bus on the 1v8 domain using 7 bit address 0x36u
 */

#ifndef MAX17261_H_
#define MAX17261_H_

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

/* Data Structures ---------------------------------------------------------------------------------------------------*/

/**
 * @brief Structure to hold fuel gauge measurement data
 */
typedef struct {
    double vcell_voltage;      // Instantaneous cell voltage in V (per cell)
    double avg_vcell_voltage;  // Average cell voltage in V (per cell)
    double pack_voltage;       // Instantaneous pack voltage in V (vcell_voltage * 2 for 2S)
    double avg_pack_voltage;   // Average pack voltage in V (avg_vcell_voltage * 2 for 2S)
    double current_ma;         // Instantaneous current in mA
    double avg_current_ma;     // Average current in mA
    double power_mw;           // Instantaneous power in mW
    double avg_power_mw;       // Average power in mW
    double temperature_c;      // Instantaneous temperature in °C
    double avg_temperature_c;  // Average temperature in °C
    double qh_mah;            // Coulomb counter in mAh
    uint16_t vcell_raw;       // Raw VCell register value
    uint16_t avg_vcell_raw;   // Raw AvgVCell register value
    uint16_t current_raw;     // Raw Current register value
    uint16_t avg_current_raw; // Raw AvgCurrent register value
    uint16_t temperature_raw; // Raw Temperature register value
    uint16_t avg_temperature_raw; // Raw AvgTA register value
    uint16_t qh_raw;          // Raw QH register value
} fuel_gauge_data_t;



/**
 * @brief    max17261_soft_reset(). Function to "soft reset" the MAX17261 device completely.
 * @retval   'E_SUCCESS` if successful, else a negative error code.
 ****************************************************************************/
int max17261_soft_reset(void);

/**
 * @brief    max17261_por_detected(). Function to check if a power-on-reset
 *           has occurred with the max17261 device.
 * @return   true on success, false on failure
 ****************************************************************************/
bool max17261_por_detected(void);

/**
 * @brief    max17261_clear_por_bit(). Function to clear a power-on-reset flag that
 *           has occurred on the max17261 device.
 ****************************************************************************/
void max17261_clear_por_bit(void);

/**
 * @brief    max17261_wait_dnr(). Function to wait for the MAX1726x to complete
 *           its startup operations. See "MAX1726x Software Implementation Guide" for details
 * @param[in] timeout_ms. Timeout in milliseconds (default 2000ms if 0 is passed)
 * @return   1 if successful (DNR cleared), 0 if timeout occurred
 ****************************************************************************/
int max17261_wait_dnr(uint32_t timeout_ms);



/**
 * @brief   max17261_read_repsoc(). This function gets the state of charge (SOC) as
 *          described in Software Implementation guide for the MAX17261. It gives how much
 *          charge is left on the battery.
 * @return the "HiByte" of RepSOC register for %1 resolution
 ****************************************************************************/
uint8_t max17261_read_repsoc(void);

/**
 * @brief    max17261_read_repcap(). This function gets the RepCap register data
 *           as described in Software Implementation guide. It gives how many milli amp
 *           hours is left on the battery.
 * @return   the 2-byte register data as a uint16_t.
 ****************************************************************************/
uint16_t max17261_read_repcap(void);

/**
 * @brief    max17261_read_designcap(). This function reads the DesignCap register
 *           to check the configured battery capacity.
 * @return   the 2-byte register data as a uint16_t.
 ****************************************************************************/
uint16_t max17261_read_designcap(void);

/**
 * @brief    max17261_reset_qh(). This function resets the QH register (coulomb counter)
 *           to zero. Use with caution as this affects fuel gauge accuracy.
 * @return   E_SUCCESS if successful, E_FAIL if failed.
 ****************************************************************************/
int max17261_reset_qh(void);


/**
 * @brief    max17261_read_tte(). This function reads the TTE register data
 *           as described in Software Implementation guide. It returns the time
 *           to empty in seconds as a time_t value.
 * @return   double representing hours until battery is empty
 ****************************************************************************/
double max17261_read_tte(void);

/**
 * @brief    max17261_calculate_tte_manual(). This function calculates TTE manually
 *           using RepCap and AvgCurrent for large batteries where the TTE register
 *           saturates at its maximum value (4.27 days).
 * @return   double representing hours until battery is empty
 ****************************************************************************/
double max17261_calculate_tte_manual(void);

/**
 * @brief    max17261_read_capacity_debug(). This function reads and displays
 *           various capacity-related registers for debugging fuel gauge behavior.
 ****************************************************************************/
void max17261_read_capacity_debug(void);

/**
 * @brief    max17261_config_ez(). This function performs EZ configuraiton
 *           as described in Software Implementation guide.
 * @param[in] timeout_ms. Timeout in milliseconds (default 2000ms if 0 is passed)
 * @return   1 if successful (configuration completed), 0 if timeout occurred
 ****************************************************************************/
int max17261_config_ez(uint32_t timeout_ms);

/**
 * @brief    Fuel_gauge_data_collect. This function reads fuel gauge registers and returns the data.
 * @param[in]   label. Label to identify the measurement (e.g., "Before", "After").
 * @return      fuel_gauge_data_t structure containing all measurements.
 ****************************************************************************/
fuel_gauge_data_t Fuel_gauge_data_collect(const char *label);

/**
 * @brief    max17261_read_filtercfg_debug(). This function reads and displays
 *           the current FilterCfg register settings for debugging averaging behavior.
 ****************************************************************************/
void max17261_read_filtercfg_debug(void);

/**
 * @brief    max17261_read_device_id(). This function reads the device ID register
 *           to verify communication with the MAX17261.
 * @return   Device ID as uint16_t, or 0 if read failed.
 ****************************************************************************/
uint16_t max17261_read_device_id(void);

/**
 * @brief    max17261_read_current_calibration(). This function reads and displays
 *           the current calibration registers (CGain and COff) for debugging.
 ****************************************************************************/
void max17261_read_current_calibration(void);

/**
 * @brief    max17261_set_current_gain(). This function sets the current gain calibration.
 * @param[in] gain_value. CGain register value (0x0400 = no adjustment, default)
 * @return   E_SUCCESS if successful, E_FAIL if failed.
 ****************************************************************************/
int max17261_set_current_gain(uint16_t gain_value);


#endif /* MAX17261_H_ */