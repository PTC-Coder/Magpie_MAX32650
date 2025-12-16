/**
 * @file environmental_sensor.h
 * @brief BME688 Environmental Sensor Driver Header
 * 
 * This file contains the interface for the BME688 environmental sensor
 * which provides temperature, humidity, pressure, and gas resistance measurements.
 * 
 * BME688 I2C Address: 0x77 (7-bit)
 * Operating Voltage: 1.8V I2C interface
 */

#ifndef ENVIRONMENTAL_SENSOR_H
#define ENVIRONMENTAL_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/* BME688 I2C Address - 0x77 when SDO connected to VDDIO, 0x76 when SDO to GND */
#define BME688_I2C_ADDR 0x77u  // Your sensor is at 0x77 (SDO connected to VDDIO)

/* BME688 Data Structure */
typedef struct {
    // Raw register values
    uint32_t temperature_raw;
    uint32_t pressure_raw;
    uint32_t humidity_raw;
    uint32_t gas_resistance_raw;
    
    // Converted values
    float temperature_c;        // Temperature in Celsius
    float pressure_pa;          // Pressure in Pascals
    float humidity_percent;     // Relative humidity in %
    float gas_resistance_ohm;   // Gas resistance in Ohms
    
    // Status
    bool valid_data;
    uint8_t gas_range;
    bool gas_valid;
    bool heat_stable;
} bme688_data_t;

/* Function Prototypes */

/**
 * @brief Initialize BME688 sensor
 * Performs complete initialization sequence:
 * 1. Soft reset
 * 2. Chip ID verification (expects 0x61)
 * 3. Calibration data reading
 * 4. Sensor configuration for forced mode operation
 * @return 0 on success, negative error code on failure
 */
int bme688_init(void);

/**
 * @brief Perform soft reset of BME688
 * @return 0 on success, negative error code on failure
 */
int bme688_soft_reset(void);

/**
 * @brief Read chip ID to verify communication
 * @return Chip ID (0x61 for BME688) or negative error code
 */
int bme688_read_chip_id(void);

/**
 * @brief Configure BME688 for continuous measurement
 * @return 0 on success, negative error code on failure
 */
int bme688_configure(void);

/**
 * @brief Read all sensor data from BME688
 * @param label Optional label for debug output
 * @return Structure containing all sensor readings
 */
bme688_data_t bme688_read_all_data(const char *label);

/**
 * @brief Read temperature from BME688
 * @return Temperature in Celsius
 */
float bme688_read_temperature(void);

/**
 * @brief Read pressure from BME688
 * @return Pressure in Pascals
 */
float bme688_read_pressure(void);

/**
 * @brief Read humidity from BME688
 * @return Relative humidity in percent
 */
float bme688_read_humidity(void);

/**
 * @brief Read gas resistance from BME688
 * @return Gas resistance in Ohms
 */
float bme688_read_gas_resistance(void);

#endif /* ENVIRONMENTAL_SENSOR_H */