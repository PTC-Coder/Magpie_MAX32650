/**
 * Description:
 * This file represents a software module for controlling the fuel gauge.
 * A MAX17261 is used in this application.
 *
 * 1)The driver helps us to write and read different parametrs from the fuel gauge.
   2)Fuel gauge parameters -- customer defined for Option 1 EZ Config.
	 See App Note: "MAX1726x Software Implementation Guide"
	 https://www.analog.com/media/en/technical-documentation/user-guides/modelgauge-m5-host-side-software-implementation-guide.pdf
	 https://www.analog.com/media/en/technical-documentation/user-guides/max1726x-modelgauge-m5-ez-user-guide.pdf
 *
 **/

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "mxc_delay.h"
#include "fuel_gauge.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"
#include <stdlib.h>
#include "bsp_i2c.h"
#include "bsp_pins.h"
#include "mxc_device.h"
#include "bsp_status_led.h"
#include "bsp_pushbutton.h"

/* Private defines ----------------------------------------------------------------------------------------------------*/
#define ShowPrintFOutput //Comment this if you want to disable printf
#define MAX17261_I2C_ADDR 0x36u // Fuel Gauge I2C 7 bit address

//Time LSB values please refere to https://www.analog.com/media/en/technical-documentation/user-guides/modelgauge-m5-host-side-software-implementation-guide.pdf
#define tte_hr 1.6				// Bits 15:10 unit = 1.6 hours
#define tte_min 1.5			// Bits 9:4 unit = 1.5 minutes
#define tte_sec 5.625			// Bits 3:0 unit = 5.625 seconds

#define CURRENT_REG_RES 781.25      // This is for 2mΩ sense resistor - 1.5625µV/0.002Ω = 0.78125mA per LSB
#define CAPACITY_LSB 2.5            // Capacity LSB for 0.002Ω sense resistor: 5µVh/0.002Ω = 2.5mAh per LSB
#define VCELL_LSB 7.8125e-5         // 16-bit register 7.8125 uV per lsb (0.0V to 5.11992V)  Table 2 in MAX17261.pdf
#define TTE_LSB  1.5625e-3          // (5.625 / 3600.0)  // Convert seconds to hours: 5.625s ÷ 3600s/hr = 0.0015625 hours
#define TEMP_LSB 3.90625e-3         // Temperature LSB = 1/256°C per bit (from user's guide Table 3)



/* Private MAX17261 specific globals ----------------------------------------------------------------------------------------------*/
uint8_t max17261_regs[256]; // For holding register value to read or write


/* Private enumerations ----------------------------------------------------------------------------------------------*/
/**
 * @brief masks to control different parameters of fuel gauge
 *
 */
enum masks
{
	ModelCfg = 0x8000, 
	DesignCap = 0x000D,   // 33 mAh: 33/2.5 = 13.2 ≈ 13 decimal = 0x000D (for 0.002Ω sense resistor)
	IChgTerm = 0x0003,    // Charge termination current: ~7.5mA (3 * 2.5mAh = 7.5mA for small battery)
	VEmpty = 0xA061,      // 3.2 V Empty / 3.88 V Recovery (keep existing)
	POR_BIT = 0x0002,
	DNR_BIT = 0x0001,
	FilterCfg_Default = 0xCEA9,  // Modified FilterCfg: CURR=9 for 3-minute AvgCurrent time constant

};

/**
 * @brief  Fuel gauge parameter register addresses
 *
 */
typedef enum uint8_t
{
	// ModelGauge M5 EZ configuration registers
	FG_ADDR_DESIGNCAP = 0x18u,   //expected capacity of the cell
	FG_ADDR_VEMPTY = 0x3Au,   //Empty Voltage Target, during load.
	FG_ADDR_MODELCFG = 0xDBu, // The ModelCFG register controls basic options of the EZ algorithm
	FG_ADDR_ICHGTERM = 0x1Eu,  //device to detect when a charge cycle of the cell has completed
	FG_ADDR_CONFIG1 = 0x1Du,
	FG_ADDR_CONFIG2 = 0xBBu,
	FG_ADDR_FILTERCFG = 0x29u,  // FilterCfg register for averaging time constants

	// ModelGauge m5 Register memory map
	FullCapNom_addr = 0x23u,
	FullCapRep_addr = 0x10u,
	FullSocThr_addr = 0x13u,
	FullCap_addr = 0x35u,

	LearnCfg_addr = 0x28u,
	FilterCfg_addr = 0x29u,
	RelaxCfg_addr = 0x2Au,
	MiscCfg_addr = 0x2Bu,
	QRTable00_addr = 0x12u,
	QRTable10_addr = 0x22u,
	QRTable20_addr = 0x32u,
	QRTable30_addr = 0x42u,
	RComp0_addr = 0x38u,
	TempCo_addr = 0x39u,

	// Current calibration registers
	CGain_addr = 0x2Eu,
	COff_addr = 0x2Fu,

	// Fuel gauge parameter other can be found in
	// https://www.analog.com/media/en/technical-documentation/user-guides/max1726x-modelgauge-m5-ez-user-guide.pdf
	FG_ADDR_FSTAT = 0x3Du,
	FG_ADDR_HibCFG = 0xBAu,
	FG_ADDR_SOFT_RESET = 0x60u,
	DevName_addr = 0x21u,
	FG_ADDR_STATUS = 0x00u,

	// Fuel Gauge parameters to read/log
	FG_ADDR_VCELL = 0x09u,  //cell voltage instantaneous
	FG_ADDR_AVGVCELL = 0x19u, //cell voltage average
	FG_ADDR_TEMP = 0x08u,
	FG_ADDR_AVGTEMP = 0x16u,
	FG_ADDR_CURRENT = 0x0Au,
	FG_ADDR_AVG_CURRENT = 0x0Bu,
	TTF_addr = 0x20u,   //Time to full
	RepCap_addr = 0x05u,
	VFRemCap_addr = 0x4Au,
	MixCap_addr = 0x0Fu,
	QResidual_addr = 0x0Cu,
	REPSOC_addr = 0x06u,
	AvSOC_addr = 0x0Eu,
	Timer_addr = 0x3Eu,
	TimerH_addr = 0xBEu,
	FG_ADDR_QH = 0x4Du,
	AvCap_addr = 0x1Fu,
	MixSOC_addr = 0x0Du,
	FG_ADDR_TTE = 0x11u,   // Time to empty
} max17261_registers_addr_t;

/* Private function declarations --------------------------------------------------------------------------------------*/

/**
 * @brief   max17261_write_reg. Generic function to read MAX17261 registers.
 * @param[out]  dev_addr. The 1-byte slave address of MAX17261.
 * @param[out]  reg_addr. The 1-byte address of the register on the I2C slave to start writing to.
 * @param[in]  *reg_data. Array of uint8_t data to write to the I2C slave.
 * @param[out]  len. Number of uint16_t registers to write.
 *  @retval      'E_NO_ERROR` if successful, else a negative error code.
 ****************************************************************************/

int max17261_write_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t len);

/**
 * @brief   max17261_read_reg. Generic function to read max17261 registers.
 * @param[out]  dev_addr. The 1-byte slave address of energy harvester.
 * @param[out]  reg_addr. The 1-byte address of the register on the I2C slave to start reading from.
 * @param[in]  *reg_data. Array of uint8_t data to read from the I2C slave.
 * @param[out]  len. Number of uint16_t registers to read.
 * @retval      'E_NO_ERROR` if successful, else a negative error code.
 ****************************************************************************/
int max17261_read_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t len);

/**
 * @brief   max17261_write_verify_reg. Generic function to read max17261 registers.
 * @param[out]  dev_addr. The 1-byte slave address of energy harvester.
 * @param[out]  reg_addr. The 1-byte address of the register on the I2C slave to start reading from.
 * @param[in]  *reg_data. Array of uint8_t data to read from the I2C slave.
 * @param[out]  num_of_byts. Number of bytes to read.
 * @param[in]   timeout_ms. Timeout in milliseconds (default 2000ms if 0 is passed)
 * @return      true on success, false on failure or timeout
 ****************************************************************************/
bool max17261_write_verify_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t num_of_bytes, uint32_t timeout_ms);

/* max17261 function definitions --------------------------------------------------------------------------------------*/

int max17261_write_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t len)
{

	int rslt = E_NO_ERROR;
	// Allocate memory for register address and data
	uint8_t *TXData = (uint8_t *)malloc(sizeof(reg_data) + sizeof(reg_addr));
	memcpy(TXData, &reg_addr, 1);	   // First sends the register address
	memcpy(TXData + 1, reg_data, len); // Then sends the data

	mxc_i2c_req_t reqMaster =
		{
			.addr = dev_addr,
			.tx_buf = TXData,
			.tx_len = len + 1,
			.callback = NULL,
			.rx_buf = NULL,
			.rx_len = 0,
			.i2c = bsp_i2c_3v3_i2c_handle,
			.restart = 0,

		};

	// Send I2C data
	if ((rslt = MXC_I2C_MasterTransaction(&reqMaster)) != E_NO_ERROR)
	{
		// Communication error
		if (rslt != 1)
		{
#if defined(ShowPrintFOutput)
			printf("Error (%d) writing data: Device = 0x%X; Register = 0x%X\n", rslt, dev_addr, reg_addr);
#endif
			return E_UNDERFLOW;
		}
		// Message not acknowledged
		else
		{
#if defined(ShowPrintFOutput)
			printf("Write was not acknowledged: Device = 0x%X; Register = 0x%X\n", dev_addr, reg_addr);
#endif
			return E_NO_RESPONSE;
		}
	}

	free(TXData);

	return E_NO_ERROR;
}

int max17261_read_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t len)
{

	int rslt = E_SUCCESS;

	mxc_i2c_req_t reqMaster =
		{
			.i2c = bsp_i2c_3v3_i2c_handle,
			.addr = dev_addr,
			.tx_buf = &reg_addr,
			.tx_len = 1,
			.callback = NULL,
			.rx_buf = reg_data,
			.rx_len = len,
			.restart = 0,

		};

	if ((rslt = MXC_I2C_MasterTransaction(&reqMaster)) != E_NO_ERROR)
	{

		// Communication error
		if (rslt != 1)
		{
#if defined(ShowPrintFOutput)
			printf("Error (%d) reading data: Device = 0x%X; Register = 0x%X\n", rslt, dev_addr, reg_addr);
#endif
			return E_UNDERFLOW;
		}
		// Message not acknowledged
		else
		{
#if defined(ShowPrintFOutput)
			printf("Read was not acknowledged: Device = 0x%X; Register = 0x%X\n", dev_addr, reg_addr);
#endif
			return E_NO_RESPONSE;
		}
	}
	MXC_Delay(200);
	return E_NO_ERROR;
}

bool max17261_write_verify_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t num_of_bytes, uint32_t timeout_ms)
{
	bool is_verified = false;
	int i = 0;
	uint8_t read_data[256];
	uint32_t timeout = (timeout_ms == 0) ? 2000 : timeout_ms; // Default 2 seconds
	uint32_t max_iterations = timeout / 3; // Each iteration takes ~3ms
	uint32_t iteration_count = 0;
	
	while (!is_verified)
	{
		// Check timeout
		if (iteration_count >= max_iterations)
		{
#if defined(ShowPrintFOutput)
			printf("max17261_write_verify_reg timeout after %d ms\r\n", timeout);
#endif
			return false; // Timeout occurred
		}
		
		max17261_write_reg(dev_addr, reg_addr, reg_data, num_of_bytes);
		// delay 3ms with timer 1
		MXC_Delay(3000);
		iteration_count++;
		
		max17261_read_reg(dev_addr, reg_addr, &read_data[0], num_of_bytes);
#if defined(ShowPrintFOutput)
		printf("write_and_verify reg_data = ");
#endif
		for (i = 0; i < num_of_bytes; i++)
		{
#if defined(ShowPrintFOutput)
			printf("%02X", *(reg_data + i));
#endif
			if (read_data[i] != *(reg_data + i))
			{
				is_verified = false;
				break; // Exit the for loop, continue while loop
			}
		}
		
		if (i == num_of_bytes) // All bytes matched
		{
#if defined(ShowPrintFOutput)
			printf("\n");
#endif
			is_verified = true;
		}
	}
	return is_verified;
}

int max17261_soft_reset(void)
{
	// The procedure for a MAX17261 reset is below:
	// Write 0x000F to 0x60.
	int result;
	max17261_regs[0] = 0x0F;
	max17261_regs[1] = 0x00;
	uint8_t message[] = {max17261_regs[0], max17261_regs[1]};
	result = max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_SOFT_RESET, &message, 2);
	if (result == E_NO_ERROR)
	{
#if defined(ShowPrintFOutput)
		printf("Soft reset succesful\r\n");
#endif
		return E_SUCCESS;
	}
	else
	{
#if defined(ShowPrintFOutput)
		printf("Soft reset failed\r\n");
#endif
		return E_FAIL;
	}
}



//Power on reset
bool max17261_por_detected(void)
{
	//POR (Power-On Reset): This bit is set to 1 when the device detects that a software or
	//hardware POR event has occurred. This bit must be cleared by system software to detect the
	//next POR event. POR is set to 1 at power-up.

	uint16_t status_register = 0;
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_STATUS, &max17261_regs[0x00], 2); // read status register
	status_register = (max17261_regs[1] << 8) + max17261_regs[0];				// make a 16-bit integer representing status register
#if defined(ShowPrintFOutput)
	printf("status_register = %04x\n", status_register);
#endif
	if ((status_register & 0x0002) > 0) // POR bit is in the 2nd position (bit 1 position of Status register)
	{
#if defined(ShowPrintFOutput)
		printf("MAX17261 POR detected\n");
#endif
		return true;
	}
#if defined(ShowPrintFOutput)
	printf("MAX17261 POR not detected\n");
#endif
	return false;
}

void max17261_clear_por_bit(void)
{
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_STATUS, &max17261_regs[0x00], 2);	 // read POR bit again
	max17261_regs[0] = max17261_regs[0] & 0xFD;									 // LSB -- Set POR bit to 0. Bit position 1
	max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_STATUS, &max17261_regs[0x00], 2); // clear POR bit
}

int max17261_wait_dnr(uint32_t timeout_ms)
{
	uint16_t FStat_register = 0;
	uint32_t timeout = (timeout_ms == 0) ? 2000 : timeout_ms; // Default 2 seconds
	uint32_t max_iterations = timeout / 11; // Each iteration takes ~11ms
	uint32_t iteration_count = 0;
	
	// max17261_write_reg(MAX17261_I2C_ADDR,FG_ADDR_FSTAT,&max17261_regs[0x00], 2);
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_FSTAT, &max17261_regs[0x00], 2);
	FStat_register = (max17261_regs[1] << 8) + max17261_regs[0];

	while ((FStat_register & 0x0001) == 0x0001)
	{
		// Check timeout
		if (iteration_count >= max_iterations)
		{
#if defined(ShowPrintFOutput)
			printf("max17261_wait_dnr timeout after %d ms\r\n", timeout);
#endif
			return 0; // Timeout occurred
		}

		// 11 ms delay
		MXC_Delay(11000);
		iteration_count++;
		
		// max17261_write_reg(MAX17261_I2C_ADDR,FG_ADDR_FSTAT,&max17261_regs[0x00], 2);
		max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_FSTAT, &max17261_regs[0x00], 2);
		FStat_register = (max17261_regs[1] << 8) + max17261_regs[0];
	}
	
	return 1; // Success
}

int max17261_config_ez(uint32_t timeout_ms)
{
	uint16_t tempdata;
	uint32_t timeout = (timeout_ms == 0) ? 2000 : timeout_ms; // Default 2 seconds
	uint32_t max_iterations = timeout / 15; // Each iteration takes ~15ms (11+2+2)
	uint32_t iteration_count = 0;
	
	/// Store original HibCFG value, read in HibCfg; prepare to load model
	uint8_t hibcfg_reg[2];
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_HibCFG, &hibcfg_reg[0x00], 2); // read hibcfg register
																			 /// Exit Hibernate Mode step
	max17261_regs[0] = 0x90;
	max17261_regs[1] = 0x00;
	max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_SOFT_RESET, &max17261_regs[0x00], 2); // Soft wakeup (Step 1)
	max17261_regs[0] = 0x00;

	max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_HibCFG, &max17261_regs[0x00], 2);	 // Exit hibernate mode Step 2
	max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_SOFT_RESET, &max17261_regs[0x00], 2); // Exit hibernate mode Step 3

	/// OPTION 1 EZ Config (No INI file is needed)
	// load DesignCap
	tempdata = DesignCap;
	max17261_regs[0] = tempdata & 0x00FF;
	max17261_regs[1] = tempdata >> 8;
#if defined(ShowPrintFOutput)
	printf("Writing DesignCap = 0x%04X (%d) = %d mAh\r\n", tempdata, tempdata, tempdata * 5);
#endif
	max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_DESIGNCAP, &max17261_regs[0x00], 2);

	// load IChgTerm
	tempdata = IChgTerm;
	max17261_regs[0] = tempdata & 0x00FF;
	max17261_regs[1] = tempdata >> 8;
	max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_ICHGTERM, &max17261_regs[0x00], 2);

	// load VEmpty
	tempdata = VEmpty;
	max17261_regs[0] = tempdata & 0x00FF;
	max17261_regs[1] = tempdata >> 8;
	max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_VEMPTY, &max17261_regs[0x00], 2);

	// load FilterCfg (configure averaging time constants before ModelCfg)
	tempdata = FilterCfg_Default;
	max17261_regs[0] = tempdata & 0x00FF;
	max17261_regs[1] = tempdata >> 8;
#if defined(ShowPrintFOutput)
	printf("Writing FilterCfg = 0x%04X (AvgCurrent time constant = 3 minutes)\r\n", tempdata);
#endif
	max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_FILTERCFG, &max17261_regs[0x00], 2);

	// load ModelCfg
	tempdata = ModelCfg;
	max17261_regs[0] = tempdata & 0x00FF;
	max17261_regs[1] = tempdata >> 8;
	max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_MODELCFG, &max17261_regs[0x00], 2);

	// Poll ModelCFG.Refresh bit, do not continue until ModelCFG.Refresh==0
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_MODELCFG, &max17261_regs[0x00], 2); // read status register
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];

	while ((tempdata & 0x8000) == 0x8000)
	{
		// Check timeout
		if (iteration_count >= max_iterations)
		{
#if defined(ShowPrintFOutput)
			printf("max17261_config_ez timeout after %d ms\r\n", timeout);
#endif
			return 0; // Timeout occurred
		}
		
		MXC_Delay(11000);															  // 11 ms delay // delay 11ms
		max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_MODELCFG, &max17261_regs[0x00], 2); // read ModelCfg register
		MXC_Delay(2000);
		tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
		MXC_Delay(2000);
		iteration_count++;
	}

	// Restore Original HibCFG value
	MXC_Delay(2000);
	max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_HibCFG, &hibcfg_reg[0x00], 2);
	MXC_Delay(2000);
	
	return 1; // Success
}

uint8_t max17261_read_repsoc(void)
{
	max17261_read_reg(MAX17261_I2C_ADDR, REPSOC_addr, &max17261_regs[0x00], 2); // Read RepSOC
	return (max17261_regs[1]);													// The RepSOC "HiByte" can be directly displayed to the user for 1% resolution.
}

uint16_t max17261_read_repcap(void)
{
	uint16_t tempdata = 0;
	max17261_read_reg(MAX17261_I2C_ADDR, 0x05, &max17261_regs[0x00], 2); // Read RepCap
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	return (tempdata);
}

uint16_t max17261_read_designcap(void)
{
	uint16_t tempdata = 0;
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_DESIGNCAP, &max17261_regs[0x00], 2); // Read DesignCap
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	return (tempdata);
}

int max17261_reset_qh(void)
{
	// Reset QH register (coulomb counter) to 0x0000
	max17261_regs[0] = 0x00;
	max17261_regs[1] = 0x00;
	
#if defined(ShowPrintFOutput)
	printf("Resetting QH register to 0x0000\r\n");
#endif
	
	int result = max17261_write_reg(MAX17261_I2C_ADDR, FG_ADDR_QH, &max17261_regs[0x00], 2);
	
	if (result == E_NO_ERROR) {
#if defined(ShowPrintFOutput)
		printf("QH register reset successful\r\n");
#endif
		return E_SUCCESS;
	} else {
#if defined(ShowPrintFOutput)
		printf("QH register reset failed\r\n");
#endif
		return E_FAIL;
	}
}

double max17261_read_tte(void)
{
	// This function gets the "Time to Empty" (TTE) value. TTE is in memory location 0x11.
	// The LSB of the TTE register is 5.625 seconds.
	uint16_t tempdata = 0;

	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_TTE, &max17261_regs[0x00], 2);
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];

	double tte_hours = (double)tempdata * TTE_LSB;

#if defined(ShowPrintFOutput)
	printf("TTE raw register = 0x%04X (%d)\r\n", tempdata, tempdata);
	printf("TTE_LSB = %f\r\n", TTE_LSB);
	printf("TTE hours = %f\r\n", tte_hours);
	printf("TTE days = %f\r\n", tte_hours / 24.0);
#endif

	return tte_hours;
}

double max17261_calculate_tte_manual(void)
{
	// Manual TTE calculation using RepCap and AvgCurrent for large batteries
	// where the fuel gauge TTE register saturates at 4.27 days
	
	uint16_t repcap_raw = 0;
	uint16_t avgcurrent_raw = 0;
	int16_t avgcurrent_signed = 0;
	
	// Read remaining capacity
	max17261_read_reg(MAX17261_I2C_ADDR, RepCap_addr, &max17261_regs[0x00], 2);
	repcap_raw = (max17261_regs[1] << 8) + max17261_regs[0];
	
	// Read average current
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_AVG_CURRENT, &max17261_regs[0x00], 2);
	avgcurrent_raw = (max17261_regs[1] << 8) + max17261_regs[0];
	avgcurrent_signed = (int16_t)avgcurrent_raw;
	
	// Convert to actual values
	double remaining_capacity_mah = (double)repcap_raw * CAPACITY_LSB; // 2.5mAh per LSB with 2mΩ sense resistor
	double avg_current_ma = (double)avgcurrent_signed * CURRENT_REG_RES / 1000.0;
	
	double tte_hours = 0.0;
	
	if (avg_current_ma < 0) // Only valid when discharging
	{
		tte_hours = remaining_capacity_mah / (-avg_current_ma);
	}
	
#if defined(ShowPrintFOutput)
	printf("Manual TTE calculation:\r\n");
	printf("  RepCap raw = 0x%04X (%d)\r\n", repcap_raw, repcap_raw);
	printf("  Remaining capacity = %.1f mAh\r\n", remaining_capacity_mah);
	printf("  Avg current = %.2f mA\r\n", avg_current_ma);
	printf("  Manual TTE = %.1f hours (%.1f days)\r\n", tte_hours, tte_hours / 24.0);
#endif
	
	return tte_hours;
}

void max17261_read_capacity_debug(void)
{
	uint16_t repcap, fullcaprep, fullcap, designcap, repsoc;
	
	// Read various capacity registers
	max17261_read_reg(MAX17261_I2C_ADDR, RepCap_addr, &max17261_regs[0x00], 2);
	repcap = (max17261_regs[1] << 8) + max17261_regs[0];
	
	max17261_read_reg(MAX17261_I2C_ADDR, FullCapRep_addr, &max17261_regs[0x00], 2);
	fullcaprep = (max17261_regs[1] << 8) + max17261_regs[0];
	
	max17261_read_reg(MAX17261_I2C_ADDR, FullCap_addr, &max17261_regs[0x00], 2);
	fullcap = (max17261_regs[1] << 8) + max17261_regs[0];
	
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_DESIGNCAP, &max17261_regs[0x00], 2);
	designcap = (max17261_regs[1] << 8) + max17261_regs[0];
	
	repsoc = max17261_read_repsoc();
	
#if defined(ShowPrintFOutput)
	printf("=== Capacity Debug Info ===\r\n");
	printf("DesignCap: 0x%04X (%d) = %.1f mAh\r\n", designcap, designcap, designcap * CAPACITY_LSB);
	printf("FullCapRep: 0x%04X (%d) = %.1f mAh\r\n", fullcaprep, fullcaprep, fullcaprep * CAPACITY_LSB);
	printf("FullCap: 0x%04X (%d) = %.1f mAh\r\n", fullcap, fullcap, fullcap * CAPACITY_LSB);
	printf("RepCap: 0x%04X (%d) = %.1f mAh\r\n", repcap, repcap, repcap * CAPACITY_LSB);
	printf("RepSOC: %d%%\r\n", repsoc);
	printf("Calculated SOC: %.1f%% (RepCap/FullCapRep)\r\n", (float)repcap * 100.0 / fullcaprep);
	printf("===========================\r\n");
#endif
}

fuel_gauge_data_t Fuel_gauge_data_collect(const char *label)
{
	// Local variables
	uint16_t tempdata = 0;
	int16_t signed_tempdata = 0;
	fuel_gauge_data_t data = {0}; // Initialize structure to zero

#if defined(ShowPrintFOutput)
	printf("=== Fuel Gauge Data (%s) ===\r\n", label);
#endif

	// Read VCell (instantaneous cell voltage)
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_VCELL, &max17261_regs[0x00], 2);
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	data.vcell_raw = tempdata;
	data.vcell_voltage = (double)tempdata * (double)VCELL_LSB;
	data.pack_voltage = data.vcell_voltage * 2.0 + 0.4; // 2S pack voltage + 0.4V drop due to protection circuit
#if defined(ShowPrintFOutput)
	printf("VCell: %.3f V, Pack: %.3f V (raw: 0x%04X = %d)\r\n", data.vcell_voltage, data.pack_voltage, tempdata, tempdata);
#endif

	// Read AvgVCell (average cell voltage)
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_AVGVCELL, &max17261_regs[0x00], 2);
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	data.avg_vcell_raw = tempdata;
	data.avg_vcell_voltage = (double)tempdata * (double)VCELL_LSB;
	data.avg_pack_voltage = data.avg_vcell_voltage * 2.0 + 0.4; // 2S pack voltage
#if defined(ShowPrintFOutput)
	printf("Avg VCell: %.3f V, Avg Pack: %.3f V (raw: 0x%04X = %d)\r\n", data.avg_vcell_voltage, data.avg_pack_voltage, tempdata, tempdata);
#endif

	// Read Current (instantaneous current in mA)
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_CURRENT, &max17261_regs[0x00], 2);
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	signed_tempdata = (int16_t)tempdata; // Current is signed
	data.current_raw = tempdata;
	data.current_ma = -1* (double)signed_tempdata * (double)CURRENT_REG_RES / 1000;
#if defined(ShowPrintFOutput)
	printf("Current: %.2f mA\r\n", data.current_ma);
#endif

	// Read AvgCurrent (average current in mA)
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_AVG_CURRENT, &max17261_regs[0x00], 2);
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	signed_tempdata = (int16_t)tempdata; // Current is signed
	data.avg_current_raw = tempdata;
	data.avg_current_ma = -1 * (double)signed_tempdata * (double)CURRENT_REG_RES / 1000;
#if defined(ShowPrintFOutput)
	printf("Avg Current: %.2f mA\r\n", data.avg_current_ma);
#endif

	//Calculate power (instantaneous power in mW) - using pack voltage for accurate power calculation
	data.power_mw = data.pack_voltage * data.current_ma;
#if defined(ShowPrintFOutput)
	printf("Power: %.2f mW (pack voltage based)\r\n", data.power_mw);
#endif

	//Calculate average power (average power in mW) - using pack voltage for accurate power calculation
	data.avg_power_mw = data.avg_pack_voltage * data.avg_current_ma;
#if defined(ShowPrintFOutput)
	printf("Avg Power: %.2f mW (pack voltage based)\r\n", data.avg_power_mw);
#endif


	// Read Temperature (instantaneous temperature in °C)
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_TEMP, &max17261_regs[0x00], 2);
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	signed_tempdata = (int16_t)tempdata; // Temperature is signed
	data.temperature_raw = tempdata;
	data.temperature_c = (double)signed_tempdata * (double)TEMP_LSB;
#if defined(ShowPrintFOutput)
	printf("Temperature: %.2f °C\r\n", data.temperature_c);
#endif

	// Read AvgTA (average temperature in °C)
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_AVGTEMP, &max17261_regs[0x00], 2);
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	signed_tempdata = (int16_t)tempdata; // Temperature is signed
	data.avg_temperature_raw = tempdata;
	data.avg_temperature_c = (double)signed_tempdata * (double)TEMP_LSB;
#if defined(ShowPrintFOutput)
	printf("Avg Temperature: %.2f °C\r\n", data.avg_temperature_c);
#endif

	// Read QH (coulomb counter)
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_QH, &max17261_regs[0x00], 2);
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	data.qh_raw = tempdata;
	data.qh_mah = (double)tempdata * 1.5625; // 1.5625 is already double
#if defined(ShowPrintFOutput)
	printf("QH (coulomb count): %.1f mAh\r\n", data.qh_mah);
	printf("===============================\r\n");
#endif

	return data;
}

void max17261_read_filtercfg_debug(void)
{
	uint16_t filtercfg_reg = 0;
	
	// Read FilterCfg register
	max17261_read_reg(MAX17261_I2C_ADDR, FG_ADDR_FILTERCFG, &max17261_regs[0x00], 2);
	filtercfg_reg = (max17261_regs[1] << 8) + max17261_regs[0];
	
	// Extract individual fields
	uint8_t curr_field = filtercfg_reg & 0x0F;        // Bits 3:0
	uint8_t volt_field = (filtercfg_reg >> 4) & 0x0F; // Bits 7:4
	uint8_t mix_field = (filtercfg_reg >> 8) & 0x0F;  // Bits 11:8
	uint8_t temp_field = (filtercfg_reg >> 12) & 0x0F; // Bits 15:12
	
	// Calculate time constants using formulas from user's guide
	double avgcurrent_time_s = 45.0 * pow(2.0, (double)((int)curr_field - 7));
	double avgvcell_time_s = 45.0 * pow(2.0, (double)((int)volt_field - 2));
	double mixing_time_s = 45.0 * pow(2.0, (double)((int)mix_field - 3));
	double avgta_time_s = 45.0 * pow(2.0, (double)((int)temp_field));
	
#if defined(ShowPrintFOutput)
	printf("=== FilterCfg Debug Info ===\r\n");
	printf("FilterCfg Register: 0x%04X\r\n", filtercfg_reg);
	printf("CURR field: %d -> AvgCurrent time constant: %.1f s (%.1f min)\r\n", 
	       curr_field, avgcurrent_time_s, avgcurrent_time_s / 60.0);
	printf("VOLT field: %d -> AvgVCell time constant: %.1f s (%.1f min)\r\n", 
	       volt_field, avgvcell_time_s, avgvcell_time_s / 60.0);
	printf("MIX field: %d -> Mixing time constant: %.1f s (%.1f hours)\r\n", 
	       mix_field, mixing_time_s, mixing_time_s / 3600.0);
	printf("TEMP field: %d -> AvgTA time constant: %.1f s (%.1f min)\r\n", 
	       temp_field, avgta_time_s, avgta_time_s / 60.0);
	printf("============================\r\n");
#endif
}

uint16_t max17261_read_device_id(void)
{
	uint16_t device_id = 0;
	
	// Read DevName register (0x21)
	if (max17261_read_reg(MAX17261_I2C_ADDR, DevName_addr, &max17261_regs[0x00], 2) == E_NO_ERROR) {
		device_id = (max17261_regs[1] << 8) + max17261_regs[0];
#if defined(ShowPrintFOutput)
		printf("MAX17261 Device ID: 0x%04X\r\n", device_id);
#endif
	} else {
#if defined(ShowPrintFOutput)
		printf("ERROR: Cannot read MAX17261 Device ID\r\n");
#endif
	}
	
	return device_id;
}

void max17261_read_current_calibration(void)
{
	uint16_t cgain_reg = 0;
	uint16_t coff_reg = 0;
	
	// Read CGain register (0x2E)
	if (max17261_read_reg(MAX17261_I2C_ADDR, CGain_addr, &max17261_regs[0x00], 2) == E_NO_ERROR) {
		cgain_reg = (max17261_regs[1] << 8) + max17261_regs[0];
	}
	
	// Read COff register (0x2F)
	if (max17261_read_reg(MAX17261_I2C_ADDR, COff_addr, &max17261_regs[0x00], 2) == E_NO_ERROR) {
		coff_reg = (max17261_regs[1] << 8) + max17261_regs[0];
	}
	
	// Calculate actual gain factor
	double gain_factor = (double)cgain_reg / 0x0400;
	
#if defined(ShowPrintFOutput)
	printf("=== Current Calibration Debug ===\r\n");
	printf("CGain Register: 0x%04X (gain factor: %.4f)\r\n", cgain_reg, gain_factor);
	printf("COff Register: 0x%04X (offset: %d µV)\r\n", coff_reg, (int16_t)coff_reg);
	printf("Default CGain: 0x0400 (no adjustment)\r\n");
	printf("Current formula: ADC_Reading × %.4f + %d µV\r\n", gain_factor, (int16_t)coff_reg);
	printf("================================\r\n");
#endif
}

int max17261_set_current_gain(uint16_t gain_value)
{
	max17261_regs[0] = gain_value & 0x00FF;
	max17261_regs[1] = gain_value >> 8;
	
#if defined(ShowPrintFOutput)
	printf("Setting CGain to 0x%04X (gain factor: %.4f)\r\n", gain_value, (double)gain_value / 0x0400);
#endif
	
	int result = max17261_write_reg(MAX17261_I2C_ADDR, CGain_addr, &max17261_regs[0x00], 2);
	
	if (result == E_NO_ERROR) {
#if defined(ShowPrintFOutput)
		printf("CGain register updated successfully\r\n");
#endif
		return E_SUCCESS;
	} else {
#if defined(ShowPrintFOutput)
		printf("CGain register update failed\r\n");
#endif
		return E_FAIL;
	}
}

