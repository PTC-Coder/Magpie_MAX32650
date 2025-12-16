/**
 * @file    Ext_Flash.c  (formerly w25.c)
 * @brief   Board layer Driver for the Macronix MX25L Serial Multi-I/O Flash Memory.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The MX25L5124tGZ2I-08G 64M-bit memory array is organized into 2048 sectors 
 * of 4KB each, 128 blocks of 64KB each. The entire page (256 bytes) can be 
 * programmed at one time. Sectors can be erased individually (4KB) or as 
 * blocks (64KB), or the entire chip can be bulk erased.
 ******************************************************************************/

/* **** Includes **** */
#include <stdint.h>
#include <stddef.h>
#include "Ext_Flash.h"
#include "mxc_delay.h"

/**
 * @ingroup w25
 * @{
 */

/* **** Definitions **** */
#define MX25L_ID_LEN (3)

#define MX25L_WIP_MASK 0x01 /**< Status Reg: Work In Progress          */
#define MX25L_WEL_MASK 0x02 /**< Status Reg: Write Enable Latch mask   */
#define MX25L_QE_MASK 0x40 /**< Status Reg: Quad-SPI enable mask      */

#define MX25L_TB_POS 5
#define MX25L_TB_MASK (1 << MX25L_TB_POS) /**< Top/Bottom Select mask         */
#define MX25L_BP_POS 2
#define MX25L_BP_MASK (0x7 << MX25L_BP_POS) /**< Block Protect mask             */
#define MX25L_SR_FP_MASK \
    (MX25L_TB_MASK | MX25L_BP_MASK) /**< Mask of all flash block protect bits in status register */

#define MX25L_DEVICE_SIZE 0x800000  //8 M-bytes (64 Megabit)
#define MX25L_BLOCK_SIZE 0x10000   //64 KB block size
#define MX25L_SECTOR_SIZE 0x1000   //4 KB sector size  
#define MX25L_PAGE_SIZE 256        //256 bytes per page
#define MX25L_TOTAL_SECTORS 2048   //2048 sectors of 4KB each
#define MX25L_TOTAL_BLOCKS 128     //128 blocks of 64KB each

#define MX25L_CMD_RST_EN 0x66 /**< Reset Enable                   */
#define MX25L_CMD_RST_MEM 0x99 /**< Reset Memory                   */
#define MX25L_CMD_ID 0x9F /**< Read ID                        */
#define MX25L_CMD_WRITE_EN 0x06 /**< Write Enable                   */
#define MX25L_CMD_WRITE_DIS 0x04 /**< Write Disable                  */

#define MX25L_CMD_READ_SR 0x05 /**< Read Status Register           */
#define MX25L_CMD_WRITE_SR 0x01 /**< Write Status Register          */

#define MX25L_CMD_READ 0x03 /**< Read Data                      */
#define MX25L_CMD_FAST_READ 0x0B /**< Fast Read                      */
#define MX25L_CMD_DUAL_READ 0x3B /**< Dual Output Fast Read         */
#define MX25L_CMD_QUAD_READ 0x6B /**< Quad Output Fast Read         */

#define MX25L_CMD_PPROG 0x02 /**< Page Program                   */
#define MX25L_CMD_QUAD_PROG 0x38 /**< Quad Input Page Program       */

#define MX25L_CMD_4K_ERASE 0x20 /**< 4KB Sector Erase              */
#define MX25L_CMD_64K_ERASE 0xD8 /**< 64KB Block Erase              */
#define MX25L_CMD_BULK_ERASE 0xC7 /**< Chip Erase                    */


/* **** Globals **** */

static Ext_Flash_Config_t g_cfg;
static uint8_t g_is_configured = 0;

/* **** Static Functions **** */

/* ************************************************************************* */
static Ext_Flash_Error_t flash_busy()
{
    uint8_t buf;

    Ext_Flash_Read_SR(&buf, Ext_Flash_StatusReg_1);

    if (buf & MX25L_WIP_MASK) {
        return EF_E_BUSY;
    } else {
        return EF_E_SUCCESS;
    }
}

/* ************************************************************************* */
static Ext_Flash_Error_t write_enable()
{
    Ext_Flash_Error_t err = EF_E_SUCCESS;
    uint8_t cmd = MX25L_CMD_WRITE_EN;
    uint8_t buf = 0;

    // Send the command
    if ((err = g_cfg.write(&cmd, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    if ((err = Ext_Flash_Read_SR(&buf, Ext_Flash_StatusReg_1)) != EF_E_SUCCESS) {
        return err;
    }

    if (buf & MX25L_WEL_MASK) {
        return EF_E_SUCCESS;
    }

    return EF_E_BAD_STATE;
}

/* ************************************************************************* */
static Ext_Flash_Error_t inline read_reg(uint8_t cmd, uint8_t *buf)
{
    Ext_Flash_Error_t err = EF_E_SUCCESS;

    if (!buf) {
        return EF_E_BAD_PARAM;
    }

    // Send the command
    if ((err = g_cfg.write(&cmd, 1, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    // Read the data
    if ((err = g_cfg.read(buf, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
static Ext_Flash_Error_t inline read_status_reg(uint8_t cmd, uint8_t *buf)
{
    Ext_Flash_Error_t err = EF_E_SUCCESS;

    if (!buf) {
        return EF_E_BAD_PARAM;
    }

    // Send the command
    if ((err = g_cfg.write(&cmd, 1, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    // Read the data
    if ((err = g_cfg.read(buf, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
static Ext_Flash_Error_t inline write_reg(uint8_t *buf, unsigned len)
{
    Ext_Flash_Error_t err = EF_E_SUCCESS;

    if (!buf || (len == 0)) {
        return EF_E_BAD_PARAM;
    }

    if ((err = write_enable()) != EF_E_SUCCESS) {
        return err;
    }

    // Send the command and data
    if ((err = g_cfg.write(buf, len, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    return EF_E_SUCCESS;
}

/* **** Functions **** */

Ext_Flash_Error_t Ext_Flash_Configure(Ext_Flash_Config_t *cfg)
{
    Ext_Flash_Error_t err = EF_E_SUCCESS;

    if (cfg == NULL) {
        return EF_E_BAD_PARAM;
    }

    g_cfg = *cfg;
    g_is_configured = 1;

    return err;
}

/* ************************************************************************* */

Ext_Flash_Error_t Ext_Flash_Init(void)
{
    if (!g_is_configured) {
        return EF_E_BAD_STATE;
    }

    return g_cfg.init();
}

/* ************************************************************************* */
Ext_Flash_Error_t Ext_Flash_Reset(void)
{
    Ext_Flash_Error_t err = EF_E_SUCCESS;
    int busy_count = 0;
    uint8_t cmd = MX25L_CMD_RST_EN;

    // Send the Enable Reset command
    if ((err = g_cfg.write(&cmd, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    // Send reset command
    cmd = MX25L_CMD_RST_MEM;
    if ((err = g_cfg.write(&cmd, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {
        busy_count++;
        if (busy_count > 20000) {
            return EF_E_TIME_OUT;
        }
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
uint32_t Ext_Flash_ID(void)
{
    Ext_Flash_Error_t err = EF_E_SUCCESS;
    uint8_t cmd = MX25L_CMD_ID;
    uint8_t id[MX25L_ID_LEN] = { 0 };

    // Send the command
    if ((err = g_cfg.write(&cmd, 1, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    // Read the data
    if ((err = g_cfg.read(id, MX25L_ID_LEN, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    /* id[0] is manufacturer ID, id[1] and id[2] are device ID */

    return ((uint32_t)(id[2] | (id[1] << 8) | (id[0] << 16)));
}

/* ************************************************************************* */
Ext_Flash_Error_t Ext_Flash_Quad(int enable)
{
    int err = EF_E_SUCCESS;
    uint8_t pre_buf = 0;
    uint8_t post_buf = 0;

    // Enable QSPI mode
    if ((err = Ext_Flash_Read_SR(&pre_buf, Ext_Flash_StatusReg_1)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {}

    if (enable) {
        if (pre_buf & MX25L_QE_MASK) {
            return EF_E_SUCCESS;
        }
        pre_buf |= MX25L_QE_MASK;
    } else {
        if (!(pre_buf & MX25L_QE_MASK)) {
            return EF_E_SUCCESS;
        }
        pre_buf &= ~MX25L_QE_MASK;
    }

    if (write_enable() != EF_E_SUCCESS) {
        return EF_E_BAD_STATE;
    }

    if ((err = Ext_Flash_Write_SR(pre_buf, Ext_Flash_StatusReg_1)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {}

    if ((err = Ext_Flash_Read_SR(&post_buf, Ext_Flash_StatusReg_1)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {}

    if (enable) {
        if (!(post_buf & MX25L_QE_MASK)) {
            return EF_E_ERROR;
        }
    } else {
        if (post_buf & MX25L_QE_MASK) {
            return EF_E_ERROR;
        }
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
// For NOR flash, DataRead is not needed as there's no page buffer
// This function is kept for compatibility but does nothing
Ext_Flash_Error_t Ext_Flash_DataRead(uint32_t address)
{
    // NOR flash doesn't require separate data read command
    // Data is read directly from memory array
    return EF_E_SUCCESS;
}

/* ************************************************************************* */
// Read data directly from NOR flash memory
Ext_Flash_Error_t Ext_Flash_Read(uint32_t address, uint8_t *rx_buf, uint32_t rx_len, Ext_Flash_DataLine_t d_line)
{
    int err = EF_E_SUCCESS;
    uint8_t cmd[4];
    uint8_t cmd_len;

    if (!rx_buf) {
        return EF_E_BAD_PARAM;
    }

    if (flash_busy()) {
        return EF_E_BUSY;
    }

    // Select appropriate read command based on data line configuration
    switch (d_line) {
        case Ext_Flash_DataLine_Quad:
            cmd[0] = MX25L_CMD_QUAD_READ;
            cmd_len = 4; // Command + 3 address bytes
            break;
        case Ext_Flash_DataLine_Dual:
            cmd[0] = MX25L_CMD_DUAL_READ;
            cmd_len = 4; // Command + 3 address bytes
            break;
        default:
            cmd[0] = MX25L_CMD_FAST_READ;
            cmd_len = 4; // Command + 3 address bytes
            break;
    }

    // Set 24-bit address
    cmd[1] = (uint8_t)((address & 0xFF0000) >> 16);
    cmd[2] = (uint8_t)((address & 0xFF00) >> 8);
    cmd[3] = (uint8_t)(address & 0xFF);

    // Send command & address
    if ((err = g_cfg.write(&cmd[0], cmd_len, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    // Send dummy bytes for fast read commands
    if (cmd[0] != MX25L_CMD_READ) {
        uint8_t dummy = 0x00;
        if ((err = g_cfg.write(&dummy, 1, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
            return err;
        }
    }

    // Read the data
    if ((err = g_cfg.read(rx_buf, rx_len, 1, d_line)) != EF_E_SUCCESS) {
        return err;
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
Ext_Flash_Error_t Ext_Flash_Program_Page(uint32_t address, uint8_t *tx_buf, uint32_t tx_len,
                           Ext_Flash_DataLine_t d_line)
{
    int err = EF_E_SUCCESS;
    volatile int timeout = 0;
    uint8_t cmd[4];
    uint32_t len = 0;
    uint32_t next_page = 0;
    uint8_t *pWrite_Data = NULL;

    if (tx_buf == NULL) {
        return EF_E_BAD_PARAM;
    }

    // if flash address is out-of-range
    if ((address >= MX25L_DEVICE_SIZE) || ((address + tx_len) > MX25L_DEVICE_SIZE)) {
        return EF_E_BAD_PARAM; // attempt to write outside flash memory size
    }

    pWrite_Data = tx_buf; // note our starting source data address

    if (flash_busy()) {
        return EF_E_BUSY;
    }

    // Now write out as many pages of flash as required to fulfil the request
    while (tx_len > 0) {
        timeout = 0;
        while (write_enable()) {
            timeout++;
            if (timeout > 100) {
                return EF_E_TIME_OUT;
            }
        }

        // Select appropriate program command based on data line configuration
        if (d_line == Ext_Flash_DataLine_Quad) {
            cmd[0] = MX25L_CMD_QUAD_PROG;
        } else {
            cmd[0] = MX25L_CMD_PPROG;
        }

        // Set 24-bit address
        cmd[1] = (uint8_t)((address & 0xFF0000) >> 16);
        cmd[2] = (uint8_t)((address & 0xFF00) >> 8);
        cmd[3] = (uint8_t)(address & 0xFF);

        // Send the command and address
        if ((err = g_cfg.write(&cmd[0], 4, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
            return err;
        }

        // calculate the next flash page boundary from our starting address
        next_page = ((address & ~(MX25L_PAGE_SIZE - 1)) + MX25L_PAGE_SIZE);

        // Now check for how much data to write on this page of flash
        if ((address + tx_len) < next_page) {
            len = tx_len; // no page boundary is crossed
        } else {
            len = next_page - address; // adjust length of this write to stay within the current page
        }

        // Write the data
        if ((err = g_cfg.write(pWrite_Data, len, 1, d_line)) != EF_E_SUCCESS) {
            return err;
        }

        if (tx_len >= len) {
            tx_len -= len; // what's left to write
        }

        // if there is more to write
        if (tx_len > 0) {
            address += len; // calculate new starting flash_address
            pWrite_Data += len; // and source data address
        }

        timeout = 0;
        while (flash_busy()) {
            timeout++;
            if (timeout > 10000) {
                return EF_E_TIME_OUT;
            }
        }
    }
    return EF_E_SUCCESS;
}

/* ************************************************************************* */
Ext_Flash_Error_t Ext_Flash_Unprotect_StatusRegister(void)
{
    uint8_t cmd[2] = {MX25L_CMD_WRITE_SR, 0x00};

    return write_reg(cmd, sizeof(cmd));
}

/* ************************************************************************* */
Ext_Flash_Error_t Ext_Flash_Bulk_Erase(void)
{
    int err = EF_E_SUCCESS;
    uint8_t cmd = MX25L_CMD_BULK_ERASE;
    volatile int timeout = 0;

    if (flash_busy()) {
        return EF_E_BUSY;
    }

    if (write_enable() != 0) {
        return EF_E_BAD_STATE;
    }

    // Send bulk erase command
    if ((err = g_cfg.write(&cmd, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    // Wait for erase to complete (can take several seconds)
    while (flash_busy()) {
        timeout++;
        if (timeout > 100000000) {
            return EF_E_TIME_OUT;
        }
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
Ext_Flash_Error_t Ext_Flash_Erase(uint32_t address, Ext_Flash_Erase_t size)
{
    Ext_Flash_Error_t err = EF_E_SUCCESS;
    uint8_t cmd[4] = { 0 };
    volatile int timeout = 0;

    if (flash_busy()) {
        return EF_E_BUSY;
    }

    if (write_enable() != 0) {
        return EF_E_BAD_STATE;
    }

    switch (size) {
    case Ext_Flash_Erase_4K:
    default:
        cmd[0] = MX25L_CMD_4K_ERASE;
        break;
    case Ext_Flash_Erase_64K:
        cmd[0] = MX25L_CMD_64K_ERASE;
        break;
    case Ext_Flash_Erase_128K:
        // MX25L doesn't have 128K erase, use 64K instead
        cmd[0] = MX25L_CMD_64K_ERASE;
        break;
    }

    // Set 24-bit address
    cmd[1] = (uint8_t)((address & 0xFF0000) >> 16);
    cmd[2] = (uint8_t)((address & 0xFF00) >> 8);
    cmd[3] = (uint8_t)(address & 0xFF);

    // Send the command and the address
    if ((err = g_cfg.write(&cmd[0], 4, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {
        timeout++;
        if (timeout > 1000000000) {
            return EF_E_TIME_OUT;
        }
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
Ext_Flash_Error_t Ext_Flash_Read_SR(uint8_t *buf, Ext_Flash_StatusReg_t reg_num)
{
    uint8_t cmd = MX25L_CMD_READ_SR;

    if (buf == NULL) {
        return EF_E_BAD_PARAM;
    }

    // MX25L only has one status register
    switch (reg_num) {
    case Ext_Flash_StatusReg_1:
    case Ext_Flash_StatusReg_2:
    case Ext_Flash_StatusReg_3:
        // All map to the same status register for MX25L
        break;
    default:
        return EF_E_BAD_PARAM;
    }

    return read_status_reg(cmd, buf);
}

/* ************************************************************************* */
//Basically just read the status register
Ext_Flash_Error_t Ext_Flash_SyncFlash(void)
{
    Ext_Flash_Error_t err = EF_E_SUCCESS;
    uint8_t sr;

    if ((err = Ext_Flash_Read_SR(&sr, Ext_Flash_StatusReg_1)) !=
        EF_E_SUCCESS) { // Get current value of status register
        return err;
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
Ext_Flash_Error_t Ext_Flash_Write_SR(uint8_t value, Ext_Flash_StatusReg_t reg_num)
{
    uint8_t cmd[2] = {MX25L_CMD_WRITE_SR, value};

    // MX25L only has one status register
    switch (reg_num) {
    case Ext_Flash_StatusReg_1:
    case Ext_Flash_StatusReg_2:
    case Ext_Flash_StatusReg_3:
        // All map to the same status register for MX25L
        break;
    default:
        return EF_E_BAD_PARAM;
    }

    return write_reg(cmd, sizeof(cmd));
}

/* ************************************************************************* */
Ext_Flash_Error_t Ext_Flash_Prog_Execute(uint32_t address)
{
    // NOR flash doesn't require separate program execute command
    // Programming happens immediately during page program
    return EF_E_SUCCESS;
}

/* ************************************************************************* */
Ext_Flash_Error_t Ext_Flash_Block_WP(uint32_t addr, uint32_t begin)
{
    Ext_Flash_Error_t err = EF_E_SUCCESS;
    uint8_t sr, bp;

    if (addr >= MX25L_DEVICE_SIZE) { // Check address valid
        return EF_E_ERROR;
    }

    // Simple block protection for MX25L
    // Calculate block protect bits based on address
    if (addr == 0) {
        bp = 0; // No protection
    } else if (addr < MX25L_DEVICE_SIZE / 32) {
        bp = 1; // Protect upper 1/32
    } else if (addr < MX25L_DEVICE_SIZE / 16) {
        bp = 2; // Protect upper 1/16
    } else if (addr < MX25L_DEVICE_SIZE / 8) {
        bp = 3; // Protect upper 1/8
    } else if (addr < MX25L_DEVICE_SIZE / 4) {
        bp = 4; // Protect upper 1/4
    } else if (addr < MX25L_DEVICE_SIZE / 2) {
        bp = 5; // Protect upper 1/2
    } else {
        bp = 7; // Protect all
    }

    if ((err = Ext_Flash_Read_SR(&sr, Ext_Flash_StatusReg_1)) !=
        EF_E_SUCCESS) { // Read current value of status register
        return err;
    }

    sr = (sr & ~MX25L_SR_FP_MASK) | (!!begin << MX25L_TB_POS) |
          (bp << MX25L_BP_POS); // Modify flash protect bits

    if ((err = Ext_Flash_Write_SR(sr, Ext_Flash_StatusReg_1)) !=
        EF_E_SUCCESS) { // Write flash protect settings back to MX25L
        return err;
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
Ext_Flash_Unblk_t Ext_Flash_GetAvailableFlash(void)
{
    Ext_Flash_Error_t err = EF_E_SUCCESS;
    uint8_t sr;
    uint32_t protected_size;
    Ext_Flash_Unblk_t free_flash;

    if ((err = Ext_Flash_Read_SR(&sr, Ext_Flash_StatusReg_1)) !=
        EF_E_SUCCESS) { // Get current value of status register
        free_flash.start_addr = err;
        free_flash.end_addr = err;
        return (free_flash);
    }

    // Use TB and BP bits to find start and end addresses
    uint8_t bp = (sr & MX25L_BP_MASK) >> MX25L_BP_POS;
    uint8_t tb = (sr & MX25L_TB_MASK) >> MX25L_TB_POS;

    if (bp == 0) { // No protection
        free_flash.start_addr = 0;
        free_flash.end_addr = MX25L_DEVICE_SIZE;
    } else if (bp == 7) { // All protected
        free_flash.start_addr = 0;
        free_flash.end_addr = 0;
    } else { // Partial protection
        // Calculate protected size based on BP bits
        switch (bp) {
            case 1: protected_size = MX25L_DEVICE_SIZE / 32; break;
            case 2: protected_size = MX25L_DEVICE_SIZE / 16; break;
            case 3: protected_size = MX25L_DEVICE_SIZE / 8; break;
            case 4: protected_size = MX25L_DEVICE_SIZE / 4; break;
            case 5: protected_size = MX25L_DEVICE_SIZE / 2; break;
            case 6: protected_size = MX25L_DEVICE_SIZE * 3 / 4; break;
            default: protected_size = 0; break;
        }

        if (tb) { // Top protect
            free_flash.start_addr = 0;
            free_flash.end_addr = MX25L_DEVICE_SIZE - protected_size;
        } else { // Bottom protect
            free_flash.start_addr = protected_size;
            free_flash.end_addr = MX25L_DEVICE_SIZE;
        }
    }

    return free_flash;
}
/**@} end of ingroup w25 */
