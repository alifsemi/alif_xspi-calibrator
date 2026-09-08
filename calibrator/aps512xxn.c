/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include <stdio.h>

#include "aps512xxn.h"
#include "sys_utils.h"
#include "sys_ctrl_aes.h" 

#ifndef DEBUG_PRINTS
#define DEBUG_PRINTS 0
#endif

/* ==========================================================================
 * APS512XXN primitives
 * ========================================================================== */

static void aps512xxn_global_reset(ospi_cfg_t *ospi_ctx)
{
    volatile uint32_t t;

    ospi_disable(ospi_ctx->regs);
    ospi_ctx->regs->OSPI_SER = 0;

    t = (0U << SPI_CTRLR0_SPI_HYPERBUS_EN)
        | (SPI_OCTAL << SPI_CTRLR0_SPI_FRF)
        | (SPI_TMOD_TO << SPI_CTRLR0_TMOD)
        | (0xFU << SPI_CTRLR0_DFS);
    ospi_ctx->regs->OSPI_CTRLR0 = t;

    t = SPI_TRANS_TYPE_FRF_DEFINED
        | (0U << SPI_CTRLR0_ADDR_L_OFFSET)
        | (2U << SPI_CTRLR0_INST_L_OFFSET)
        | (APS512XXN_RESET_WAIT_CYCLES << SPI_CTRLR0_WAIT_CYCLES_OFFSET)
        | (1U << SPI_CTRLR0_SPI_DDR_EN_OFFSET)
        | (1U << SPI_CTRLR0_SPI_RXDS_EN_OFFSET)
        | (1U << SPI_CTRLR0_SPI_DM_EN_OFFSET)
        | (0U << SPI_CTRLR0_SPI_RXDS_SIG_EN_OFFSET);
    ospi_ctx->regs->OSPI_SPI_CTRLR0 = t;

    ospi_enable(ospi_ctx->regs);

    ospi_ctx->tx_buff[0] = APS512XXN_CMD_GLOBAL_RESET;
    ospi_ctx->regs->OSPI_DR[0] = ospi_ctx->tx_buff[0];

    ospi_ctx->regs->OSPI_SER = 1;

    while (ospi_ctx->regs->OSPI_SR & SR_BUSY)
        ;
    while ((ospi_ctx->regs->OSPI_SR & SR_TF_EMPTY) != SR_TF_EMPTY)
        ;

    ospi_ctx->regs->OSPI_SER = 0;
}

/*
 * The APS512XXN x16 device requires an address "gap" between bit 10 and
 * bit 11 
 */
static uint32_t insert_addr_gap(uint32_t addr)
{
    uint32_t lower = addr & 0x7FF;  /* bits [10:0]  */
    uint32_t upper = addr & ~0x7FF; /* bits [31:11] */

    upper <<= 1;                    /* create gap between bit10 and bit11 */

    return ((upper | lower) >> 1);
}

void aps512xxn_write_data(ospi_cfg_t *ospi_ctx, uint32_t address, uint32_t *data, uint32_t length)
{
    uint32_t l = 0;

    uint32_t trans_type = ospi_ctx->is_dual_octal ? SPI_TRANS_TYPE_FRF_DUAL_OCTAL : SPI_TRANS_TYPE_FRF_DEFINED;

    ospi_setup_write(ospi_ctx->regs, ospi_ctx->dfs, trans_type, SPI_INST_L_8_BIT, SPI_ADDR_L_32_BIT,
                     APS512XXN_WRITE_WAIT_CYCLES - 1, false);

    /* Form a linear burst mem write transaction command */
    ospi_ctx->tx_buff[0] = APS512XXN_CMD_LINEAR_BURST_WRITE;
    ospi_ctx->tx_buff[1] = ospi_ctx->is_dual_octal ? insert_addr_gap(address) : address;

    while (l < 2) {
        ospi_ctx->regs->OSPI_DR[0] = ospi_ctx->tx_buff[l];
        l++;
    }

    l = 0;
    while (l < length) {
        ospi_ctx->regs->OSPI_DR[0] = data[l];
        l++;
    }

    ospi_ctx->regs->OSPI_SER = 1;

    while (ospi_ctx->regs->OSPI_SR & SR_BUSY)
        ;
    while ((ospi_ctx->regs->OSPI_SR & SR_TF_EMPTY) != SR_TF_EMPTY)
        ;

    ospi_ctx->regs->OSPI_SER = 0;
}

int aps512xxn_read_data(ospi_cfg_t *ospi_ctx, uint32_t address, volatile uint32_t *data,
                        uint32_t length)
{
    uint32_t l = 0;
    int timeout = 16;
    uint32_t trans_type = ospi_ctx->is_dual_octal ? SPI_TRANS_TYPE_FRF_DUAL_OCTAL : SPI_TRANS_TYPE_FRF_DEFINED;

    ospi_setup_read(ospi_ctx->regs, ospi_ctx->dfs, trans_type, SPI_INST_L_8_BIT, SPI_ADDR_L_32_BIT,
                    APS512XXN_READ_WAIT_CYCLES - 1, false, length);

    /* Form a linear burst mem read transaction command */
    ospi_ctx->tx_buff[0] = APS512XXN_CMD_LINEAR_BURST_READ;
    ospi_ctx->tx_buff[1] = ospi_ctx->is_dual_octal ? insert_addr_gap(address) : address;

    while (l < 2) {
        ospi_ctx->regs->OSPI_DR[0] = ospi_ctx->tx_buff[l];
        l++;
    }

    ospi_ctx->regs->OSPI_SER = 1;

    for (int wait = 0; wait < 500; wait++) {
        if (!(ospi_ctx->regs->OSPI_SR & SR_BUSY))
            break;
        sys_busy_loop_us(1);
    }
    if (ospi_ctx->regs->OSPI_SR & SR_BUSY) {
        ospi_ctx->regs->OSPI_SER = 0;
        return -1;
    }

    l = 0;

    /* Wait until there is valid data in the FIFO */
    while (ospi_ctx->regs->OSPI_RXFLR == 0)
        ;

    do {
        if (ospi_ctx->regs->OSPI_RXFLR > 0) {
            ospi_ctx->rx_buff[l] = ospi_ctx->regs->OSPI_DR[0];
            *data++ = ospi_ctx->rx_buff[l];
            l++;
        }
        timeout--;
    } while ((ospi_ctx->regs->OSPI_RXFLR > 0) && timeout);

    ospi_ctx->regs->OSPI_SER = 0;

    return 0;
}

static void aps512xxn_write_reg(ospi_cfg_t *ospi_ctx, uint32_t reg_addr, uint8_t data)
{
    uint32_t l = 0;
    volatile uint32_t t;

    ospi_disable(ospi_ctx->regs);
    ospi_ctx->regs->OSPI_SER = 0;

    t = (0U << SPI_CTRLR0_SPI_HYPERBUS_EN)
        | (SPI_OCTAL << SPI_CTRLR0_SPI_FRF)
        | (SPI_TMOD_TO << SPI_CTRLR0_TMOD)
        | (0xFU << SPI_CTRLR0_DFS);
    ospi_ctx->regs->OSPI_CTRLR0 = t;

    t = SPI_TRANS_TYPE_FRF_DEFINED
        | (8U << SPI_CTRLR0_ADDR_L_OFFSET)
        | (2U << SPI_CTRLR0_INST_L_OFFSET)
        | (0U << SPI_CTRLR0_WAIT_CYCLES_OFFSET)
        | (1U << SPI_CTRLR0_SPI_DDR_EN_OFFSET)
        | (1U << SPI_CTRLR0_SPI_RXDS_EN_OFFSET)
        | (0U << SPI_CTRLR0_SPI_DM_EN_OFFSET)
        | (0U << SPI_CTRLR0_SPI_RXDS_SIG_EN_OFFSET);
    ospi_ctx->regs->OSPI_SPI_CTRLR0 = t;

    ospi_enable(ospi_ctx->regs);

    /* Form the command to write into the configuration register */
    ospi_ctx->tx_buff[0] = APS512XXN_CMD_MODE_REGISTER_WRITE;
    ospi_ctx->tx_buff[1] = reg_addr;
    ospi_ctx->tx_buff[2] = ((uint32_t)data << 8) | data;

    l = 0;
    while (l < 3) {
        ospi_ctx->regs->OSPI_DR[0] = ospi_ctx->tx_buff[l];
        l++;
    }

    ospi_ctx->regs->OSPI_DR[0] = data;

    ospi_ctx->regs->OSPI_SER = 1;

    while (ospi_ctx->regs->OSPI_SR & SR_BUSY)
        ;
    while ((ospi_ctx->regs->OSPI_SR & SR_TF_EMPTY) != SR_TF_EMPTY)
        ;

    ospi_ctx->regs->OSPI_SER = 0;
}

static uint32_t aps512xxn_read_reg(ospi_cfg_t *ospi_ctx, uint32_t reg_addr, uint8_t wait_cycles)
{
    uint32_t l = 0;
    uint32_t t;
    uint16_t reg_val = 0;

    ospi_disable(ospi_ctx->regs);
    ospi_ctx->regs->OSPI_SER = 0;

    t = (0U << SPI_CTRLR0_SPI_HYPERBUS_EN)
        | (SPI_OCTAL << SPI_CTRLR0_SPI_FRF)
        | (SPI_TMOD_RO << SPI_CTRLR0_TMOD)
        | (0xFU << SPI_CTRLR0_DFS);
    ospi_ctx->regs->OSPI_CTRLR0 = t;
    ospi_ctx->regs->OSPI_CTRLR1 = 0;

    t = SPI_TRANS_TYPE_FRF_DEFINED
        | (8U << SPI_CTRLR0_ADDR_L_OFFSET)
        | (2U << SPI_CTRLR0_INST_L_OFFSET)
        | ((uint32_t)(wait_cycles - 1U) << SPI_CTRLR0_WAIT_CYCLES_OFFSET)
        | (1U << SPI_CTRLR0_SPI_DDR_EN_OFFSET)
        | (1U << SPI_CTRLR0_SPI_RXDS_EN_OFFSET)
        | (0U << SPI_CTRLR0_SPI_DM_EN_OFFSET)
        | (0U << SPI_CTRLR0_SPI_RXDS_SIG_EN_OFFSET);
    ospi_ctx->regs->OSPI_SPI_CTRLR0 = t;
    ospi_ctx->regs->OSPI_RX_SAMPLE_DELAY = 0x0;

    ospi_enable(ospi_ctx->regs);

    ospi_ctx->tx_buff[0] = APS512XXN_CMD_MODE_REGISTER_READ;
    ospi_ctx->tx_buff[1] = reg_addr;

    l = 0;
    while (l < 2) {
        ospi_ctx->regs->OSPI_DR[0] = ospi_ctx->tx_buff[l];
        l++;
    }

    ospi_ctx->regs->OSPI_SER = 1;

    while ((ospi_ctx->regs->OSPI_SR & SR_TF_EMPTY) != SR_TF_EMPTY)
        ;

    for (int wait = 0; wait < 10000; wait++) {
        if (ospi_ctx->regs->OSPI_RXFLR != 0) {
            break;
        }
        sys_busy_loop_us(1);
    }

    if (ospi_ctx->regs->OSPI_RXFLR == 0) {
        ospi_ctx->regs->OSPI_SER = 0;
        return -1;
    }

    l = 0;
    while (l < 1) {
        reg_val = (uint16_t)ospi_ctx->regs->OSPI_DR[0];
        l++;
    }

    ospi_ctx->regs->OSPI_SER = 0;
    return (uint8_t)(reg_val >> 8);
}

int aps512xxn_setup(ospi_cfg_t *ospi_ctx)
{
    uint8_t reg_value;

    /* APS512XXN global reset */
    aps512xxn_global_reset(ospi_ctx);

    reg_value = (uint8_t)aps512xxn_read_reg(ospi_ctx, APS512XXN_MODE_REG2_ADDR,
                                            APS512XXN_INIT_REG_READ_WAIT_CYCLES);
    if (reg_value != APS512XXN_ID) {
        return -1;
    }

    aps512xxn_write_reg(ospi_ctx, APS512XXN_MODE_REG0_ADDR, 0x30); /* fixed lat, read_wait_cycle=7 */
    aps512xxn_write_reg(ospi_ctx, APS512XXN_MODE_REG4_ADDR, 0x20); /* write_wait_cycle=7            */

    reg_value = (uint8_t)(((ospi_ctx->is_dual_octal ? 0 : 1) << APS512XXN_MODE_REG8_BURST_LEN) |
                          (0 << APS512XXN_MODE_REG8_BURST_TYPE) |
                          (0 << APS512XXN_MODE_REG8_RBX_READ_EN) |
                          ((ospi_ctx->is_dual_octal ? 1 : 0) << APS512XXN_MODE_REG8_TRANSFER_MODE));
    aps512xxn_write_reg(ospi_ctx, APS512XXN_MODE_REG8_ADDR, reg_value);


    /* Configure the XIP instruction opcodes for the APS512XXN. Instruction */
    ospi_disable(ospi_ctx->regs);
    ospi_ctx->regs->OSPI_XIP_INCR_INST =
        (APS512XXN_CMD_LINEAR_BURST_READ << 8) | APS512XXN_CMD_LINEAR_BURST_READ;
    ospi_ctx->regs->OSPI_XIP_WRAP_INST =
        (APS512XXN_CMD_SYNC_READ << 8) | APS512XXN_CMD_SYNC_READ;
    ospi_ctx->regs->OSPI_XIP_WRITE_INCR_INST =
        (APS512XXN_CMD_LINEAR_BURST_WRITE << 8) | APS512XXN_CMD_LINEAR_BURST_WRITE;
    ospi_ctx->regs->OSPI_XIP_WRITE_WRAP_INST =
        (APS512XXN_CMD_SYNC_WRITE << 8) | APS512XXN_CMD_SYNC_WRITE;
    ospi_ctx->regs->OSPI_XIP_CNT_TIME_OUT = 255;


    /* Configure the AES address-control shim for the APS512XXN x16 in dual-
     * octal mode. */
    if (ospi_ctx->is_dual_octal)
    {
        aes_addr_ctrl ram_addr_ctrl;
        ram_addr_ctrl.addr_lower_bits   = 11;
        ram_addr_ctrl.addr_upper_shift  = 12;
        ram_addr_ctrl.ss0_array_mode_en = 1;
        ram_addr_ctrl.ss1_array_mode_en = 0;
        ram_addr_ctrl.addr_mask         = 0x7FF;
        aes_control_address(ospi_ctx->aes, &ram_addr_ctrl);
    }

    ospi_enable(ospi_ctx->regs);
    return 0;
}
