/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*
 * APS512XXN OSPI/XSPI pseudo-SRAM device driver (register/FIFO path).
 */

#ifndef APS512XXN_H
#define APS512XXN_H

#include <stdint.h>

#include "ospi_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * APS512XXN device definitions
 * ========================================================================== */

/* APS512XXN Device ID */
#define APS512XXN_ID                          0xDE

/* APS512XXN wait cycle macros (device counts wait cycles from last addr cycle) */
#define APS512XXN_INIT_REG_READ_WAIT_CYCLES   5
#define APS512XXN_REG_WRITE_WAIT_CYCLES       0
#define APS512XXN_RESET_WAIT_CYCLES           3

// See aps512xxn_setup, fixed latency
#define APS512XXN_WRITE_WAIT_CYCLES           7
#define APS512XXN_READ_WAIT_CYCLES            14

/* APS512XXN Register addresses */
#define APS512XXN_MODE_REG0_ADDR              0x0
#define APS512XXN_MODE_REG1_ADDR              0x1
#define APS512XXN_MODE_REG2_ADDR              0x2
#define APS512XXN_MODE_REG3_ADDR              0x3
#define APS512XXN_MODE_REG4_ADDR              0x4
#define APS512XXN_MODE_REG6_ADDR              0x6
#define APS512XXN_MODE_REG8_ADDR              0x8

/* APS512XXN Register bit positions */
#define APS512XXN_MODE_REG0_DRIVE_STR          0
#define APS512XXN_MODE_REG0_READ_LATENCY_CODE  2
#define APS512XXN_MODE_REG0_LATENCY_TYPE       5
#define APS512XXN_MODE_REG4_WRITE_LATENCY_CODE 5
#define APS512XXN_MODE_REG4_READ_RF_RATE       3
#define APS512XXN_MODE_REG4_PASR               0
#define APS512XXN_MODE_REG8_TRANSFER_MODE      6
#define APS512XXN_MODE_REG8_RBX_READ_EN        3
#define APS512XXN_MODE_REG8_BURST_TYPE         2
#define APS512XXN_MODE_REG8_BURST_LEN          0

/* APS512XXN command opcodes */
#define APS512XXN_CMD_SYNC_READ           0x00
#define APS512XXN_CMD_SYNC_WRITE          0x80
#define APS512XXN_CMD_LINEAR_BURST_READ   0x20
#define APS512XXN_CMD_LINEAR_BURST_WRITE  0xA0
#define APS512XXN_CMD_MODE_REGISTER_READ  0x40
#define APS512XXN_CMD_MODE_REGISTER_WRITE 0xC0
#define APS512XXN_CMD_GLOBAL_RESET        0xFF

/* ==========================================================================
 * Public API
 * ========================================================================== */

int aps512xxn_setup(ospi_cfg_t *ospi_ctx);

/* Linear-burst write of `length` 32-bit words to `address` (FIFO path). */
void aps512xxn_write_data(ospi_cfg_t *ospi_ctx, uint32_t address, uint32_t *data,
                          uint32_t length);

/* Linear-burst read of `length` 32-bit words from `address` (FIFO path).
 * Returns 0 on success, -1 if the controller stayed busy. */
int aps512xxn_read_data(ospi_cfg_t *ospi_ctx, uint32_t address, volatile uint32_t *data,
                        uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* APS512XXN_H */
