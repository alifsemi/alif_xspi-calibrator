/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef MX66UW1G_H
#define MX66UW1G_H

#include <stdint.h>

#include "ospi_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * DTR-OPI command opcodes (16-bit: command byte + complement)
 * ========================================================================== */
#define MX66UW1G_CMD_READ_DATA            0xEE11U /* 8DTRD octal DDR read        */
#define MX66UW1G_CMD_PAGE_PROGRAM         0x12EDU /* PP4B  page program          */
#define MX66UW1G_CMD_SECTOR_ERASE         0x21DEU /* SE4B  4KB sector erase      */
#define MX66UW1G_CMD_WRITE_ENABLE         0x06F9U /* WREN                        */
#define MX66UW1G_CMD_READ_STATUS          0x05FAU /* RDSR  status register       */
#define MX66UW1G_CMD_READ_SECURITY        0x2BD4U /* RDSCUR security register    */

/* Status register bits */
#define MX66UW1G_STATUS_WIP               0x01U /* write/erase in progress       */
#define MX66UW1G_STATUS_WEL               0x02U /* write enable latch            */

/* Security register error bits */
#define MX66UW1G_SECURITY_P_FAIL          0x20U
#define MX66UW1G_SECURITY_E_FAIL          0x40U

/* Register reads (RDSR/RDSCUR) carry a 32-bit dummy address and a fixed
 * 4-cycle latency in DTR-OPI, per the DFP driver. */
#define MX66UW1G_STATUS_WAIT_CYCLES       4U

/* Program and erase commands carry no read latency. */
#define MX66UW1G_WRITE_WAIT_CYCLES        0U

/* Controller clock used for the completion handshake and erase, so those can
 * never fail while the caller drives the bus at an uncalibrated speed. */
#define MX66UW1G_REG_WRITE_SCLK           16666666U

/* ==========================================================================
 * Device geometry
 * ========================================================================== */
#define MX66UW1G_PAGE_SIZE                256U
#define MX66UW1G_SECTOR_SIZE              (4U * 1024U)
#define MX66UW1G_BLOCK_SIZE               (64U * 1024U)

/* ==========================================================================
 * Public API (mirrors is25w.*)
 * ========================================================================== */

/* Bring the device up in DTR-OPI via the DFP flash driver, then program the
 * controller-side context and XIP read opcodes. Returns 0 on success. */
int mx66uw1g_setup(ospi_cfg_t *ospi_ctx);

/* Linear octal-DDR read of `length` 32-bit words from `address` (FIFO path).
 * Returns 0 on success, -1 if the controller stayed busy. */
int mx66uw1g_read_data(ospi_cfg_t *ospi_ctx, uint32_t address, volatile uint32_t *data,
                       uint32_t length);

/* Program `length` 32-bit words to `address` (WREN + page program + poll).
 * Handles page-boundary splitting. */
void mx66uw1g_write_data(ospi_cfg_t *ospi_ctx, uint32_t address, uint32_t *data,
                         uint32_t length);

/* Erase the 4KB sector containing `address` (WREN + sector erase + poll). */
int mx66uw1g_erase_sector(ospi_cfg_t *ospi_ctx, uint32_t address);

#ifdef __cplusplus
}
#endif

#endif /* MX66UW1G_H */
