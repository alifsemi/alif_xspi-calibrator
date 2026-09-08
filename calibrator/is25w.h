/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef IS25W_H
#define IS25W_H

#include <stdint.h>

#include "ospi_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * IS25WX command opcodes
 * ========================================================================== */
#define IS25W_CMD_WRITE_ENABLE            0x06U /* WREN                          */
#define IS25W_CMD_READ_STATUS             0x05U /* RDSR (status register)        */
#define IS25W_CMD_READ_FLAG_STATUS        0x70U /* RDFSR (flag status register)  */
#define IS25W_CMD_READ_ID                 0x9FU /* RDID (JEDEC read identity)    */
#define IS25W_CMD_RESET_ENABLE            0x66U /* RSTEN                         */
#define IS25W_CMD_RESET                   0x99U /* RST                           */
#define IS25W_CMD_WRITE_VOL_CONFIG        0x81U /* WRVCR (volatile config reg)   */
#define IS25W_CMD_READ_DATA               0x7CU /* 8DTRD octal DDR read          */
#define IS25W_CMD_PAGE_PROGRAM            0x84U /* octal DDR page program        */
#define IS25W_CMD_SECTOR_ERASE            0x21U /* 4KB sector erase              */
#define IS25W_CMD_BULK_ERASE              0xC7U /* whole-chip erase              */

/* JEDEC manufacturer ID (RDID byte 0). 0x9D == ISSI. */
#define IS25W_MANUFACTURER_ISSI           0x9DU

/* Controller clock used for the direct-register VCR write path (is25w_write_reg).
 * The DFP may leave OSPI1 at an unknown baud after PowerControl, so a known slow
 * clock is programmed for the register write. Matches the tool's slow BAUD. */
#ifndef IS25W_REG_WRITE_BAUD
#define IS25W_REG_WRITE_BAUD              40U
#endif
#ifndef IS25W_REG_WRITE_DRIVE_EDGE
#define IS25W_REG_WRITE_DRIVE_EDGE        1U
#endif

/* Flag status register bits */
#define IS25W_FLAG_STATUS_BUSY            0x80U /* 0 = program/erase in progress */
#define IS25W_FLAG_STATUS_ERROR           0x30U /* program(0x10) | erase(0x20)   */

/* Status/flag reads use a fixed 8-cycle latency (per the DFP driver). */
#define IS25W_STATUS_WAIT_CYCLES          8U

/* Program and erase commands carry no read latency. */
#define IS25W_WRITE_WAIT_CYCLES           0U

/* XIP linear/wrap read instruction (duplicated 16-bit DDR opcode). Both the
 * 8DTRD (0x7C) and the octal fast-read (0x0B) families are valid per the
 * datasheet; the XIP path sends the opcode duplicated into both DDR beats. */
#ifndef IS25W_XIP_READ_INST
#define IS25W_XIP_READ_INST               ((IS25W_CMD_READ_DATA << 8) | IS25W_CMD_READ_DATA)
#endif

/* ==========================================================================
 * Device geometry
 * ========================================================================== */
#define IS25W_PAGE_SIZE                   256U
#define IS25W_SECTOR_SIZE                 (4U * 1024U)
#define IS25W_BLOCK_SIZE                  (64U * 1024U)

/* ==========================================================================
 * Public API (mirrors aps512xxn.* / is66.*)
 * ========================================================================== */

/* Reset */
int is25w_setup(ospi_cfg_t *ospi_ctx);

/* Linear octal-DDR read  */
int is25w_read_data(ospi_cfg_t *ospi_ctx, uint32_t address, volatile uint32_t *data,
                    uint32_t length);

/* Program `length` 32-bit words to `address`  */
void is25w_write_data(ospi_cfg_t *ospi_ctx, uint32_t address, uint32_t *data,
                      uint32_t length);

/* Erase the 4KB sector . */
int is25w_erase_sector(ospi_cfg_t *ospi_ctx, uint32_t address);

#ifdef __cplusplus
}
#endif

#endif /* IS25W_H */
