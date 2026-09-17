/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef OSPI_DRV_H
#define OSPI_DRV_H

#include <stdint.h>

#include "ospi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * OSPI controller status / framing helper values
 * ========================================================================== */

/* SPI frame format (FRF) value */
#define SPI_OCTAL                             0x3U

/* Transfer mode (TMOD) values */
#define SPI_TMOD_TO                           0x1U /* transmit only */
#define SPI_TMOD_RO                           0x2U /* receive only  */

/* Status register (SR) bits */
#define SR_BUSY                               (1U << 0)
#define SR_TF_EMPTY                           (1U << 2)

#define DELAY_CFG_16(v)                                                             \
    ((const uint8_t[16]){(v), (v), (v), (v), (v), (v), (v), (v), (v), (v), (v),     \
                         (v), (v), (v), (v), (v)})

/* ==========================================================================
 * OSPI controller context
 * (OSPI_Type / AES_Type come from the Alif DFP soc.h)
 * ========================================================================== */

/* Forward declaration so the device read/write handler pointers below can take
 * a pointer to the context they are stored in. */
struct ospi_cfg;

typedef void (*ospi_write_data_fn)(struct ospi_cfg *ospi_ctx, uint32_t address,
                                   uint32_t *data, uint32_t length);
typedef int  (*ospi_read_data_fn)(struct ospi_cfg *ospi_ctx, uint32_t address,
                                  volatile uint32_t *data, uint32_t length);
typedef int  (*ospi_erase_sector_fn)(struct ospi_cfg *ospi_ctx, uint32_t address);

typedef struct ospi_cfg {
    OSPI_Type        *regs;              /* OSPI0 / OSPI1 controller registers   */
    AES_Type         *aes;               /* OSPI delay (AES) module registers    */
    uint32_t          tx_buff[8];        /* scratch tx command buffer            */
    volatile uint32_t rx_buff[8];        /* scratch rx buffer                    */
    uint32_t          dfs;               /* DFS for non-XIP read/write           */
    uint32_t          xip_dfs;           /* XIP frame size (DFS)                 */
    uint32_t          read_wait_cycles;  /* device read wait cycles (XIP)        */
    uint32_t          write_wait_cycles; /* device write wait cycles (XIP)       */
    bool              is_dual_octal;     /* transfer type is dual-octal (HexSPI) */
    bool              is_hyperram;       /* device is a HyperBus HyperRAM (IS66) */
    bool              is_flash;          /* device is NOR flash (IS25W / MX66)   */
    bool              calibrate_ssi_oe;  /* track SSI OE_N delays with TXD       */
    ospi_write_data_fn write_data;       /* selected device write handler        */
    ospi_read_data_fn  read_data;        /* selected device read handler         */
    ospi_erase_sector_fn erase_sector;   /* selected flash erase handler         */
} ospi_cfg_t;

/* OSPI delay configuration: 16 individual per-line delay taps for each of the
 * TXD, RXD and SSI output-enable (OE_N) paths, plus the two RXDS strobe delays
 * and the two TXD data-mask (DM) data delays. Passed to ospi_delay_cfg(). */
#pragma pack(push, 1)
typedef struct ospi_delay_cfg {
    uint32_t idx;       /* OSPI controller the config belongs to (0 or 1) */
    uint32_t sclk_freq; /* OSPI controller SCLK frequency */
    uint8_t txd[16];    /* per-line TXD delay taps (index 0..15)     */
    uint8_t rxd[16];    /* per-line RXD delay taps (index 0..15)     */
    uint8_t ssioen[16]; /* per-line OE_N delay taps (index 0..15)    */
    uint8_t rxds[2];    /* RXDS strobe delays (index 0..1)           */
    uint8_t txddm[2];   /* TXD DM data delays (index 0..1)           */
} ospi_delay_cfg_t;
#pragma pack(pop)

/* On-flash container for the calibration results: one entry per calibrated
 * controller, identified by its `idx` field. Written to the last flash sector
 * (CAL_RESULT_BASE_OFFSET) so another application can read the values back and
 * run the bus at the calibrated clock. */
#define OSPI_DELAY_BLOB_MAGIC   0xFA57C10CU

#pragma pack(push, 1)
typedef struct {
    uint32_t         magic;   /* OSPI_DELAY_BLOB_MAGIC                  */
    uint32_t         count;   /* number of delay configurations stored  */
    ospi_delay_cfg_t cfg[];   /* `count` entries                        */
} ospi_delay_blob_t;
#pragma pack(pop)

/* Setup OSPI for reading. See ospi_drv.c for why the DFP ospi_receive() is not
 * used as a drop-in here.
 *   inst_len  Instruction phase width (SPI_INST_L_*)
 *   addr_len  Address phase width (SPI_ADDR_L_*)
 *   hyperbus  Enable HyperBus framing (HyperRAM) instead of JEDEC-octal */
void ospi_setup_read(OSPI_Type *ospi, uint32_t dfs, uint32_t frf_type, uint32_t inst_len,
                     uint32_t addr_len, uint32_t wait_cycles, bool hyperbus, uint32_t length);

/* Setup OSPI for writing. See ospi_drv.c for why the DFP ospi_send() is not
 * used as a drop-in here.
 *   inst_len  Instruction phase width (SPI_INST_L_*)
 *   addr_len  Address phase width (SPI_ADDR_L_*)
 *   hyperbus  Enable HyperBus framing (HyperRAM) instead of JEDEC-octal */
void ospi_setup_write(OSPI_Type *ospi, uint32_t dfs, uint32_t frf_type, uint32_t inst_len,
                      uint32_t addr_len, uint32_t wait_cycles, bool hyperbus);

/* Initialize OSPI XIP (memory-mapped) configuration for the RAM device.
 *   ospi              Pointer to the OSPI register map
 *   read_wait_cycles  Read wait cycles for the RAM device
 *   write_wait_cycles Write wait cycles for the RAM device
 *   is_dual_octal     OSPI transfer type is Dual Octal
 *   is_hyperram       Device is a HyperBus HyperRAM (uses HyperBus XIP framing) */
void ospi_xip_cfg(OSPI_Type *ospi,
                  uint32_t dfs,
                  uint32_t read_wait_cycles, uint32_t write_wait_cycles,
                  bool is_dual_octal, bool is_hyperram);

void ospi_xip_enter(ospi_cfg_t *ospi_ctx);
void ospi_xip_exit(ospi_cfg_t *ospi_ctx);

void ospi_clk_cfg(OSPI_Type *ospi, uint32_t clk_div, uint32_t drive_edge);

/* Configure the 16 individual RXD delay taps.
 *   aes        Pointer to the AES peripheral
 *   delay_val  Array of 16 per-line RXD delay values (index 0..15) */
void ospi_delay_cfg_rxd(AES_Type *aes, const uint8_t delay_val[16]);

/* Configure the 16 individual TXD delay taps.
 *   aes        Pointer to the AES peripheral
 *   delay_val  Array of 16 per-line TXD delay values (index 0..15) */
void ospi_delay_cfg_txd(AES_Type *aes, const uint8_t delay_val[16]);

/* Configure the 16 individual SSI output-enable (OE_N) delay taps.
 *   aes        Pointer to the AES peripheral
 *   delay_val  Array of 16 per-line OE_N delay values (index 0..15) */
void ospi_delay_cfg_ssioen(AES_Type *aes, const uint8_t delay_val[16]);

/* Configure the two RXDS (read data strobe) delay values.
 *   aes         Pointer to the AES peripheral
 *   rxds_delay  Array of 2 RXDS delay values: index 0 -> register bits 7:0,
 *               index 1 -> register bits 15:8 */
void ospi_delay_cfg_rxds(AES_Type *aes, const uint8_t rxds_delay[2]);

/* Configure the two TXD data-mask (DM) data delays, leaving the DM OE_N delay
 * fields untouched (read-modify-write).
 *   aes          Pointer to the AES peripheral
 *   txddm_delay  Array of 2 DM data delays: index 0 -> TXD_DM_0,
 *                index 1 -> TXD_DM_1 */
void ospi_delay_cfg_txddm(AES_Type *aes, const uint8_t txddm_delay[2]);

/* Apply a full TXD + RXD delay configuration.
 *   aes    Pointer to the AES peripheral
 *   cfg    Delay configuration holding the TXD and RXD tap values */
void ospi_delay_cfg(AES_Type *aes, const ospi_delay_cfg_t *cfg);

/* Print an ospi_delay_cfg_t as a pasteable C designated initializer, so a
 * calibrated configuration can be copied into another project as a compile-time
 * constant.
 *   cfg   Delay configuration to print
 *   name  Variable name to emit (NULL -> "calibrated_delay_cfg") */
void ospi_delay_cfg_print(const ospi_delay_cfg_t *cfg, const char *name);




#ifdef __cplusplus
}
#endif

#endif /* OSPI_DRV_H */
