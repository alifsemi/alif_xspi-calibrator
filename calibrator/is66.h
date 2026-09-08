/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef IS66_H
#define IS66_H

#include <stdint.h>

#include "ospi_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * IS66 device definitions
 * ========================================================================== */

/* ID Register 0 manufacturer field (ID0[3:0]); 0011b == ISSI. */
#define IS66_ID0_MANUFACTURER_ISSI        0x3U
/* ID Register 1 device-type field (ID1[3:0]); 0001b == HyperRAM 2.0. */
#define IS66_ID1_TYPE_HYPERRAM            0x1U

/* HyperBus initial latency count (clocks) programmed into CR0[7:4].
 *
 */
#ifndef IS66_INITIAL_LATENCY
#define IS66_INITIAL_LATENCY              7U
#endif

/* Output drive strength code for CR0[14:12]; 000b == 34 ohm (reset default). */
#ifndef IS66_DRIVE_STRENGTH
#define IS66_DRIVE_STRENGTH               0x0U
#endif

/* Controller wait-cycle counts
 */
#define IS66_READ_WAIT_CYCLES             (2U * IS66_INITIAL_LATENCY)
#define IS66_WRITE_WAIT_CYCLES            (2U * IS66_INITIAL_LATENCY)

/* ==========================================================================
 * Public API (mirrors aps512xxn.*)
 * ========================================================================== */

int is66_setup(ospi_cfg_t *ospi_ctx);

/* Linear-burst write of `length` 32-bit words */
void is66_write_data(ospi_cfg_t *ospi_ctx, uint32_t address, uint32_t *data,
                     uint32_t length);

/* Linear-burst read of `length` 32-bit words */
int is66_read_data(ospi_cfg_t *ospi_ctx, uint32_t address, volatile uint32_t *data,
                   uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* IS66_H */
