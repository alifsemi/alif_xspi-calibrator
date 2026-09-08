/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include "RTE_Components.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include CMSIS_device_header

/* Alif DFP CMSIS-pack headers */
#include "sys_ctrl_ospi.h"
#include "sys_ctrl_aes.h"
#include "sys_utils.h"

#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
#include "retarget_stdout.h"
#include <retarget_init.h>
#endif

#include "power_management.h"
#include "se_services_port.h"
#include "ram_test.h"

#include "ospi.h"
#include "ospi_drv.h"
#include "mpu.h"
#include "aps512xxn.h"
#include "is66.h"
#include "is25w.h"
#include "mx66uw1g.h"
#include "cal_pattern.h"
#include "board_config.h"

#define DEBUG_PRINTS 0
#define FAST_INTEGRITY_CHECK 1
#define XIP_MEMORY_TESTS 1

#define CAL_XIP_TEST_SIZE                     (16 * 1024)
#define CAL_XIP_TEST_ITERS                    (64 * 1024)

/* Passes over the whole calibration region in the final flash XIP read test. */
#define CAL_XIP_FLASH_PASSES                  64


#define BAUDR_FAST                            2
#define DRIVE_EDGE_FAST                       0
/* OSPI clock configuration (slow default) */
#define BAUD                                  40
#define DRIVE_EDGE                            1

// The 'slow' default delay configuration
#define DEFAULT_RXDS_DELAY                  11U

ospi_delay_cfg_t delay_cfg_default = {
    .rxds = {DEFAULT_RXDS_DELAY, DEFAULT_RXDS_DELAY},
};

// Manually tested AppKit-E8 DM delay value at 200MHz
#define DEFAULT_TXD_DM_DELAY                7U

ospi_delay_cfg_t delay_cfg_ram = {.idx = 0};

/* Separate calibrated config for the flash device (OSPI1 / AES1). */
ospi_delay_cfg_t delay_cfg_flash = {.idx = 1};

#define OSPI0_XIP_BASE                          0xA0000000UL
#define OSPI1_XIP_BASE                          0xC0000000UL
static ospi_cfg_t ospi;       /* RAM device   (OSPI0 / AES0) */
static ospi_cfg_t ospi_flash; /* Flash device (OSPI1 / AES1) */

/* ==========================================================================
 * 1 ms tick / delay helpers
 * ========================================================================== */
static volatile uint32_t g_ms_ticks = 0;

void SysTick_Handler(void)
{
    g_ms_ticks++;
}

static void configure_delay(void)
{
    SysTick_Config(SystemCoreClock / 1000);
}

static void delay(uint32_t nticks)
{
    uint32_t c_ticks = g_ms_ticks;
    while ((g_ms_ticks - c_ticks) < nticks)
        ;
}


/* ==========================================================================
 * Delay calibration ("training") routines
 * ========================================================================== */

static void lane_pattern(int lane, int num_lanes, uint32_t *mask, uint32_t *expected)
{
    uint32_t m = 0, e = 0;
    for (int pos = lane; pos < 32; pos += num_lanes) {
        m |= (1U << pos);
        if (pos < 16) {
            e |= (1U << pos);
        }
    }
    *mask = m;
    *expected = e;
}

static uint32_t tx_delay_cal(ospi_cfg_t *ospi_ctx, ospi_delay_cfg_t *cfg)
{
    uint32_t before = g_ms_ticks;
    uint32_t test_addr = 0;
    uint32_t min_delay[16];
    uint32_t max_delay[16];
    uint32_t adj_delay[16];
    memset(min_delay, -1, sizeof(min_delay));
    memset(max_delay, -1, sizeof(max_delay));
    memset(adj_delay, -1, sizeof(adj_delay));

    uint32_t write_val[2] = {0x0000FFFF, 0x0000FFFF};
    uint32_t inv_write_val[2] = {~write_val[0], ~write_val[1]};
    uint32_t read_data[2];
    memset(read_data, -1, sizeof(read_data));

    const uint32_t datalines = ospi_ctx->is_dual_octal ? 16 : 8;

    /* Flash programs 1->0 only, so the RAM pre-clear write is impossible. Erase
     * one scratch sector up front and step the address every pass instead. */
    if (ospi_ctx->is_flash) {
        if (ospi_ctx->erase_sector(ospi_ctx, CAL_TX_SCRATCH_ADDR) != 0) {
            printf("TX Tuning: scratch sector erase failed\n");
            return 1;
        }
    }

    for (int i = 0; i < 24; i++) {
        if (ospi_ctx->is_flash) {
            test_addr = CAL_TX_SCRATCH_ADDR + (uint32_t)i * sizeof(write_val);
        } else {
            /* Pre-clear: write complement at low speed (guaranteed correct) */
            ospi_clk_cfg(ospi_ctx->regs, BAUD, DRIVE_EDGE);
            ospi_delay_cfg(ospi_ctx->aes, &delay_cfg_default);
            ospi_ctx->write_data(ospi_ctx, test_addr, inv_write_val, 2);
        }

        /* Test write at high speed with TX delay = i, OE_N delay = i */
        ospi_clk_cfg(ospi_ctx->regs, BAUDR_FAST, DRIVE_EDGE_FAST);
        ospi_delay_cfg_txd(ospi_ctx->aes, DELAY_CFG_16(i));
        if (ospi_ctx->calibrate_ssi_oe) {
            ospi_delay_cfg_ssioen(ospi_ctx->aes, DELAY_CFG_16(i));
        }
        ospi_ctx->write_data(ospi_ctx, test_addr, write_val, 2);

        /* Read back at low speed */
        ospi_clk_cfg(ospi_ctx->regs, BAUD, DRIVE_EDGE);
        ospi_delay_cfg(ospi_ctx->aes, &delay_cfg_default);

        if (ospi_ctx->read_data(ospi_ctx, test_addr, read_data, 2)) {
#if DEBUG_PRINTS
            printf("TX Tuning: Failed to read data at low speed\n");
#endif
            return 1;
        } else {
#if DEBUG_PRINTS
            printf("0x%x 0x%x\n", read_data[0], read_data[1]);
#endif
            for (int j = 0; j < (int)datalines; j++) {
                uint32_t mask, expected;
                lane_pattern(j, datalines, &mask, &expected);

                if ((read_data[1] & mask) == expected && (read_data[0] & mask) == expected) {
                    if (min_delay[j] == (uint32_t)-1) {
                        min_delay[j] = i;
                    }
                    max_delay[j] = i;
                }
            }
        }
    }
    for (int i = 0; i < (int)datalines; i++) {
        if (min_delay[i] == (uint32_t)-1 || max_delay[i] == (uint32_t)-1) {
            adj_delay[i] = 0;
            continue;
        }
        if ((max_delay[i] - min_delay[i]) > 0x8) {
            adj_delay[i] = min_delay[i] + 0x4;
        } else {
            adj_delay[i] = (min_delay[i] + max_delay[i]) / 2;
        }
    }
    uint32_t after = g_ms_ticks;
#if DEBUG_PRINTS
    printf("TX Tuning took %d ms\n", after - before);
#endif
    for (int i = 0; i < (int)datalines; i++) {
        printf("Calibrated TX %d delay - MIN: 0x%x MAX: 0x%x AVG: 0x%x ADJ: 0x%x\n", i,
               min_delay[i], max_delay[i], (min_delay[i] + max_delay[i]) / 2, adj_delay[i]);
        cfg->txd[i] = adj_delay[i];
        if (ospi_ctx->calibrate_ssi_oe) {
            /* OE_N must track TXD for proper output driver timing */
            cfg->ssioen[i] = adj_delay[i];
        }
    }

    return 0;
}



/* Sweep a single DM (data-mask) lane and return the passing delay window.
 *
 */
static uint32_t dm_sweep_lane(ospi_cfg_t *ospi_ctx, int lane, uint32_t byte_off,
                              uint8_t new_val, uint32_t bg, uint32_t expected,
                              uint8_t other_delay, uint32_t *min_out, uint32_t *max_out)
{
    volatile uint8_t  *xip8  = (volatile uint8_t  *)OSPI0_XIP_BASE;
    volatile uint32_t *xip32 = (volatile uint32_t *)OSPI0_XIP_BASE;

    uint32_t min_delay = (uint32_t)-1;
    uint32_t max_delay = (uint32_t)-1;

    for (int i = 0; i < 24; i++) {
        uint8_t  dm[2];
        uint32_t read_back;

        dm[lane]     = (uint8_t)i;
        dm[lane ^ 1] = other_delay;

        /* ---- Pre-fill background (unmasked full-word store) ---- */
        xip32[0] = bg;
        __DSB();

        /* Set candidate DM delay on the lane under test ---- */
        ospi_delay_cfg_txddm(ospi_ctx->aes, dm);

        /* Only the lane-under-test byte is written (others masked). */
        xip8[byte_off] = new_val;

        /* ---- Read back (masking not involved on reads) ---- */
        __DSB();
        read_back = xip32[0];

#if DEBUG_PRINTS
        printf("DM%d %2d: 0x%08x (exp 0x%08x)\n", lane, i, read_back, expected);
#endif
        if (read_back == expected) {
            if (min_delay == (uint32_t)-1) {
                min_delay = (uint32_t)i;
            }
            max_delay = (uint32_t)i;
        }
    }

    *min_out = min_delay;
    *max_out = max_delay;
    return (min_delay == (uint32_t)-1 || max_delay == (uint32_t)-1) ? 1 : 0;
}

/* Single-octal x8 DM sweep across ALL four byte lanes. */
static uint32_t dm_sweep_all_bytes(ospi_cfg_t *ospi_ctx, uint32_t *adj_out)
{
    const uint32_t bg = 0x11223344u;
    const uint8_t  v[4] = {0xAAu, 0xBBu, 0xCCu, 0x55u};

    volatile uint8_t  *xip8  = (volatile uint8_t  *)OSPI0_XIP_BASE;
    volatile uint32_t *xip32 = (volatile uint32_t *)OSPI0_XIP_BASE;

    uint32_t lmin[4], lmax[4];
    uint32_t cmin = (uint32_t)-1, cmax = (uint32_t)-1; /* combined (all-lane) window */
    for (int b = 0; b < 4; b++) {
        lmin[b] = (uint32_t)-1;
        lmax[b] = (uint32_t)-1;
    }

    for (int i = 0; i < 24; i++) {
        uint8_t dm[2] = {(uint8_t)i, (uint8_t)i};
        int all_pass = 1;

        ospi_delay_cfg_txddm(ospi_ctx->aes, dm);

        for (int b = 0; b < 4; b++) {
            uint32_t expected = (bg & ~(0xFFu << (b * 8))) | ((uint32_t)v[b] << (b * 8));
            uint32_t read_back;

            xip32[0] = bg;          /* full-word background (unmasked) */
            __DSB();
            xip8[b] = v[b];         /* byte-masked partial write on lane b */
            __DSB();
            read_back = xip32[0];

            int pass = (read_back == expected);
#if DEBUG_PRINTS
            printf("DMx8 d=%2d b=%d: 0x%08x (exp 0x%08x) %s\n", i, b, read_back, expected,
                   pass ? "ok" : "BAD");
#endif
            if (pass) {
                if (lmin[b] == (uint32_t)-1) {
                    lmin[b] = (uint32_t)i;
                }
                lmax[b] = (uint32_t)i;
            } else {
                all_pass = 0;
            }
        }

        if (all_pass) {
            if (cmin == (uint32_t)-1) {
                cmin = (uint32_t)i;
            }
            cmax = (uint32_t)i;
        }
    }

    for (int b = 0; b < 4; b++) {
        if (lmin[b] == (uint32_t)-1) {
            printf("DM x8 lane %d - no working delay found\n", b);
        } else {
            printf("DM x8 lane %d - MIN: 0x%x MAX: 0x%x\n", b, lmin[b], lmax[b]);
        }
    }

    if (cmin == (uint32_t)-1) {
        printf("DM x8 - NO common delay works for all 4 byte lanes\n");
        *adj_out = DEFAULT_TXD_DM_DELAY;
        return 1;
    }

    *adj_out = (cmin + cmax) / 2;
    printf("DM x8 combined window - MIN: 0x%x MAX: 0x%x ADJ: 0x%x\n", cmin, cmax, *adj_out);
    return 0;
}

static uint32_t tx_dm_delay_cal(ospi_cfg_t *ospi_ctx, ospi_delay_cfg_t *cfg)
{
    /* Data-mask (DM) write-window tuning.
     *
     */
    const uint32_t bg        = 0x11223344u; /* background: b0=44 b1=33 b2=22 b3=11 */
    const uint8_t  v0        = 0xAAu;        /* new byte 0 (DM0 lane)               */
    const uint8_t  v3        = 0x55u;        /* new byte 3 (DM1 lane)               */
    const uint32_t expected0 = 0x112233AAu;  /* DM0 pass: only b0 -> AA, rest bg    */
    const uint32_t expected3 = 0x55223344u;  /* DM1 pass: only b3 -> 55, rest bg    */

    uint32_t before = g_ms_ticks;
    uint32_t min0, max0, min3 = 0, max3 = 0;

    // Make sure we are running at the high speed
    ospi_clk_cfg(ospi_ctx->regs, BAUDR_FAST, DRIVE_EDGE_FAST);
    ospi_delay_cfg(ospi_ctx->aes, cfg);

    /* Enter memory-mapped XIP mode (runs at the calibrated high speed). */
    ospi_xip_enter(ospi_ctx);

    /* --- Single-octal x8: one DM line drives all four byte-beats of a word.
     * Calibrate against all four byte lanes and use the intersection so
     * byte-masked partial-word writes at any offset work. --- */
    if (!ospi_ctx->is_dual_octal) {
        uint32_t adj;
        uint32_t fail = dm_sweep_all_bytes(ospi_ctx, &adj);

        ospi_xip_exit(ospi_ctx);

        uint32_t after = g_ms_ticks;
#if DEBUG_PRINTS
        printf("DM Tuning took %d ms\n", after - before);
#endif
        (void)after;

        cfg->txddm[0] = (uint8_t)adj;
        cfg->txddm[1] = 0;
        ospi_delay_cfg_txddm(ospi_ctx->aes, cfg->txddm);
        return fail;
    }

    /* Sweep DM0 with DM1 parked at the default (DM1 stays fully masked, so its
     * value is not exercised during this pass). */
    uint32_t fail0 = dm_sweep_lane(ospi_ctx, 0, 0, v0, bg, expected0,
                                   DEFAULT_TXD_DM_DELAY, &min0, &max0);
    uint32_t adj0  = fail0 ? DEFAULT_TXD_DM_DELAY : (uint8_t)((min0 + max0) / 2);

    uint32_t fail3 = 0;
    uint32_t adj3;
    if (ospi_ctx->is_dual_octal) {
        /* Sweep DM1 with DM0 parked at its freshly-calibrated centre. */
        fail3 = dm_sweep_lane(ospi_ctx, 1, 3, v3, bg, expected3,
                              (uint8_t)adj0, &min3, &max3);
        adj3  = fail3 ? DEFAULT_TXD_DM_DELAY : (uint8_t)((min3 + max3) / 2);
    } else {
        adj3 = 0;
    }

    ospi_xip_exit(ospi_ctx);

    uint32_t after = g_ms_ticks;
#if DEBUG_PRINTS
    printf("DM Tuning took %d ms\n", after - before);
#endif

    if (fail0) {
        printf("Calibrated DM0 delay - no working delay found, falling back to 0x%x\n",
               DEFAULT_TXD_DM_DELAY);
    } else {
        printf("Calibrated DM0 delay - MIN: 0x%x MAX: 0x%x ADJ: 0x%x\n", min0, max0, adj0);
    }

    if (ospi_ctx->is_dual_octal) {
        if (fail3) {
            printf("Calibrated DM1 delay - no working delay found, falling back to 0x%x\n",
                   DEFAULT_TXD_DM_DELAY);
        } else {
            printf("Calibrated DM1 delay - MIN: 0x%x MAX: 0x%x ADJ: 0x%x\n", min3, max3, adj3);
        }
    }

    /* Apply the per-lane centred DM data delays; OE_N delays stay at 0. */
    cfg->txddm[0] = (uint8_t)adj0;
    cfg->txddm[1] = (uint8_t)adj3;
    ospi_delay_cfg_txddm(ospi_ctx->aes, cfg->txddm);

    /* Signal failure if either exercised lane had no working window. */
    return (fail0 || fail3) ? 1 : 0;
}

/* Sweep the per-lane RXD windows at the currently applied RXDS/clock */
static void rx_sweep_windows(ospi_cfg_t *ospi_ctx, int datalines,
                             uint32_t *min_delay, uint32_t *max_delay)
{
    uint32_t write_val[2] = {0x0000FFFF, 0x0000FFFF};
    uint32_t read_data[2];

    for (int j = 0; j < datalines; j++) {
        min_delay[j] = (uint32_t)-1;
        max_delay[j] = (uint32_t)-1;
    }

    /* RAM: prime the test word by writing it. Flash: read a pre-programmed
     * lane-toggle block instead (no writes during calibration). */
    const uint32_t test_addr = ospi_ctx->is_flash ? CAL_LANE_TOGGLE_ADDR : 0;
    if (!ospi_ctx->is_flash) {
        ospi_ctx->write_data(ospi_ctx, test_addr, write_val, 2);
    }

    for (int i = 0; i < 24; i++) {
        ospi_delay_cfg_rxd(ospi_ctx->aes, DELAY_CFG_16(i));

        memset(read_data, -1, sizeof(read_data));
        if (ospi_ctx->read_data(ospi_ctx, test_addr, read_data, 2)) {
            continue;
        }
        for (int j = 0; j < datalines; j++) {
            uint32_t mask, expected;
            lane_pattern(j, datalines, &mask, &expected);
            if ((read_data[1] & mask) == expected) {
                if (min_delay[j] == (uint32_t)-1) {
                    min_delay[j] = i;
                }
                if ((max_delay[j] == (uint32_t)-1) || (max_delay[j] == (uint32_t)(i - 1))) {
                    max_delay[j] = i;
                }
            }
        }
    }
}

/* Short cacheable-XIP burst validation. */
static int cal_validate_xip(ospi_cfg_t *ospi_ctx)
{
    int mism;
    MPU_Set_OSPI0_XIP_Cacheable();
    ospi_xip_enter(ospi_ctx);
    mism = ram_random_test((uint8_t *)OSPI0_XIP_BASE + (8 * 1024 * 1024),
                           CAL_XIP_TEST_SIZE, CAL_XIP_TEST_ITERS, 0);
    MPU_Set_OSPI0_XIP_Device();
    ospi_xip_exit(ospi_ctx);
    return mism;
}

/* Read-only XIP burst validation for flash: compares the pre-programmed
 * increment/burst block against its expected address-tagged words. Returns the
 * mismatch count (capped). The flash XIP region (0xC0000000) is RO-cacheable in
 * the static MPU table, so the D-cache is invalidated to force fresh reads. */
static int cal_validate_xip_flash(ospi_cfg_t *ospi_ctx)
{
    volatile uint32_t *xip =
        (volatile uint32_t *)(OSPI1_XIP_BASE + CAL_PATTERN_BASE_OFFSET);
    int mism = 0;

    ospi_xip_enter(ospi_ctx);
    SCB_InvalidateDCache_by_Addr((void *)xip, CAL_PATTERN_REGION_SIZE);
    __DSB();

    for (uint32_t off = CAL_OFF_INCREMENT; off < CAL_PATTERN_REGION_SIZE; off += 4) {
        if (xip[off / 4] != cal_increment_word(off)) {
            if (++mism > 64) {
                break;
            }
        }
    }

    ospi_xip_exit(ospi_ctx);
    return mism;
}

#if XIP_MEMORY_TESTS
/* Sustained read-only XIP test over the whole calibration region, run at the
 * calibrated clock once training is done. Every pass invalidates the D-cache
 * first so the words really come off the bus. Returns the mismatch count. */
static int flash_xip_read_test(ospi_cfg_t *ospi_ctx, uint32_t passes)
{
    volatile uint32_t *xip =
        (volatile uint32_t *)(OSPI1_XIP_BASE + CAL_PATTERN_BASE_OFFSET);
    uint32_t errors = 0;

    ospi_xip_enter(ospi_ctx);

    for (uint32_t p = 0; p < passes; p++) {
        SCB_InvalidateDCache_by_Addr((void *)xip, CAL_PATTERN_REGION_SIZE);
        __DSB();

        for (uint32_t off = 0; off < CAL_PATTERN_REGION_SIZE; off += 4) {
            uint32_t got = xip[off / 4];
            uint32_t exp = cal_pattern_word(off);

            if (got != exp) {
                if (errors < 16) {
                    printf("  pass %lu: +0x%04lX got 0x%08lX exp 0x%08lX\n",
                           (unsigned long)p, (unsigned long)off,
                           (unsigned long)got, (unsigned long)exp);
                }
                errors++;
            }
        }
    }

    ospi_xip_exit(ospi_ctx);

    printf("Read %lu KB, %lu mismatches\n",
           (unsigned long)((passes * CAL_PATTERN_REGION_SIZE) / 1024U),
           (unsigned long)errors);

    return (int)errors;
}
#endif

/* Combined RXDS + RXD delay calibration using a maximin nested sweep.
 *
 * Results are written into `cfg` and the matching AES delay module. For flash
 * (ospi_ctx->is_flash) the sweeps read a pre-programmed pattern instead of
 * writing test words, and validation uses a read-only XIP compare. */
static uint32_t rxds_rx_delay_cal(ospi_cfg_t *ospi_ctx, ospi_delay_cfg_t *cfg)
{
    const int datalines        = ospi_ctx->is_dual_octal ? 16 : 8;
    const int num_strobes      = ospi_ctx->is_dual_octal ? 2 : 1;
    const int lanes_per_strobe = datalines / num_strobes;
    uint32_t before = g_ms_ticks;

    uint8_t  rxds_delay[2] = {0, 0};
    uint32_t min_delay[16], max_delay[16];

    // Make sure we are running at the high speed
    ospi_clk_cfg(ospi_ctx->regs, BAUDR_FAST, DRIVE_EDGE_FAST);
    ospi_delay_cfg(ospi_ctx->aes, cfg);

    // puukko
    //if (ospi_ctx->is_flash)
    //    ospi_delay_cfg_txd(ospi_ctx->aes, DELAY_CFG_16(3));

    /* -------- 1. Find RXDS working window ----------
     *
     * RXDS0 and RXDS1 are separate read strobes, each clocking in its own byte
     * They can have DIFFERENT working ranges, so each strobe is swept on its own. */
    uint32_t rxds_lo[2] = {(uint32_t)-1, (uint32_t)-1};
    uint32_t rxds_hi[2] = {(uint32_t)-1, (uint32_t)-1};
    {
        uint32_t write_data[2] = {0x0000FFFF, 0x0000FFFF};
        /* RAM: prime the test word. Flash: read the pre-programmed block. */
        uint32_t test_addr = ospi_ctx->is_flash ? CAL_LANE_TOGGLE_ADDR : 0;
        ospi_delay_cfg_rxd(ospi_ctx->aes, DELAY_CFG_16(0));
        if (!ospi_ctx->is_flash) {
            ospi_ctx->write_data(ospi_ctx, test_addr, write_data, 2);
        }

        for (int s = 0; s < num_strobes; s++) {
            const int lane0 = s * lanes_per_strobe;
            const int lane1 = lane0 + lanes_per_strobe;

            for (int i = 0; i < 24; i++) {
                uint32_t read_data[2];
                int ok;

                rxds_delay[s] = (uint8_t)i;
                ospi_delay_cfg_rxds(ospi_ctx->aes, rxds_delay);

                memset(read_data, -1, sizeof(read_data));
                ok = (ospi_ctx->read_data(ospi_ctx, test_addr, read_data, 2) == 0);
                for (int j = lane0; ok && j < lane1; j++) {
                    uint32_t mask, expected;
                    lane_pattern(j, datalines, &mask, &expected);
                    if ((read_data[1] & mask) != expected) {
                        ok = 0;
                    }
                }
                if (ok) {
                    if (rxds_lo[s] == (uint32_t)-1) {
                        rxds_lo[s] = (uint32_t)i;
                    }
                    rxds_hi[s] = (uint32_t)i;
                }
            }

            if (rxds_lo[s] == (uint32_t)-1) {
                printf("RXDS/RX cal: no working RXDS%d found\n", s);
                return 1;
            }
            printf("RXDS%d working window - MIN: 0x%x MAX: 0x%x\n",
                   s, rxds_lo[s], rxds_hi[s]);
        }
    }

    /* -------- 2. Baseline both strobes ---------------------------------------
     *
     * Park each strobe at the centre of its own window and centre every lane's
     * RXD window. This is only a starting point; step 3 refines each strobe. */
    for (int s = 0; s < num_strobes; s++) {
        rxds_delay[s] = (uint8_t)((rxds_lo[s] + rxds_hi[s]) / 2);
    }
    ospi_delay_cfg_rxds(ospi_ctx->aes, rxds_delay);
    rx_sweep_windows(ospi_ctx, datalines, min_delay, max_delay);
    for (int j = 0; j < datalines; j++) {
        if (min_delay[j] == (uint32_t)-1 || max_delay[j] == (uint32_t)-1) {
            printf("RXDS/RX cal: lane %d has no RXD window at baseline\n", j);
            return 1;
        }
        cfg->rxd[j] = (uint8_t)((min_delay[j] + max_delay[j]) / 2);
    }
    ospi_delay_cfg_rxd(ospi_ctx->aes, cfg->rxd);

    /* -------- 3. Per-strobe refinement ---------------------------------------
     *
     * For each strobe independently: sweep its own RXDS working window; at each
     * candidate re-centre only the RXD lanes it owns */
    for (int s = 0; s < num_strobes; s++) {
        const int lane0 = s * lanes_per_strobe;
        const int lane1 = lane0 + lanes_per_strobe;

        uint8_t cand_rxd[24][16];
        int     cand_valid[24];
        int     cand_worst[24];
        int     cand_mism[24];
        for (int c = 0; c < 24; c++) {
            cand_valid[c] = 0;
            cand_worst[c] = -1;
            cand_mism[c]  = INT32_MAX;
        }

        for (uint32_t c = rxds_lo[s]; c <= rxds_hi[s]; c++) {
            rxds_delay[s] = (uint8_t)c;
            ospi_delay_cfg_rxds(ospi_ctx->aes, rxds_delay);

            rx_sweep_windows(ospi_ctx, datalines, min_delay, max_delay);

            /* Every owned lane must have a window; score by the worst one. */
            int valid = 1, worst_margin = INT32_MAX;
            for (int j = lane0; j < lane1; j++) {
                if (min_delay[j] == (uint32_t)-1 || max_delay[j] == (uint32_t)-1) {
                    valid = 0;
                    break;
                }
                int centre = (int)((min_delay[j] + max_delay[j]) / 2);
                int lo_margin = centre - (int)min_delay[j];
                int hi_margin = (int)max_delay[j] - centre;
                int margin = (lo_margin < hi_margin) ? lo_margin : hi_margin;
                if (margin < worst_margin) {
                    worst_margin = margin;
                }
                cand_rxd[c][j] = (uint8_t)centre;
            }
            if (!valid) {
                printf("RXDS%d 0x%x - no RXD window, skipping\n", s, c);
                continue;
            }

            /* Apply the owned lanes' RXD centres and validate real XIP bursts. */
            for (int j = lane0; j < lane1; j++) {
                cfg->rxd[j] = cand_rxd[c][j];
            }
            ospi_delay_cfg_rxd(ospi_ctx->aes, cfg->rxd);
            int mism = ospi_ctx->is_flash ? cal_validate_xip_flash(ospi_ctx)
                                          : cal_validate_xip(ospi_ctx);

            cand_valid[c] = 1;
            cand_worst[c] = worst_margin;
            cand_mism[c]  = mism;
            printf("RXDS%d 0x%x - worst RXD margin %d, XIP mismatches %d\n",
                   s, c, worst_margin, mism);
        }

        /* Usable window = candidates that both scored and validated clean. */
        uint32_t use_lo = (uint32_t)-1, use_hi = (uint32_t)-1;
        for (uint32_t c = rxds_lo[s]; c <= rxds_hi[s]; c++) {
            if (cand_valid[c] && cand_mism[c] == 0) {
                if (use_lo == (uint32_t)-1) {
                    use_lo = c;
                }
                use_hi = c;
            }
        }
        if (use_lo == (uint32_t)-1) {
            printf("RXDS/RX cal: no RXDS%d candidate validated clean\n", s);
            return 1;
        }

        const int mid2 = (int)(use_lo + use_hi); /* 2x window midpoint */
        uint32_t best_rxds  = use_lo;
        int      best_score = -1;
        int      best_dist2 = INT32_MAX;
        int      have_best  = 0;
        for (uint32_t c = use_lo; c <= use_hi; c++) {
            if (!cand_valid[c] || cand_mism[c] != 0) {
                continue;
            }
            int rxds_margin = (int)((c - use_lo) < (use_hi - c) ? (c - use_lo) : (use_hi - c));
            int score = (rxds_margin < cand_worst[c]) ? rxds_margin : cand_worst[c];
            int d = 2 * (int)c - mid2;
            int dist2 = (d < 0) ? -d : d;

            int better = 0;
            if (score > best_score) {
                better = 1;
            } else if (score == best_score) {
                if (dist2 < best_dist2) {
                    better = 1;              /* prefer more central RXDS */
                } else if (dist2 == best_dist2 && c > best_rxds) {
                    better = 1;              /* then prefer higher RXDS  */
                }
            }
            if (better) {
                best_score = score;
                best_dist2 = dist2;
                best_rxds  = c;
                have_best  = 1;
            }
        }
        if (!have_best) {
            printf("RXDS/RX cal: RXDS%d selection failed\n", s);
            return 1;
        }

        /* Apply this strobe's winner (delay + owned RXD lanes) before moving on
         * so the next strobe is refined against a settled "other" group. */
        rxds_delay[s] = (uint8_t)best_rxds;
        ospi_delay_cfg_rxds(ospi_ctx->aes, rxds_delay);
        for (int j = lane0; j < lane1; j++) {
            cfg->rxd[j] = cand_rxd[best_rxds][j];
        }
        ospi_delay_cfg_rxd(ospi_ctx->aes, cfg->rxd);

        int best_rxds_margin =
            (int)((best_rxds - use_lo) < (use_hi - best_rxds) ? (best_rxds - use_lo)
                                                              : (use_hi - best_rxds));
        printf("RXDS%d usable window 0x%x..0x%x; selected 0x%x "
               "(RXDS margin %d, worst RXD margin %d, score %d)\n",
               s, use_lo, use_hi, best_rxds, best_rxds_margin,
               cand_worst[best_rxds], best_score);
    }

    /* -------- 4. Finalize --------------------------------------------------- */
    cfg->rxds[0] = rxds_delay[0];
    /* In single-octal (x8) the 2nd RXDS strobe is unconnected; leave it zeroed. */
    cfg->rxds[1] = ospi_ctx->is_dual_octal ? rxds_delay[1] : 0;
    ospi_delay_cfg_rxds(ospi_ctx->aes, cfg->rxds);

    /* Re-measure per-lane windows with both strobes at their final values. */
    rx_sweep_windows(ospi_ctx, datalines, min_delay, max_delay);
    ospi_delay_cfg_rxd(ospi_ctx->aes, cfg->rxd);

    printf("RXDS/RX calibration done, RXDS = { 0x%x, 0x%x }\n",
           cfg->rxds[0], cfg->rxds[1]);
    for (int j = 0; j < datalines; j++) {
        printf("  RX %d delay -> 0x%x (window 0x%x..0x%x)\n", j,
               cfg->rxd[j], min_delay[j], max_delay[j]);
    }

    return 0;
}

/* Run the full delay calibration for one device and leave it running at the
 * calibrated high speed. Flash needs its read-only pattern in place before the
 * RX sweep and has no data-mask write path to tune. */
static int calibrate_device(ospi_cfg_t *ctx, ospi_delay_cfg_t *cfg, const char *name)
{
    printf("\n%s calibration\n", name);
    printf("--------------------------------------------\n");

    if (tx_delay_cal(ctx, cfg)) {
        printf("%s: Tx delay calibration failed\n", name);
        return -1;
    }

    if (ctx->is_flash) {
        /* The TXD sweep leaves an uncalibrated clock applied. */
        ospi_clk_cfg(ctx->regs, BAUD, DRIVE_EDGE);
        ospi_delay_cfg(ctx->aes, &delay_cfg_default);
        if (cal_program_flash_pattern(ctx) != 0) {
            printf("%s: calibration pattern programming failed\n", name);
            return -1;
        }
    }

    if (rxds_rx_delay_cal(ctx, cfg)) {
        printf("%s: RXDS/RX delay calibration failed\n", name);
        return -1;
    }

    if (!ctx->is_flash && tx_dm_delay_cal(ctx, cfg)) {
        printf("%s: Tx DM delay calibration failed\n", name);
        return -1;
    }

    ospi_clk_cfg(ctx->regs, BAUDR_FAST, DRIVE_EDGE_FAST);
    ospi_delay_cfg(ctx->aes, cfg);

    return 0;
}

#if FAST_INTEGRITY_CHECK
static uint32_t integrity_word(int off)
{
    return ((uint32_t)(off + 3) << 24) | ((uint32_t)(off + 2) << 16) |
           ((uint32_t)(off + 1) << 8) | (uint32_t)off;
}

static void integrity_write_pattern(ospi_cfg_t *ctx)
{
    for (int i = 0; i < 0x800; i += 4) {
        uint32_t data = integrity_word(i);
        ctx->write_data(ctx, i, &data, 1);
    }
}

/* Read the pattern back at whatever clock the caller has applied. */
static void integrity_verify_pattern(ospi_cfg_t *ctx)
{
    for (int i = 0; i < 0x800; i += 4) {
        uint32_t data = 0;
        uint32_t expected = integrity_word(i);

        if (ctx->read_data(ctx, i, &data, 1)) {
            printf("Failed read at addr: 0x%x\tContinuing...\t", i);
            if (ctx->read_data(ctx, i, &data, 1)) {
                printf("Retry failed...\n");
                continue;
            }
            printf("Retry succeeded!\n");
        }
        if (expected != data) {
            printf("Data mismatch: 0x%x 0x%x 0x%x 0x%x\n", i, expected, data, expected ^ data);
        }
    }
}
#endif

/* ==========================================================================
 * main
 * ========================================================================== */

int main(void)
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    if (stdout_init() != 0) {
        while (1) {
            __WFE();
        }
    }
#endif

    printf("XSPI calibration tool\n");

    // Initialize pinmuxes
    int32_t ret = board_pins_config();
    if (ret != 0) {
        printf("Error: board pins configuration failed. ret=%d\n", ret);
        goto wait_forever;
    }

    se_services_port_init();

    if (!init_power_management()) {
        printf("Error: power management init failed.\n");
    }

    configure_delay();

    ospi_cfg_t *ospi_ram = &ospi;
    ospi_ram->regs = OSPI0;
    ospi_ram->aes = AES0;
    ospi_ram->rx_buff[0] = 0;

    ospi_ram->is_dual_octal = BOARD_OSPI0_SPI_FRAME_FORMAT == 4;
    ospi_ram->calibrate_ssi_oe = false;

    enable_ospi_clk(OSPI_INSTANCE_0);
    ospi_clk_cfg(ospi_ram->regs, BAUD, DRIVE_EDGE);
    ospi_delay_cfg(ospi_ram->aes, &delay_cfg_default);

    /* Flash device context (OSPI1 / AES1) */
    ospi_cfg_t *flash_ctx = &ospi_flash;
    flash_ctx->regs = OSPI1;
    flash_ctx->aes = AES1;
    flash_ctx->rx_buff[0] = 0;
    flash_ctx->is_hyperram = false;
    flash_ctx->is_flash = true;
    flash_ctx->is_dual_octal = false;

    enable_ospi_clk(OSPI_INSTANCE_1);
    ospi_clk_cfg(flash_ctx->regs, BAUD, DRIVE_EDGE);
    ospi_delay_cfg(flash_ctx->aes, &delay_cfg_default);

    /* Probe the ISSI IS25W NOR flash. Optional: absence is not fatal so the
     * tool can still calibrate RAM-only boards. */
    bool have_flash = false;

#ifdef RTE_Drivers_ISSI_FLASH
    have_flash = (is25w_setup(flash_ctx) == 0);
    if (have_flash) {
        flash_ctx->read_data    = is25w_read_data;
        flash_ctx->write_data   = is25w_write_data;
        flash_ctx->erase_sector = is25w_erase_sector;
        printf("IS25W flash detected on OSPI1\n");
    } else {
        printf("No IS25W flash detected on OSPI1 (skipping flash calibration)\n");
    }
#endif

#ifdef RTE_Drivers_MX66UW1G_FLASH
    have_flash = (mx66uw1g_setup(flash_ctx) == 0);
    if (have_flash) {
        flash_ctx->read_data    = mx66uw1g_read_data;
        flash_ctx->write_data   = mx66uw1g_write_data;
        flash_ctx->erase_sector = mx66uw1g_erase_sector;
        printf("MX66UW1G flash detected on OSPI1\n");
    } else {
        printf("No MX66UW1G flash detected on OSPI1 (skipping flash calibration)\n");
    }
#endif

    /* Runtime external-RAM selection. Optional, like the flash, so a missing
     * RAM still lets the flash be calibrated. */
    bool have_ram = true;
    ospi_ram->is_hyperram = false;
    ospi_ram->dfs = 32;
    ospi_ram->xip_dfs = 32;
    ospi_ram->read_wait_cycles = APS512XXN_READ_WAIT_CYCLES - 1;
    ospi_ram->write_wait_cycles = APS512XXN_WRITE_WAIT_CYCLES - 1;
    if (aps512xxn_setup(ospi_ram) == 0) {
        /* APS512XXN pseudo-SRAM */
        ospi_ram->write_data = aps512xxn_write_data;
        ospi_ram->read_data = aps512xxn_read_data;
    } else {
        /* ISSI IS66 HyperRAM */
        ospi_ram->is_hyperram = true;
        ospi_ram->xip_dfs = 16;
        ospi_ram->read_wait_cycles = IS66_READ_WAIT_CYCLES;
        ospi_ram->write_wait_cycles = IS66_WRITE_WAIT_CYCLES;
        if (is66_setup(ospi_ram) != 0) {
            printf("No external RAM detected on OSPI0 (APS512XXN / IS66 HyperRAM)\n");
            have_ram = false;
        } else {
            ospi_ram->write_data = is66_write_data;
            ospi_ram->read_data = is66_read_data;
        }
    }

    bool ram_ok = have_ram && (calibrate_device(ospi_ram, &delay_cfg_ram, "RAM") == 0);

    /* Independent of the RAM result: the flash sits on its own controller. */
    bool flash_ok = have_flash &&
                    (calibrate_device(flash_ctx, &delay_cfg_flash, "Flash") == 0);

    if (ram_ok) {
#if XIP_MEMORY_TESTS
        /* The RAM tests exercise the PSRAM as normal cacheable memory */
        MPU_Set_OSPI0_XIP_Cacheable();

        ospi_xip_enter(ospi_ram);
        printf("\nLinear RAM test XIP (CPU cache enabled)\n");
        printf("---------------------------------------\n");
        ram_linear_test((uint8_t *) OSPI0_XIP_BASE, 1024 * 1024, 1);

        printf("\nRandom RAM test XIP (CPU cache enabled)\n");
        printf("---------------------------------------\n");
        ram_random_test((uint8_t *) OSPI0_XIP_BASE + (8*1024*1024), 0x20000, 1024 * 1024, 1);

        MPU_Set_OSPI0_XIP_Device();
        ospi_xip_exit(ospi_ram);
#endif

#if FAST_INTEGRITY_CHECK
        printf("\nFast integrity check (non-XIP)\n");
        printf("------------------------------\n");
        integrity_write_pattern(ospi_ram);

        printf("Read pattern at high speed\n");
        integrity_verify_pattern(ospi_ram);

        printf("Read again at low speed\n");
        ospi_clk_cfg(ospi_ram->regs, BAUD, DRIVE_EDGE);
        ospi_delay_cfg(ospi_ram->aes, &delay_cfg_default);
        integrity_verify_pattern(ospi_ram);
#endif
    }

#if XIP_MEMORY_TESTS
    if (flash_ok) {
        printf("\nFlash XIP read test (calibration region, CPU cache enabled)\n");
        printf("----------------------------------------------------------\n");
        flash_xip_read_test(flash_ctx, CAL_XIP_FLASH_PASSES);
    }
#endif

    /* Log the calibration results as pasteable C */
    if (ram_ok) {
        printf("\n/* Calibrated RAM OSPI delays */\n");
        ospi_delay_cfg_print(&delay_cfg_ram, "calibrated_delay_cfg");
        printf("\n");
    }
    if (flash_ok) {
        printf("\n/* Calibrated flash OSPI delays */\n");
        ospi_delay_cfg_print(&delay_cfg_flash, "flash_calibrated_delay_cfg");
        printf("\n");
    }

    /* Persist the results in the flash result sector for other applications. */
    if (flash_ok) {
        ospi_delay_cfg_t cfgs[2];
        uint32_t         count = 0;

        if (ram_ok) {
            cfgs[count++] = delay_cfg_ram;
        }
        cfgs[count++] = delay_cfg_flash;

        printf("\nStoring calibration results\n");
        printf("---------------------------\n");
        ospi_clk_cfg(flash_ctx->regs, BAUD, DRIVE_EDGE);
        ospi_delay_cfg(flash_ctx->aes, &delay_cfg_default);
        cal_store_delay_cfgs(flash_ctx, cfgs, count);
    }

wait_forever:
    while (1) {
        __WFE();
    }
}
