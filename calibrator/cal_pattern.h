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
 * On-device calibration pattern layout for flash delay calibration.
 *
 * Flash cannot be written cheaply during calibration (program/erase are slow
 * and 1->0 only), so the RX-path (RXDS/RXD) calibration reads back a fixed
 * pattern
 */
#ifndef CAL_PATTERN_H
#define CAL_PATTERN_H

#include <stdbool.h>
#include <stdint.h>
#include "RTE_Components.h"

/* Size of the calibration region: one 64KB sector/block at the end of flash. */
#define CAL_PATTERN_REGION_SIZE      (64U * 1024U)

/* Erase granularity used when (re)programming the region. Both supported
 * flashes (IS25W, MX66UW1G) have 4KB sectors. */
#define CAL_FLASH_SECTOR_SIZE        (4U * 1024U)

/* Total flash device size. IS25WX01G = 128 MiB. Override from the board/build
 * layer for other densities. */
#ifndef CAL_FLASH_SIZE

#ifdef RTE_Drivers_ISSI_FLASH
#define CAL_FLASH_SIZE               (128U * 1024U * 1024U)
#endif

#ifdef RTE_Drivers_MX66UW1G_FLASH
#define CAL_FLASH_SIZE               (128U * 1024U * 1024U)
#endif

#endif

/* -------- Calibration result blob -----------------------------------------
 * The calibrated delay configurations are stored in the LAST sector of the
 * flash (as an ospi_delay_blob_t) so another application can read them back
 * and run the buses at the calibrated clock. */
#ifndef CAL_RESULT_BASE_OFFSET
#define CAL_RESULT_BASE_OFFSET       (CAL_FLASH_SIZE - CAL_FLASH_SECTOR_SIZE)
#endif
#define CAL_RESULT_MAX_CFGS          4U

#ifndef CAL_PATTERN_BASE_OFFSET
#define CAL_PATTERN_BASE_OFFSET      (CAL_RESULT_BASE_OFFSET - CAL_PATTERN_REGION_SIZE)
#endif

/* Byte address of the pre-programmed lane-toggle block used by the read-only
 * (flash) RX-path calibration. */
#define CAL_LANE_TOGGLE_ADDR   (CAL_PATTERN_BASE_OFFSET + CAL_OFF_LANE_TOGGLE)

/* Scratch sector for the flash TXD sweep: erased once, then written at a
 * stepping address so no pass ever reprograms an already-written word. */
#define CAL_TX_SCRATCH_ADDR    (CAL_PATTERN_BASE_OFFSET - CAL_FLASH_SECTOR_SIZE)

/* Geometry recorded in the signature block (informational / self-describe). */
#ifndef CAL_PATTERN_LANES
#define CAL_PATTERN_LANES            8U   /* single-octal x8 flash              */
#endif
#ifndef CAL_PATTERN_DUMMY
#define CAL_PATTERN_DUMMY            16U  /* matches IS25W_READ_WAIT_CYCLES      */
#endif

/* -------- Block offsets (relative to CAL_PATTERN_BASE_OFFSET) -------------- */
#define CAL_OFF_SIGNATURE            0x0000U /* magic + geometry self-describe   */
#define CAL_OFF_LANE_TOGGLE          0x0040U /* RXDS + per-lane RXD windows       */
#define CAL_OFF_WALKING              0x0080U /* walking-1: per-line skew isolation*/
#define CAL_OFF_BYTE_PATTERN         0x00C0U /* per-byte-lane values              */
#define CAL_OFF_INCREMENT            0x0100U /* address-tagged burst/integrity    */
#define CAL_OFF_BURST_FILL           0x0200U /* region-filling sustained burst    */

/* -------- Signature block ------------------------------------------------- */
#define CAL_SIGNATURE_MAGIC          0xC0DECA1BU
#define CAL_SIGNATURE_VERSION        0x00000001U
/* word2 self-describes geometry: [7:0]=lane count, [15:8]=read dummy cycles */
#define CAL_SIGNATURE_GEOM(lanes, dummy) \
    (((uint32_t)(lanes) & 0xFFU) | (((uint32_t)(dummy) & 0xFFU) << 8))
#define CAL_SIGNATURE_MAGIC_INV      (~CAL_SIGNATURE_MAGIC)

/* -------- Lane-toggle block ----------------------------------------------- */
/* Every beat carries the low-16-bits-set word that lane_pattern() expects:
 * for data line j the mask selects bits {j, j+8, j+16, j+24} and the pass
 * value keeps only the bits below 16. 0x0000FFFF satisfies this for all lanes
 * in both x8 and x16 framing. */
#define CAL_LANE_WORD                0x0000FFFFU
#define CAL_LANE_WORD_COUNT          16U /* words in the block                   */

/* -------- Walking-1 block: word k = (1u << (k & 31)) ----------------------- */
#define CAL_WALKING_WORD(k)          ((uint32_t)1U << ((k) & 31U))
#define CAL_WALKING_WORD_COUNT       32U

/* -------- Byte-pattern block: distinct value per byte lane, rotated -------- */
/* Fixed per-word byte pattern (b0..b3) that rotates the {AA,55,CC,33} set so
 * every byte lane sees every value across the block. */
static inline uint32_t cal_byte_pattern_word(uint32_t k)
{
    static const uint8_t v[4] = {0xAAu, 0x55u, 0xCCu, 0x33u};
    uint32_t r = k & 3U;
    return ((uint32_t)v[(0 + r) & 3])
         | ((uint32_t)v[(1 + r) & 3] << 8)
         | ((uint32_t)v[(2 + r) & 3] << 16)
         | ((uint32_t)v[(3 + r) & 3] << 24);
}
#define CAL_BYTE_PATTERN_WORD_COUNT  16U

/* -------- Increment / burst block: address-tagged, "1->0 rich" ------------ */
/* Word at byte offset `off` = low half carries the offset, high half its
 * complement, so no word is all-ones (an erased-flash 0xFFFFFFFF fails). */
static inline uint32_t cal_increment_word(uint32_t off)
{
    uint32_t half = (off >> 2) & 0xFFFFU; /* word index */
    return half | ((~half & 0xFFFFU) << 16);
}

/* -------- Whole-region generator ------------------------------------------
 * Returns the expected 32-bit word at byte offset `off` (0 <= off <
 * CAL_PATTERN_REGION_SIZE) within the calibration region. Both the on-target
 * programmer (cal_program_flash_pattern) and every verifier derive their data
 * from this single function so the written image and the checks cannot drift.
 */
static inline uint32_t cal_pattern_word(uint32_t off)
{
    if (off < CAL_OFF_LANE_TOGGLE) {
        /* Signature block: magic / version / geometry / inverse-magic, then a
         * non-all-ones filler so an erased region never reads back as valid. */
        switch (off) {
        case CAL_OFF_SIGNATURE + 0x00U: return CAL_SIGNATURE_MAGIC;
        case CAL_OFF_SIGNATURE + 0x04U: return CAL_SIGNATURE_VERSION;
        case CAL_OFF_SIGNATURE + 0x08U:
            return CAL_SIGNATURE_GEOM(CAL_PATTERN_LANES, CAL_PATTERN_DUMMY);
        case CAL_OFF_SIGNATURE + 0x0CU: return CAL_SIGNATURE_MAGIC_INV;
        default:                        return cal_increment_word(off);
        }
    } else if (off < CAL_OFF_WALKING) {
        return CAL_LANE_WORD;                              /* lane-toggle block */
    } else if (off < CAL_OFF_BYTE_PATTERN) {
        return CAL_WALKING_WORD((off - CAL_OFF_WALKING) >> 2);
    } else if (off < CAL_OFF_INCREMENT) {
        return cal_byte_pattern_word((off - CAL_OFF_BYTE_PATTERN) >> 2);
    }
    return cal_increment_word(off);       /* increment + burst-fill tail block */
}

/* -------- Target-side programmer / verifier (cal_pattern.c) ---------------
 * Both need a live flash context with read_data/write_data installed, and a
 * controller clock the device can accept; the caller selects the clock. */
struct ospi_cfg;
struct ospi_delay_cfg;

/* Read the whole calibration region back and compare it against the generator.
 * `report` controls whether the first mismatch is printed. Returns 0 on match. */
int cal_verify_flash_pattern(struct ospi_cfg *flash_ctx, bool report);

/* Erase and program the calibration region, unless a full read-back already
 * matches. Returns 0 on success or already-present, -1 on error. */
int cal_program_flash_pattern(struct ospi_cfg *flash_ctx);

/* Store `count` calibrated delay configurations into the result sector as an
 * ospi_delay_blob_t and verify the read-back. Each entry's `idx` field tells
 * the consumer which OSPI controller it belongs to. The caller must have a
 * clock the device accepts selected; the write is not speed-calibrated.
 * Returns 0 on success, -1 on error. */
int cal_store_delay_cfgs(struct ospi_cfg *flash_ctx, const struct ospi_delay_cfg *cfgs,
                         uint32_t count);

#endif /* CAL_PATTERN_H */
