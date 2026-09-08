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
#include <string.h>

#include "cal_pattern.h"
#include "ospi_drv.h"


#define CAL_VERIFY_CHUNK    2U

int cal_verify_flash_pattern(ospi_cfg_t *flash_ctx, bool report)
{
    uint32_t buf[CAL_VERIFY_CHUNK];

    for (uint32_t off = 0; off < CAL_PATTERN_REGION_SIZE; off += CAL_VERIFY_CHUNK * 4U) {
        if (flash_ctx->read_data(flash_ctx, CAL_PATTERN_BASE_OFFSET + off, buf,
                                 CAL_VERIFY_CHUNK) != 0) {
            if (report) {
                printf("  verify read failed at +0x%04lX\n", (unsigned long)off);
            }
            return -1;
        }
        for (uint32_t i = 0; i < CAL_VERIFY_CHUNK; i++) {
            uint32_t word_off = off + i * 4U;
            uint32_t exp = cal_pattern_word(word_off);
            if (buf[i] != exp) {
                if (report) {
                    printf("  verify failed at +0x%04lX: got 0x%08lX exp 0x%08lX\n",
                           (unsigned long)word_off, (unsigned long)buf[i],
                           (unsigned long)exp);
                }
                return -1;
            }
        }
    }
    return 0;
}

int cal_program_flash_pattern(ospi_cfg_t *flash_ctx)
{
    static uint32_t sector_buf[CAL_FLASH_SECTOR_SIZE / 4U];

    if (cal_verify_flash_pattern(flash_ctx, false) == 0) {
        printf("Calibration pattern already present at 0x%08lX\n",
               (unsigned long)CAL_PATTERN_BASE_OFFSET);
        return 0;
    }

    printf("Programming calibration pattern at 0x%08lX (%lu bytes)...\n",
           (unsigned long)CAL_PATTERN_BASE_OFFSET,
           (unsigned long)CAL_PATTERN_REGION_SIZE);

    /* Erase the whole region, one 4KB sector at a time. */
    for (uint32_t off = 0; off < CAL_PATTERN_REGION_SIZE; off += CAL_FLASH_SECTOR_SIZE) {
        if (flash_ctx->erase_sector(flash_ctx, CAL_PATTERN_BASE_OFFSET + off) != 0) {
            printf("  erase failed at +0x%04lX\n", (unsigned long)off);
            return -1;
        }
    }

    /* Fill each sector in RAM from the shared generator, then program it. */
    for (uint32_t base = 0; base < CAL_PATTERN_REGION_SIZE; base += CAL_FLASH_SECTOR_SIZE) {
        for (uint32_t i = 0; i < CAL_FLASH_SECTOR_SIZE / 4U; i++) {
            sector_buf[i] = cal_pattern_word(base + i * 4U);
        }
        flash_ctx->write_data(flash_ctx, CAL_PATTERN_BASE_OFFSET + base,
                              sector_buf, CAL_FLASH_SECTOR_SIZE / 4U);
    }

    if (cal_verify_flash_pattern(flash_ctx, true) != 0) {
        return -1;
    }

    printf("Calibration pattern verified (%lu bytes).\n",
           (unsigned long)CAL_PATTERN_REGION_SIZE);
    return 0;
}

int cal_store_delay_cfgs(ospi_cfg_t *flash_ctx, const ospi_delay_cfg_t *cfgs, uint32_t count)
{
    static uint32_t blob_buf[(sizeof(ospi_delay_blob_t) +
                              CAL_RESULT_MAX_CFGS * sizeof(ospi_delay_cfg_t)) / 4U];
    ospi_delay_blob_t *blob = (ospi_delay_blob_t *)blob_buf;
    uint32_t words;

    if (count == 0U || count > CAL_RESULT_MAX_CFGS) {
        return -1;
    }

    blob->magic = OSPI_DELAY_BLOB_MAGIC;
    blob->count = count;
    memcpy(blob->cfg, cfgs, count * sizeof(ospi_delay_cfg_t));

    words = (uint32_t)(sizeof(ospi_delay_blob_t) + count * sizeof(ospi_delay_cfg_t)) / 4U;

    printf("Storing %lu calibrated delay set(s) at 0x%08lX...\n",
           (unsigned long)count, (unsigned long)CAL_RESULT_BASE_OFFSET);

    if (flash_ctx->erase_sector(flash_ctx, CAL_RESULT_BASE_OFFSET) != 0) {
        printf("  result sector erase failed\n");
        return -1;
    }

    flash_ctx->write_data(flash_ctx, CAL_RESULT_BASE_OFFSET, blob_buf, words);

    for (uint32_t i = 0; i < words; i += CAL_VERIFY_CHUNK) {
        uint32_t buf[CAL_VERIFY_CHUNK];
        uint32_t chunk = ((words - i) < CAL_VERIFY_CHUNK) ? (words - i) : CAL_VERIFY_CHUNK;

        if (flash_ctx->read_data(flash_ctx, CAL_RESULT_BASE_OFFSET + i * 4U, buf, chunk) != 0) {
            printf("  result verify read failed at +0x%04lX\n", (unsigned long)(i * 4U));
            return -1;
        }
        for (uint32_t j = 0; j < chunk; j++) {
            if (buf[j] != blob_buf[i + j]) {
                printf("  result verify failed at +0x%04lX: got 0x%08lX exp 0x%08lX\n",
                       (unsigned long)((i + j) * 4U), (unsigned long)buf[j],
                       (unsigned long)blob_buf[i + j]);
                return -1;
            }
        }
    }

    printf("Calibration results stored and verified (%lu bytes).\n",
           (unsigned long)(words * 4U));
    return 0;
}
