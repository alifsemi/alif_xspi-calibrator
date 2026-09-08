/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>

#include <RTE_Device.h>
#include <RTE_Components.h>
#include CMSIS_device_header

#include "ram_test.h"

/* Maximum randomized-test region. The check buffer below is statically sized
 * to this; ram_random_test() accepts any power-of-two size up to this value. */
#define RANDOM_TEST_MAX_SIZE 0x0020000

// Address bit test - primarily to check the address shim for parts that need it
void ram_address_test(volatile uint8_t *ram, uint32_t size)
{
    ram[0] = 'K';
    __DSB();
    ram[size - 1] = 'B';
    __DSB();
    for (unsigned bit = 0; (UINT32_C(1) << bit) < size; bit++) {
        ram[UINT32_C(1) << bit] = bit;
        __DSB();
    }
    unsigned errors = 0, good = 0;
    if (ram[0] != 'K')
    {
        printf("RAM start failure\n");
        errors++;
    }
    if (ram[size - 1] != 'B')
    {
        printf("RAM end failure\n");
        errors++;
    }
    for (unsigned bit = 0; (UINT32_C(1) << bit) < size; bit++) {
        if (ram[UINT32_C(1) << bit] != bit) {
            printf("RAM address bit %u failure\n", bit);
            errors++;
        } else {
            good++;
        }
    }
    if (errors == 0) {
        printf("%u address bits all good\n", good);
    }
}

int ram_linear_test(uint8_t *ram, uint32_t size, int verbose)
{
    if (verbose) {
        printf("Writing %uK\n", (unsigned)(size / 1024));
    }
    for (uint32_t i = 0; i < size; i++) {
        ram[i] = i;
    }
    if (verbose) {
        printf("Reading %uK\n", (unsigned)(size / 1024));
    }

    int err_cnt = 0;
    for (uint32_t i = 0; i < size; i++) {
        uint8_t expected = i;
        if (ram[i] != expected) {
            ++err_cnt;
            if (verbose) {
                if (err_cnt < 20) {
                    printf("Mismatch at %p: Got %02X expected %02X\n", (void*) &ram[i], ram[i], expected);
                } else if (err_cnt == 20) {
                    printf("...\n");
                }
            }
        }
    }
    if (verbose) {
        printf("Finished linear test (%u matches, %d mismatches)\n", (unsigned)size - err_cnt, err_cnt);
    }
    return err_cnt;
}

int ram_random_test(uint8_t *test_ram, uint32_t size, uint32_t iters, int verbose)
{
    static uint8_t check_ram[RANDOM_TEST_MAX_SIZE] __attribute__((section(".bss.large_ram"))) __attribute__((aligned(16)));

    if (size > RANDOM_TEST_MAX_SIZE || (size & (size - 1)) != 0) {
        printf("ram_random_test: invalid size %u (must be power of two <= %u)\n",
               (unsigned)size, (unsigned)RANDOM_TEST_MAX_SIZE);
        return -1;
    }

    if (verbose) {
        printf("Init test (%u iterations in %u bytes)\n", (unsigned)iters, (unsigned)size);
    }
    memset((uint8_t *)test_ram, 0xAA, size);
    memset(check_ram, 0xAA, size);
    if (verbose) {
        printf("Start\n");
    }
    for (uint32_t i = 0; i < iters; i++) {
        size_t pos = rand() & (size - 1);
        uint32_t val = rand();

        switch (rand() & 7) {
        case 0:
            check_ram[pos] = test_ram[pos] = val;
            break;
        case 1:
            if (pos + 2 > size) break;
            __UNALIGNED_UINT16_WRITE(check_ram+pos, val);
            __UNALIGNED_UINT16_WRITE(test_ram+pos, val);
            break;
        case 2:
            if (pos + 4 > size) break;
            __UNALIGNED_UINT32_WRITE(check_ram+pos, val);
            __UNALIGNED_UINT32_WRITE(test_ram+pos, val);
            break;
        case 3:
            if (pos + 11 > size) break;
            memset(check_ram+pos, val, 11);
            memset((uint8_t *)test_ram+pos, val, 11);
            break;
        case 4:
            if (pos + 3 > size) break;
            check_ram[pos] = check_ram[pos + 2] = val;
            test_ram[pos] = test_ram[pos + 2] = val;
            break;
        case 5:
            if (pos + 6 > size) break;
            check_ram[pos] = check_ram[pos + 5] = val;
            test_ram[pos] = val; test_ram[pos + 5] = val;
            break;
        case 6:
            if (pos + 7 > size) break;
            check_ram[pos] = test_ram[pos] = val;
            check_ram[pos + 3] = test_ram[pos + 3] = val;
            check_ram[pos + 6] = test_ram[pos + 6] = val;
            break;
        case 7:
            if (!(rand() & 0xF) && pos + 1024 <= size) {
                memset(check_ram+pos, val, 1024);
                memset((uint8_t *)test_ram+pos, val, 1024);
            } else {
                if (pos + 17 > size) break;
                check_ram[pos] = test_ram[pos] = test_ram[pos + 16];
            }
            break;
        }
    }

    int err_cnt = 0;
    for (uint32_t i = 0; i < size; i++) {
        if (test_ram[i] != check_ram[i]) {
            ++err_cnt;
            if (verbose) {
                if (err_cnt < 20) {
                    printf("Mismatch at %p: Got %02X expected %02X\n", (void*) &test_ram[i], test_ram[i], check_ram[i]);
                } else if (err_cnt == 20) {
                    printf("...\n");
                }
            }
        }
    }
    if (verbose) {
        printf("Test complete (%u matches, %d mismatches)\n", (unsigned)size - err_cnt, err_cnt);
    }
    return err_cnt;
}
