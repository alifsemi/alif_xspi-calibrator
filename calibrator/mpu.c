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

#include CMSIS_device_header

#include "app_mem_regions.h"


/* ==========================================================================
 * MPU region table override
 *
 * ========================================================================== */
void MPU_Load_Regions(void)
{
/* Define the memory attribute index with the below properties */
#define MEMATTRIDX_NORMAL_WT_RA_TRANSIENT 0
#define MEMATTRIDX_DEVICE_nGnRE           1
#define MEMATTRIDX_NORMAL_WB_RA_WA        2
#define MEMATTRIDX_NORMAL_WT_RA           3

    static const ARM_MPU_Region_t mpu_table[] = {
#if defined(APP_SRAM0_BASE) && defined(APP_SRAM1_BASE)
#if defined(SRAM0_SRAM1_COMBINED) && (SRAM0_SRAM1_COMBINED == 1)
        {/* SRAM - 8MB : RO-0, NP-1, XN-0 */
         .RBAR = ARM_MPU_RBAR(APP_SRAM_BASE, ARM_MPU_SH_NON, 0, 1, 0),
         .RLAR = ARM_MPU_RLAR((APP_SRAM_BASE + APP_SRAM_SIZE - 1),
                              MEMATTRIDX_NORMAL_WT_RA_TRANSIENT)},
#else
        {/* SRAM0 - 4MB : RO-0, NP-1, XN-0 */
         .RBAR = ARM_MPU_RBAR(APP_SRAM0_BASE, ARM_MPU_SH_NON, 0, 1, 0),
         .RLAR = ARM_MPU_RLAR((APP_SRAM0_BASE + APP_SRAM0_SIZE - 1),
                              MEMATTRIDX_NORMAL_WT_RA_TRANSIENT)},
        {/* SRAM1 - 2.5MB : RO-0, NP-1, XN-0 */
         .RBAR = ARM_MPU_RBAR(APP_SRAM1_BASE, ARM_MPU_SH_NON, 0, 1, 0),
         .RLAR = ARM_MPU_RLAR((APP_SRAM1_BASE + APP_SRAM1_SIZE - 1),
                              MEMATTRIDX_NORMAL_WB_RA_WA)},
#endif
#endif
        {/* Host Peripherals - 16MB : RO-0, NP-1, XN-1 */
         .RBAR = ARM_MPU_RBAR(0x1A000000, ARM_MPU_SH_NON, 0, 1, 1),
         .RLAR = ARM_MPU_RLAR(0x1AFFFFFF, MEMATTRIDX_DEVICE_nGnRE)},
#if defined(RTSS_HP)
        {/* RTSS HE ITCM - 256K(SRAM4) : RO-0, NP-1, XN-0  */
         .RBAR = ARM_MPU_RBAR(APP_SRAM4_BASE, ARM_MPU_SH_OUTER, 0, 1, 0),
         .RLAR = ARM_MPU_RLAR((APP_SRAM4_BASE + APP_SRAM4_SIZE - 1),
                              MEMATTRIDX_NORMAL_WB_RA_WA)},
        {/* RTSS HE DTCM - 256K(SRAM5) : RO-0, NP-1, XN-0  */
         .RBAR = ARM_MPU_RBAR(APP_SRAM5_BASE, ARM_MPU_SH_OUTER, 0, 1, 0),
         .RLAR = ARM_MPU_RLAR((APP_SRAM5_BASE + APP_SRAM5_SIZE - 1),
                              MEMATTRIDX_NORMAL_WB_RA_WA)},
#elif defined(RTSS_HE) && defined(APP_SRAM2_BASE) && defined(APP_SRAM3_BASE)
        {/* RTSS HP ITCM - 256K(SRAM2) : RO-0, NP-1, XN-0  */
         .RBAR = ARM_MPU_RBAR(APP_SRAM2_BASE, ARM_MPU_SH_OUTER, 0, 1, 0),
         .RLAR = ARM_MPU_RLAR((APP_SRAM2_BASE + APP_SRAM2_SIZE - 1),
                              MEMATTRIDX_NORMAL_WB_RA_WA)},
        {/* RTSS HP DTCM - 1MB(SRAM3) : RO-0, NP-1, XN-0  */
         .RBAR = ARM_MPU_RBAR(APP_SRAM3_BASE, ARM_MPU_SH_OUTER, 0, 1, 0),
         .RLAR = ARM_MPU_RLAR((APP_SRAM3_BASE + APP_SRAM3_SIZE - 1),
                              MEMATTRIDX_NORMAL_WB_RA_WA)},
#endif
        {/* MRAM - 5.5MB : RO-1, NP-1, XN-0  */
         .RBAR = ARM_MPU_RBAR(APP_MRAM_HE_BASE, ARM_MPU_SH_NON, 1, 1, 0),
         .RLAR = ARM_MPU_RLAR(APP_MRAM_USER_BASE - 1, MEMATTRIDX_NORMAL_WT_RA)},
        {/* MRAM User - 1.5MB: RO-0, NP-1, XN-1  */
         .RBAR = ARM_MPU_RBAR(APP_MRAM_USER_BASE, ARM_MPU_SH_NON, 0, 1, 1),
         .RLAR = ARM_MPU_RLAR((APP_MRAM_USER_BASE + APP_MRAM_USER_SIZE - 1),
                              MEMATTRIDX_DEVICE_nGnRE)},
        {/* OSPI Regs - 16MB : RO-0, NP-1, XN-1  */
         .RBAR = ARM_MPU_RBAR(0x83000000, ARM_MPU_SH_NON, 0, 1, 1),
         .RLAR = ARM_MPU_RLAR(0x83FFFFFF, MEMATTRIDX_DEVICE_nGnRE)},
        {/* OSPI0 XIP(eg:hyperram) - 512MB : RO-0, NP-1, XN-0
          * NOTE: Device-nGnRE (was NORMAL_WB_RA_WA) so sub-word XIP writes
          * used by tx_dm_delay_cal() bypass the cache and appear on the OSPI
          * bus as byte-masked writes. */
         .RBAR = ARM_MPU_RBAR(0xA0000000, ARM_MPU_SH_NON, 0, 1, 0),
         .RLAR = ARM_MPU_RLAR(0xBFFFFFFF, MEMATTRIDX_DEVICE_nGnRE)},
        {/* OSPI1 XIP(eg:flash) - 512MB : RO-1, NP-1, XN-0  */
         .RBAR = ARM_MPU_RBAR(0xC0000000, ARM_MPU_SH_NON, 1, 1, 0),
         .RLAR = ARM_MPU_RLAR(0xDFFFFFFF, MEMATTRIDX_NORMAL_WT_RA)},
    };

    /* Mem Attribute for 0th index */
    ARM_MPU_SetMemAttr(MEMATTRIDX_NORMAL_WT_RA_TRANSIENT,
                       ARM_MPU_ATTR(
                           /* NT=0, WB=0, RA=1, WA=0 */
                           ARM_MPU_ATTR_MEMORY_(0, 0, 1, 0),
                           ARM_MPU_ATTR_MEMORY_(0, 0, 1, 0)));

    /* Mem Attribute for 1st index */
    ARM_MPU_SetMemAttr(MEMATTRIDX_DEVICE_nGnRE,
                       ARM_MPU_ATTR(
                           /* Device Memory */
                           ARM_MPU_ATTR_DEVICE,
                           ARM_MPU_ATTR_DEVICE_nGnRE));

    /* Mem Attribute for 2nd index */
    ARM_MPU_SetMemAttr(MEMATTRIDX_NORMAL_WB_RA_WA,
                       ARM_MPU_ATTR(
                           /* NT=1, WB=1, RA=1, WA=1 */
                           ARM_MPU_ATTR_MEMORY_(1, 1, 1, 1),
                           ARM_MPU_ATTR_MEMORY_(1, 1, 1, 1)));

    /* Mem Attribute for 3th index */
    ARM_MPU_SetMemAttr(MEMATTRIDX_NORMAL_WT_RA,
                       ARM_MPU_ATTR(
                           /* NT=1, WB=0, RA=1, WA=0 */
                           ARM_MPU_ATTR_MEMORY_(1, 0, 1, 0),
                           ARM_MPU_ATTR_MEMORY_(1, 0, 1, 0)));

    /* Load the regions from the table */
    ARM_MPU_Load(0, mpu_table, sizeof(mpu_table) / sizeof(ARM_MPU_Region_t));
}

#if defined(APP_SRAM0_BASE) && defined(APP_SRAM1_BASE)
#if defined(SRAM0_SRAM1_COMBINED) && (SRAM0_SRAM1_COMBINED == 1)
#define MPU_SRAM_REGION_COUNT 1
#else
#define MPU_SRAM_REGION_COUNT 2
#endif
#else
#define MPU_SRAM_REGION_COUNT 0
#endif

#if defined(RTSS_HP) ||                                                         \
    (defined(RTSS_HE) && defined(APP_SRAM2_BASE) && defined(APP_SRAM3_BASE))
#define MPU_TCM_REGION_COUNT 2
#else
#define MPU_TCM_REGION_COUNT 0
#endif

#define MPU_OSPI0_XIP_REGION (MPU_SRAM_REGION_COUNT + 1 + MPU_TCM_REGION_COUNT + 3)

static void mpu_set_ospi0_xip_attr(uint8_t attr_idx)
{
    uint32_t ctrl = MPU->CTRL;

    __DSB();
    SCB_CleanInvalidateDCache();

    __DMB();
    MPU->CTRL = 0;
    __DSB();
    __ISB();

    ARM_MPU_SetRegion(MPU_OSPI0_XIP_REGION,
                      ARM_MPU_RBAR(0xA0000000, ARM_MPU_SH_NON, 0, 1, 0),
                      ARM_MPU_RLAR(0xBFFFFFFF, attr_idx));

    MPU->CTRL = ctrl;
    __DSB();
    __ISB();
}

/* Map the OSPI0 XIP window as Normal Write-Back Read/Write-Allocate cacheable
 * memory (the application-facing attribute). */
void MPU_Set_OSPI0_XIP_Cacheable(void)
{
    mpu_set_ospi0_xip_attr(MEMATTRIDX_NORMAL_WB_RA_WA);
}

/* Restore the OSPI0 XIP window to Device-nGnRE (needed by tx_dm_delay_cal). */
void MPU_Set_OSPI0_XIP_Device(void)
{
    mpu_set_ospi0_xip_attr(MEMATTRIDX_DEVICE_nGnRE);
}
