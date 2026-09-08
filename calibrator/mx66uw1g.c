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
#include <string.h>
#include <inttypes.h>

#include "mx66uw1g.h"
#include "ospi.h"
#include "sys_utils.h"
#include "board_config.h"

#if defined(RTE_Drivers_MX66UW1G_FLASH)

#include "RTE_Device.h"
#include "Driver_Flash.h"

extern ARM_DRIVER_FLASH ARM_Driver_Flash_(BOARD_OSPI_FLASH_INSTANCE);
static ARM_DRIVER_FLASH *ptrDrvFlash = &ARM_Driver_Flash_(BOARD_OSPI_FLASH_INSTANCE);

#ifndef DEBUG_PRINTS
#define DEBUG_PRINTS 0
#endif

/* ==========================================================================
 * Hardware reset (optional, board-gated exactly like is25w.c)
 * ========================================================================== */
#ifdef BOARD_OSPI_FLASH_RESET_GPIO_PORT

#include "Driver_IO.h"

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_OSPI_FLASH_RESET_GPIO_PORT);
static ARM_DRIVER_GPIO *const GPIODrv = &ARM_Driver_GPIO_(BOARD_OSPI_FLASH_RESET_GPIO_PORT);

static int mx66uw1g_reset(void)
{
    const uint8_t pin = BOARD_OSPI_FLASH_RESET_GPIO_PIN;

    int32_t ret = GPIODrv->Initialize(pin, NULL);
    if (ret != ARM_DRIVER_OK) { return ret; }

    ret = GPIODrv->PowerControl(pin, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) { return ret; }

    ret = GPIODrv->SetDirection(pin, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) { return ret; }

    ret = GPIODrv->SetValue(pin, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) { return ret; }
    sys_busy_loop_us(10);
    ret = GPIODrv->SetValue(pin, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) { return ret; }
    sys_busy_loop_us(100);

    return ret;
}
#else
static int mx66uw1g_reset(void)
{
    return 0;
}
#endif

/* ==========================================================================
 * Transaction framing
 * ========================================================================== */
static void mx66uw1g_setup_recv(OSPI_Type *regs, uint32_t dfs, uint32_t addr_len,
                                uint32_t wait_cycles, uint32_t frames)
{
    ospi_disable(regs);
    regs->OSPI_SER = 0;

    regs->OSPI_CTRLR0 = SPI_CTRLR0_SPI_OCTAL_ENABLE
                      | SPI_CTRLR0_TMOD_RECEIVE_ONLY
                      | ((dfs - 1) << SPI_CTRLR0_DFS);
    regs->OSPI_CTRLR1 = frames - 1U;

    regs->OSPI_SPI_CTRLR0 =
          SPI_TRANS_TYPE_FRF_DEFINED
        | (addr_len << SPI_CTRLR0_ADDR_L_OFFSET)
        | (SPI_INST_L_16_BIT << SPI_CTRLR0_INST_L_OFFSET)
        | (wait_cycles << SPI_CTRLR0_WAIT_CYCLES_OFFSET)
        | (1U << SPI_CTRLR0_SPI_DDR_EN_OFFSET)
        | (1U << SPI_CTRLR0_INST_DDR_EN_OFFSET)
        | (SPI_CTRLR0_SPI_RXDS_ENABLE << SPI_CTRLR0_SPI_RXDS_EN_OFFSET);

    /* The DFP leaves the TX start threshold at the item count of its last
     * transfer; a shorter one would never start. Start as soon as SER is set. */
    regs->OSPI_TXFTLR &= ~SPI_TXFTLR_TXFTHR_MASK;

    ospi_enable(regs);
}

static void mx66uw1g_setup_send(OSPI_Type *regs, uint32_t dfs, uint32_t addr_len)
{
    ospi_disable(regs);
    regs->OSPI_SER = 0;

    regs->OSPI_CTRLR0 = SPI_CTRLR0_SPI_OCTAL_ENABLE
                      | SPI_CTRLR0_TMOD_SEND_ONLY
                      | ((dfs - 1) << SPI_CTRLR0_DFS);
    regs->OSPI_CTRLR1 = 0;

    regs->OSPI_SPI_CTRLR0 =
          SPI_TRANS_TYPE_FRF_DEFINED
        | (addr_len << SPI_CTRLR0_ADDR_L_OFFSET)
        | (SPI_INST_L_16_BIT << SPI_CTRLR0_INST_L_OFFSET)
        | (1U << SPI_CTRLR0_SPI_DDR_EN_OFFSET)
        | (1U << SPI_CTRLR0_INST_DDR_EN_OFFSET)
        | (SPI_CTRLR0_SPI_RXDS_ENABLE << SPI_CTRLR0_SPI_RXDS_EN_OFFSET);
        /* NOTE: no SPI_CTRLR0_SPI_DM_EN - a flash command has no DM lane. */

    regs->OSPI_TXFTLR &= ~SPI_TXFTLR_TXFTHR_MASK;

    ospi_enable(regs);
}

/* Wait for a transmit-only transaction to drain: TX FIFO empty and not busy. */
static int mx66uw1g_wait_tx_done(OSPI_Type *regs)
{
    for (int wait = 0; wait < 100000; wait++) {
        if ((regs->OSPI_SR & (SR_BUSY | SR_TF_EMPTY)) == SR_TF_EMPTY) {
            return 0;
        }
        sys_busy_loop_us(1);
    }
    return -1;
}


/* Low-level octal-DDR read  */
int mx66uw1g_read_data(ospi_cfg_t *ospi_ctx, uint32_t address, volatile uint32_t *data,
                       uint32_t length)
{
    const uint32_t fpw       = 32U / ospi_ctx->dfs;
    const uint32_t frames    = length * fpw;
    const uint32_t item_mask = (ospi_ctx->dfs >= 32U) ? 0xFFFFFFFFU
                                                      : ((1U << ospi_ctx->dfs) - 1U);
    uint32_t got     = 0;
    uint32_t acc     = 0;
    int      timeout = 16;

    mx66uw1g_setup_recv(ospi_ctx->regs, ospi_ctx->dfs, SPI_ADDR_L_32_BIT,
                        ospi_ctx->read_wait_cycles, frames);

    ospi_ctx->regs->OSPI_DR[0] = MX66UW1G_CMD_READ_DATA;
    ospi_ctx->regs->OSPI_DR[0] = address;

    ospi_ctx->regs->OSPI_SER = 1;

    for (int wait = 0; wait < 500; wait++) {
        if (!(ospi_ctx->regs->OSPI_SR & SR_BUSY)) {
            break;
        }
        sys_busy_loop_us(1);
    }
    if (ospi_ctx->regs->OSPI_SR & SR_BUSY) {
        ospi_ctx->regs->OSPI_SER = 0;
        return -1;
    }

    while (ospi_ctx->regs->OSPI_RXFLR == 0)
        ;

    do {
        if (ospi_ctx->regs->OSPI_RXFLR > 0) {
            uint32_t item = ospi_ctx->regs->OSPI_DR[0] & item_mask;
            uint32_t idx  = got % fpw;
            acc |= item << (idx * ospi_ctx->dfs);
            if (idx == fpw - 1U) {
                data[got / fpw] = acc;
                acc = 0;
            }
            got++;
        }
        timeout--;
    } while ((got < frames) && (ospi_ctx->regs->OSPI_RXFLR > 0 || timeout));

    ospi_ctx->regs->OSPI_SER = 0;
    return 0;
}

/* Read one 8-bit register (RDSR / RDSCUR). Both take a 32-bit dummy address
 * and the fixed status latency; the device repeats the register byte on every
 * beat, so the low byte of the first frame is the value. */
static int mx66uw1g_read_reg(ospi_cfg_t *ospi_ctx, uint32_t command, uint8_t *value)
{
    OSPI_Type *regs = ospi_ctx->regs;
    int        ret  = -1;

    mx66uw1g_setup_recv(regs, ospi_ctx->dfs, SPI_ADDR_L_32_BIT,
                        MX66UW1G_STATUS_WAIT_CYCLES, 1U);

    regs->OSPI_DR[0] = command;
    regs->OSPI_DR[0] = 0;
    regs->OSPI_SER   = 1;

    for (int wait = 0; wait < 100000; wait++) {
        if (regs->OSPI_RXFLR > 0) {
            *value = (uint8_t)regs->OSPI_DR[0];
            ret    = 0;
            break;
        }
        sys_busy_loop_us(1);
    }

    regs->OSPI_SER = 0;
    return ret;
}

static int mx66uw1g_write_enable(ospi_cfg_t *ospi_ctx)
{
    OSPI_Type *regs = ospi_ctx->regs;

    mx66uw1g_setup_send(regs, ospi_ctx->dfs, SPI_ADDR_L_0_BIT);

    regs->OSPI_DR[0] = MX66UW1G_CMD_WRITE_ENABLE;
    regs->OSPI_SER   = 1;

    int ret        = mx66uw1g_wait_tx_done(regs);
    regs->OSPI_SER = 0;

    return ret;
}

/* Poll the status register  */
static int mx66uw1g_wait_ready(ospi_cfg_t *ospi_ctx, uint8_t err_flags)
{
    OSPI_Type     *regs       = ospi_ctx->regs;
    const uint32_t saved_baud = regs->OSPI_BAUDR;
    const uint32_t saved_edge = regs->OSPI_DDR_DRIVE_EDGE;
    int            ret        = -1;

    ospi_clk_cfg(regs, MX66UW1G_REG_WRITE_BAUD, MX66UW1G_REG_WRITE_DRIVE_EDGE);

    for (int poll = 0; poll < 100000; poll++) {
        uint8_t val = 0;

        if (mx66uw1g_read_reg(ospi_ctx, MX66UW1G_CMD_READ_STATUS, &val) != 0) {
            break;
        }
        if ((val & (MX66UW1G_STATUS_WIP | MX66UW1G_STATUS_WEL)) == 0U) {
            ret = 0;
            break;
        }
        sys_busy_loop_us(10);
    }

    if (ret == 0) {
        uint8_t sec = 0;

        if (mx66uw1g_read_reg(ospi_ctx, MX66UW1G_CMD_READ_SECURITY, &sec) != 0 ||
            (sec & err_flags)) {
            ret = -1;
        }
    }

    ospi_clk_cfg(regs, saved_baud, saved_edge);
    return ret;
}

/* Program `length` 32-bit words at byte `address` */
void mx66uw1g_write_data(ospi_cfg_t *ospi_ctx, uint32_t address, uint32_t *data,
                         uint32_t length)
{
    OSPI_Type     *regs      = ospi_ctx->regs;
    const uint32_t fpw       = 32U / ospi_ctx->dfs;
    const uint32_t item_mask = (ospi_ctx->dfs >= 32U) ? 0xFFFFFFFFU
                                                      : ((1U << ospi_ctx->dfs) - 1U);
    uint32_t       done      = 0;

    while (done < length) {
        uint32_t addr       = address + done * 4U;
        uint32_t page_words = (MX66UW1G_PAGE_SIZE - (addr % MX66UW1G_PAGE_SIZE)) / 4U;
        uint32_t chunk      = ((length - done) < page_words) ? (length - done) : page_words;
        int      ret;

        if (mx66uw1g_write_enable(ospi_ctx) != 0) {
#if DEBUG_PRINTS
            printf("MX66UW1G: write enable failed at 0x%" PRIx32 "\n", addr);
#endif
            break;
        }

        mx66uw1g_setup_send(regs, ospi_ctx->dfs, SPI_ADDR_L_32_BIT);

        regs->OSPI_DR[0] = MX66UW1G_CMD_PAGE_PROGRAM;
        regs->OSPI_DR[0] = addr;

        for (uint32_t i = 0; i < chunk; i++) {
            uint32_t word = data[done + i];
            for (uint32_t f = 0; f < fpw; f++) {
                regs->OSPI_DR[0] = (word >> (f * ospi_ctx->dfs)) & item_mask;
            }
        }

        regs->OSPI_SER = 1;
        ret            = mx66uw1g_wait_tx_done(regs);
        regs->OSPI_SER = 0;

        if (ret != 0 || mx66uw1g_wait_ready(ospi_ctx, MX66UW1G_SECURITY_P_FAIL) != 0) {
#if DEBUG_PRINTS
            printf("MX66UW1G: program failed at 0x%" PRIx32 "\n", addr);
#endif
            break;
        }

        done += chunk;
    }
}

/* Erase the 4KB sector  */
int mx66uw1g_erase_sector(ospi_cfg_t *ospi_ctx, uint32_t address)
{
    OSPI_Type     *regs       = ospi_ctx->regs;
    const uint32_t saved_baud = regs->OSPI_BAUDR;
    const uint32_t saved_edge = regs->OSPI_DDR_DRIVE_EDGE;
    int            ret;

    ospi_clk_cfg(regs, MX66UW1G_REG_WRITE_BAUD, MX66UW1G_REG_WRITE_DRIVE_EDGE);

    ret = mx66uw1g_write_enable(ospi_ctx);
    if (ret == 0) {
        mx66uw1g_setup_send(regs, ospi_ctx->dfs, SPI_ADDR_L_32_BIT);

        regs->OSPI_DR[0] = MX66UW1G_CMD_SECTOR_ERASE;
        regs->OSPI_DR[0] = address & ~(MX66UW1G_SECTOR_SIZE - 1U);

        regs->OSPI_SER = 1;
        ret            = mx66uw1g_wait_tx_done(regs);
        regs->OSPI_SER = 0;

        if (ret == 0) {
            ret = mx66uw1g_wait_ready(ospi_ctx, MX66UW1G_SECURITY_E_FAIL);
        }
    }

    ospi_clk_cfg(regs, saved_baud, saved_edge);

#if DEBUG_PRINTS
    if (ret != 0) {
        printf("MX66UW1G: erase failed at 0x%" PRIx32 "\n", address);
    }
#endif

    return ret;
}

/* ==========================================================================
 * Setup / probe
 * ========================================================================== */

int mx66uw1g_setup(ospi_cfg_t *ospi_ctx)
{
    mx66uw1g_reset();

    int32_t ret = ptrDrvFlash->Initialize(NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("OSPI Flash: Init failed, error = %" PRIi32 "\n", ret);
        return ret;
    }

    ret = ptrDrvFlash->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("OSPI Flash: Power up failed, error = %" PRIi32 "\n", ret);
        return ret;
    }

    /* Controller-side settings for this device. */
    ospi_ctx->is_flash          = true;
    ospi_ctx->is_hyperram       = false;
    ospi_ctx->calibrate_ssi_oe  = true;
    ospi_ctx->dfs               = 16;
    ospi_ctx->xip_dfs           = 16;
    ospi_ctx->read_wait_cycles  = RTE_MX66UW1G_FLASH_WAIT_CYCLES;
    ospi_ctx->write_wait_cycles = MX66UW1G_WRITE_WAIT_CYCLES;

    /* XIP linear/wrap read opcodes (used when memory-mapped). */
    ospi_disable(ospi_ctx->regs);
    ospi_ctx->regs->OSPI_XIP_INCR_INST    = MX66UW1G_CMD_READ_DATA;
    ospi_ctx->regs->OSPI_XIP_WRAP_INST    = MX66UW1G_CMD_READ_DATA;
    ospi_ctx->regs->OSPI_XIP_CNT_TIME_OUT = 255;
    ospi_enable(ospi_ctx->regs);

    return 0;
}

#endif /* RTE_Drivers_MX66UW1G_FLASH */
