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
#include <inttypes.h>

#include "is25w.h"
#include "ospi.h"
#include "sys_utils.h"
#include "board_config.h"

#include "Driver_Flash.h"

extern ARM_DRIVER_FLASH ARM_Driver_Flash_(BOARD_OSPI_FLASH_INSTANCE);
static ARM_DRIVER_FLASH *ptrDrvFlash = &ARM_Driver_Flash_(BOARD_OSPI_FLASH_INSTANCE);


#ifndef DEBUG_PRINTS
#define DEBUG_PRINTS 0
#endif

/* In octal DDR the instruction phase is 16-bit (2 DDR beats): the 8-bit opcode
 * is duplicated into both bytes. */
#define IS25W_DUP(op)   ((uint32_t)(((uint32_t)(op) << 8) | (uint32_t)(op)))


#ifdef BOARD_OSPI_FLASH_RESET_GPIO_PORT

#include "Driver_IO.h"

#define IS25W_RESET_PORT    BOARD_OSPI_FLASH_RESET_GPIO_PORT
#define IS25W_RESET_PIN     BOARD_OSPI_FLASH_RESET_GPIO_PIN

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(IS25W_RESET_PORT);
static ARM_DRIVER_GPIO *const GPIODrv = &ARM_Driver_GPIO_(IS25W_RESET_PORT);

static int is25w_reset(void)
{
    int32_t ret = GPIODrv->Initialize(IS25W_RESET_PIN, NULL);
    if (ret != ARM_DRIVER_OK) { return ret; }

    ret = GPIODrv->PowerControl(IS25W_RESET_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) { return ret; }

    ret = GPIODrv->SetDirection(IS25W_RESET_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) { return ret; }

    ret = GPIODrv->SetValue(IS25W_RESET_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) { return ret; }
    sys_busy_loop_us(10);
    ret = GPIODrv->SetValue(IS25W_RESET_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) { return ret; }
    sys_busy_loop_us(100);

    return ret;
}
#else
static int is25w_reset(void)
{
    return 0;
}
#endif

/* Low-level octal-DDR read */
int is25w_read_data(ospi_cfg_t *ospi_ctx, uint32_t address, volatile uint32_t *data,
                    uint32_t length)
{
    const uint32_t fpw       = 32U / ospi_ctx->dfs; /* frames per word: 1 (dfs32) or 2 (dfs16) */
    const uint32_t frames    = length * fpw;
    const uint32_t item_mask = (ospi_ctx->dfs >= 32U) ? 0xFFFFFFFFU
                                                      : ((1U << ospi_ctx->dfs) - 1U);
    uint32_t got  = 0;
    uint32_t acc  = 0;
    int timeout = 16;

    /* CTRLR1 = frames - 1 (see note above: length*2 for 16-bit frames). */
    ospi_setup_read(ospi_ctx->regs, ospi_ctx->dfs, SPI_TRANS_TYPE_FRF_DEFINED,
                    SPI_INST_L_8_BIT, SPI_ADDR_L_32_BIT,
                    ospi_ctx->read_wait_cycles, false, frames);

    ospi_ctx->tx_buff[0] = IS25W_CMD_READ_DATA;
    ospi_ctx->tx_buff[1] = address;

    ospi_ctx->regs->OSPI_DR[0] = ospi_ctx->tx_buff[0];
    ospi_ctx->regs->OSPI_DR[0] = ospi_ctx->tx_buff[1];

    ospi_ctx->regs->OSPI_SER = 1;

    for (int wait = 0; wait < 500; wait++) {
        if (!(ospi_ctx->regs->OSPI_SR & SR_BUSY))
            break;
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
            uint32_t idx  = got % fpw;                    /* sub-word position   */
            acc |= item << (idx * ospi_ctx->dfs);         /* pack low half first */
            if (idx == fpw - 1U) {
                data[got / fpw] = acc;                    /* whole word assembled */
                acc = 0;
            }
            got++;
        }
        timeout--;
    } while ((got < frames) && (ospi_ctx->regs->OSPI_RXFLR > 0 || timeout));

    ospi_ctx->regs->OSPI_SER = 0;
    return 0;
}

/* Set up an octal-DDR transmit (SEND-only) transaction */
static void is25w_setup_cmd_send(OSPI_Type *regs, uint32_t dfs,
                                 uint32_t inst_len, uint32_t addr_len)
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
        | (inst_len << SPI_CTRLR0_INST_L_OFFSET)
        | (1U << SPI_CTRLR0_SPI_DDR_EN_OFFSET)
        | (SPI_CTRLR0_SPI_RXDS_ENABLE << SPI_CTRLR0_SPI_RXDS_EN_OFFSET);

    regs->OSPI_TXFTLR &= ~SPI_TXFTLR_TXFTHR_MASK;

    ospi_enable(regs);
}


static void is25w_setup_cmd_recv(OSPI_Type *regs, uint32_t dfs, uint32_t inst_len,
                                 uint32_t addr_len, uint32_t wait_cycles, uint32_t frames)
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
        | (inst_len << SPI_CTRLR0_INST_L_OFFSET)
        | (wait_cycles << SPI_CTRLR0_WAIT_CYCLES_OFFSET)
        | (1U << SPI_CTRLR0_SPI_DDR_EN_OFFSET)
        | (SPI_CTRLR0_SPI_RXDS_ENABLE << SPI_CTRLR0_SPI_RXDS_EN_OFFSET);

    regs->OSPI_TXFTLR &= ~SPI_TXFTLR_TXFTHR_MASK;

    ospi_enable(regs);
}

/* Wait for a transmit-only transaction to drain: TX FIFO empty and not busy. */
static int is25w_wait_tx_done(OSPI_Type *regs)
{
    for (int wait = 0; wait < 100000; wait++) {
        if ((regs->OSPI_SR & (SR_BUSY | SR_TF_EMPTY)) == SR_TF_EMPTY) {
            return 0;
        }
        sys_busy_loop_us(1);
    }
    return -1;
}

static int is25w_write_enable(ospi_cfg_t *ospi_ctx)
{
    OSPI_Type *regs = ospi_ctx->regs;

    is25w_setup_cmd_send(regs, ospi_ctx->dfs, SPI_INST_L_8_BIT, SPI_ADDR_L_0_BIT);

    regs->OSPI_DR[0] = IS25W_CMD_WRITE_ENABLE;
    regs->OSPI_SER   = 1;

    int ret          = is25w_wait_tx_done(regs);
    regs->OSPI_SER   = 0;

    return ret;
}

static int is25w_read_flag_status(ospi_cfg_t *ospi_ctx, uint8_t *value)
{
    OSPI_Type *regs = ospi_ctx->regs;
    int        ret  = -1;

    is25w_setup_cmd_recv(regs, ospi_ctx->dfs, SPI_INST_L_8_BIT, SPI_ADDR_L_0_BIT,
                         IS25W_STATUS_WAIT_CYCLES, 1U);

    regs->OSPI_DR[0] = IS25W_CMD_READ_FLAG_STATUS;
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

/* Poll the flag status register until the device reports ready */
static int is25w_wait_ready(ospi_cfg_t *ospi_ctx)
{
    OSPI_Type     *regs       = ospi_ctx->regs;
    const uint32_t saved_baud = regs->OSPI_BAUDR;
    const uint32_t saved_edge = regs->OSPI_DDR_DRIVE_EDGE;
    int            ret        = -1;

    ospi_clk_cfg(regs, IS25W_REG_WRITE_BAUD, IS25W_REG_WRITE_DRIVE_EDGE);

    for (int poll = 0; poll < 100000; poll++) {
        uint8_t val = 0;

        if (is25w_read_flag_status(ospi_ctx, &val) != 0) {
            break;
        }
        if (val & IS25W_FLAG_STATUS_BUSY) {
            ret = (val & IS25W_FLAG_STATUS_ERROR) ? -1 : 0;
            break;
        }
        sys_busy_loop_us(10);
    }

    ospi_clk_cfg(regs, saved_baud, saved_edge);
    return ret;
}

/* Program `length` 32-bit words at byte `address`  */
void is25w_write_data(ospi_cfg_t *ospi_ctx, uint32_t address, uint32_t *data, uint32_t length)
{
    OSPI_Type     *regs      = ospi_ctx->regs;
    const uint32_t fpw       = 32U / ospi_ctx->dfs;
    const uint32_t item_mask = (ospi_ctx->dfs >= 32U) ? 0xFFFFFFFFU
                                                      : ((1U << ospi_ctx->dfs) - 1U);
    uint32_t       done      = 0;

    while (done < length) {
        uint32_t addr       = address + done * 4U;
        uint32_t page_words = (IS25W_PAGE_SIZE - (addr % IS25W_PAGE_SIZE)) / 4U;
        uint32_t chunk      = ((length - done) < page_words) ? (length - done) : page_words;
        int      ret;

        if (is25w_write_enable(ospi_ctx) != 0) {
#if DEBUG_PRINTS
            printf("IS25W: write enable failed at 0x%" PRIx32 "\n", addr);
#endif
            break;
        }

        is25w_setup_cmd_send(regs, ospi_ctx->dfs, SPI_INST_L_8_BIT, SPI_ADDR_L_32_BIT);

        regs->OSPI_DR[0] = IS25W_CMD_PAGE_PROGRAM;
        regs->OSPI_DR[0] = addr;

        for (uint32_t i = 0; i < chunk; i++) {
            uint32_t word = data[done + i];
            for (uint32_t f = 0; f < fpw; f++) {
                regs->OSPI_DR[0] = (word >> (f * ospi_ctx->dfs)) & item_mask;
            }
        }

        regs->OSPI_SER = 1;
        ret            = is25w_wait_tx_done(regs);
        regs->OSPI_SER = 0;

        if (ret != 0 || is25w_wait_ready(ospi_ctx) != 0) {
#if DEBUG_PRINTS
            printf("IS25W: program failed at 0x%" PRIx32 "\n", addr);
#endif
            break;
        }

        done += chunk;
    }
}

/* Erase the 4KB sector  */
int is25w_erase_sector(ospi_cfg_t *ospi_ctx, uint32_t address)
{
    OSPI_Type     *regs       = ospi_ctx->regs;
    const uint32_t saved_baud = regs->OSPI_BAUDR;
    const uint32_t saved_edge = regs->OSPI_DDR_DRIVE_EDGE;
    int            ret;

    ospi_clk_cfg(regs, IS25W_REG_WRITE_BAUD, IS25W_REG_WRITE_DRIVE_EDGE);

    ret = is25w_write_enable(ospi_ctx);
    if (ret == 0) {
        is25w_setup_cmd_send(regs, ospi_ctx->dfs, SPI_INST_L_8_BIT, SPI_ADDR_L_32_BIT);

        regs->OSPI_DR[0] = IS25W_CMD_SECTOR_ERASE;
        regs->OSPI_DR[0] = address & ~(IS25W_SECTOR_SIZE - 1U);

        regs->OSPI_SER = 1;
        ret            = is25w_wait_tx_done(regs);
        regs->OSPI_SER = 0;

        if (ret == 0) {
            ret = is25w_wait_ready(ospi_ctx);
        }
    }

    ospi_clk_cfg(regs, saved_baud, saved_edge);

#if DEBUG_PRINTS
    if (ret != 0) {
        printf("IS25W: erase failed at 0x%" PRIx32 "\n", address);
    }
#endif

    return ret;
}

/* ==========================================================================
 * Setup / probe
 * ========================================================================== */

int is25w_setup(ospi_cfg_t *ospi_ctx)
{
    is25w_reset();
  
    int32_t ret = ptrDrvFlash->Initialize(NULL);  // Initialize ext-FLASH
    if (ret != ARM_DRIVER_OK) {
        printf("OSPI Flash: Init failed, error = %" PRIi32 "\n", ret);
        return ret;
    }

    ret = ptrDrvFlash->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("OSPI Flash: Power up failed, error = %" PRIi32 "\n", ret);
        return ret;
    }

    /* 5. Controller-side settings for this device. */
    ospi_ctx->is_flash          = true;
    ospi_ctx->is_hyperram       = false;
    ospi_ctx->calibrate_ssi_oe  = true;
    ospi_ctx->dfs               = 16;
    ospi_ctx->xip_dfs           = 16;
    ospi_ctx->read_wait_cycles  = RTE_ISSI_FLASH_WAIT_CYCLES;
    ospi_ctx->write_wait_cycles = IS25W_WRITE_WAIT_CYCLES;

    /* 6. XIP linear/wrap read opcodes (used when memory-mapped). */
    ospi_disable(ospi_ctx->regs);
    ospi_ctx->regs->OSPI_XIP_INCR_INST    = 0x0B0B;
    ospi_ctx->regs->OSPI_XIP_WRAP_INST    = 0x0B0B;
    ospi_ctx->regs->OSPI_XIP_CNT_TIME_OUT = 255;
    ospi_enable(ospi_ctx->regs);

    return 0;
}
