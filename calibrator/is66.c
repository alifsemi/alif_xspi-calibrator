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

#include "is66.h"
#include "ospi.h"
#include "gpio.h"
#include "sys_utils.h"
#include "board_config.h"

#ifndef DEBUG_PRINTS
#define DEBUG_PRINTS 0
#endif


#define IS66_CA_CR0_WRITE_HI  0x60000100U   /* W, reg, linear, reg-addr 0x01 */
#define IS66_CA_CR0_READ_HI   0xE0000100U   /* R, reg, linear, reg-addr 0x01 */
#define IS66_CA_ID0_READ_HI   0xE0000000U   /* R, reg, linear, reg-addr 0x00 */
#define IS66_CA_REG_LO        0x00000000U

/* ==========================================================================
 * Hardware reset
 * ========================================================================== */
#ifdef BOARD_IS66_HYPERRAM_RESET_GPIO_PORT

#include "pinconf.h"
#include "Driver_IO.h"

#define OSPI_RESET_PORT     BOARD_IS66_HYPERRAM_RESET_GPIO_PORT
#define OSPI_RESET_PIN      BOARD_IS66_HYPERRAM_RESET_GPIO_PIN

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(OSPI_RESET_PORT);
static ARM_DRIVER_GPIO* const GPIODrv = &ARM_Driver_GPIO_(OSPI_RESET_PORT);

static int is66_reset(void)
{
    int32_t ret = GPIODrv->Initialize(OSPI_RESET_PIN, NULL);
    if (ret != ARM_DRIVER_OK) { return ret; }

    ret = GPIODrv->PowerControl(OSPI_RESET_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) { return ret; }

    ret = GPIODrv->SetDirection(OSPI_RESET_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) { return ret; }

    ret = GPIODrv->SetValue(OSPI_RESET_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) { return ret; }
    sys_busy_loop_us(10);
    ret = GPIODrv->SetValue(OSPI_RESET_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) { return ret; }

    return ret;
}
#else

static int is66_reset(void)
{
    return 0;
}

#endif

/* ==========================================================================
 * Register access (DFP HyperBus helpers)
 * ========================================================================== */

static void is66_write_reg(ospi_cfg_t *ospi_ctx, uint32_t ca_hi, uint32_t ca_lo, uint16_t data)
{
    uint32_t        buff[3] = {ca_hi, ca_lo, data};
    ospi_transfer_t xfer;

    memset(&xfer, 0, sizeof(xfer));
    xfer.spi_frf        = SPI_FRF_OCTAL;
    xfer.ddr            = 1;
    xfer.inst_len       = SPI_INST_L_0_BIT;
    xfer.addr_len       = SPI_ADDR_L_48_BIT;
    xfer.dummy_cycle    = 0; /* register writes have zero initial latency */
    xfer.tx_total_cnt   = 3; /* 2 CA words + 1 data word                  */
    xfer.tx_current_cnt = 0;
    xfer.tx_buff        = buff;

    ospi_set_dfs(ospi_ctx->regs, 16);
    ospi_ctx->regs->OSPI_SER = 1; /* select the slave for the DFP polling loop */
    ospi_hyperbus_send(ospi_ctx->regs, &xfer);
    ospi_ctx->regs->OSPI_SER = 0;
}

static uint16_t is66_read_reg(ospi_cfg_t *ospi_ctx, uint32_t ca_hi, uint32_t ca_lo,
                              uint8_t initial_latency)
{
    uint32_t        buff[2] = {ca_hi, ca_lo};
    uint16_t        val     = 0;
    ospi_transfer_t xfer;

    memset(&xfer, 0, sizeof(xfer));
    xfer.spi_frf        = SPI_FRF_OCTAL;
    xfer.ddr            = 1;
    xfer.inst_len       = SPI_INST_L_0_BIT;
    xfer.addr_len       = SPI_ADDR_L_48_BIT;
    xfer.dummy_cycle    = initial_latency; /* ospi_hyperbus_receive() doubles this */
    xfer.rx_total_cnt   = 1;
    xfer.rx_current_cnt = 0;
    xfer.rx_buff        = &val;
    xfer.tx_buff        = buff;            /* the two CA words */

    ospi_set_dfs(ospi_ctx->regs, 16);
    ospi_ctx->regs->OSPI_SER = 1;
    ospi_hyperbus_receive(ospi_ctx->regs, &xfer);
    ospi_ctx->regs->OSPI_SER = 0;

    return val;
}

/* ==========================================================================
 * Memory data path (direct HyperBus programming)
 * ========================================================================== */

/* Build the 48-bit memory-space Command-Address for byte `address`, split as
 * { CA[47:16], CA[15:0] } to load into the controller FIFO.
 *
 *   CA[47]    R/W#  (1 = read, 0 = write)
 *   CA[46]    0     (memory space)
 *   CA[45]    1     (linear burst)
 *   CA[44:16] word address A31:A3   (HyperBus is 16-bit-word addressed)
 *   CA[2:0]   word address A2:A0
 */
static void is66_build_mem_ca(uint32_t address, int is_read, uint32_t ca[2])
{
    uint32_t word_addr = address >> 1; /* byte -> 16-bit word address */

    ca[0] = ((uint32_t)(is_read ? 1U : 0U) << 31) /* CA[47] R/W#            */
          | (1U << 29)                            /* CA[45] linear burst    */
          | ((word_addr >> 3) & 0x1FFFFFFFU);     /* CA[44:16] A31:A3       */
    ca[1] = word_addr & 0x7U;                     /* CA[15:0]  (A2:A0)      */
}

void is66_write_data(ospi_cfg_t *ospi_ctx, uint32_t address, uint32_t *data, uint32_t length)
{
    uint32_t ca[2];
    uint32_t l;

    is66_build_mem_ca(address, 0, ca);

    ospi_setup_write(ospi_ctx->regs,
                     ospi_ctx->dfs,
                     SPI_TRANS_TYPE_FRF_DEFINED, SPI_INST_L_0_BIT,
                     SPI_ADDR_L_48_BIT, IS66_WRITE_WAIT_CYCLES, true);

    ospi_ctx->regs->OSPI_DR[0] = ca[0];
    ospi_ctx->regs->OSPI_DR[0] = ca[1];

    l = 0;
    while (l < length) {
        ospi_ctx->regs->OSPI_DR[0] = data[l];
        l++;
    }

    ospi_ctx->regs->OSPI_SER = 1;

    while (ospi_ctx->regs->OSPI_SR & SR_BUSY)
        ;
    while ((ospi_ctx->regs->OSPI_SR & SR_TF_EMPTY) != SR_TF_EMPTY)
        ;

    ospi_ctx->regs->OSPI_SER = 0;
}

int is66_read_data(ospi_cfg_t *ospi_ctx, uint32_t address, volatile uint32_t *data, uint32_t length)
{
    uint32_t ca[2];
    uint32_t l;
    int      timeout = 16;

    is66_build_mem_ca(address, 1, ca);

    ospi_setup_read(ospi_ctx->regs, ospi_ctx->dfs, SPI_TRANS_TYPE_FRF_DEFINED, SPI_INST_L_0_BIT,
                    SPI_ADDR_L_48_BIT, IS66_READ_WAIT_CYCLES, true, length);

    ospi_ctx->regs->OSPI_DR[0] = ca[0];
    ospi_ctx->regs->OSPI_DR[0] = ca[1];

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

    l = 0;

    /* Wait until there is valid data in the FIFO */
    while (ospi_ctx->regs->OSPI_RXFLR == 0)
        ;

    do {
        if (ospi_ctx->regs->OSPI_RXFLR > 0) {
            *data++ = ospi_ctx->regs->OSPI_DR[0];
            l++;
        }
        timeout--;
    } while ((ospi_ctx->regs->OSPI_RXFLR > 0) && timeout);

    ospi_ctx->regs->OSPI_SER = 0;

    (void)l;
    return 0;
}

/* ==========================================================================
 * Setup
 * ========================================================================== */

int is66_setup(ospi_cfg_t *ospi_ctx)
{
    uint16_t cr0;
    uint16_t cr0_rd;
    uint16_t id0;
    int      ok;

    if (ospi_ctx->is_dual_octal) {
        printf("IS66: dual-octal is not supported (x8 device)\n");
        return -1;
    }

    if (is66_reset() != 0) {
        printf("IS66: reset failed\n");
        return -1;
    }

    cr0 = (uint16_t)((1U << 15)
                     | ((uint32_t)IS66_DRIVE_STRENGTH << 12)
                     | (0xFU << 8)
                     | ((((uint32_t)IS66_INITIAL_LATENCY - 5U) & 0xFU) << 4)
                     | (1U << 3)   /* fixed 2x latency  */
                     | (1U << 2)
                     | (3U << 0)); /* 32-byte wrap */

    is66_write_reg(ospi_ctx, IS66_CA_CR0_WRITE_HI, IS66_CA_REG_LO, cr0);

    cr0_rd = is66_read_reg(ospi_ctx, IS66_CA_CR0_READ_HI, IS66_CA_REG_LO, IS66_INITIAL_LATENCY);
    id0    = is66_read_reg(ospi_ctx, IS66_CA_ID0_READ_HI, IS66_CA_REG_LO, IS66_INITIAL_LATENCY);

    printf("IS66 setup: wrote CR0=0x%04x, read back CR0=0x%04x, ID0=0x%04x\n", cr0, cr0_rd, id0);

    ok = (cr0_rd == cr0) || ((id0 & 0xFU) == IS66_ID0_MANUFACTURER_ISSI);
    if (!ok) {
        printf("IS66 not detected\n");
        return -1;
    }

    return 0;
}
