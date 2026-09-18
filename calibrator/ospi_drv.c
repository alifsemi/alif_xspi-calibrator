/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include "ospi_drv.h"
#include "soc_features.h" /* SOC_FEAT_AES_OSPI_HAS_XIP_WRITE_HC_DFS */

#include "sys_ctrl_aes.h" 
#include <stdio.h>

/* Setup OSPI for reading.
 */
void ospi_setup_read(OSPI_Type *ospi, uint32_t dfs,
                     uint32_t frf_type, uint32_t inst_len,
                     uint32_t addr_len, uint32_t wait_cycles, bool hyperbus, uint32_t length)
{
    uint32_t t;

    ospi_disable(ospi);
    ospi->OSPI_SER = 0;

    t = SPI_CTRLR0_SPI_OCTAL_ENABLE                   /* SPI_FRF = octal               */
        | SPI_CTRLR0_TMOD_RECEIVE_ONLY                /* Receive-only mode             */
        | ((dfs - 1) << SPI_CTRLR0_DFS);
    if (hyperbus) {
        t |= SPI_CTRLR0_SPI_HYPERBUS_ENABLE;          /* HyperBus framing (HyperRAM)   */
    }
    ospi->OSPI_CTRLR0 = t;
    ospi->OSPI_CTRLR1 = length - 1;

    t = frf_type                                                       /* OctalSPI transfer type */
        | (addr_len << SPI_CTRLR0_ADDR_L_OFFSET)                       /* Address length         */
        | (inst_len << SPI_CTRLR0_INST_L_OFFSET)                       /* Instruction length     */
        | (wait_cycles << SPI_CTRLR0_WAIT_CYCLES_OFFSET)               /* Wait/Dummy cycles      */
        | (1U << SPI_CTRLR0_SPI_DDR_EN_OFFSET)                         /* SPI DDR Enable         */
        | (SPI_CTRLR0_SPI_RXDS_ENABLE << SPI_CTRLR0_SPI_RXDS_EN_OFFSET); /* Data Strobe Enable   */

    ospi->OSPI_SPI_CTRLR0 = t;

    ospi_enable(ospi);
}

/* Setup OSPI for writing. */
void ospi_setup_write(OSPI_Type *ospi, uint32_t dfs, uint32_t frf_type, uint32_t inst_len,
                      uint32_t addr_len, uint32_t wait_cycles, bool hyperbus)
{
    volatile uint32_t t;

    ospi_disable(ospi);
    ospi->OSPI_SER = 0;

    t = SPI_CTRLR0_SPI_OCTAL_ENABLE                   /* SPI_FRF = octal               */
        | SPI_CTRLR0_TMOD_SEND_ONLY                   /* Transmit-only mode            */
        | ((dfs - 1) << SPI_CTRLR0_DFS);
    if (hyperbus) {
        t |= SPI_CTRLR0_SPI_HYPERBUS_ENABLE;          /* HyperBus framing (HyperRAM)   */
    }
    ospi->OSPI_CTRLR0 = t;

    t = frf_type
        | (addr_len << SPI_CTRLR0_ADDR_L_OFFSET)
        | (inst_len << SPI_CTRLR0_INST_L_OFFSET)
        | (wait_cycles << SPI_CTRLR0_WAIT_CYCLES_OFFSET)
        | (1U << SPI_CTRLR0_SPI_DDR_EN_OFFSET)
        | (1U << SPI_CTRLR0_SPI_DM_EN_OFFSET);
    
    ospi->OSPI_SPI_CTRLR0 = t;

    ospi_enable(ospi);
}

void ospi_clk_cfg(OSPI_Type *ospi, uint32_t clk_div, uint32_t drive_edge)
{
    ospi_disable(ospi);
    ospi->OSPI_BAUDR = clk_div;
    ospi->OSPI_DDR_DRIVE_EDGE = drive_edge;
    ospi_enable(ospi);
}

/* Set OSPI XIP configuration */
void ospi_xip_cfg(OSPI_Type *ospi,
                  uint32_t dfs,
                  uint32_t read_wait_cycles,
                  uint32_t write_wait_cycles,
                  bool is_dual_octal,
                  bool is_hyperram)
{
    uint8_t trans_type;
    uint32_t val;

    if (is_dual_octal) {
        trans_type = SPI_TRANS_TYPE_FRF_DUAL_OCTAL;
    } else {
        trans_type = SPI_TRANS_TYPE_FRF_DEFINED;
    }

    ospi_disable(ospi);
    val = (1 << SPI_CTRLR0_SSI_IS_MST)
        | (SPI_FRAME_FORMAT_OCTAL << SPI_CTRLR0_SPI_FRF)
        | SPI_CTRLR0_SCPOL_LOW
        | SPI_CTRLR0_SCPH_LOW
        | (0 << SPI_CTRLR0_SSTE)
        | (SPI_TMOD_RX << SPI_CTRLR0_TMOD)
        | ((dfs - 1) << SPI_CTRLR0_DFS);

    ospi->OSPI_CTRLR0 = val;

    if (is_hyperram) {

        /* XIP read: HyperBus, RWDS-signalled strobe, chosen DFS framing. */
        ospi->OSPI_XIP_CTRL =
              (1U << XIP_CTRL_XIP_HYPERBUS_EN_OFFSET)
            | (0U << XIP_CTRL_RXDS_SIG_EN_OFFSET)
            | ((uint32_t)read_wait_cycles << XIP_CTRL_WAIT_CYCLES_OFFSET)
            | (1U << XIP_CTRL_DFS_HC_OFFSET)
            | (trans_type << XIP_CTRL_TRANS_TYPE_OFFSET);

        ospi->OSPI_XIP_WRITE_CTRL =
              (1U << XIP_WRITE_CTRL_XIPWR_HYPERBUS_EN_OFFSET)
            | (1U << XIP_WRITE_CTRL_XIPWR_DM_EN_OFFSET)
            | (0U << XIP_WRITE_CTRL_XIPWR_RXDS_SIG_EN_OFFSET)
            | (trans_type << XIP_WRITE_CTRL_WR_TRANS_TYPE_OFFSET)
#if (SOC_FEAT_AES_OSPI_HAS_XIP_WRITE_HC_DFS)
            | (1U << XIP_WRITE_CTRL_XIPWR_DFS_HC_OFFSET)
#endif
            | ((uint32_t)write_wait_cycles << XIP_WRITE_CTRL_XIPWR_WAIT_CYCLES);

        ospi_enable(ospi);

        return;
    }

    val = (SPI_FRAME_FORMAT_OCTAL << XIP_CTRL_FRF_OFFSET)
            | (trans_type << XIP_CTRL_TRANS_TYPE_OFFSET)
            | (SPI_ADDR_L_32_BIT << XIP_CTRL_ADDR_L_OFFSET)
            | (SPI_INST_L_16_BIT << XIP_CTRL_INST_L_OFFSET)
            | (0x0 << XIP_CTRL_MD_BITS_EN_OFFSET)
            | (read_wait_cycles << XIP_CTRL_WAIT_CYCLES_OFFSET)
            | (0x1 << XIP_CTRL_DFS_HC_OFFSET)
            | (0x1 << XIP_CTRL_DDR_EN_OFFSET)
            | (0x1 << XIP_CTRL_INST_DDR_EN_OFFSET)
            | (0x1 << XIP_CTRL_RXDS_EN_OFFSET)
            | (0x1 << XIP_CTRL_INST_EN_OFFSET)
            | (0x0 << XIP_CTRL_CONT_XFER_EN_OFFSET)
            | (0x0 << XIP_CTRL_XIP_HYPERBUS_EN_OFFSET)
            | (0x0 << XIP_CTRL_RXDS_SIG_EN_OFFSET)
            | (0x0 << XIP_CTRL_XIP_MBL_OFFSET)
            | (0x0 << XIP_CTRL_XIP_PREFETCH_EN_OFFSET)
            | (0x0 << XIP_CTRL_RXDS_VL_EN_OFFSET);

    ospi->OSPI_XIP_CTRL = val;

    val = (SPI_FRAME_FORMAT_OCTAL << XIP_WRITE_CTRL_WR_FRF_OFFSET)
            | (trans_type << XIP_WRITE_CTRL_WR_TRANS_TYPE_OFFSET)
            | (SPI_ADDR_L_32_BIT << XIP_WRITE_CTRL_WR_ADDR_L_OFFSET)
            | (SPI_INST_L_16_BIT << XIP_WRITE_CTRL_WR_INST_L_OFFSET)
            | (0x1 << XIP_WRITE_CTRL_WR_SPI_DDR_EN_OFFSET)
            | (0x1 << XIP_WRITE_CTRL_WR_INST_DDR_EN_OFFSET)
            | (0x0 << XIP_WRITE_CTRL_XIPWR_HYPERBUS_EN_OFFSET)
            | (0x0 << XIP_WRITE_CTRL_XIPWR_RXDS_SIG_EN_OFFSET)
            | (0x1 << XIP_WRITE_CTRL_XIPWR_DM_EN_OFFSET)
#if (SOC_FEAT_AES_OSPI_HAS_XIP_WRITE_HC_DFS)
            | (0x1 << XIP_WRITE_CTRL_XIPWR_DFS_HC_OFFSET)
#endif
            | (write_wait_cycles << XIP_WRITE_CTRL_XIPWR_WAIT_CYCLES);

    ospi->OSPI_XIP_WRITE_CTRL = val;

    ospi_enable(ospi);
}

/* Enter memory-mapped XIP mode */
void ospi_xip_enter(ospi_cfg_t *ospi_ctx)
{
    /* Enter XIP mode (dual-octal x16) with the device's write wait cycles. */
    ospi_xip_cfg(ospi_ctx->regs,
                 ospi_ctx->xip_dfs,
                 ospi_ctx->read_wait_cycles,
                 ospi_ctx->write_wait_cycles,
                 ospi_ctx->is_dual_octal, ospi_ctx->is_hyperram);

    ospi_ctx->regs->OSPI_TXFTLR &= ~0xFFFFU; /* TFT = 0 */

    ospi_disable(ospi_ctx->regs);
    ospi_ctx->regs->OSPI_SER = (1U << 0); /* SS0 */

    /* Match the known-good flash XIP reference: clear the controller RX sample
     * delay (the AES taps do the fine sampling) and the XIP mode bits, and -
     * critically - select the slave for XIP transactions via OSPI_XIP_SER.
     * XIP uses its OWN slave-enable register, separate from OSPI_SER; without
     * it, memory-mapped reads target no device and return bus garbage on every
     * word regardless of the AES delay taps. */
    ospi_ctx->regs->OSPI_RX_SAMPLE_DELAY = 0;
    ospi_ctx->regs->OSPI_XIP_MODE_BITS   = 0;
#if defined(SOC_FEAT_OSPI_HAS_XIP_SER) && SOC_FEAT_OSPI_HAS_XIP_SER
    ospi_ctx->regs->OSPI_XIP_SER = (1U << 0); /* SS0 */
#endif

    ospi_enable(ospi_ctx->regs);

    aes_enable_xip(ospi_ctx->aes);
    __DSB();
}


/* Leave memory-mapped XIP mode  */
void ospi_xip_exit(ospi_cfg_t *ospi_ctx)
{
    aes_disable_xip(ospi_ctx->aes);
    ospi_disable(ospi_ctx->regs);
    ospi_ctx->regs->OSPI_SER = 0;
    ospi_ctx->regs->OSPI_XIP_CTRL = 0;
    ospi_ctx->regs->OSPI_XIP_WRITE_CTRL = 0;
    ospi_enable(ospi_ctx->regs);
}

// TODO: Remove when the delay configuration functions are available in DFP 
void ospi_delay_cfg_rxd(AES_Type *aes, const uint8_t delay_val[16])
{
    aes->AES_RXD_DELAY_0 =
        ((uint32_t)delay_val[3] << 24) | ((uint32_t)delay_val[2] << 16) | ((uint32_t)delay_val[1] << 8) | ((uint32_t)delay_val[0] << 0);
    aes->AES_RXD_DELAY_1 =
        ((uint32_t)delay_val[7] << 24) | ((uint32_t)delay_val[6] << 16) | ((uint32_t)delay_val[5] << 8) | ((uint32_t)delay_val[4] << 0);
    aes->AES_RXD_DELAY_2 =
        ((uint32_t)delay_val[11] << 24) | ((uint32_t)delay_val[10] << 16) | ((uint32_t)delay_val[9] << 8) | ((uint32_t)delay_val[8] << 0);
    aes->AES_RXD_DELAY_3 =
        ((uint32_t)delay_val[15] << 24) | ((uint32_t)delay_val[14] << 16) | ((uint32_t)delay_val[13] << 8) | ((uint32_t)delay_val[12] << 0);
}

void ospi_delay_cfg_txd(AES_Type *aes, const uint8_t delay_val[16])
{
    aes->AES_TXD_DELAY_0 =
        ((uint32_t)delay_val[3] << 24) | ((uint32_t)delay_val[2] << 16) | ((uint32_t)delay_val[1] << 8) | ((uint32_t)delay_val[0] << 0);
    aes->AES_TXD_DELAY_1 =
        ((uint32_t)delay_val[7] << 24) | ((uint32_t)delay_val[6] << 16) | ((uint32_t)delay_val[5] << 8) | ((uint32_t)delay_val[4] << 0);
    aes->AES_TXD_DELAY_2 =
        ((uint32_t)delay_val[11] << 24) | ((uint32_t)delay_val[10] << 16) | ((uint32_t)delay_val[9] << 8) | ((uint32_t)delay_val[8] << 0);
    aes->AES_TXD_DELAY_3 =
        ((uint32_t)delay_val[15] << 24) | ((uint32_t)delay_val[14] << 16) | ((uint32_t)delay_val[13] << 8) | ((uint32_t)delay_val[12] << 0);
}

void ospi_delay_cfg_ssioen(AES_Type *aes, const uint8_t delay_val[16])
{
    aes->AES_SSI_OE_N_DELAY_0 =
        ((uint32_t)delay_val[3] << 24) | ((uint32_t)delay_val[2] << 16) | ((uint32_t)delay_val[1] << 8) | ((uint32_t)delay_val[0] << 0);
    aes->AES_SSI_OE_N_DELAY_1 =
        ((uint32_t)delay_val[7] << 24) | ((uint32_t)delay_val[6] << 16) | ((uint32_t)delay_val[5] << 8) | ((uint32_t)delay_val[4] << 0);
    aes->AES_SSI_OE_N_DELAY_2 =
        ((uint32_t)delay_val[11] << 24) | ((uint32_t)delay_val[10] << 16) | ((uint32_t)delay_val[9] << 8) | ((uint32_t)delay_val[8] << 0);
    aes->AES_SSI_OE_N_DELAY_3 =
        ((uint32_t)delay_val[15] << 24) | ((uint32_t)delay_val[14] << 16) | ((uint32_t)delay_val[13] << 8) | ((uint32_t)delay_val[12] << 0);
}

void ospi_delay_cfg_rxds(AES_Type *aes, const uint8_t rxds_delay[2])
{
    aes->AES_RXDS_DELAY = (uint32_t)rxds_delay[0] | ((uint32_t)rxds_delay[1] << 8);
}

/* Program the two TXD data-mask (DM) */
void ospi_delay_cfg_txddm(AES_Type *aes, const uint8_t txddm_delay[2])
{
    uint32_t reg = aes->AES_TXD_DM_DELAY;
    reg &= ~(((uint32_t)0x1Fu << AES_TXD_DM_0_DELAY_POS) |
             ((uint32_t)0x1Fu << AES_TXD_DM_1_DELAY_POS));
    reg |= ((uint32_t)txddm_delay[0] << AES_TXD_DM_0_DELAY_POS) |
           ((uint32_t)txddm_delay[1] << AES_TXD_DM_1_DELAY_POS);
    aes->AES_TXD_DM_DELAY = reg;
}

/* Apply a full TXD + RXD delay configuration by delegating to the per-path
 * helpers. */
void ospi_delay_cfg(AES_Type *aes, const ospi_delay_cfg_t *cfg)
{
    ospi_delay_cfg_txd(aes, cfg->txd);
    ospi_delay_cfg_rxd(aes, cfg->rxd);
    ospi_delay_cfg_ssioen(aes, cfg->ssioen);
    ospi_delay_cfg_rxds(aes, cfg->rxds);
    ospi_delay_cfg_txddm(aes, cfg->txddm);
}

/* Print a full 16-tap delay array as a C initializer body, e.g. "0, 1, ...". */
static void print_u8_array(const uint8_t *arr, int count)
{
    for (int i = 0; i < count; i++) {
        printf("%s0x%02x", (i == 0) ? "" : ", ", arr[i]);
    }
}

/* Print an ospi_delay_cfg_t as a pasteable C designated initializer.
 *
 * The emitted text is valid C that reconstructs the given configuration, e.g.
 *
 *   ospi_delay_cfg_t <name> = {
 *       .idx    = 0,
 *       .txd    = { 0x04, 0x04, ... },
 *       .rxd    = { 0x03, 0x04, ... },
 *       .ssioen = { 0x00, ... },
 *       .rxds   = { 0x08, 0x08 },
 *       .txddm  = { 0x07, 0x00 },
 *   };
 *
 * If `name` is NULL a default identifier is used. */
void ospi_delay_cfg_print(const ospi_delay_cfg_t *cfg, const char *name)
{
    if (cfg == NULL) {
        return;
    }
    if (name == NULL) {
        name = "calibrated_delay_cfg";
    }

    printf("ospi_delay_cfg_t %s = {\n", name);

    printf("    .idx    = %u,\n", (unsigned)cfg->idx);
    printf("    .sclk_freq = %u,\n", (unsigned)cfg->sclk_freq);

    printf("    .txd    = { ");
    print_u8_array(cfg->txd, 16);
    printf(" },\n");

    printf("    .rxd    = { ");
    print_u8_array(cfg->rxd, 16);
    printf(" },\n");

    printf("    .ssioen = { ");
    print_u8_array(cfg->ssioen, 16);
    printf(" },\n");

    printf("    .rxds   = { ");
    print_u8_array(cfg->rxds, 2);
    printf(" },\n");

    printf("    .txddm  = { ");
    print_u8_array(cfg->txddm, 2);
    printf(" },\n");

    printf("    .dmoen  = { ");
    print_u8_array(cfg->dmoen, 2);
    printf(" },\n");

    printf("    .sclk   = 0x%02x,\n", cfg->sclk);
    printf("    .sclkn  = 0x%02x,\n", cfg->sclkn);

    printf("    .ssn    = { ");
    print_u8_array(cfg->ssn, 2);
    printf(" },\n");

    printf("};\n");
}
