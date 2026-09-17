# alif_xspi-calibrator

Calibrates the OSPI/HEXSPI (xSPI) delay lines on Alif Ensemble E8|E4 (Cortex-M55) so external PSRAM/HyperRAM and flash devices can use high bus clock (200 MHz).
On DevKit-E8 External RAM is auto-detected: AP Memory APS512XXN PSRAM or ISSI IS66 HyperRAM. Supports octal (x8) and dual-octal (x16) frame formats.

- Tested with GCC 13.3.1
- Targets: `DevKit-E8`, `AppKit-E8`
- Supported flash devices are ISSI IS25WX01G and MX66UW1G
- Build with CMSIS csolution 
- Calibration output and RAM-test results are printed to retargeted UART as a pasteable `ospi_delay_cfg_t` C initializer
- Output is also saved to external flash so that other applications can use the calibrated values

> **Note:** The flash device signal delay parameter calibration (TXD) modifies flash contents and may destroy existing data. The last 72KiB at the end of the flash is used as TXD tuning scratch buffer, RXD calibration pattern storage and finally for the calibration result. Calibration result is written at the start of the last 4KiB sector (0x07FFF000 on a typical 128MiB device). Even though calibration knowingly changes only the mentioned area it is possible that other areas are touched due to the nature of the calibration (In order to find correct timings a range of signal timings are sweeped including invalid and outside of the normal working values).

> **Note:** This is an example, not a production-grade solution. It has not been validated over a wide temperature range or across a large number of boards. Treat the delay values it produces as a starting point and qualify them for your own hardware and operating conditions.

## Calibration flow

The tool trains each delay group using a known pattern, picks the centre of each passing window, then validates the result.
In flash calibration flow the TXD test values are not written to a single address as in RAM case. Instead a scratch sector is erased and the values are written to sequential addresses.
The datamask signal calibration is skipped completely for flash and the memory tests also differ. The RAM is tested more thoroughly while the flash final test is based on a 64KiB XIP pattern read.

- **TXD** — delay swept 0–23 across all data lines together; passing window measured for each data line (8 or 16) and each line set to its own mid-window value.
- **RXDS/RX** — per-strobe sweep with RXD lanes centred, scored by worst-lane margin and validated against live XIP bursts.
- **TX DM** — data-mask write delay; x8 uses the intersection across all 4 byte lanes, x16 sweeps DM0/DM1 independently.
- **Validation** — XIP linear + random RAM tests (cacheable) and a non-XIP high/low-speed integrity check.

## Calibration output structure

**Delay configuration**

```
typedef struct ospi_delay_cfg {
    uint32_t idx;       /* OSPI controller the config belongs to (0 or 1) */
    uint32_t sclk_freq; /* OSPI controller SCLK frequency */
    uint8_t txd[16];    /* per-line TXD delay taps (index 0..15)     */
    uint8_t rxd[16];    /* per-line RXD delay taps (index 0..15)     */
    uint8_t ssioen[16]; /* per-line OE_N delay taps (index 0..15)    */
    uint8_t rxds[2];    /* RXDS strobe delays (index 0..1)           */
    uint8_t txddm[2];   /* TXD DM data delays (index 0..1)           */
} ospi_delay_cfg_t;

```

**Container for the calibration results (Saved to 0x07FFF000 on a typical 128MiB device)**

```
#define OSPI_DELAY_BLOB_MAGIC   0xFA57C10CU

typedef struct {
    uint32_t         magic;   /* OSPI_DELAY_BLOB_MAGIC                  */
    uint32_t         count;   /* number of delay configurations stored  */
    ospi_delay_cfg_t cfg[];   /* `count` entries                        */
} ospi_delay_blob_t;

```
