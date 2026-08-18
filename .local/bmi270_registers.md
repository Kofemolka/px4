# Bosch BMI270 — register map & device addressing

Source: official datasheet, BST-BMI270-DS000-08, Rev 1.6, March 2026
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf
(register map: chapter 5, p.71-120; addressing: chapter 6, p.121-131)

Cross-checked against the addresses already used by PX4's existing driver
(`src/drivers/imu/bosch/bmi270/Bosch_BMI270_registers.hpp`) - they match.

## Device addressing

- **Chip ID** (reg 0x00, read-only): `0x24`
- **I2C**: 7-bit, default `0x68` (0b1101000) when SDO pin -> GND;
  alternate `0x69` (0b1101001) when SDO pin -> VDDIO. Same scheme as
  BMI160. Std/Fast/Fast+ modes up to 1 MHz.
- **SPI**: 4-wire (default) or 3-wire (set via `IF_CONF.spi3=1`), up to
  10 MHz. Protocol auto-selected by CSB pin behavior after power-up (same
  as BMI160): tied to VDDIO -> I2C; rising edge seen -> SPI (switches
  ~200us after power-up, until next reset). First byte of a transaction:
  bit7 = R/W (0=write,1=read), bits6:0 = 7-bit register address. Recommend
  a dummy SPI read of CHIP_ID (0x00) before the first real transaction -
  the value obtained on that first read is invalid.
- Interrupt pins: INT1 (pin 4), INT2 (pin 9) - same pinout as BMI160.

## Register map (address, name) — from datasheet chapter 5 TOC

| Addr        | Name           |
|-------------|----------------|
| 0x00        | CHIP_ID        |
| 0x02        | ERR_REG        |
| 0x03        | STATUS         |
| 0x04-0x17   | DATA_0..DATA_19 (mag/aux x2, gyr x2, acc x2 per axis, LSB then MSB - see note) |
| 0x18-0x1A   | SENSORTIME_0..2 |
| 0x1B        | EVENT          |
| 0x1C        | INT_STATUS_0   |
| 0x1D        | INT_STATUS_1   |
| 0x1E        | SC_OUT_0       |
| 0x1F        | SC_OUT_1       |
| 0x20        | WR_GEST_ACT    |
| 0x21        | INTERNAL_STATUS|
| 0x22        | TEMPERATURE_0 (TEMP_LSB) |
| 0x23        | TEMPERATURE_1 (TEMP_MSB) |
| 0x24        | FIFO_LENGTH_0  |
| 0x25        | FIFO_LENGTH_1  |
| 0x26        | FIFO_DATA      |
| 0x2F        | FEAT_PAGE      |
| 0x30-0x3F   | FEATURES[16]   |
| 0x40        | ACC_CONF       |
| 0x41        | ACC_RANGE      |
| 0x42        | GYR_CONF       |
| 0x43        | GYR_RANGE      |
| 0x44        | AUX_CONF       |
| 0x45        | FIFO_DOWNS     |
| 0x46        | FIFO_WTM_0     |
| 0x47        | FIFO_WTM_1     |
| 0x48        | FIFO_CONFIG_0  |
| 0x49        | FIFO_CONFIG_1  |
| 0x4A        | SATURATION     |
| 0x4B        | AUX_DEV_ID     |
| 0x4C        | AUX_IF_CONF    |
| 0x4D        | AUX_RD_ADDR    |
| 0x4E        | AUX_WR_ADDR    |
| 0x4F        | AUX_WR_DATA    |
| 0x52        | ERR_REG_MSK    |
| 0x53        | INT1_IO_CTRL   |
| 0x54        | INT2_IO_CTRL   |
| 0x55        | INT_LATCH      |
| 0x56        | INT1_MAP_FEAT  |
| 0x57        | INT2_MAP_FEAT  |
| 0x58        | INT_MAP_DATA   |
| 0x59        | INIT_CTRL      |
| 0x5B        | INIT_ADDR_0    |
| 0x5C        | INIT_ADDR_1    |
| 0x5E        | INIT_DATA      |
| 0x5F        | INTERNAL_ERROR |
| 0x68        | AUX_IF_TRIM    |
| 0x69        | GYR_CRT_CONF   |
| 0x6A        | NVM_CONF       |
| 0x6B        | IF_CONF        |
| 0x6C        | DRV            |
| 0x6D        | ACC_SELF_TEST  |
| 0x6E        | GYR_SELF_TEST_AXES |
| 0x70        | NV_CONF        |
| 0x71-0x77   | OFFSET_0..6    |
| 0x7C        | PWR_CONF       |
| 0x7D        | PWR_CTRL       |
| 0x7E        | CMD            |

Note: DATA_0..DATA_19 (0x04-0x17) carry AUX (mag) x/y/z, GYR x/y/z, then
ACC x/y/z, each as LSB-then-MSB byte pairs - same layout convention as
BMI160's DATA block, just in a different axis order (BMI160 = mag, gyr,
acc; BMI270 has no dedicated mag slot in DATA, since AUX data width there
is smaller/generic - verify exact sub-layout against p.76-80 of the
datasheet before relying on it for the port).

## Key structural difference from BMI160

BMI270 is **not** usable right after a plain register-level reset. It
requires uploading Bosch's binary "config file" blob through
`INIT_ADDR_0/1` + `INIT_DATA` while `PWR_CONF` is in a specific state, then
polling `INTERNAL_STATUS` for `message == 0x1` (init OK) before any
sensor/feature register does anything - see the existing driver's
`BMI270.cpp` (`Reset()`/`ConfigureInit()` or equivalent) for how PX4
already does this. BMI160 has no such requirement: all motion-detection
features live directly in fixed registers (0x50-0x7B), no blob upload, no
FEAT_PAGE paging. This is the main logic (not just register-address) that
has to be stripped out when turning the cloned BMI270 driver into a real
BMI160 one - a 1:1 register rename is not enough.
