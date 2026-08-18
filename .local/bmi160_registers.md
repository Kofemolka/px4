# Bosch BMI160 — register map & device addressing

Source: official datasheet, BST-BMI160-DS000-09, Rev 1.0, Nov 2020
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf
(register map: Figure 20, p.48-49; addressing: chapter 3, p.88-93)

## Device addressing

- **Chip ID** (reg 0x00, read-only): `0xD1`
- **I2C**: 7-bit, default `0x68` (0b1101000) when SDO/SAO pin -> GND;
  alternate `0x69` (0b1101001) when SDO/SAO pin -> VDDIO. Std/Fast/Fast+
  modes up to 1 MHz.
- **SPI**: 4-wire (default) or 3-wire (set via `IF_CONF.spi3=1`), up to
  10 MHz. Protocol auto-selected by CSB pin behavior after power-up: tied
  to VDDIO -> I2C; rising edge seen -> SPI (until next reset). First byte
  of a transaction: bit7 = R/W (0=write,1=read, MSB-first as sent), bits
  6:0 = 7-bit register address. Do a dummy read of 0x7F before the first
  real SPI transaction after power-up.
- Interrupt pins: INT1 (pin 4), INT2 (pin 9).

## Register map (address, name, reset value, bitfield contents)

| Addr | Name          | Reset | Contents (bit7..bit0 / high..low byte) |
|------|---------------|-------|------------------------------------------|
| 0x00 | CHIP_ID       | 0xD1  | chip_id[7:0] |
| 0x01 | -             | -     | reserved |
| 0x02 | ERR_REG       | 0x00  | mag_drdy_err, drop_cmd_err, i2c_fail_err, err_code[3:0], fatal_err |
| 0x03 | PMU_STATUS    | 0x00  | acc_pmu_status[1:0], gyr_pmu_status[1:0], mag_pmu_status[1:0] |
| 0x04 | DATA_0        | 0x00  | mag_x_7_0 |
| 0x05 | DATA_1        | 0x00  | mag_x_15_8 |
| 0x06 | DATA_2        | 0x00  | mag_y_7_0 |
| 0x07 | DATA_3        | 0x00  | mag_y_15_8 |
| 0x08 | DATA_4        | 0x00  | mag_z_7_0 |
| 0x09 | DATA_5        | 0x00  | mag_z_15_8 |
| 0x0A | DATA_6        | 0x00  | rhall_7_0 |
| 0x0B | DATA_7        | 0x00  | rhall_15_8 |
| 0x0C | DATA_8        | 0x00  | gyr_x_7_0 |
| 0x0D | DATA_9        | 0x00  | gyr_x_15_8 |
| 0x0E | DATA_10       | 0x00  | gyr_y_7_0 |
| 0x0F | DATA_11       | 0x00  | gyr_y_15_8 |
| 0x10 | DATA_12       | 0x00  | gyr_z_7_0 |
| 0x11 | DATA_13       | 0x00  | gyr_z_15_8 |
| 0x12 | DATA_14       | 0x00  | acc_x_7_0 |
| 0x13 | DATA_15       | 0x00  | acc_x_15_8 |
| 0x14 | DATA_16       | 0x00  | acc_y_7_0 |
| 0x15 | DATA_17       | 0x00  | acc_y_15_8 |
| 0x16 | DATA_18       | 0x00  | acc_z_7_0 |
| 0x17 | DATA_19       | 0x00  | acc_z_15_8 |
| 0x18 | SENSORTIME_0  | 0x00  | sensor_time_7_0 |
| 0x19 | SENSORTIME_1  | 0x00  | sensor_time_15_8 |
| 0x1A | SENSORTIME_2  | 0x00  | sensor_time_23_16 |
| 0x1B | STATUS        | 0x01  | drdy_acc, drdy_gyr, drdy_mag, nvm_rdy, foc_rdy, mag_man_op, gyr_self_test_ok, reserved |
| 0x1C | INT_STATUS_0  | 0x00  | step_int, sigmot_int, anym_int, pmu_trigger_int, d_tap_int, s_tap_int, orient_int, flat_int |
| 0x1D | INT_STATUS_1  | 0x00  | reserved, high_int, low_int, drdy_int, ffull_int, fwm_int, nomo_int |
| 0x1E | INT_STATUS_2  | 0x00  | anym_first_x, anym_first_y, anym_first_z, anym_sign, tap_first_x, tap_first_y, tap_first_z, tap_sign |
| 0x1F | INT_STATUS_3  | 0x00  | high_first_x, high_first_y, high_first_z, high_sign, orient_1_0, orient_2, flat |
| 0x20 | TEMPERATURE_0 | 0x00  | temperature_7_0 |
| 0x21 | TEMPERATURE_1 | 0x80  | temperature_15_8 |
| 0x22 | FIFO_LENGTH_0 | 0x00  | fifo_byte_counter_7_0 |
| 0x23 | FIFO_LENGTH_1 | 0x00  | reserved, fifo_byte_counter_10_8 |
| 0x24 | FIFO_DATA     | 0x00  | fifo_data |
| 0x25 | -             | -     | reserved |
| 0x3F | -             | -     | reserved |
| 0x40 | ACC_CONF      | 0x28  | acc_us, acc_bwp[2:0], acc_odr[3:0] |
| 0x41 | ACC_RANGE     | 0x03  | reserved, acc_range[3:0] |
| 0x42 | GYR_CONF      | 0x28  | reserved, gyr_bwp[1:0], gyr_odr[3:0] |
| 0x43 | GYR_RANGE     | 0x0B  | reserved, gyr_range[2:0] |
| 0x44 | MAG_CONF      | 0x0B  | reserved, mag_odr[3:0] |
| 0x45 | FIFO_DOWNS    | 0x88  | acc_fifo_filt_data, acc_fifo_downs[2:0], gyr_fifo_filt_data, gyr_fifo_downs[2:0] |
| 0x46 | FIFO_CONFIG_0 | 0x80  | fifo_water_mark[7:0] |
| 0x47 | FIFO_CONFIG_1 | 0x10  | fifo_gyr_en, fifo_acc_en, fifo_mag_en, fifo_header_en, fifo_tag_int1_en, fifo_tag_int2_en, fifo_time_en, reserved |
| 0x48 | MAG_IF_0      | 0x20  | i2c_device_addr[6:0], reserved |
| 0x49 | -             | -     | reserved |
| 0x4A | -             | -     | reserved |
| 0x4B | MAG_IF_1      | 0x80  | mag_manual_en, reserved, mag_offset[3:0], mag_rd_burst[1:0] |
| 0x4C | MAG_IF_2      | 0x42  | read_addr |
| 0x4D | MAG_IF_3      | 0x4C  | write_addr |
| 0x4E | MAG_IF_4      | 0x00  | write_data |
| 0x4F | -             | -     | reserved |
| 0x50 | INT_EN_0      | 0x00  | flat_en, orient_en, s_tap_en, d_tap_en, reserved, anymo_z_en, anymo_y_en, anymo_x_en |
| 0x51 | INT_EN_1      | 0x00  | fwm_en, ffull_en, drdy_en, low_en, reserved, high_z_en, high_y_en, high_x_en |
| 0x52 | INT_EN_2      | 0x00  | reserved, nomox_en, nomoy_en, nomoz_en, step_det_en |
| 0x53 | INT_OUT_CTRL  | 0x00  | int2_output_en, int2_od, int2_lvl, int2_edge_ctrl, int1_output_en, int1_od, int1_lvl, int1_edge_ctrl |
| 0x54 | INT_LATCH     | 0x00  | reserved, int_latch |
| 0x55 | INT_MAP_0     | 0x00  | int1_flat, int1_orient, int1_s_tap, int1_d_tap, int1_pmu_trig, int1_ffull, int1_fwm, int1_drdy |
| 0x56 | INT_MAP_1     | 0x00  | int2_drdy, int1_fwm, int1_ffull, int1_pmu_trig, int2_anymotion, int2_highg, int2_lowg, int2_pmu_trig |
| 0x57 | INT_MAP_2     | 0x00  | int2_flat, int2_orient, int2_s_tap, int2_d_tap, int2_anymotion, int2_highg, int2_lowg, int2_step |
| 0x58 | INT_DATA_0    | 0x00  | int_tap_src, reserved, int_low_high_src |
| 0x59 | INT_DATA_1    | 0x00  | int_motion_src, reserved |
| 0x5A | INT_LOWHIGH_0 | 0x07  | int_low_dur[7:0] |
| 0x5B | INT_LOWHIGH_1 | 0x30  | int_low_hy[1:0], reserved, int_low_th[7:0] (shared) |
| 0x5C | INT_LOWHIGH_2 | 0x81  | int_high_hy[1:0], int_high_dur[7:0] |
| 0x5D | INT_LOWHIGH_3 | 0x0B  | int_high_th[7:0] |
| 0x5E | INT_LOWHIGH_4 | 0xC0  | int_high_th[7:0] (cont.) |
| 0x5F | INT_MOTION_0  | 0x00  | int_anym_dur[1:0], int_slo_nomo_sel |
| 0x60 | INT_MOTION_1  | 0x14  | int_anym_th[7:0] |
| 0x61 | INT_MOTION_2  | 0x14  | int_slo_nomo_th[7:0] |
| 0x62 | INT_MOTION_3  | 0x24  | int_sig_mot_skip[1:0], int_sig_mot_proof[1:0], reserved |
| 0x63 | INT_TAP_0     | 0x04  | int_tap_dur[2:0], reserved, int_tap_quiet, int_tap_shock |
| 0x64 | INT_TAP_1     | 0x0A  | int_tap_th[4:0] |
| 0x65 | INT_ORIENT_0  | 0x18  | int_orient_mode[1:0], int_orient_blocking[1:0], int_orient_hy[3:0] |
| 0x66 | INT_ORIENT_1  | 0x48  | int_orient_theta[5:0] |
| 0x67 | INT_FLAT_0    | 0x08  | reserved, int_flat_theta[5:0] |
| 0x68 | INT_FLAT_1    | 0x11  | reserved, int_flat_hold[1:0], reserved, int_flat_hy[3:0] |
| 0x69 | FOC_CONF      | 0x00  | foc_gyr_en, foc_acc_x[1:0], foc_acc_y[1:0], foc_acc_z[1:0], nvm_prog_en |
| 0x6A | CONF          | 0x00  | reserved, spi3, reserved |
| 0x6B | IF_CONF       | 0x00  | reserved, if_mode[1:0], reserved |
| 0x6C | PMU_TRIGGER   | 0x00  | reserved, gyr_sleep_trigger[2:0], gyr_wakeup_trigger[1:0], gyr_sleep_state, wakeup_int |
| 0x6D | SELF_TEST     | 0x00  | reserved, acc_self_test_amp, acc_self_test_sign, acc_self_test_enable, reserved, gyr_self_test_enable |
| 0x6E | -             | -     | reserved |
| 0x6F | -             | -     | reserved |
| 0x70 | NV_CONF       | 0x00  | reserved, u_spare_0, i2c_wdt_en, i2c_wdt_sel, spi_en |
| 0x71 | OFFSET_0      | 0x00  | off_acc_x[7:0] |
| 0x72 | OFFSET_1      | 0x00  | off_acc_y[7:0] |
| 0x73 | OFFSET_2      | 0x00  | off_acc_z[7:0] |
| 0x74 | OFFSET_3      | 0x00  | off_gyr_x_7_0 |
| 0x75 | OFFSET_4      | 0x00  | off_gyr_y_7_0 |
| 0x76 | OFFSET_5      | 0x00  | off_gyr_z_7_0 |
| 0x77 | OFFSET_6      | 0x00  | gyr_off_en, acc_off_en, off_gyr_z_9_8, off_gyr_y_9_8, off_gyr_x_9_8 |
| 0x78 | STEP_CNT_0    | 0x00  | step_cnt_7_0 |
| 0x79 | STEP_CNT_1    | 0x00  | step_cnt_15_8 |
| 0x7A | STEP_CONF_0   | 0x15  | step_conf_7_0 |
| 0x7B | STEP_CONF_1   | 0x03  | reserved, step_cnt_en, step_conf_10_8 |
| 0x7C | -             | -     | reserved |
| 0x7D | -             | -     | reserved |
| 0x7E | CMD           | 0x00  | cmd[7:0] |

Note: BMI160 has no on-chip FEATURES/advanced-detection page mechanism
like BMI270 (0x30 FEATURES[16] window) - all the motion-detection config
(tap, orientation, flat, motion, step) lives directly in fixed registers
0x50-0x7B above. This is the single biggest structural difference from
BMI270 for a driver port: BMI270 requires uploading a config file blob at
init (`INIT_CTRL`/`INIT_ADDR`/`INIT_DATA`) and paging through FEATURES via
`FEAT_PAGE`; BMI160 does not - it's usable directly after reset.
