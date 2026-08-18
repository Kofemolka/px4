/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Bosch_BMI160_registers.hpp
 *
 * Bosch BMI160 registers.
 *
 * Addresses and bit layouts per BST-BMI160-DS000-09 (Rev 1.0, Nov 2020).
 * Unlike BMI270, BMI160's SPI read protocol has no dummy byte between the
 * command byte and the returned data byte (see chapter 3.2.2/3.2.3), and
 * the device needs no config-file upload - it's usable directly after the
 * PMU (power management) commands below are issued.
 */

#pragma once

#include <cstdint>
#include <cstddef>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace Bosch_BMI160
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10MHz SPI serial interface
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t chip_id = 0xD1;

enum class Register : uint8_t {
	CHIP_ID            = 0x00,

	ERR_REG            = 0x02,
	PMU_STATUS         = 0x03,

	TEMPERATURE_0       = 0x20, // temperature LSB
	TEMPERATURE_1       = 0x21, // temperature MSB

	FIFO_LENGTH_0      = 0x22,
	FIFO_LENGTH_1      = 0x23,
	FIFO_DATA          = 0x24,

	ACC_CONF           = 0x40,
	ACC_RANGE          = 0x41,
	GYR_CONF           = 0x42,
	GYR_RANGE          = 0x43,

	FIFO_CONFIG_0      = 0x46, // fifo_water_mark, unit: 4 bytes
	FIFO_CONFIG_1      = 0x47, // fifo_gyr_en, fifo_acc_en, fifo_mag_en, fifo_header_en

	INT_EN_1           = 0x51,
	INT_OUT_CTRL       = 0x53,
	INT_MAP_0          = 0x55,

	CMD                = 0x7E,

	// Not a real register - used only for the mandatory dummy read that
	// switches the primary interface to SPI mode after power-up
	// (datasheet 3.2.1: "perform a SPI single read access to the ADDRESS
	// 0x7F before the actual communication").
	SPI_COMM_INIT      = 0x7F,
};

// Register (0x7E) CMD command codes
enum CMD_BIT : uint8_t {
	cmd_start_foc         = 0x03,
	cmd_acc_pmu_normal     = 0x11,
	cmd_acc_pmu_lowpower   = 0x12,
	cmd_gyr_pmu_normal     = 0x15,
	cmd_gyr_pmu_fast_startup = 0x17,
	cmd_prog_nvm           = 0xA0,
	cmd_fifo_flush         = 0xB0,
	cmd_int_reset          = 0xB1,
	cmd_softreset          = 0xB6,
	cmd_step_cnt_clr       = 0xB2,
};

// ACC_CONF
enum ACC_CONF_BIT : uint8_t {
	// [3:0] acc_odr - output data rate is 100 / 2^(8-val)
	acc_odr_1600 = Bit3 | Bit2, // 0b1100 -> 1600 Hz
};

// ACC_RANGE - 4 bit field, not a simple 2-bit power-of-two like BMI270
enum ACC_RANGE_BIT : uint8_t {
	acc_range_2g  = Bit1 | Bit0,        // 0b0011
	acc_range_4g  = Bit2 | Bit0,        // 0b0101
	acc_range_8g  = Bit3,               // 0b1000
	acc_range_16g = Bit3 | Bit2,        // 0b1100
};

// GYR_CONF
enum GYR_CONF_BIT : uint8_t {
	// [3:0] gyr_odr - output data rate is 100 / 2^(8-val), same encoding as acc_odr
	gyr_odr_1600 = Bit3 | Bit2, // 0b1100 -> 1600 Hz
};

// GYR_RANGE
enum GYR_RANGE_BIT : uint8_t {
	gyr_range_2000dps = 0, // 0b000
};

// FIFO_CONFIG_1
enum FIFO_CONFIG_1_BIT : uint8_t {
	fifo_gyr_en    = Bit7,
	fifo_acc_en    = Bit6,
	fifo_mag_en    = Bit5,
	fifo_header_en = Bit4,
};

// INT_EN_1
enum INT_EN_1_BIT : uint8_t {
	fwm_en  = Bit7,
	ffull_en = Bit6,
	drdy_en  = Bit5,
};

// INT_OUT_CTRL - low nibble configures INT1, high nibble configures INT2
enum INT_OUT_CTRL_BIT : uint8_t {
	int1_edge_ctrl = Bit0,
	int1_lvl       = Bit1,
	int1_od        = Bit2,
	int1_output_en = Bit3,
};

// INT_MAP_0
enum INT_MAP_0_BIT : uint8_t {
	int1_drdy = Bit0,
	int1_fwm  = Bit1,
	int1_ffull = Bit2,
};

// ERR_REG
enum ERR_REG_BIT : uint8_t {
	fatal_err   = Bit0,
	// bits [4:1] error_code
	drop_cmd_err = Bit6,
	mag_drdy_err = Bit7,
};

namespace FIFO
{
static constexpr size_t SIZE = 1024;

// BMI160 is run in "headerless" FIFO mode (FIFO_CONFIG_1.fifo_header_en=0)
// with both gyro and accel enabled at the same ODR - every frame is
// therefore a fixed 12-byte [gyro xyz][accel xyz] record with no header
// byte and no per-frame parsing needed, unlike BMI270's tagged frames.
// Order matches Register (0x04-0x17) DATA: gyro before accel.
struct Data {
	uint8_t gyr_x_lsb;
	uint8_t gyr_x_msb;
	uint8_t gyr_y_lsb;
	uint8_t gyr_y_msb;
	uint8_t gyr_z_lsb;
	uint8_t gyr_z_msb;
	uint8_t acc_x_lsb;
	uint8_t acc_x_msb;
	uint8_t acc_y_lsb;
	uint8_t acc_y_msb;
	uint8_t acc_z_lsb;
	uint8_t acc_z_msb;
};

} // namespace FIFO

} // namespace Bosch_BMI160
