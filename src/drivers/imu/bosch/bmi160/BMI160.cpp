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
#include "BMI160.hpp"
#define BMI160_DEBUG

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

BMI160::BMI160(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

BMI160::~BMI160()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int BMI160::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool BMI160::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

// Debug helper - BMI160's ERR_REG has a much simpler error model than
// BMI270's (no config-file/INTERNAL_STATUS state machine to decode).
void BMI160::CheckErrorRegister()
{
#ifdef BMI160_DEBUG
	uint8_t err = RegisterRead(Register::ERR_REG);

	if (err) {
		if (err & ERR_REG_BIT::fatal_err) {
			// Chip not operable. Per datasheet this flag is only cleared by
			// a power-on-reset - a soft reset (CMD 0xB6) will not fix it.
			PX4_DEBUG("BMI160: fatal_err - chip not operable, power cycle required");
		}

		const uint8_t error_code = (err >> 1) & 0x0F;

		if (error_code != 0) {
			PX4_DEBUG("BMI160: error_code 0x%X", error_code);
		}

		if (err & ERR_REG_BIT::drop_cmd_err) {
			PX4_DEBUG("BMI160: drop_cmd_err - dropped command to CMD register");
		}

		if (err & ERR_REG_BIT::mag_drdy_err) {
			PX4_DEBUG("BMI160: mag_drdy_err");
		}
	}

#endif
}

void BMI160::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void BMI160::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int BMI160::probe()
{
	// Per datasheet 3.2.1: a rising edge on CSB is required to switch the
	// primary interface into SPI mode after power-up. A single dummy read
	// of address 0x7F (not a real register) triggers that edge.
	RegisterRead(Register::SPI_COMM_INIT);

	const uint8_t CHIP_ID = RegisterRead(Register::CHIP_ID);

	if (CHIP_ID != chip_id) {
		DEVICE_DEBUG("unexpected CHIP_ID 0x%02x", CHIP_ID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void BMI160::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// 0xB6 is written to the CMD register for a soft reset
		RegisterWrite(Register::CMD, CMD_BIT::cmd_softreset);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(10_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		if (RegisterRead(Register::CHIP_ID) == chip_id) {
			PX4_DEBUG("Read from CHIP_ID register and the IDs match");

			// Unlike BMI270, BMI160 needs no config-file upload - only its
			// accelerometer and gyroscope power modes need to be switched
			// from suspend to normal via the CMD register.
			RegisterWrite(Register::CMD, CMD_BIT::cmd_acc_pmu_normal);
			_state = STATE::ACCEL_PMU_WAIT;
			ScheduleDelayed(10_ms); // datasheet: max 3.8 ms execution time

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::ACCEL_PMU_WAIT:
		RegisterWrite(Register::CMD, CMD_BIT::cmd_gyr_pmu_normal);
		_state = STATE::GYRO_PMU_WAIT;
		ScheduleDelayed(100_ms); // datasheet: max 80 ms execution time for gyro PMU switch
		break;

	case STATE::GYRO_PMU_WAIT: {
			const uint8_t pmu_status = RegisterRead(Register::PMU_STATUS);
			// PMU_STATUS: acc_pmu_status[5:4], gyr_pmu_status[3:2], mag_pmu_status[1:0].
			// 0b01 == Normal for both.
			const bool acc_normal = (pmu_status & (Bit5 | Bit4)) == Bit4;
			const bool gyr_normal = (pmu_status & (Bit3 | Bit2)) == Bit2;

			if (acc_normal && gyr_normal) {
				_state = STATE::CONFIGURE;
				ScheduleNow();

			} else {
				PX4_DEBUG("PMU_STATUS 0x%02hhX, accel/gyro not yet in normal mode, resetting", pmu_status);
				_state = STATE::RESET;
				ScheduleDelayed(10_ms);
			}
		}
		break;

	case STATE::CONFIGURE:

		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();
			_state = STATE::FIFO_READ;

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_READ: {

			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			bool success = false;
			const uint16_t fifo_count = FIFOReadCount();

			// more bytes than what the buffer takes so an overflow
			if (fifo_count >= FIFO::SIZE) {
				FIFOReset();
				perf_count(_fifo_overflow_perf);

			} else if (fifo_count == 0) {
				perf_count(_fifo_empty_perf);

			} else {

				uint8_t samples = fifo_count / sizeof(FIFO::Data);

				// tolerate minor jitter, leave sample to next iteration if behind by only 1
				if (samples == _fifo_gyro_samples + 1) {
					timestamp_sample -= static_cast<int>(FIFO_SAMPLE_DT);
					samples--;
				}

				if (samples > FIFO_MAX_SAMPLES) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (samples >= _fifo_gyro_samples) {
					if (FIFORead(timestamp_sample, fifo_count)) {
						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					PX4_DEBUG("failure count > 10, resetting...");
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					PX4_DEBUG("register check failed, resetting...");
					perf_count(_bad_register_perf);
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void BMI160::SetAccelScaleAndRange()
{
	// ACC_RANGE is a 4 bit field (unlike BMI270's 2 bit field)
	const uint8_t ACC_RANGE = RegisterRead(Register::ACC_RANGE) & (Bit3 | Bit2 | Bit1 | Bit0);

	switch (ACC_RANGE) {
	case acc_range_2g:
		_px4_accel.set_scale(2.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(2.f * CONSTANTS_ONE_G);
		break;

	case acc_range_4g:
		_px4_accel.set_scale(4.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(4.f * CONSTANTS_ONE_G);
		break;

	case acc_range_8g:
		_px4_accel.set_scale(8.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(8.f * CONSTANTS_ONE_G);
		break;

	case acc_range_16g:
		_px4_accel.set_scale(16.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
		break;
	}
}

void BMI160::SetGyroScale()
{
	// GYR_RANGE is configured for its widest range, 2000 dps (see _register_cfg)
	const float scale = math::radians(2000.0f) / 32767.0f;
	_px4_gyro.set_scale(scale);
}

void BMI160::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = math::min((float)_fifo_empty_interval_us / (1e6f / RATE), (float)FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

// when this register is set an interrupt is triggered when the FIFO reaches this many samples
void BMI160::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO_CONFIG_0.fifo_water_mark is a single 8 bit register, unlike
	// BMI270's split 13 bit FIFO_WTM_0/FIFO_WTM_1. Its unit is 4 bytes
	// (BMI270's is 1 byte), so divide the byte threshold by 4.
	const uint16_t fifo_watermark_threshold = samples * sizeof(FIFO::Data);
	const uint8_t fifo_watermark_reg_value = fifo_watermark_threshold / 4;

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_CONFIG_0) {
			r.set_bits = fifo_watermark_reg_value;
			r.clear_bits = ~r.set_bits;
		}
	}
}

bool BMI160::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured correctly
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	SetAccelScaleAndRange();
	SetGyroScale();

	return success;
}

int BMI160::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<BMI160 *>(arg)->DataReady();
	return 0;
}

void BMI160::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool BMI160::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge (INT1 configured active-low, see INT_OUT_CTRL)
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool BMI160::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool BMI160::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t BMI160::RegisterRead(Register reg)
{
	// BMI160 SPI reads are a plain 2 byte transaction: command byte
	// immediately followed by the data byte - no dummy byte like BMI270
	// (datasheet 3.2.2/3.2.3, Figure 24/25).
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void BMI160::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void BMI160::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

// Checks how many bytes are in the FIFO
uint16_t BMI160::FIFOReadCount()
{
	CheckErrorRegister();

	// FIFO_LENGTH_1[2:0] and FIFO_LENGTH_0 contain the 11 bit FIFO byte count
	FIFOLengthReadBuffer buffer {};

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return combine(buffer.FIFO_LENGTH_1 & 0x07, buffer.FIFO_LENGTH_0);
}

// writes a gyro frame into the FIFO buffer the first argument points to
void BMI160::ProcessGyro(sensor_gyro_fifo_s *gyro, FIFO::Data *frame)
{
	const uint8_t samples = gyro->samples;

	const int16_t gyro_x = combine(frame->gyr_x_msb, frame->gyr_x_lsb);
	const int16_t gyro_y = combine(frame->gyr_y_msb, frame->gyr_y_lsb);
	const int16_t gyro_z = combine(frame->gyr_z_msb, frame->gyr_z_lsb);

	// Rotate from FLU to NED
	gyro->x[samples] = gyro_x;
	gyro->y[samples] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
	gyro->z[samples] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

	gyro->samples++;
}

// writes an accelerometer frame into the FIFO buffer the first argument points to
void BMI160::ProcessAccel(sensor_accel_fifo_s *accel, FIFO::Data *frame)
{
	const uint8_t samples = accel->samples;

	const int16_t accel_x = combine(frame->acc_x_msb, frame->acc_x_lsb);
	const int16_t accel_y = combine(frame->acc_y_msb, frame->acc_y_lsb);
	const int16_t accel_z = combine(frame->acc_z_msb, frame->acc_z_lsb);

	// Rotate from FLU to NED
	accel->x[samples] = accel_x;
	accel->y[samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
	accel->z[samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

	accel->samples++;
}

bool BMI160::FIFORead(const hrt_abstime &timestamp_sample, uint16_t fifo_bytes)
{
	FIFOReadBuffer buffer{};

	// Reads from the FIFO_DATA register as much data as is available,
	// plus 1 byte for the sent command (no dummy byte for BMI160).
	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, fifo_bytes + 1) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	sensor_accel_fifo_s accel_buffer{};
	accel_buffer.timestamp_sample = timestamp_sample;
	accel_buffer.dt = FIFO_SAMPLE_DT;

	sensor_gyro_fifo_s gyro_buffer{};
	gyro_buffer.timestamp_sample = timestamp_sample;
	gyro_buffer.dt = FIFO_SAMPLE_DT;

	// Headerless mode with both sensors enabled at the same ODR: every
	// entry is a fixed-size combined gyro+accel frame, so unlike BMI270
	// there's no per-frame header byte to parse - just walk fixed-size
	// records. fifo_bytes is always a multiple of sizeof(FIFO::Data)
	// because FIFOReadCount()/RunImpl() only calls FIFORead() when
	// `samples = fifo_count / sizeof(FIFO::Data)` is within range.
	const uint8_t samples = fifo_bytes / sizeof(FIFO::Data);

	for (uint8_t i = 0; i < samples; i++) {
		ProcessGyro(&gyro_buffer, &buffer.f[i]);
		ProcessAccel(&accel_buffer, &buffer.f[i]);
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if ((accel_buffer.samples == 0) && (gyro_buffer.samples == 0)) {
		return false;

	} else {
		if (accel_buffer.samples > 0) {
			_px4_accel.updateFIFO(accel_buffer);
		}

		if (gyro_buffer.samples > 0) {
			_px4_gyro.updateFIFO(gyro_buffer);
		}

		return true;
	}
}

void BMI160::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// fifo_flush: clears all FIFO data, keeps FIFO_CONFIG/FIFO_DOWNS settings
	RegisterWrite(Register::CMD, CMD_BIT::cmd_fifo_flush);

	_drdy_timestamp_sample.store(0);
}

void BMI160::UpdateTemperature()
{
	// TEMPERATURE_0 (LSB) followed by TEMPERATURE_1 (MSB), no dummy byte
	uint8_t temperature_buf[3] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::TEMPERATURE_0) | DIR_READ;

	if (transfer(&temperature_buf[0], &temperature_buf[0], sizeof(temperature_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	const uint16_t temp = temperature_buf[1] | (temperature_buf[2] << 8);

	float temperature;

	if (temp == 0x8000) {
		// invalid
		temperature = NAN;

	} else {
		constexpr float lsb = 0.001953125f; // 1/2^9
		temperature = 23.0f + (int16_t)temp * lsb;
	}

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);
}
