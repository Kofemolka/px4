/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file gnss_analyzer.cpp
 * Implementation of the GnssAnalyzer.
 *
 * @author
 */

#include "gnss_analyzer.hpp"

#include <px4_platform_common/log.h>

GnssSpoofingState GnssAnalyzer::state() const
{
	return _state;
}

void GnssAnalyzer::reset()
{
	_gnss_kf.reset();
	_high_freq_imu_history.reset();
	_low_freq_gps_history.reset();
	_imu_cumulative_velocity_ned.setZero();

	_state = GnssSpoofingState::Uninitialized;
}

void GnssAnalyzer::pushIMU(const DeltaVelocityEarth &sample)
{
	if (!_high_freq_imu_history.empty() && _high_freq_imu_history.newest().time_us >= sample.time_us)
	{
		//PX4_WARN("GnssAnalyzer: dropping IMU sample with the time_us <= latest_saved_imu.time_us");
		return;
	}

	_imu_cumulative_velocity_ned += sample.delta_velocity_ned;

	[[maybe_unused]]
	auto res = _high_freq_imu_history.push(IMUCumulativeVelocityEndpoint{
			.time_us = sample.time_us,
			.cumulative_velocity = _imu_cumulative_velocity_ned
		});
}

matrix::Vector3f GnssAnalyzer::lerp(
	const IMUCumulativeVelocityEndpoint& a,
	const IMUCumulativeVelocityEndpoint& b,
	uint64_t target_time_us)
{
	if (a.time_us == b.time_us)
	{
		return a.cumulative_velocity;
	}

	const float fraction =
		static_cast<float>(target_time_us - a.time_us)
		/
		static_cast<float>(b.time_us - a.time_us);

	return a.cumulative_velocity + (b.cumulative_velocity - a.cumulative_velocity) * fraction;
}

void GnssAnalyzer::pushGnss(const GnssKalmanFilter::Measurement &sample)
{
	const auto res = _gnss_kf.process(sample);

	if (!res)
	{
		PX4_WARN("GnssAnalyzer: failed to process Gnss sample in KF. Skipping.");
		return;
	}

	// find the IMU samples near the current Gnss timestamp for further comparison
	const auto bracket_indices = _high_freq_imu_history.findBracket(sample.time_us);
	if (!bracket_indices)
	{
		PX4_WARN("GnssAnalyzer: cannot find the nearby IMU samples for this Gnss sample.");
		return;
	}

	const auto& before = _high_freq_imu_history.atOldestOffset(bracket_indices->before);
	const auto& after = _high_freq_imu_history.atOldestOffset(bracket_indices->after);
	const auto imu_cumulative_at_gps = lerp(before, after, sample.time_us);

	// get the latest filtered gnss velocity
	const auto& state = _gnss_kf.state();
	matrix::Vector3f filtered_gnss_vel{};
	filtered_gnss_vel(0) = state(3);
	filtered_gnss_vel(1) = state(4);
	filtered_gnss_vel(2) = state(5);

	// pushing cumulative_imu and filtered_gnss velocities as a new checkpoint into the queue
	_low_freq_gps_history.push(VelocityEndpoint{
			.time_us = sample.time_us,
			.gnss_velocity_ned = filtered_gnss_vel,
			.imu_cumulative_delta_velocity_ned = imu_cumulative_at_gps
		});
}
