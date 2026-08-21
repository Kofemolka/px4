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

namespace
{
constexpr float kVelWindowDurationS = 2.f; // 2 second window
constexpr float kOldestPossibleSampleToCompareS = 2.5f; // 2.5 seconds
constexpr float kVelSafeResidual = 0.4; // m/s
constexpr float kVelSevereResidual = 1.2; // m/s

constexpr float kSafeSuspicionDecreasePerWindow = 0.1f;
constexpr float kMaxSuspicionIncreasePerWindow = 0.4f;

constexpr float kVelSpoofThreshold = 0.8f;
constexpr float kVelUnspoofThreshold = 0.2f;
} // namespace

GnssSpoofingState GnssAnalyzer::state() const
{
	return _state;
}

void GnssAnalyzer::reset(bool origin_valid)
{
	_gnss_kf.reset();
	_high_freq_imu_history.reset();
	_low_freq_gps_history.reset();
	_imu_cumulative_velocity_ned.setZero();
	_last_vel_analysis_time_us = 0;

	if (origin_valid)
	{
		_velocity_suspicion = kVelSpoofThreshold;
		_state = GnssSpoofingState::Untrusted;
	}
	else
	{
		_velocity_suspicion = 0.f;
		_state = GnssSpoofingState::NoOrigin;
	}
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

	analyzeVelAnomalies();
}


void GnssAnalyzer::analyzeVelAnomalies()
{
	if (_state == GnssSpoofingState::NoOrigin || _low_freq_gps_history.empty())
	{
		return;
	}

	const auto& new_sample = _low_freq_gps_history.newest();

	// first time call after reset()
	if (_last_vel_analysis_time_us == 0)
	{
		_last_vel_analysis_time_us = new_sample.time_us;
		return;
	}

	if (new_sample.time_us <= static_cast<uint64_t>(kVelWindowDurationS * 1e+6f))
	{
		return;
	}

	const auto& old_idx_opt = _low_freq_gps_history.findLastAtOrBefore(new_sample.time_us - static_cast<uint64_t>(kVelWindowDurationS * 1e+6f));

	// not enough samples yet
	if (!old_idx_opt)
	{
		return;
	}

	const auto& old_sample = _low_freq_gps_history.atOldestOffset(*old_idx_opt);

	// the old sample is too old to compare. Wait for another sample that is around window_duration old
	if ((new_sample.time_us - old_sample.time_us) > static_cast<uint64_t>(kOldestPossibleSampleToCompareS * 1e+6f))
	{
		return;
	}

	const float score_dt = (new_sample.time_us - _last_vel_analysis_time_us) * 1e-6f;
	const float window_fraction = math::constrain(score_dt / kVelWindowDurationS, 0.f, 1.f);

	const matrix::Vector3f gnss_delta_velocity = new_sample.gnss_velocity_ned - old_sample.gnss_velocity_ned;
	const matrix::Vector3f imu_delta_velocity = new_sample.imu_cumulative_delta_velocity_ned - old_sample.imu_cumulative_delta_velocity_ned;

	const matrix::Vector3f residual = gnss_delta_velocity - imu_delta_velocity;
	const float horizontal_residual = sqrtf(residual(0) * residual(0) + residual(1) * residual(1));

	float suspicion_delta = 0;

	if (horizontal_residual <= kVelSafeResidual)
	{
		suspicion_delta = -kSafeSuspicionDecreasePerWindow * window_fraction;
	}
	else
	{
		const float severity_weight = math::constrain(
			(horizontal_residual - kVelSafeResidual) / (kVelSevereResidual - kVelSafeResidual),
			0.f,
			1.f);
		suspicion_delta = kMaxSuspicionIncreasePerWindow * severity_weight * window_fraction;
	}

	_velocity_suspicion = math::constrain(_velocity_suspicion + suspicion_delta, 0.f, 1.f);

	// Hysteresis
	if (_velocity_suspicion >= kVelSpoofThreshold)
	{
		_state = GnssSpoofingState::Untrusted;
	}
	else if (_velocity_suspicion <= kVelUnspoofThreshold)
	{
		_state = GnssSpoofingState::Trusted;
	}

	_last_vel_analysis_time_us = new_sample.time_us;
}
