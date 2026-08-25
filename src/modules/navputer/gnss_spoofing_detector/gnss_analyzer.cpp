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
#include <matrix/Vector.hpp>

namespace
{
constexpr float kVelWindowDurationS = 2.f; // 2 second window
constexpr float kOldestPossibleSampleToCompareS = 2.5f; // 2.5 seconds
constexpr float kVelSafeError = 0.4; // m/s
constexpr float kVelSevereError = 1.2; // m/s

constexpr float kSafeSuspicionDecreasePerWindow = 0.1f;
constexpr float kMaxSuspicionIncreasePerWindow = 0.4f;

constexpr float kSpoofThreshold = 0.8f;
constexpr float kUnspoofThreshold = 0.2f;

constexpr float kPosSafeNormalizedError = 2.5f;  // sigma
constexpr float kPosSevereNormalizedError = 5.f; // sigma

constexpr float kSafePositionSuspicionDecrease = 0.1f;
constexpr float kMaxPositionSuspicionIncrease = 0.4f;

constexpr uint64_t kPosMaxGnssInterpolationGapUs = 250'000;
} // namespace

GnssSpoofingState GnssAnalyzer::state() const
{
	return _state;
}

void GnssAnalyzer::reset(bool origin_valid)
{
	_gnss_kf.reset();
	_high_freq_imu_history.reset();
	_gnss_endpoint_history.reset();
	_trusted_position_history.reset();
	_imu_cumulative_velocity_ned.setZero();
	_last_vel_analysis_time_us = 0;
	_last_pos_analysis_time_us = 0;

	if (origin_valid)
	{
		_velocity_suspicion = kSpoofThreshold;
		_position_suspicion = kSpoofThreshold;
		_state = GnssSpoofingState::Untrusted;
	}
	else
	{
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

void GnssAnalyzer::pushTrustedPosition(const TrustedPositionSample &sample)
{
	_trusted_position_history.push(sample);
	analyzePosAnomalies();
}

template<typename T, size_t Size>
matrix::Vector<T, Size> lerp(
	const matrix::Vector<T, Size> &before,
	const matrix::Vector<T, Size> &after,
	uint64_t before_time_us,
	uint64_t after_time_us,
	uint64_t target_time_us)
{
	if (before_time_us == after_time_us)
	{
		return before;
	}

	const float fraction = static_cast<float>(target_time_us - before_time_us) / static_cast<float>(after_time_us - before_time_us);

	return before + (after - before) * fraction;
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
	const auto imu_cumulative_at_gps = lerp<float, 3>(before.cumulative_velocity, after.cumulative_velocity, before.time_us, after.time_us, sample.time_us);

	// get the latest filtered gnss velocity
	const auto& state = _gnss_kf.state();
	matrix::Vector3f filtered_gnss_pos{
		state(0),
		state(1),
		state(2)
	};
	matrix::Vector3f filtered_gnss_vel{
		state(3),
		state(4),
		state(5)
	};

	const auto& P = _gnss_kf.covariance();

	_gnss_endpoint_history.push(GnssEndpoint{
			.time_us = sample.time_us,
			.gnss_position_ned = filtered_gnss_pos,
			.gnss_position_ned_variance = {P(0, 0), P(1, 1), P(2, 2)},
			.gnss_velocity_ned = filtered_gnss_vel,
			.imu_cumulative_delta_velocity_ned = imu_cumulative_at_gps
		});

	analyzeVelAnomalies();
	analyzePosAnomalies();
}

void GnssAnalyzer::recalculateState()
{
	// Hysteresis
	const auto total_suspicion = math::max(_velocity_suspicion, _position_suspicion);

	if (total_suspicion >= kSpoofThreshold)
	{
		_state = GnssSpoofingState::Untrusted;
	}
	else if (total_suspicion <= kUnspoofThreshold)
	{
		_state = GnssSpoofingState::Trusted;
	}

}

bool GnssAnalyzer::getVelEndpoints(GnssEndpoint& recent, GnssEndpoint& old)
{
	if (_state == GnssSpoofingState::NoOrigin || _gnss_endpoint_history.empty())
	{
		return false;
	}

	recent = _gnss_endpoint_history.newest();

	// first time call after reset()
	if (_last_vel_analysis_time_us == 0)
	{
		_last_vel_analysis_time_us = recent.time_us;
		return false;
	}

	if (recent.time_us <= static_cast<uint64_t>(kVelWindowDurationS * 1e+6f))
	{
		return false;
	}

	const auto& old_idx_opt = _gnss_endpoint_history.findLastAtOrBefore(recent.time_us - static_cast<uint64_t>(kVelWindowDurationS * 1e+6f));

	// not enough samples yet
	if (!old_idx_opt)
	{
		return false;
	}

	old = _gnss_endpoint_history.atOldestOffset(*old_idx_opt);

	// the old sample is too old to compare. Wait for another sample that is around window_duration old
	if ((recent.time_us - old.time_us) > static_cast<uint64_t>(kOldestPossibleSampleToCompareS * 1e+6f))
	{
		return false;
	}

	return true;
}

void GnssAnalyzer::analyzeVelAnomalies()
{
	GnssEndpoint new_sample, old_sample;

	if (!getVelEndpoints(new_sample, old_sample))
	{
		return;
	}

	const float score_dt = (new_sample.time_us - _last_vel_analysis_time_us) * 1e-6f;
	const float window_fraction = math::constrain(score_dt / kVelWindowDurationS, 0.f, 1.f);

	const matrix::Vector3f gnss_delta_velocity = new_sample.gnss_velocity_ned - old_sample.gnss_velocity_ned;
	const matrix::Vector3f imu_delta_velocity = new_sample.imu_cumulative_delta_velocity_ned - old_sample.imu_cumulative_delta_velocity_ned;

	const matrix::Vector3f residual = gnss_delta_velocity - imu_delta_velocity;
	const float error = sqrtf(residual(0) * residual(0) + residual(1) * residual(1));

	float suspicion_delta = 0;

	if (error <= kVelSafeError)
	{
		suspicion_delta = -kSafeSuspicionDecreasePerWindow * window_fraction;
	}
	else
	{
		const float severity_weight = math::constrain(
			(error - kVelSafeError) / (kVelSevereError - kVelSafeError),
			0.f,
			1.f);
		suspicion_delta = kMaxSuspicionIncreasePerWindow * severity_weight * window_fraction;
	}

	_velocity_suspicion = math::constrain(_velocity_suspicion + suspicion_delta, 0.f, 1.f);

	recalculateState();
	_last_vel_analysis_time_us = new_sample.time_us;
}

bool GnssAnalyzer::getPosEndpoints(TrustedPositionSample& trusted, GnssEndpoint& before, GnssEndpoint& after)
{
	if (_state == GnssSpoofingState::NoOrigin
		|| _trusted_position_history.empty()
		|| _gnss_endpoint_history.empty())
	{
		return false;
	}

	trusted = _trusted_position_history.newest();

	if (trusted.time_us <= _last_pos_analysis_time_us)
	{
		return false;
	}
	// wait for more gnss samples
	if (trusted.time_us > _gnss_endpoint_history.newest().time_us)
	{
		return false;
	}
	// trusted sample is too old to be covered by gnss samples
	if (trusted.time_us < _gnss_endpoint_history.oldest().time_us)
	{
		_last_pos_analysis_time_us = trusted.time_us;
		return false;
	}

	const auto gnss_bracket_indices = _gnss_endpoint_history.findBracket(trusted.time_us);

	if (!gnss_bracket_indices)
	{
		return false;
	}

	before = _gnss_endpoint_history.atOldestOffset(gnss_bracket_indices->before);
	after =_gnss_endpoint_history.atOldestOffset(gnss_bracket_indices->after);

	if ((after.time_us - before.time_us) > kPosMaxGnssInterpolationGapUs)
	{
		_last_pos_analysis_time_us = trusted.time_us;
		return false;
	}

	return true;
}

void GnssAnalyzer::analyzePosAnomalies()
{
	TrustedPositionSample trusted;
	GnssEndpoint before, after;

	if (!getPosEndpoints(trusted, before, after))
	{
		return;
	}

	matrix::Vector3f gnss_position_ned = lerp(
		before.gnss_position_ned,
		after.gnss_position_ned,
		before.time_us,
		after.time_us,
		trusted.time_us);
	matrix::Vector2f gnss_position_ne = {gnss_position_ned(0), gnss_position_ned(1)};
	const matrix::Vector2f position_residual = gnss_position_ne - trusted.position_ne;

	const matrix::Vector3f gnss_position_ned_variance = lerp(
		before.gnss_position_ned_variance,
		after.gnss_position_ned_variance,
		before.time_us,
		after.time_us,
		trusted.time_us);
	const float variance_n = gnss_position_ned_variance(0) + trusted.position_variance_ne(0);
	const float variance_e = gnss_position_ned_variance(1) + trusted.position_variance_ne(1);

	const float normalized_error = sqrtf(position_residual(0) * position_residual(0) / variance_n
						+ position_residual(1) * position_residual(1) / variance_e);

	float suspicion_delta;

	if (normalized_error <= kPosSafeNormalizedError)
	{
		suspicion_delta = -kSafePositionSuspicionDecrease;
	}
	else
	{
		const float severity = math::constrain((normalized_error - kPosSafeNormalizedError)
							/ (kPosSevereNormalizedError - kPosSafeNormalizedError),
							0.f,
							1.f);
		suspicion_delta = kMaxPositionSuspicionIncrease * severity;
	}

	_position_suspicion = math::constrain(_position_suspicion + suspicion_delta, 0.f, 1.f);

	recalculateState();
	_last_pos_analysis_time_us = trusted.time_us;
}
