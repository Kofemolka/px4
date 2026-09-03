/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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

#include "gnss_analyzer.hpp"

#include <drivers/drv_hrt.h>
#include <matrix/Vector.hpp>
#include <px4_platform_common/log.h>

namespace
{
constexpr uint64_t kOldestPossibleSampleToCompareUs = 2'500'000;

constexpr float kVelSafeSigma = 1.0f;
constexpr float kVelSevereSigma = 3.0f;
constexpr float kRawVelPosSafeError = 1.0f;
constexpr float kRawVelPosSevereError = 3.0f;
constexpr float kSafeSuspicionDecreasePerWindow = 0.1f;
constexpr float kMaxSuspicionIncreasePerWindow = 0.8f;

constexpr float kPosSafeSigma = 2.5f;
constexpr float kPosSevereSigma = 5.f;
constexpr float kSafePositionSuspicionDecrease = 0.1f;
constexpr float kMaxPositionSuspicionIncrease = 0.4f;
constexpr uint64_t kPosMaxGnssInterpolationGapUs = 250'000;
constexpr uint64_t kDiagnosticLogPeriodUs = 5'000'000;

template<typename T, size_t Size>
matrix::Vector<T, Size> lerp(const matrix::Vector<T, Size> &before,
			     const matrix::Vector<T, Size> &after,
			     uint64_t before_time_us, uint64_t after_time_us,
			     uint64_t target_time_us)
{
	if (before_time_us == after_time_us)
	{
		return before;
	}

	const float fraction = static_cast<float>(target_time_us - before_time_us)
		/ static_cast<float>(after_time_us - before_time_us);
	return before + (after - before) * fraction;
}

template<typename History, typename Sample>
bool grabSamplesForWindow(const History &history, Sample &recent, Sample &old,
			  uint64_t window_duration_us, uint64_t oldest_possible_us)
{
	if (history.empty())
	{
		return false;
	}

	recent = history.newest();

	if (recent.time_us <= window_duration_us)
	{
		return false;
	}

	const auto old_index = history.findLastAtOrBefore(recent.time_us - window_duration_us);

	if (!old_index)
	{
		return false;
	}

	old = history.atOldestOffset(*old_index);
	return (recent.time_us - old.time_us) <= oldest_possible_us;
}
} // namespace

using namespace GnssAnalyzerTypes;

void BasicAnomalyAnalyzer::reset(float initial_suspicion)
{
	_suspicion = initial_suspicion;
}

float BasicAnomalyAnalyzer::suspicion() const
{
	return _suspicion;
}

void BasicAnomalyAnalyzer::updateWindowedSuspicion(float error, float safe_error,
		float severe_error, float window_fraction)
{
	float suspicion_delta = 0.f;

	if (error <= safe_error)
	{
		suspicion_delta = -kSafeSuspicionDecreasePerWindow * window_fraction;

	}
	else
	{
		const float severity = math::constrain((error - safe_error) / (severe_error - safe_error), 0.f, 1.f);
		suspicion_delta = kMaxSuspicionIncreasePerWindow * severity * window_fraction;
	}

	_suspicion = math::constrain(_suspicion + suspicion_delta, 0.f, 1.f);
}

void GnssImuDeltaVelocityAnalyzer::reset(float initial_suspicion)
{
	BasicAnomalyAnalyzer::reset(initial_suspicion);
	_last_analysis_time_us = 0;
}

void GnssImuDeltaVelocityAnalyzer::analyze(const GnssEndpointHistory &history)
{
	if (_last_analysis_time_us == 0)
	{
		if (!history.empty())
		{
			_last_analysis_time_us = history.newest().time_us;
		}
		return;
	}

	GnssEndpoint recent;
	GnssEndpoint old;

	if (!grabSamplesForWindow(history, recent, old, kVelWindowDurationUs,
			kOldestPossibleSampleToCompareUs))
	{
		return;
	}

	const uint64_t score_dt_us = recent.time_us - _last_analysis_time_us;
	const float window_fraction = math::constrain(static_cast<float>(score_dt_us) / static_cast<float>(kVelWindowDurationUs), 0.f, 1.f);

	const matrix::Vector3f gnss_delta_velocity = recent.gnss_velocity_ned - old.gnss_velocity_ned;
	const matrix::Vector3f imu_delta_velocity = recent.imu_cumulative_delta_velocity_ned - old.imu_cumulative_delta_velocity_ned;

	const matrix::Vector3f residual = gnss_delta_velocity - imu_delta_velocity;
	const matrix::Vector3f gnss_delta_velocity_variance = recent.gnss_velocity_ned_variance + old.gnss_velocity_ned_variance;
	const matrix::Vector3f imu_delta_velocity_variance = recent.imu_cumulative_delta_velocity_variance - old.imu_cumulative_delta_velocity_variance;
	const matrix::Vector3f residual_variance = gnss_delta_velocity_variance + imu_delta_velocity_variance;

	if (!PX4_ISFINITE(residual_variance(0))
		|| !PX4_ISFINITE(residual_variance(1))
		|| residual_variance(0) <= 0.f
		|| residual_variance(1) <= 0.f)
	{
		return;
	}

	const float error = sqrtf(residual(0) * residual(0) / residual_variance(0) + residual(1) * residual(1) / residual_variance(1));

	updateWindowedSuspicion(error, kVelSafeSigma, kVelSevereSigma, window_fraction);
	_last_analysis_time_us = recent.time_us;
}

void RawGnssVelocityAnalyzer::reset(float initial_suspicion)
{
	BasicAnomalyAnalyzer::reset(initial_suspicion);
	_last_analysis_time_us = 0;
}

void RawGnssVelocityAnalyzer::analyze(const GnssRawHistory &history)
{
	if (_last_analysis_time_us == 0)
	{
		if (!history.empty())
		{
			_last_analysis_time_us = history.newest().time_us;
		}
		return;
	}

	GnssRaw recent;
	GnssRaw old;

	if (!grabSamplesForWindow(history, recent, old, kVelWindowDurationUs, kOldestPossibleSampleToCompareUs))
	{
		return;
	}

	const float window_dt_s = (recent.time_us - old.time_us) * 1e-6f;

	if (!PX4_ISFINITE(window_dt_s) || window_dt_s <= 0.f)
	{
		return;
	}

	const matrix::Vector3f velocity_from_position = (recent.gnss_position_ned - old.gnss_position_ned) / window_dt_s;
	const auto old_index = history.findLastAtOrBefore(old.time_us);
	const auto recent_index = history.findLastAtOrBefore(recent.time_us);

	matrix::Vector3f integrated_position_change;
	integrated_position_change.setZero();

	for (size_t i = *old_index; i < *recent_index; ++i)
	{
		const GnssRaw &a = history.atOldestOffset(i);
		const GnssRaw &b = history.atOldestOffset(i + 1);
		const float interval_dt_s = (b.time_us - a.time_us) * 1e-6f;

		if (!PX4_ISFINITE(interval_dt_s) || interval_dt_s <= 0.f)
		{
			return;
		}

		integrated_position_change += (a.gnss_velocity_ned + b.gnss_velocity_ned) * (0.5f * interval_dt_s);
	}

	const matrix::Vector3f reported_average_velocity = integrated_position_change / window_dt_s;

	const matrix::Vector3f residual = reported_average_velocity - velocity_from_position;
	const float error = sqrtf(residual(0) * residual(0) + residual(1) * residual(1));

	const float score_dt_s = (recent.time_us - _last_analysis_time_us) * 1e-6f;
	const float window_fraction = math::constrain(score_dt_s / (kVelWindowDurationUs * 1e-6f), 0.f, 1.f);

	updateWindowedSuspicion(error, kRawVelPosSafeError, kRawVelPosSevereError, window_fraction);
	_last_analysis_time_us = recent.time_us;
}

void GnssMlatPosAnalyzer::reset(float initial_suspicion)
{
	BasicAnomalyAnalyzer::reset(initial_suspicion);
	_last_analysis_time_us = 0;
}

void GnssMlatPosAnalyzer::updateSuspicion(float normalized_error)
{
	float suspicion_delta = 0.f;

	if (normalized_error <= kPosSafeSigma)
	{
		suspicion_delta = -kSafePositionSuspicionDecrease;
	}
	else
	{
		const float severity = math::constrain((normalized_error - kPosSafeSigma)
			/ (kPosSevereSigma - kPosSafeSigma), 0.f, 1.f);
		suspicion_delta = kMaxPositionSuspicionIncrease * severity;
	}

	_suspicion = math::constrain(_suspicion + suspicion_delta, 0.f, 1.f);
}

bool GnssMlatPosAnalyzer::grabSamples(const GnssEndpointHistory &gnss_history,
	const TrustedPositionHistory &trusted_history, TrustedPositionSample &trusted,
	GnssEndpoint &before, GnssEndpoint &after)
{
	if (trusted_history.empty() || gnss_history.empty())
	{
		return false;
	}

	trusted = trusted_history.newest();

	if (trusted.time_us <= _last_analysis_time_us
		|| trusted.time_us > gnss_history.newest().time_us)
	{
		return false;
	}

	if (trusted.time_us < gnss_history.oldest().time_us)
	{
		_last_analysis_time_us = trusted.time_us;
		return false;
	}

	const auto bracket = gnss_history.findBracket(trusted.time_us);

	if (!bracket)
	{
		return false;
	}

	before = gnss_history.atOldestOffset(bracket->before);
	after = gnss_history.atOldestOffset(bracket->after);

	if ((after.time_us - before.time_us) > kPosMaxGnssInterpolationGapUs)
	{
		_last_analysis_time_us = trusted.time_us;
		return false;
	}

	return true;
}

void GnssMlatPosAnalyzer::analyze(const GnssEndpointHistory &gnss_history,
	const TrustedPositionHistory &trusted_history)
{
	TrustedPositionSample trusted;
	GnssEndpoint before;
	GnssEndpoint after;

	if (!grabSamples(gnss_history, trusted_history, trusted, before, after))
	{
		return;
	}

	// finding pos residual
	const matrix::Vector3f gnss_position_ned = lerp(
		before.gnss_position_ned,
		after.gnss_position_ned,
		before.time_us,
		after.time_us,
		trusted.time_us);
	const matrix::Vector2f gnss_position_ne{gnss_position_ned(0), gnss_position_ned(1)};
	const matrix::Vector2f position_residual = gnss_position_ne - trusted.position_ne;
	// finding pos variance residual
	const matrix::Vector3f gnss_position_variance = lerp(
		before.gnss_position_ned_variance,
		after.gnss_position_ned_variance,
		before.time_us,
		after.time_us,
		trusted.time_us);
	const float variance_n = gnss_position_variance(0) + trusted.position_variance_ne(0);
	const float variance_e = gnss_position_variance(1) + trusted.position_variance_ne(1);
	// all together error
	const float normalized_error = sqrtf(position_residual(0) * position_residual(0) / variance_n
		+ position_residual(1) * position_residual(1) / variance_e);

	updateSuspicion(normalized_error);
	_last_analysis_time_us = trusted.time_us;
}

GnssSpoofingState GnssAnalyzer::state() const
{
	return _state;
}

float GnssAnalyzer::suspicion() const
{
	return math::max(
		_imu_velocity_analyzer.suspicion(),
		_raw_velocity_analyzer.suspicion(),
		_position_analyzer.suspicion());
}

void GnssAnalyzer::transitionTo(GnssSpoofingState new_state)
{
	if (new_state != _state)
	{
		PX4_INFO("GNSSAnalyzer: %d -> %d (vel_sus1=%.2f vel_sus2=%.2f pos_sus=%.2f)",
			static_cast<int>(_state), static_cast<int>(new_state),
			static_cast<double>(_imu_velocity_analyzer.suspicion()),
			static_cast<double>(_raw_velocity_analyzer.suspicion()),
			static_cast<double>(_position_analyzer.suspicion()));
		_state = new_state;
	}
}

void GnssAnalyzer::maybeLogSuspicion()
{
	if (hrt_elapsed_time(&_last_diaglog_us) < kDiagnosticLogPeriodUs)
	{
		return;
	}

	_last_diaglog_us = hrt_absolute_time();
	PX4_INFO("GNSSAnalyzer: state=%u (vel_sus1=%.2f vel_sus2=%.2f pos_sus=%.2f)",
		static_cast<unsigned>(_state),
		static_cast<double>(_imu_velocity_analyzer.suspicion()),
		static_cast<double>(_raw_velocity_analyzer.suspicion()),
		static_cast<double>(_position_analyzer.suspicion()));
}

void GnssAnalyzer::reset(bool origin_valid)
{
	_gnss_kf.reset();
	_high_freq_imu_history.reset();
	_gnss_endpoint_history.reset();
	_gnss_raw_history.reset();
	_trusted_position_history.reset();
	_imu_cumulative_velocity_ned.setZero();
	_imu_cumulative_velocity_variance.setZero();

	const float initial_suspicion = origin_valid ? kSpoofThreshold : 0.f;
	_imu_velocity_analyzer.reset(initial_suspicion);
	_raw_velocity_analyzer.reset(initial_suspicion);
	_position_analyzer.reset(initial_suspicion);

	transitionTo(origin_valid ? GnssSpoofingState::Untrusted : GnssSpoofingState::NoOrigin);
}

void GnssAnalyzer::pushIMU(const DeltaVelocityEarth &sample)
{
	if (!_high_freq_imu_history.empty() && _high_freq_imu_history.newest().time_us >= sample.time_us)
	{
		return;
	}

	_imu_cumulative_velocity_ned += sample.delta_velocity_ned;
	_imu_cumulative_velocity_variance += sample.delta_velocity_variance_ned;
	_high_freq_imu_history.push(IMUCumulativeVelocityEndpoint{
		.time_us = sample.time_us,
		.cumulative_velocity = _imu_cumulative_velocity_ned,
		.cumulative_velocity_variance = _imu_cumulative_velocity_variance});
}

void GnssAnalyzer::pushTrustedPosition(const TrustedPositionSample &sample)
{
	_trusted_position_history.push(sample);

	if (_state != GnssSpoofingState::NoOrigin)
	{
		_position_analyzer.analyze(_gnss_endpoint_history, _trusted_position_history);
		recalculateState();
		maybeLogSuspicion();
	}
}

void GnssAnalyzer::pushGnss(const GnssKalmanFilter::Measurement &sample)
{
	if (!_gnss_raw_history.empty() && sample.time_us <= _gnss_raw_history.newest().time_us)
	{
		return;
	}

	_gnss_raw_history.push(GnssRaw{
		.time_us = sample.time_us,
		.gnss_position_ned = sample.pos_ned,
		.gnss_velocity_ned = sample.vel_ned});

	const auto result = _gnss_kf.process(sample);

	if (!result)
	{
		PX4_WARN("GnssAnalyzer: failed to process Gnss sample in KF. Skipping.");
		return;
	}

	const auto bracket = _high_freq_imu_history.findBracket(sample.time_us);

	if (!bracket)
	{
		PX4_WARN("GnssAnalyzer: cannot find the nearby IMU samples for this Gnss sample.");
		return;
	}

	const IMUCumulativeVelocityEndpoint &before = _high_freq_imu_history.atOldestOffset(bracket->before);
	const IMUCumulativeVelocityEndpoint &after = _high_freq_imu_history.atOldestOffset(bracket->after);
	const matrix::Vector3f imu_velocity = lerp(
		before.cumulative_velocity,
		after.cumulative_velocity,
		before.time_us,
		after.time_us,
		sample.time_us);
	const auto &state = _gnss_kf.state();
	const auto &covariance = _gnss_kf.covariance();

	_gnss_endpoint_history.push(GnssEndpoint{
		.time_us = sample.time_us,
		.gnss_position_ned = {state(0), state(1), state(2)},
		.gnss_position_ned_variance = {covariance(0, 0), covariance(1, 1), covariance(2, 2)},
		.gnss_velocity_ned = {state(3), state(4), state(5)},
		.gnss_velocity_ned_variance = {covariance(3, 3), covariance(4, 4), covariance(5, 5)},
		.imu_cumulative_delta_velocity_ned = imu_velocity,
		.imu_cumulative_delta_velocity_variance = lerp(
			before.cumulative_velocity_variance,
			after.cumulative_velocity_variance,
			before.time_us,
			after.time_us,
			sample.time_us)});

	if (_state != GnssSpoofingState::NoOrigin)
	{
		_imu_velocity_analyzer.analyze(_gnss_endpoint_history);
		_raw_velocity_analyzer.analyze(_gnss_raw_history);
		_position_analyzer.analyze(_gnss_endpoint_history, _trusted_position_history);
		recalculateState();
		maybeLogSuspicion();
	}
}

void GnssAnalyzer::recalculateState()
{
	const float total_suspicion = math::max(
		_imu_velocity_analyzer.suspicion(),
		_raw_velocity_analyzer.suspicion(),
		_position_analyzer.suspicion());

	if (total_suspicion >= kSpoofThreshold)
	{
		transitionTo(GnssSpoofingState::Untrusted);

	}
	else if (total_suspicion <= kUnspoofThreshold)
	{
		transitionTo(GnssSpoofingState::Trusted);
	}
}
