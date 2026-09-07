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
 * @file gnss_analyzer.hpp
 * Implementation of the GnssAnalyzer.
 *
 * @author
 */

#ifndef GNSS_ANALYZER_HPP
#define GNSS_ANALYZER_HPP

#include <ekf.h>

#include "history_ring_buffer.hpp"
#include "gnss_kf.hpp"

// NoOrigin <-> Spoofed <-> Healthy
enum class GnssSpoofingState
{
	NoOrigin,
	Trusted,
	Untrusted
};

namespace GnssAnalyzerTypes
{
constexpr float kSpoofThreshold = 0.8f;
constexpr float kUnspoofThreshold = 0.2f;

constexpr uint64_t kGpsFreqHz = 8;
constexpr uint64_t kImuFreqHz = 300;
constexpr uint64_t kTwiceGpsPeriodUs = 2'000'000ULL / kGpsFreqHz;
constexpr uint64_t kImuPeriodUs = 1'000'000ULL / kImuFreqHz;

constexpr uint64_t kVelWindowDurationUs = 2'000'000; // 2 second window

// Twice larger than the sufficient imu history capacity that should encompass two GPS periods ~250 ms
constexpr size_t kHighFreqImuQueueSize = (kTwiceGpsPeriodUs / kImuPeriodUs) * 2ULL;
// Twice larger than the sufficient gnss history
constexpr size_t kGnssQueueSize = (kVelWindowDurationUs / 1'000'000ULL) * kGpsFreqHz * 2ULL;
constexpr size_t kMlatPosQueueSize = 2;

// internal
struct GnssEndpoint
{
	uint64_t time_us{0};
	matrix::Vector3f gnss_position_ned{};
	matrix::Vector3f gnss_position_ned_variance{};
	matrix::Vector3f gnss_velocity_ned{};
	matrix::Vector3f gnss_velocity_ned_variance{};
	matrix::Vector3f imu_cumulative_delta_velocity_ned{};
	matrix::Vector3f imu_cumulative_delta_velocity_variance{};
};
struct GnssRaw
{
	uint64_t time_us{0};
	matrix::Vector3f gnss_position_ned{};
	matrix::Vector3f gnss_velocity_ned{};
};
struct ImuCumulativeVelocityEndpoint
{
	uint64_t time_us{0};
	matrix::Vector3f cumulative_velocity{};
	matrix::Vector3f cumulative_velocity_variance{};
};
struct MlatPositionSample
{
	uint64_t time_us;
	matrix::Vector2f position_ne;
	matrix::Vector2f position_variance_ne;
};

using CummulativeImuHistory = HistoryRingBuffer<
	ImuCumulativeVelocityEndpoint,
	GnssAnalyzerTypes::kHighFreqImuQueueSize>;
using GnssEndpointHistory = HistoryRingBuffer<
	GnssEndpoint,
	GnssAnalyzerTypes::kGnssQueueSize>;
using GnssRawHistory = HistoryRingBuffer<
	GnssRaw,
	GnssAnalyzerTypes::kGnssQueueSize>;
using MlatPositionHistory = HistoryRingBuffer<
	MlatPositionSample,
	GnssAnalyzerTypes::kMlatPosQueueSize>;

class BasicAnomalyAnalyzer
{
public:
	void reset(float initial_suspicion);
	float suspicion() const;

protected:
	void updateWindowedSuspicion(float error, float safe_error, float severe_error, float window_fraction);

	float _suspicion{0.f};
	uint64_t _last_analysis_time_us{0};
};

class GnssImuDeltaVelocityAnalyzer final : public BasicAnomalyAnalyzer
{
public:
	void reset(float initial_suspicion);
	void analyze(const GnssAnalyzerTypes::GnssEndpointHistory &history);
};

class GnssVelocityConsistencyAnalyzer final : public BasicAnomalyAnalyzer
{
public:
	void reset(float initial_suspicion);
	void analyze(const GnssAnalyzerTypes::GnssRawHistory &history);
};

class GnssMlatPosAnalyzer final : public BasicAnomalyAnalyzer
{
public:
	void reset(float initial_suspicion);
	void analyze(const GnssAnalyzerTypes::GnssEndpointHistory &gnss_history,
		     const GnssAnalyzerTypes::MlatPositionHistory &mlat_history);
	uint64_t lastSuccessfulAnalysisTime() const;

private:
	void updateSuspicion(float normalized_error);

	bool grabSamples(const GnssAnalyzerTypes::GnssEndpointHistory &gnss_history,
			 const GnssAnalyzerTypes::MlatPositionHistory &mlat_history,
			 GnssAnalyzerTypes::MlatPositionSample &mlat,
			 GnssAnalyzerTypes::GnssEndpoint &before,
			 GnssAnalyzerTypes::GnssEndpoint &after);
private:
	uint64_t _last_successful_analysis_time_us{0};
};

struct IndependentRecoveryLatch
{
	bool canBeTrusted(const uint64_t last_successful_mlat_analysis_us) const
	{
		return !_recovery_needs_mlat_confirmation
			|| last_successful_mlat_analysis_us > _recovery_required_after_us;
	}

	void setNeedForRecoveryAfter(const uint64_t gnss_time_us)
	{
		_recovery_needs_mlat_confirmation = true;
		_recovery_required_after_us = gnss_time_us;
	}

	void clear()
	{
		_recovery_needs_mlat_confirmation = false;
	}

	bool _recovery_needs_mlat_confirmation{false};
	uint64_t _recovery_required_after_us{0};
};

} // namespace GnssAnalyzerTypes

class GnssAnalyzer
{
public:
	GnssSpoofingState state() const;
	float suspicion() const;

	void reset(bool origin_valid);

	void pushImu(const DeltaVelocityEarth &sample);
	void pushGnss(const GnssKalmanFilter::Measurement &sample);
	void pushMlatPosition(const GnssAnalyzerTypes::MlatPositionSample &sample);
private:
	void transitionTo(GnssSpoofingState new_state);
	void maybeLogSuspicion();
	void recalculateState(const uint64_t last_gnss_sample);
	void resetInternalGnssKF();
private:
	GnssSpoofingState _state{GnssSpoofingState::NoOrigin};
	GnssKalmanFilter _gnss_kf;

	GnssAnalyzerTypes::CummulativeImuHistory _high_freq_imu_history;
	GnssAnalyzerTypes::GnssEndpointHistory _gnss_endpoint_history;
	GnssAnalyzerTypes::GnssRawHistory _gnss_raw_history;
	GnssAnalyzerTypes::MlatPositionHistory _mlat_position_history;

	GnssAnalyzerTypes::IndependentRecoveryLatch _recovery_latch;

	matrix::Vector3f _imu_cumulative_velocity_ned{};
	matrix::Vector3f _imu_cumulative_velocity_variance{};

	GnssAnalyzerTypes::GnssImuDeltaVelocityAnalyzer _imu_velocity_analyzer;
	GnssAnalyzerTypes::GnssVelocityConsistencyAnalyzer _gnss_velocity_consistency_analyzer;
	GnssAnalyzerTypes::GnssMlatPosAnalyzer _position_analyzer;

	uint64_t _last_diaglog_us{0};
};

#endif
