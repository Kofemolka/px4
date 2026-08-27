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
constexpr uint64_t kGpsFreqHz = 8;
constexpr uint64_t kImuFreqHz = 300;
constexpr uint64_t kTwiceGpsPeriodUs = 2'000'000ULL / kGpsFreqHz;
constexpr uint64_t kImuPeriodUs = 1'000'000ULL / kImuFreqHz;

constexpr uint64_t kVelWindowDurationUs = 2'000'000; // 2 second window

// Twice larger than the sufficient imu history capacity that should encompass two GPS periods ~250 ms
constexpr size_t kHighFreqIMUQueueSize = (kTwiceGpsPeriodUs / kImuPeriodUs) * 2ULL;
// Twice larger than the sufficient gnss history
constexpr size_t kGnssQueueSize = (kVelWindowDurationUs / 1'000'000ULL) * kGpsFreqHz * 2ULL;
constexpr size_t kTrustedPosQueueSize = 2;

// internal
struct GnssEndpoint
{
	uint64_t time_us{0};
	matrix::Vector3f gnss_position_ned{};	matrix::Vector3f gnss_position_ned_variance{};
	matrix::Vector3f gnss_velocity_ned{};
	matrix::Vector3f imu_cumulative_delta_velocity_ned{};
};
struct GnssRaw
{
	uint64_t time_us{0};
	matrix::Vector3f gnss_position_ned{};
	matrix::Vector3f gnss_velocity_ned{};
};
struct IMUCumulativeVelocityEndpoint
{
	uint64_t time_us{0};
	matrix::Vector3f cumulative_velocity{};
};
struct TrustedPositionSample
{
	uint64_t time_us;
	matrix::Vector2f position_ne;
	matrix::Vector2f position_variance_ne;
};

using CummulativeImuHistory = HistoryRingBuffer<
	IMUCumulativeVelocityEndpoint,
	GnssAnalyzerTypes::kHighFreqIMUQueueSize>;
using GnssEndpointHistory = HistoryRingBuffer<
	GnssEndpoint,
	GnssAnalyzerTypes::kGnssQueueSize>;
using GnssRawHistory = HistoryRingBuffer<
	GnssRaw,
	GnssAnalyzerTypes::kGnssQueueSize>;
using TrustedPositionHistory = HistoryRingBuffer<
	TrustedPositionSample,
	GnssAnalyzerTypes::kTrustedPosQueueSize>;
} // GnssAnalyzerTypes


class BasicAnomalyAnalyzer
{
public:
	void reset(float initial_suspicion)
	{
		_suspicion = initial_suspicion;
	}

	float suspicion() const
	{
		return _suspicion;
	}

protected:
	float _suspicion;
};

class GnssImuDeltaVelocityAnalyzer final : public BasicAnomalyAnalyzer
{
public:
	void analyze(const GnssAnalyzerTypes::GnssEndpointHistory& history);
};

class RawGnssVelocityAnalyzer final : public BasicAnomalyAnalyzer
{
public:
	void analyze(const GnssAnalyzerTypes::GnssEndpointHistory& history);
};

class GnssMlatPosAnalyzer final : public BasicAnomalyAnalyzer
{
public:
	void analyze(const GnssAnalyzerTypes::TrustedPositionHistory& history);
};

class GnssAnalyzer
{
public:
	// input
public:
	GnssSpoofingState state() const;
	void reset(bool origin_valid);
	void pushIMU(const DeltaVelocityEarth &sample);
	void pushGnss(const GnssKalmanFilter::Measurement &sample);
	void pushTrustedPosition(const GnssAnalyzerTypes::TrustedPositionSample &sample);
private:
private:
	void transitionTo(GnssSpoofingState new_state);
	void maybeLogSuspicion();

	void analyzeVelAnomalies();
	void compareVelDeltaImuWithGnssKF();
	void compareVelAvgRawGnss();

	void analyzePosAnomalies();

	void recalculateState();
private:
	GnssSpoofingState _state{GnssSpoofingState::NoOrigin};
	GnssKalmanFilter _gnss_kf;

	GnssAnalyzerTypes::CummulativeImuHistory _high_freq_imu_history;
	GnssAnalyzerTypes::GnssEndpointHistory _gnss_endpoint_history;
	GnssAnalyzerTypes::GnssRawHistory _gnss_raw_history;
	GnssAnalyzerTypes::TrustedPositionHistory _trusted_position_history;

	matrix::Vector3f _imu_cumulative_velocity_ned{};

	uint64_t _last_vel_analysis_time_us{0};
	uint64_t _last_vel_raw_analysis_time_us{0};
	uint64_t _last_pos_analysis_time_us{0};
	uint64_t _last_diaglog_us{0};

	float _vel_delta_imu_with_gnsskf_suspicion{0};
	float _vel_raw_suspicion{0};
	float _position_suspicion{0};
};

#endif
