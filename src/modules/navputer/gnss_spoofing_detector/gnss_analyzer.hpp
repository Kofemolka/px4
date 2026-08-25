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

/*
 * Uninitialized <-> Spoofed
 * Uninitialized <-> Healthy
 * Healthy <-> Spoofed
 */

enum class GnssSpoofingState
{
	NoOrigin,
	Trusted,
	Untrusted
};

namespace GnssAnalyzerConstants
{
constexpr size_t kHighFreqIMUQueueSize = 512;
constexpr size_t kLowFreqGPSQueueSize = 128;
constexpr size_t kLowFreqTrustedPosQueueSize = 64;
} // GnssAnalyzerConstants

class GnssAnalyzer
{
public:
	// input
	struct TrustedPositionSample {
		uint64_t time_us;
		matrix::Vector2f position_ne;
		matrix::Vector2f position_variance_ne;
	};
public:
	GnssSpoofingState state() const;
	void reset(bool origin_valid);
	void pushIMU(const DeltaVelocityEarth &sample);
	void pushGnss(const GnssKalmanFilter::Measurement &sample);
	void pushTrustedPosition(const TrustedPositionSample &sample);
private:
	// internal
	struct GnssEndpoint
	{
		uint64_t time_us{0};
		matrix::Vector3f gnss_position_ned{};
		matrix::Vector3f gnss_position_ned_variance{};
		matrix::Vector3f gnss_velocity_ned{};
		matrix::Vector3f imu_cumulative_delta_velocity_ned{};
	};
	struct IMUCumulativeVelocityEndpoint
	{
		uint64_t time_us{0};
		matrix::Vector3f cumulative_velocity{};
	};
private:
	bool getVelEndpoints(GnssEndpoint& recent, GnssEndpoint& old);
	void analyzeVelAnomalies();

	bool getPosEndpoints(TrustedPositionSample& trusted, GnssEndpoint& before, GnssEndpoint& after);
	void analyzePosAnomalies();

	void recalculateState();
private:
	GnssSpoofingState _state{GnssSpoofingState::NoOrigin};
	GnssKalmanFilter _gnss_kf;

	HistoryRingBuffer<IMUCumulativeVelocityEndpoint,
		GnssAnalyzerConstants::kHighFreqIMUQueueSize> _high_freq_imu_history;
	HistoryRingBuffer<GnssEndpoint,
		GnssAnalyzerConstants::kLowFreqGPSQueueSize> _gnss_endpoint_history;
	HistoryRingBuffer<TrustedPositionSample,
		GnssAnalyzerConstants::kLowFreqTrustedPosQueueSize> _trusted_position_history;

	matrix::Vector3f _imu_cumulative_velocity_ned{};

	uint64_t _last_vel_analysis_time_us{0};
	uint64_t _last_pos_analysis_time_us{0};

	float _velocity_suspicion{0};
	float _position_suspicion{0};
};

#endif
