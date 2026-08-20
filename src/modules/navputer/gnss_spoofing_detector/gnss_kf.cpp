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
 * @file gnss_kf.cpp
 * Implementation of the GnssKalmanFilter.
 *
 * @author
 */

#include <px4_platform_common/log.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#include "gnss_kf.hpp"

namespace
{
constexpr float kAccelerationNoiseDensitySq = 0.25f; // m^2/s^3
constexpr float kMaxGnssGapS = 5.f; // sec

constexpr float kMinHorizontalPositionStd = 0.5f;  // m
constexpr float kMinVerticalPositionStd   = 0.75f; // m

constexpr float kMinHorizontalVelocityStd = 0.3f;  // m/s
constexpr float kMinVerticalVelocityStd   = 0.45f; // m/s
} // namespace 

void GnssKalmanFilter::initialize(const Measurement& sample)
{
	// rebuilding the nominal state
	_state(0) = sample.pos_ned(0);
	_state(1) = sample.pos_ned(1);
	_state(2) = sample.pos_ned(2);
	_state(3) = sample.vel_ned(0);
	_state(4) = sample.vel_ned(1);
	_state(5) = sample.vel_ned(2);

	// rebuilding the covariance matrix
	_P.setZero();
	_P(0, 0) = math::max(sample.pos_var(0), math::sq(kMinHorizontalPositionStd));
	_P(1, 1) = math::max(sample.pos_var(1), math::sq(kMinHorizontalPositionStd));
	_P(2, 2) = math::max(sample.pos_var(2), math::sq(kMinVerticalPositionStd));
	_P(3, 3) = math::max(sample.vel_var(0), math::sq(kMinHorizontalVelocityStd));
	_P(4, 4) = math::max(sample.vel_var(1), math::sq(kMinHorizontalVelocityStd));
	_P(5, 5) = math::max(sample.vel_var(2), math::sq(kMinVerticalVelocityStd));

	_time_us = sample.time_us;
	_initialized = true;
}

bool GnssKalmanFilter::calcTimeDelta(uint64_t sample_ts_us, float& dt)
{
	if (sample_ts_us <= _time_us)
	{
		return false;
	}
	dt = (sample_ts_us - _time_us) * 1e-6f;

	if (dt > kMaxGnssGapS)
	{
		PX4_WARN("GnssKalmanFilter: Too large time delta. Resetting.");
		reset();
		return false;
	}

	return PX4_ISFINITE(dt) && dt > 0;
}

bool GnssKalmanFilter::isMeasurementValid(const Measurement& sample)
{
	const bool valid =
		PX4_ISFINITE(sample.pos_var(0))
		&& PX4_ISFINITE(sample.pos_var(1))
		&& PX4_ISFINITE(sample.pos_var(2))
		&& PX4_ISFINITE(sample.vel_var(0))
		&& PX4_ISFINITE(sample.vel_var(1))
		&& PX4_ISFINITE(sample.vel_var(2));
	return valid;
}

void GnssKalmanFilter::process(const Measurement& sample)
{
	if (!isMeasurementValid(sample))
	{
		PX4_WARN("GnssKalmanFilter: Skipping invalid Gnss measurement");
		return;
	}

	if (!_initialized)
	{
		initialize(sample);
		return;
	}

	float dt = 0;
	if (!calcTimeDelta(sample.time_us, dt))
	{
		return;
	}
	
	predict(dt);

	const bool successful_update = update(sample);
	if (successful_update)
	{
		_time_us = sample.time_us;
	}
}

void GnssKalmanFilter::reset()
{
	_state.setZero();
	_P.setZero();

	_time_us = 0;
	_initialized = false;
}

GnssKalmanFilter::Matrix6f GnssKalmanFilter::buildMotionMatrixF(const float dt)
{
	// F = [ I3   dt*I3 ]
	//     [ 03    I3   ]
	Matrix6f F;
	F.setIdentity();

	F(0, 3) = dt;
	F(1, 4) = dt;
	F(2, 5) = dt;

	return F;
}

GnssKalmanFilter::Matrix6f GnssKalmanFilter::buildProcessNoiseMatrixQ(const float dt)
{
	Matrix6f Q;
	Q.setZero();

	const float q = kAccelerationNoiseDensitySq;
	const float dt2 = dt * dt;
	const float dt3 = dt2 * dt;

	for (int axis = 0; axis < 3; ++axis)
	{
		const int p = axis;
		const int v = axis + 3;

		Q(p, p) = q * dt3 / 3.f;
		Q(p, v) = q * dt2 / 2.f;
		Q(v, p) = q * dt2 / 2.f;
		Q(v, v) = q * dt;
	}

	return Q;
}

GnssKalmanFilter::Matrix6f GnssKalmanFilter::buildMeasurementNoiseMatrixR(const Measurement& sample)
{
	Matrix6f R;
	R.setZero();

	R(0, 0) = math::max(sample.pos_var(0), math::sq(kMinHorizontalPositionStd));
	R(1, 1) = math::max(sample.pos_var(1), math::sq(kMinHorizontalPositionStd));
	R(2, 2) = math::max(sample.pos_var(2), math::sq(kMinVerticalPositionStd));
	R(3, 3) = math::max(sample.vel_var(0), math::sq(kMinHorizontalVelocityStd));
	R(4, 4) = math::max(sample.vel_var(1), math::sq(kMinHorizontalVelocityStd));
	R(5, 5) = math::max(sample.vel_var(2), math::sq(kMinVerticalVelocityStd));

	return R;
}

void GnssKalmanFilter::predict(const float dt)
{
	const auto F = buildMotionMatrixF(dt);

	// move the nominal state
	_state = F * _state;

	// grow the covariance
	const auto Q = buildProcessNoiseMatrixQ(dt);
	_P = F * _P * F.transpose() + Q;
}

bool GnssKalmanFilter::update(const Measurement& sample)
{
	// building measurement vector
	Vector6f z{};
	z(0) = sample.pos_ned(0);
	z(1) = sample.pos_ned(1);
	z(2) = sample.pos_ned(2);
	z(3) = sample.vel_ned(0);
	z(4) = sample.vel_ned(1);
	z(5) = sample.vel_ned(2);

	// innovation = z - H * predicted
	// But we have H = IdentityMatrix
	Vector6f innovation = z - _state;
	const auto R = buildMeasurementNoiseMatrixR(sample);

	// S = H * P * H^T + R
	// Since, H = IdentityMatrix
	Matrix6f S = _P + R;

	// K = P * H^T * S^-1
	decltype(S) S_inv;

	if (!S.I(S_inv))
	{
		PX4_WARN("GnssKalmanFilter: Error with inverting the S matrix. Resetting.");
		reset();
		return false;
	}
	const Matrix6f K = _P * S_inv;

	// correcting nominal state
	_state += K * innovation;

	// shrink covariance
	Matrix6f I;
	I.setIdentity();

	const decltype(K) IK = I - K;

	_P = IK * _P * IK.transpose() + K * R * K.transpose();

	return true;
}
