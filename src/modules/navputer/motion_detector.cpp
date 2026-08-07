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
 * @file motion_detector.cpp
 * Implementation of the motion detector.
 *
 * @author
 */

#include "motion_detector.hpp"

#include <lib/geo/geo.h>
#include <px4_platform_common/log.h>

#include <cmath>

MotionDetector::MotionDetector(ModuleParams *parent)
	: ModuleParams(parent)
{
}

bool MotionDetector::is_imu_valid(const estimator::imuSample& input) const
{
	const bool valid = input.delta_ang_dt > 0
		&& input.delta_vel_dt > 0
		&& input.delta_ang.isAllFinite()
		&& input.delta_vel.isAllFinite();
	return valid;
}

void MotionDetector::update(const estimator::imuSample& input)
{
	if (_state == State::AirborneMoving)
	{
		return;
	}

	if (!is_imu_valid(input))
	{
		_motion_candidate_started_at = 0;
		return;
	}

	const matrix::Vector3f gyro_rate = input.delta_ang / input.delta_ang_dt;
	const matrix::Vector3f accel_rate = input.delta_vel / input.delta_vel_dt;

	const float gyro_magnitude = gyro_rate.norm();
	const float accel_magnitude = fabsf(accel_rate.norm() - CONSTANTS_ONE_G);

	const bool definitely_moving =
		gyro_magnitude > _param_motion_gyro.get()
		|| accel_magnitude > _param_motion_accel.get();

	if (definitely_moving)
	{
		const hrt_abstime confirmation_time_us =
			static_cast<hrt_abstime>(_param_motion_confirmation_time_ms.get()) * 1000ULL;

		if (_motion_candidate_started_at == 0)
		{
			_motion_candidate_started_at = input.time_us;
		}
		else if (input.time_us >= _motion_candidate_started_at + confirmation_time_us)
		{
			const State previous_state = _state;
			_state = State::AirborneMoving;

			PX4_INFO("state changed: %s -> %s",
				 previous_state == State::LandedStationary ? "LandedStationary" : "AirborneMoving",
				 _state == State::LandedStationary ? "LandedStationary" : "AirborneMoving");
		}
	}
	else
	{
		_motion_candidate_started_at = 0;
	}
}

MotionDetector::State MotionDetector::state() const
{
	return _state;
}
