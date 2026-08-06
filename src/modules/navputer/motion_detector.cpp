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
 * Implementation of the attitude and position estimator.
 *
 * @author
 */

#include "motion_detector.hpp"

#include <matrix/Vector2.hpp>

#include <cmath>

MotionDetector::MotionDetector(ModuleParams *parent)
	: ModuleParams(parent)
{
}

void MotionDetector::reset()
{
	_state = State::Moving;
	_stationary_candidate_started_at = 0;
}

bool MotionDetector::is_imu_valid(const Input& input) const
{
	const bool valid = input.delta_angle_dt > 0
		&& input.delta_velocity_dt > 0
		&& input.delta_angle.isAllFinite()
		&& input.delta_velocity.isAllFinite();
	return valid;
}

void MotionDetector::recalculate_state(const bool stationary_enough, const bool definitely_moving,
				       const hrt_abstime timestamp)
{
	switch (_state) {
		case State::Unknown:
		case State::Moving:
			if (stationary_enough)
			{
				_state = State::MaybeStationary;
				_stationary_candidate_started_at = timestamp;
			}
			else
			{
				_state = State::Moving;
				_stationary_candidate_started_at = 0;
			}
			break;

		case State::MaybeStationary:
			if (definitely_moving || !stationary_enough)
			{
				_state = State::Moving;
				_stationary_candidate_started_at = 0;

			}
			else if (timestamp >= _stationary_candidate_started_at
				 + static_cast<hrt_abstime>(_param_motdet_stat_confirmation_time.get()))
			{
				_state = State::Stationary;
			}
			break;

		case State::Stationary:
			if (definitely_moving)
			{
				_state = State::Moving;
				_stationary_candidate_started_at = 0;
			}
			break;
	}
}

void MotionDetector::update(const Input& input)
{
	if (!is_imu_valid(input))
	{
		reset();
		return;
	}

	const matrix::Vector3f gyro_rate = input.delta_angle / input.delta_angle_dt;
	const matrix::Vector3f accel_rate = input.delta_velocity / input.delta_velocity_dt;

	const float gyro_magnitude = gyro_rate.norm();
	const float accel_magnitude = fabsf(accel_rate.norm() - kGravityM_s2);

	float hor_speed = 0.f;

	if (input.velocity_valid && input.velocity.isAllFinite())
	{
		hor_speed = matrix::Vector2f{input.velocity(0), input.velocity(1)}.norm();
	}

	const bool stationary_enough =
		gyro_magnitude <= _param_motdet_gate_stat_gyro.get()
		&& accel_magnitude <= _param_motdet_gate_stat_accel.get()
		&& (!input.velocity_valid || hor_speed <= _param_motdet_gate_stat_speed.get());

	const bool definitely_moving =
		gyro_magnitude > _param_motdet_gate_mot_gyro.get()
		|| accel_magnitude > _param_motdet_gate_mot_accel.get()
		|| (input.velocity_valid && hor_speed > _param_motdet_gate_mot_speed.get());

	recalculate_state(stationary_enough, definitely_moving, input.timestamp);
}

MotionDetector::State MotionDetector::state() const
{
	return _state;
}
