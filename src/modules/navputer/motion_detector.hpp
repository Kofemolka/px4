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
 * @file motion_detector.hpp
 * Implementation of the attitude and position estimator.
 *
 * @author
 */

#include "EKF/common.h"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

#include <matrix/Vector3.hpp>

#ifndef MOTION_DETECTOR_1234_HPP
#define MOTION_DETECTOR_1234_HPP

class MotionDetector : public ModuleParams
{
public:
	enum class State
	{
		LandedStationary,
		AirborneMoving,
	};
public:
	explicit MotionDetector(ModuleParams *parent);

	void update(const estimator::imuSample& input);
	State state() const;

private:
	bool is_imu_valid(const estimator::imuSample& input) const;

private:
	DEFINE_PARAMETERS(
		// gates for MotionDetector
		(ParamFloat<px4::params::NPT_MD_MOT_GYR>) _param_motion_gyro,
		(ParamFloat<px4::params::NPT_MD_MOT_ACC>) _param_motion_accel,
		(ParamInt<px4::params::NPT_MD_CF_TIME>) _param_motion_confirmation_time_ms
	)

private:
	State _state{State::LandedStationary};
	hrt_abstime _motion_candidate_started_at{0};
};

#endif // !MOTION_DETECTOR_1234_HPP
