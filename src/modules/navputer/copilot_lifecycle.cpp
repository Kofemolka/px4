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
 * @file copilot_lifecycle.cpp
 * Implementation of the global state controller.
 *
 * @author
 */

#include "copilot_lifecycle.hpp"

#include <px4_platform_common/log.h>

void CopilotLifecycle::update(MotionDetector::State state)
{
	vehicle_status_s vehicle_status;
	const bool vehicle_status_updated = _vehicle_status_sub.update(&vehicle_status);
	if (vehicle_status_updated)
	{
		_vehicle_status = vehicle_status;
	}

	vehicle_control_mode_s vehicle_control_mode;
	const bool vehicle_control_mode_updated = _vehicle_control_mode_sub.update(&vehicle_control_mode);
	if (vehicle_control_mode_updated)
	{
		_vehicle_control_mode = vehicle_control_mode;
	}

	const bool state_changed = state != _state;

	if (state_changed)
	{
		_state = state;
		PX4_INFO("copilot calibration lifecycle armed: %s", isArmed() ? "true" : "false");
	}

	if (state_changed || vehicle_status_updated || vehicle_control_mode_updated)
	{
		auto timestamp = hrt_absolute_time();
		publishVehicleStatus(timestamp);
		publishVehicleControlMode(timestamp);
	}
}

bool CopilotLifecycle::isArmed() const
{
	return _state == MotionDetector::State::AirborneMoving
	       || _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
	       || _vehicle_control_mode.flag_armed;
}

void CopilotLifecycle::publishVehicleStatus(hrt_abstime timestamp)
{
	vehicle_status_s navput_vehicle_status{_vehicle_status};
	navput_vehicle_status.timestamp = timestamp;
	navput_vehicle_status.arming_state = isArmed()
			? vehicle_status_s::ARMING_STATE_ARMED
			: vehicle_status_s::ARMING_STATE_DISARMED;

	if (!_navput_vehicle_status_pub.publish(navput_vehicle_status))
	{
		PX4_WARN("failed to publish navput_vehicle_status");
	}
}

void CopilotLifecycle::publishVehicleControlMode(hrt_abstime timestamp)
{
	vehicle_control_mode_s navput_vehicle_control_mode{_vehicle_control_mode};
	navput_vehicle_control_mode.timestamp = timestamp;
	navput_vehicle_control_mode.flag_armed = isArmed();

	if (!_navput_vehicle_control_mode_pub.publish(navput_vehicle_control_mode))
	{
		PX4_WARN("failed to publish navput_vehicle_control_mode");
	}
}
