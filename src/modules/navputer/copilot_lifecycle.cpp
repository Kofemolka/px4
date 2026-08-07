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

namespace {
using namespace time_literals;
constexpr float kForceArmParam = 21196.f;
constexpr uint8_t kBroadcastTarget = 0;
constexpr uint8_t kSourceSystem = 1;
constexpr uint16_t kSourceComponent = 1;
constexpr hrt_abstime kArmRequestInterval{500_ms};
} // namespace

void CopilotLifecycle::update(MotionDetector::State state)
{
	vehicle_status_s vehicle_status;
	if (_vehicle_status_sub.update(&vehicle_status))
	{
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}

	if (state == MotionDetector::State::AirborneMoving
		&& !_armed
		&& hrt_elapsed_time(&_last_arm_request) >= kArmRequestInterval)
	{
		publishArmCommand();
	}
}

void CopilotLifecycle::publishArmCommand()
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();

	command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	command.param1 = static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM);
	command.param2 = kForceArmParam;
	command.target_system = kBroadcastTarget;
	command.target_component = kBroadcastTarget;
	command.source_system = kSourceSystem;
	command.source_component = kSourceComponent;
	command.from_external = false;

	if (_vehicle_command_pub.publish(command))
	{
		_last_arm_request = command.timestamp;
		PX4_INFO("published copilot arm command");
	}
	else
	{
		PX4_WARN("failed to publish copilot arm command");
	}
}
