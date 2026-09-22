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
 * @file rngbcn_health_monitor.cpp
 * Implementation of the ranging beacons health monitor.
 *
 * @author
 */

#include "rngbcn_health_monitor.hpp"

#include <lib/mathlib/mathlib.h>

namespace
{
constexpr uint64_t kFreshnessWindowUs = 3'000'000;
} // namespace

void RngBcnHealthMonitor::updateRecent(const uint64_t time_us, const uint8_t id)
{
	if (_recent_bcn_updates[0].id == id)
	{
		_recent_bcn_updates[0].time_us = time_us;
		return;
	}
	else if (_recent_bcn_updates[1].id == id)
	{
		_recent_bcn_updates[1].time_us = time_us;
		return;
	}

	if (_recent_bcn_updates[0].time_us <= _recent_bcn_updates[1].time_us
		&& _recent_bcn_updates[0].time_us < time_us)
	{
		_recent_bcn_updates[0].id = id;
		_recent_bcn_updates[0].time_us = time_us;
	}
	else if (_recent_bcn_updates[1].time_us < _recent_bcn_updates[0].time_us
		&& _recent_bcn_updates[1].time_us < time_us)
	{
		_recent_bcn_updates[1].id = id;
		_recent_bcn_updates[1].time_us = time_us;
	}
}

void RngBcnHealthMonitor::update()
{
	const uint64_t now = hrt_absolute_time();

	// Initial case. We allow beacon fusion from the start.
	if (_recent_bcn_updates[0].time_us == 0UL && _recent_bcn_updates[1].time_us == 0UL)
	{
		_recent_bcn_updates[0].time_us = now;
		_recent_bcn_updates[1].time_us = now;
		return;
	}

	ranging_beacon_s sample;

	if (!_ranging_beacon_sub.update(&sample))
	{
		return;
	}

	const bool measurement_valid =
		PX4_ISFINITE(sample.range)
		&& PX4_ISFINITE(sample.lat)
		&& PX4_ISFINITE(sample.lon)
		&& PX4_ISFINITE(sample.alt);

	if (!measurement_valid)
	{
		return;
	}

	updateRecent(now, sample.beacon_id);
}

bool RngBcnHealthMonitor::healthy() const
{
	const uint64_t now = hrt_absolute_time();
	const bool bcn1_fresh = (now - _recent_bcn_updates[0].time_us) < kFreshnessWindowUs;
	const bool bcn2_fresh = (now - _recent_bcn_updates[1].time_us) < kFreshnessWindowUs;
	return bcn1_fresh && bcn2_fresh;
}
