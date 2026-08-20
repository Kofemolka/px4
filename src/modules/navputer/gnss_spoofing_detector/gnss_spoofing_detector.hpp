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
 * @file gnss_spoofing_detector.hpp
 * Implementation of the GnssSpoofingDetector.
 *
 * @author
 */

#ifndef GNSS_SPOOFING_DETECTOR_HPP
#define GNSS_SPOOFING_DETECTOR_HPP

#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/navput_local_position.h>
#include <uORB/topics/navput_status_flags.h>
#include <uORB/topics/ranging_beacon.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <ekf.h>

#include "gnss_analyzer.hpp"

class GnssSpoofingDetector
{
public:
	void update(const ImmediateDeltaVelocityEarth &imu_ned);
	GnssSpoofingState state() const;
private:
	void maybeUpdateOrigin();
	void maybeFuseGnss();
private:
	GnssAnalyzer _analyzer;

	// origin fields
	MapProjection _origin_projection{};
	float _origin_alt{NAN};
	uint64_t _origin_timestamp{0};
	bool _origin_valid{false};

	// subscriptions
	uORB::Subscription _gps_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _local_position_sub{ORB_ID(navput_local_position)};
	uORB::Subscription _status_sub{ORB_ID(navput_status_flags)};
};

#endif
