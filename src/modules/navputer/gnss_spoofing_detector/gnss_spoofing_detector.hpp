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

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/navput_local_position.h>
#include <uORB/topics/ranging_beacon.h>
#include <uORB/topics/aux_global_position.h>
#include <uORB/topics/navput_gnss_spoof_detector.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <ekf.h>

#include "gnss_analyzer.hpp"

class GnssSpoofingDetector
{
public:
	void update(const DeltaVelocityEarth &imu_ned);
	void setGnssInstance(const int gnss_instance);
	void setAllowedAuxSources(const int32_t mask);
	SpoofReport report() const;
private:
	void maybeUpdateOrigin();
	void maybeFuseGnss();
	void maybeGrabAuxPosition();
	void maybePublishExtendedState();
private:
	GnssAnalyzer _analyzer;

	// origin fields
	double _origin_lat_deg{NAN};
	double _origin_lon_deg{NAN};
	MapProjection _origin_projection{};
	bool _origin_valid{false};

	int32_t _allowed_aux_sources_mask{0};
	uint64_t _last_diaglog_us{0};

	// subscriptions
	uORB::Subscription _gps_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _local_position_sub{ORB_ID(navput_local_position)};
	uORB::SubscriptionMultiArray<aux_global_position_s, 4> _aux_global_pos_subs{ORB_ID::aux_global_position};

	// publication
	uORB::Publication<navput_gnss_spoof_detector_s> _state_pub{ORB_ID(navput_gnss_spoof_detector)};
};

#endif
