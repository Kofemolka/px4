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
 * @file gnss_spoofing_detector.cpp
 * Implementation of the GnssSpoofingDetector.
 *
 * @author
 */

#include "gnss_spoofing_detector.hpp"

#include <matrix/helper_functions.hpp>

namespace
{
constexpr double kOriginEpsilon = 1e-8;
constexpr float kMaxStddevMultiplier = 10.f;
} // namespace

void GnssSpoofingDetector::setGnssInstance(const int instance)
{
	if (instance == _gps_sub.get_instance())
	{
		return;
	}

	_gps_sub = uORB::Subscription{ORB_ID(vehicle_gps_position), static_cast<uint8_t>(instance)};
	_analyzer.reset(_origin_valid);
}

void GnssSpoofingDetector::maybeUpdateOrigin()
{
	navput_local_position_s local_position{};

	// Here we check only horizontal changes without an altitude
	if (_local_position_sub.update(&local_position))
	{
		const bool new_origin_valid = local_position.xy_global
					  && PX4_ISFINITE(local_position.ref_lat)
					  && PX4_ISFINITE(local_position.ref_lon);

		// invalidate origin if it is not valid
		if (!new_origin_valid)
		{
			if (_origin_valid)
			{
				_analyzer.reset(new_origin_valid);
			}
			_origin_valid = false;
		}
		// update the origin if it is valid
		else if (!_origin_valid
				|| !matrix::isEqualF(local_position.ref_lat, _origin_lat_deg, kOriginEpsilon)
				|| !matrix::isEqualF(local_position.ref_lon, _origin_lon_deg, kOriginEpsilon))
		{
			_origin_projection.initReference(
				local_position.ref_lat,
				local_position.ref_lon,
				local_position.ref_timestamp);

			_origin_lat_deg = local_position.ref_lat;
			_origin_lon_deg = local_position.ref_lon;
			_origin_valid = true;

			_analyzer.reset(new_origin_valid);
		}
	}
}

void GnssSpoofingDetector::maybeFuseGnss()
{
	sensor_gps_s gps{};
	const bool gps_updated = _gps_sub.update(&gps);

	if (_origin_valid
	    && gps_updated && gps.vel_ned_valid
	    && PX4_ISFINITE(gps.latitude_deg)
	    && PX4_ISFINITE(gps.longitude_deg)
	    && PX4_ISFINITE(gps.altitude_msl_m))
	{
		const matrix::Vector2f gps_pos_ne = _origin_projection.project(gps.latitude_deg, gps.longitude_deg);
		const uint64_t gps_time_us = gps.timestamp_sample > 0 ? gps.timestamp_sample : gps.timestamp;
		const float gps_pos_down_msl = -static_cast<float>(gps.altitude_msl_m);

		_analyzer.pushGnss({
			.time_us = gps_time_us,
			.pos_ned = {gps_pos_ne(0), gps_pos_ne(1), gps_pos_down_msl},
			.vel_ned = {gps.vel_n_m_s, gps.vel_e_m_s, gps.vel_d_m_s},
			.pos_var = {gps.eph * gps.eph, gps.eph * gps.eph, gps.epv * gps.epv},
			.vel_var = {gps.s_variance_m_s * gps.s_variance_m_s,
				    gps.s_variance_m_s * gps.s_variance_m_s,
				    gps.s_variance_m_s * gps.s_variance_m_s}
		});
	}
}

void GnssSpoofingDetector::maybeGrabTrustedPosition()
{
	if (!_origin_valid)
	{
		return;
	}

	for (size_t instance = 0; instance < _aux_global_pos_subs.size(); ++instance)
	{
		aux_global_position_s aux_global_pos{};

		if (!_aux_global_pos_subs[instance].update(&aux_global_pos))
		{
			continue;
		}
		if (aux_global_pos.source != aux_global_position_s::SOURCE_PSEUDOLITES)
		{
			continue;
		}
		if (!PX4_ISFINITE(aux_global_pos.lat)
			|| !PX4_ISFINITE(aux_global_pos.lon)
			|| !PX4_ISFINITE(aux_global_pos.eph)
			|| aux_global_pos.eph < 0.f)
		{
			continue;
		}

		const uint64_t time_us = aux_global_pos.timestamp_sample > 0 ? aux_global_pos.timestamp_sample : aux_global_pos.timestamp;

		if (time_us == 0)
		{
			continue;
		}

		const matrix::Vector2f position_ne = _origin_projection.project(aux_global_pos.lat, aux_global_pos.lon);
		// splitting total variance between N and E
		const float position_variance_per_axis = aux_global_pos.eph * aux_global_pos.eph * 0.5f;

		_analyzer.pushTrustedPosition(GnssAnalyzerTypes::TrustedPositionSample{
			.time_us = time_us,
			.position_ne = position_ne,
			.position_variance_ne = {
				position_variance_per_axis,
				position_variance_per_axis
			}
		});
	}
}

void GnssSpoofingDetector::update(const DeltaVelocityEarth &imu_ned)
{
	maybeUpdateOrigin();

	_analyzer.pushIMU(imu_ned);
	maybeGrabTrustedPosition();
	maybeFuseGnss();
}

GnssSpoofingDetector::SpoofReport GnssSpoofingDetector::report() const
{
	const float suspicion = _analyzer.suspicion();
	const float normalized = math::constrain(suspicion / GnssAnalyzerTypes::kSpoofThreshold, 0.f, 1.f);
	// normalized^2 gives gentler earlier response comparing to linear
	const float stddev_multiplier = 1.f + (kMaxStddevMultiplier - 1.f) * normalized * normalized;

	return SpoofReport {
		.state = _analyzer.state(),
		.pos_stddev_mult = stddev_multiplier,
		.vel_stddev_mult = stddev_multiplier
	};
}

