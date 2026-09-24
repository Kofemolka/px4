/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef GLOBAL_POSITION_INT_COV_HPP
#define GLOBAL_POSITION_INT_COV_HPP

#ifdef CONFIG_MAVLINK_SOURCE_NAVPUTER

#include <lib/geo/geo.h>
#include <uORB/topics/navput_local_position.h>
#include <uORB/topics/vehicle_status.h>

class MavlinkStreamGlobalPositionIntCov : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGlobalPositionIntCov(mavlink); }

	static constexpr const char *get_name_static() { return "GLOBAL_POSITION_INT_COV"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _lpos_sub.advertised() ? MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamGlobalPositionIntCov(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _lpos_sub{ORB_ID(navput_local_position)};
	uORB::Subscription _status_sub{ORB_ID(navput_vehicle_status)};

	navput_local_position_s _lpos{};
	bool _lpos_valid{false};

	MapProjection _projection{};
	bool _projection_initialized{false};

	bool _was_armed{false};
	bool _home_capture_pending{false};
	bool _home_valid{false};

	float _home_z{NAN};

	void update_data() override
	{
		navput_local_position_s lpos{};

		if (_lpos_sub.update(&lpos))
		{
			_lpos = lpos;
			_lpos_valid = true;
		}

		vehicle_status_s status{};

		if (_status_sub.update(&status))
		{
			const bool armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

			if (armed && !_was_armed)
			{
				_home_valid = false;
				_home_capture_pending = true;

			}
			else if (!armed)
			{
				_home_valid = false;
				_home_capture_pending = false;
			}

			_was_armed = armed;
		}

		// Capture the latest valid Navputer NED position as home.
		if (_home_capture_pending
			&& _lpos_valid
			&& _lpos.z_valid)
		{
			_home_z = _lpos.z;

			_home_valid = true;
			_home_capture_pending = false;
		}
	}

	bool send() override
	{
		if (!_mavlink->isNavputerOutputEnabled(NAVPUTER_OUTPUT::GLOBAL_POSITION_INT_COV) || !_lpos_valid)
		{
			return false;
		}

		const navput_local_position_s &lpos = _lpos;

		if (!lpos.xy_global || !lpos.z_global)
		{
			return false;
		}

		if (!_projection_initialized
			|| _projection.getProjectionReferenceTimestamp() != lpos.ref_timestamp)
		{
			_projection.initReference(lpos.ref_lat, lpos.ref_lon, lpos.ref_timestamp);
			_projection_initialized = true;
		}

		double lat_deg;
		double lon_deg;
		_projection.reproject(lpos.x, lpos.y, lat_deg, lon_deg);

		mavlink_global_position_int_cov_t msg{};

		msg.time_usec = lpos.timestamp;
		msg.estimator_type = MAV_ESTIMATOR_TYPE_UNKNOWN;
		msg.lat = static_cast<int32_t>(lat_deg * 1e7);
		msg.lon = static_cast<int32_t>(lon_deg * 1e7);
		msg.alt = static_cast<int32_t>((lpos.ref_alt - lpos.z) * 1000.f);
		msg.relative_alt = _home_valid ? static_cast<int32_t>((_home_z - lpos.z) * 1000.f) : 0;
		msg.vx = lpos.vx;
		msg.vy = lpos.vy;
		msg.vz = lpos.vz;

		fillCovariance(msg, lat_deg, lpos);
		mavlink_msg_global_position_int_cov_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}

	static void fillCovariance(
		mavlink_global_position_int_cov_t &msg,
		const double lat_deg,
		const navput_local_position_s &lpos)
	{
		if (!PX4_ISFINITE(lpos.eph)
			|| !PX4_ISFINITE(lpos.epv)
			|| !PX4_ISFINITE(lpos.evh)
			|| !PX4_ISFINITE(lpos.evv)
			|| lpos.eph < 0.f
			|| lpos.epv < 0.f
			|| lpos.evh < 0.f
			|| lpos.evv < 0.f)
		{
			msg.covariance[0] = NAN;
			return;
		}

		const double lat_rad = math::radians(lat_deg);
		const double cos_lat = cos(lat_rad);

		if (fabs(cos_lat) < 1e-6)
		{
			msg.covariance[0] = NAN;
			return;
		}

		// EPH and EVH are 2D errors. Approximate them as equal, uncorrelated N/E errors.
		const double pos_horizontal_variance = static_cast<double>(lpos.eph) * lpos.eph * 0.5;
		const double vel_horizontal_variance = static_cast<double>(lpos.evh) * lpos.evh * 0.5;
		const double radians_to_degrees = 180.0 / M_PI;
		const double lat_degrees_per_meter = radians_to_degrees / CONSTANTS_RADIUS_OF_EARTH;
		const double lon_degrees_per_meter = lat_degrees_per_meter / cos_lat;

		// State order: latitude, longitude, altitude, vx, vy, vz.
		msg.covariance[0] = pos_horizontal_variance * lat_degrees_per_meter * lat_degrees_per_meter;
		msg.covariance[7] = pos_horizontal_variance * lon_degrees_per_meter * lon_degrees_per_meter;
		msg.covariance[14] = static_cast<double>(lpos.epv) * lpos.epv;
		msg.covariance[21] = vel_horizontal_variance;
		msg.covariance[28] = vel_horizontal_variance;
		msg.covariance[35] = static_cast<double>(lpos.evv) * lpos.evv;
	}
};

#endif // CONFIG_MAVLINK_SOURCE_NAVPUTER

#endif // GLOBAL_POSITION_INT_COV_HPP
