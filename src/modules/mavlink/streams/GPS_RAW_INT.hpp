/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef GPS_RAW_INT_HPP
#define GPS_RAW_INT_HPP

#include <lib/gnss/SensorGpsSelector.hpp>
#include <uORB/topics/sensor_gps.h>

#ifdef CONFIG_MAVLINK_SOURCE_NAVPUTER
#include <lib/geo/geo.h>
#include <uORB/topics/navput_local_position.h>
#endif

using namespace time_literals;

class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGPSRawInt(mavlink); }

	static constexpr const char *get_name_static() { return "GPS_RAW_INT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GPS_RAW_INT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

#ifdef CONFIG_MAVLINK_SOURCE_NAVPUTER
	unsigned get_size() override
	{
		return _lpos_sub.advertised() ? MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}
#else //CONFIG_MAVLINK_SOURCE_NAVPUTER
	unsigned get_size() override
	{
		return _sensor_gps_sub.advertised() ? (MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}
#endif //CONFIG_MAVLINK_SOURCE_NAVPUTER

private:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps), 0};
	SensorGpsSelector _gps_selector{};
	hrt_abstime _last_send_ts {};
	static constexpr hrt_abstime kNoGpsSendInterval {1_s};

	bool send() override
	{
		return sendImpl();
	}

#ifdef CONFIG_MAVLINK_SOURCE_NAVPUTER
	uORB::Subscription _lpos_sub{ORB_ID(navput_local_position)};

	navput_local_position_s _lpos{};
	bool _lpos_valid{false};

	MapProjection _projection{};
	bool _projection_initialized{false};

	void update_data() override
	{
		navput_local_position_s lpos{};

		if (_lpos_sub.update(&lpos)) {
			_lpos = lpos;
			_lpos_valid = true;
		}
	}

	bool sendImpl()
	{
		if (!_mavlink->isNavputerOutputEnabled(Mavlink::NAVPUTER_OUTPUT::GPS_RAW_INT) || !_lpos_valid) {
			return false;
		}

		const navput_local_position_s &lpos = _lpos;

		if (!lpos.xy_global || !lpos.z_global || !lpos.xy_valid || !lpos.z_valid) {
			const hrt_abstime now = hrt_absolute_time();

			if ((_last_send_ts == 0) || (now > _last_send_ts + kNoGpsSendInterval)) {
				mavlink_gps_raw_int_t msg{};
				msg.time_usec = now;
				msg.fix_type = GPS_FIX_TYPE_NO_FIX;
				msg.eph = UINT16_MAX;
				msg.epv = UINT16_MAX;
				msg.vel = UINT16_MAX;
				msg.cog = UINT16_MAX;
				msg.satellites_visible = UINT8_MAX;
				msg.h_acc = UINT32_MAX;
				msg.v_acc = UINT32_MAX;
				msg.vel_acc = UINT32_MAX;
				msg.hdg_acc = UINT32_MAX;
				mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);
				_last_send_ts = now;
				return true;
			}

			return false;
		}

		if (!_projection_initialized || _projection.getProjectionReferenceTimestamp() != lpos.ref_timestamp) {
			_projection.initReference(lpos.ref_lat, lpos.ref_lon, lpos.ref_timestamp);
			_projection_initialized = true;
		}

		double lat_deg;
		double lon_deg;
		_projection.reproject(lpos.x, lpos.y, lat_deg, lon_deg);

		mavlink_gps_raw_int_t msg{};
		msg.time_usec = lpos.timestamp;
		msg.fix_type = GPS_FIX_TYPE_3D_FIX;
		msg.lat = static_cast<int32_t>(round(lat_deg * 1e7));
		msg.lon = static_cast<int32_t>(round(lon_deg * 1e7));
		msg.alt = static_cast<int32_t>(round((lpos.ref_alt - lpos.z) * 1e3f));
		msg.eph = UINT16_MAX; // Navputer has EPH, not GNSS HDOP.
		msg.epv = UINT16_MAX; // Navputer has EPV, not GNSS VDOP.
		msg.satellites_visible = UINT8_MAX;
		msg.h_acc = UINT32_MAX;
		msg.v_acc = UINT32_MAX;
		msg.vel_acc = UINT32_MAX;
		msg.hdg_acc = UINT32_MAX;

		if (lpos.v_xy_valid && PX4_ISFINITE(lpos.vx) && PX4_ISFINITE(lpos.vy)) {
			const float groundspeed = sqrtf(lpos.vx * lpos.vx + lpos.vy * lpos.vy);
			msg.vel = static_cast<uint16_t>(math::min(groundspeed * 100.f, static_cast<float>(UINT16_MAX)));

			if (groundspeed > FLT_EPSILON) {
				msg.cog = static_cast<uint16_t>(math::degrees(matrix::wrap_2pi(atan2f(lpos.vy, lpos.vx))) * 100.f);
			} else {
				msg.cog = UINT16_MAX;
			}

			if (PX4_ISFINITE(lpos.evh) && lpos.evh >= 0.f) {
				msg.vel_acc = static_cast<uint32_t>(math::min(lpos.evh * 1e3f, static_cast<float>(UINT32_MAX)));
			}

		} else {
			msg.vel = UINT16_MAX;
			msg.cog = UINT16_MAX;
		}

		if (PX4_ISFINITE(lpos.eph) && lpos.eph >= 0.f) {
			msg.h_acc = static_cast<uint32_t>(math::min(lpos.eph * 1e3f, static_cast<float>(UINT32_MAX)));
		}

		if (PX4_ISFINITE(lpos.epv) && lpos.epv >= 0.f) {
			msg.v_acc = static_cast<uint32_t>(math::min(lpos.epv * 1e3f, static_cast<float>(UINT32_MAX)));
		}

		if (lpos.heading_good_for_control && PX4_ISFINITE(lpos.heading)) {
			msg.yaw = static_cast<uint16_t>(math::degrees(matrix::wrap_2pi(lpos.heading)) * 100.f);

			if (PX4_ISFINITE(lpos.heading_var) && lpos.heading_var >= 0.f) {
				msg.hdg_acc = static_cast<uint32_t>(math::min(math::degrees(sqrtf(lpos.heading_var)) * 1e5f,
											 static_cast<float>(UINT32_MAX)));
			}
		}

		mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);
		_last_send_ts = hrt_absolute_time();
		return true;
	}
#else //CONFIG_MAVLINK_SOURCE_NAVPUTER
	bool sendImpl()
	{
		const uint8_t primary = _gps_selector.primary_instance();

		if (primary != _sensor_gps_sub.get_instance()) {
			_sensor_gps_sub.ChangeInstance(primary);
		}

		sensor_gps_s gps;
		mavlink_gps_raw_int_t msg{};
		hrt_abstime now{};

		// only report the primary receiver, never another instance's data
		if ((_sensor_gps_sub.get_instance() == primary) && _sensor_gps_sub.update(&gps)) {
			if (gps.time_utc_usec <= 0) {
				msg.time_usec = gps.timestamp;

			} else {
				msg.time_usec = gps.time_utc_usec;
			}

			msg.fix_type = gps.fix_type;
			msg.lat = static_cast<int32_t>(round(gps.latitude_deg * 1e7));
			msg.lon = static_cast<int32_t>(round(gps.longitude_deg * 1e7));
			msg.alt = static_cast<int32_t>(round(gps.altitude_msl_m * 1e3)); // convert [m] to [mm]
			msg.eph = gps.hdop * 100; // GPS HDOP horizontal dilution of position (unitless)
			msg.epv = gps.vdop * 100; // GPS VDOP vertical dilution of position (unitless)

			if (PX4_ISFINITE(gps.vel_m_s) && (fabsf(gps.vel_m_s) >= 0.f)) {
				msg.vel = gps.vel_m_s * 100.f; // cm/s

			} else {
				msg.vel = UINT16_MAX; // If unknown, set to: UINT16_MAX
			}

			msg.cog = math::degrees(matrix::wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;
			msg.alt_ellipsoid = static_cast<int32_t>(round(gps.altitude_ellipsoid_m * 1e3)); // convert [m] to [mm]
			msg.h_acc = gps.eph * 1e3f;              // position uncertainty in mm
			msg.v_acc = gps.epv * 1e3f;              // altitude uncertainty in mm
			msg.vel_acc = gps.s_variance_m_s * 1e3f; // speed uncertainty in mm

			if (PX4_ISFINITE(gps.heading)) {
				if (fabsf(gps.heading) < FLT_EPSILON) {
					msg.yaw = 36000; // Use 36000 for north.

				} else {
					msg.yaw = math::degrees(matrix::wrap_2pi(gps.heading)) * 100.0f; // centidegrees
				}

				if (PX4_ISFINITE(gps.heading_accuracy)) {
					msg.hdg_acc = math::degrees(gps.heading_accuracy) * 1e5f; // Heading / track uncertainty in degE5
				}
			}

			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);
			_last_send_ts = gps.timestamp;

			return true;

		} else if (_last_send_ts != 0 && (now = hrt_absolute_time()) > _last_send_ts + kNoGpsSendInterval) {
			msg.fix_type = GPS_FIX_TYPE_NO_GPS;
			msg.eph = UINT16_MAX;
			msg.epv = UINT16_MAX;
			msg.vel = UINT16_MAX;
			msg.cog = UINT16_MAX;
			msg.satellites_visible = UINT8_MAX;
			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);
			_last_send_ts = now;

			return true;
		}

		return false;
	}
#endif //CONFIG_MAVLINK_SOURCE_NAVPUTER
};

#endif // GPS_RAW_INT_HPP
