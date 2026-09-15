/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file NavputTunnel.hpp
 * CDR-serializes a fixed set of navput/estimator uORB topics and streams them
 * to the GCS as fragmented MAVLink TUNNEL messages (via the mavlink_tunnel_out
 * uORB topic, relayed by MavlinkTunnelStream in src/modules/mavlink).
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/mavlink_tunnel.h>
#include <uORB/topics/navput_local_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/aux_global_position.h>
#include <uORB/topics/navput_fusion_control.h>
#include <uORB/topics/navput_gnss_spoof_detector.h>
#include <uORB/topics/ranging_beacon.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/navput_aid_source2d.h>
#include <uORB/topics/navput_aid_source1d.h>
#include <uORB/topics/navput_attitude.h>

using namespace time_literals;

class NavputTunnel : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	NavputTunnel();
	~NavputTunnel() override = default;

	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:
	// Permanent, append-only wire IDs (TUNNEL.payload_type). Never reassign an
	// existing ID; a new topic instance of an already-listed struct gets the
	// next free ID instead of reusing one.
	enum PayloadType : uint16_t {
		PAYLOAD_TYPE_NAVPUT_LOCAL_POSITION      = 1,
		PAYLOAD_TYPE_VEHICLE_LOCAL_POSITION     = 2,
		PAYLOAD_TYPE_VEHICLE_GPS_POSITION       = 3,
		PAYLOAD_TYPE_AUX_GLOBAL_POSITION        = 4,
		PAYLOAD_TYPE_NAVPUT_FUSION_CONTROL      = 5,
		PAYLOAD_TYPE_NAVPUT_GNSS_SPOOF_DETECTOR = 6,
		PAYLOAD_TYPE_RANGING_BEACON             = 7,
		PAYLOAD_TYPE_ESTIMATOR_AID_SOURCE_2D    = 8,
		PAYLOAD_TYPE_NAVPUT_AID_SOURCE_2D       = 9,
		PAYLOAD_TYPE_NAVPUT_AID_SOURCE_1D       = 10,
		PAYLOAD_TYPE_NAVPUT_ATTITUDE            = 11,
	};

	// TUNNEL.payload[0]=frag_total, [1]=frag_index, [2..]=CDR bytes; TUNNEL.payload is 128B.
	static constexpr int MAX_CDR_BYTES_PER_FRAGMENT = 125;

	void Run() override;

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::NPT_LOG_EN>) _param_npt_log_en
	)

	/** CDR-serialize msg (msg_size bytes, described by cdr_ops) and publish its fragments. */
	void serialize_and_publish(uint16_t payload_type, const void *msg, size_t msg_size, const uint32_t *cdr_ops);

	/** Split cdr into MAX_CDR_BYTES_PER_FRAGMENT chunks and publish one mavlink_tunnel_out frame per chunk.
	 * mavlink_tunnel_out has ORB_QUEUE_LENGTH set, so back-to-back publishes here don't overwrite each
	 * other before MavlinkTunnelStream (running on its own, independently-scheduled loop) drains them. */
	void publish_fragments(uint16_t payload_type, const uint8_t *cdr, size_t cdr_len);

	// Per-topic output rate caps, to keep high-rate estimator topics (attitude
	// especially) from clogging the MAVLink link. Fixed at compile time; bump
	// these if a link with more headroom needs more detail.
	uORB::SubscriptionInterval _navput_local_position_sub{ORB_ID(navput_local_position), 200_ms};             // 5 Hz
	uORB::SubscriptionInterval _vehicle_local_position_sub{ORB_ID(vehicle_local_position), 200_ms};           // 5 Hz
	uORB::SubscriptionInterval _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position), 200_ms};               // 5 Hz
	uORB::SubscriptionInterval _aux_global_position_sub{ORB_ID(aux_global_position), 200_ms};                 // 5 Hz
	uORB::SubscriptionInterval _navput_fusion_control_sub{ORB_ID(navput_fusion_control), 1_s};                // 1 Hz
	uORB::SubscriptionInterval _navput_gnss_spoof_detector_sub{ORB_ID(navput_gnss_spoof_detector), 1_s};      // 1 Hz
	uORB::SubscriptionInterval _ranging_beacon_sub{ORB_ID(ranging_beacon), 200_ms};                           // 5 Hz
	uORB::SubscriptionInterval _estimator_aid_src_aux_global_position_sub{ORB_ID(estimator_aid_src_aux_global_position), 200_ms}; // 5 Hz
	uORB::SubscriptionInterval _navput_aid_src_aux_global_position_sub{ORB_ID(navput_aid_src_aux_global_position), 200_ms};       // 5 Hz
	uORB::SubscriptionInterval _navput_aid_src_ranging_beacon_sub{ORB_ID(navput_aid_src_ranging_beacon), 200_ms};                 // 5 Hz
	uORB::SubscriptionInterval _navput_attitude_sub{ORB_ID(navput_attitude), 100_ms};                         // 10 Hz

	uORB::Publication<mavlink_tunnel_s> _tunnel_out_pub{ORB_ID(mavlink_tunnel_out)};
};
