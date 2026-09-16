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

#include "NavputTunnel.hpp"

#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <dds_serializer.h>

#include <px4/msg/NavputLocalPosition.h>
#include <px4/msg/VehicleLocalPosition.h>
#include <px4/msg/SensorGps.h>
#include <px4/msg/AuxGlobalPosition.h>
#include <px4/msg/NavputFusionControl.h>
#include <px4/msg/NavputGnssSpoofDetector.h>
#include <px4/msg/RangingBeacon.h>
#include <px4/msg/EstimatorAidSource2d.h>
#include <px4/msg/NavputAidSource2d.h>
#include <px4/msg/NavputAidSource1d.h>
#include <px4/msg/NavputAttitude.h>
#include <px4/msg/NavputStatusFlags.h>

using namespace time_literals;

ModuleBase::Descriptor NavputTunnel::desc{task_spawn, custom_command, print_usage};

NavputTunnel::NavputTunnel() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

int NavputTunnel::task_spawn(int argc, char *argv[])
{
	NavputTunnel *obj = new NavputTunnel();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	desc.object.store(obj);
	desc.task_id = task_id_is_work_queue;

	obj->ScheduleOnInterval(5_ms);

	return 0;
}

void NavputTunnel::serialize_and_publish(uint16_t payload_type, const void *msg, size_t msg_size,
		const uint32_t *cdr_ops)
{
	// Largest message handled here is 220B of CDR; sized with generous headroom.
	static constexpr size_t MAX_CDR_BUFFER = 320;

	if (msg_size + 4 + CDR_SAFETY_MARGIN > MAX_CDR_BUFFER) {
		PX4_ERR("navput_tunnel: message too large for payload_type %u (%zu bytes)", payload_type, msg_size);
		return;
	}

	uint8_t buf[MAX_CDR_BUFFER];
	memcpy(buf, ros2_header, sizeof(ros2_header));

	dds_ostream_t os;
	os.m_buffer = &buf[4];
	os.m_index = 0;
	os.m_size = (uint32_t)(sizeof(buf) - 4);
	os.m_xcdr_version = DDSI_RTPS_CDR_ENC_VERSION_1;

	if (!dds_stream_write(&os, &dds_allocator, (const char *)msg, cdr_ops)) {
		PX4_ERR("navput_tunnel: CDR serialization failed for payload_type %u", payload_type);
		return;
	}

	// Send the full CDR blob (4-byte encapsulation header + fields) as-is; the
	// receiver's deserialize() expects the header and skips it at a fixed offset.
	publish_fragments(payload_type, buf, 4 + os.m_index);
}

void NavputTunnel::publish_fragments(uint16_t payload_type, const uint8_t *cdr, size_t cdr_len)
{
	const uint8_t frag_total = (uint8_t)((cdr_len + MAX_CDR_BYTES_PER_FRAGMENT - 1) / MAX_CDR_BYTES_PER_FRAGMENT);

	for (uint8_t frag_index = 0; frag_index < frag_total; ++frag_index) {
		const size_t offset = (size_t)frag_index * MAX_CDR_BYTES_PER_FRAGMENT;
		const size_t remaining = cdr_len - offset;
		const size_t chunk_len = remaining < MAX_CDR_BYTES_PER_FRAGMENT ? remaining : MAX_CDR_BYTES_PER_FRAGMENT;

		mavlink_tunnel_s tunnel{};
		tunnel.timestamp = hrt_absolute_time();
		tunnel.payload_type = payload_type;
		tunnel.target_system = 255; // GCS
		tunnel.target_component = 0;
		tunnel.payload_length = 2 + chunk_len;
		tunnel.payload[0] = frag_total;
		tunnel.payload[1] = frag_index;
		memcpy(&tunnel.payload[2], cdr + offset, chunk_len);

		_tunnel_out_pub.publish(tunnel);
	}
}

void NavputTunnel::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	_param_npt_log_en.update();

	if (!_param_npt_log_en.get()) {
		return;
	}

	{
		navput_local_position_s msg;

		if (_navput_local_position_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_NAVPUT_LOCAL_POSITION, &msg, sizeof(msg),
					      px4_msgs_msg_NavputLocalPosition_cdrstream_desc.ops.ops);
		}
	}

	{
		vehicle_local_position_s msg;

		if (_vehicle_local_position_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_VEHICLE_LOCAL_POSITION, &msg, sizeof(msg),
					      px4_msgs_msg_VehicleLocalPosition_cdrstream_desc.ops.ops);
		}
	}

	{
		sensor_gps_s msg;

		if (_vehicle_gps_position_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_VEHICLE_GPS_POSITION, &msg, sizeof(msg),
					      px4_msgs_msg_SensorGps_cdrstream_desc.ops.ops);
		}
	}

	{
		aux_global_position_s msg;

		if (_aux_global_position_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_AUX_GLOBAL_POSITION, &msg, sizeof(msg),
					      px4_msgs_msg_AuxGlobalPosition_cdrstream_desc.ops.ops);
		}
	}

	{
		navput_fusion_control_s msg;

		if (_navput_fusion_control_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_NAVPUT_FUSION_CONTROL, &msg, sizeof(msg),
					      px4_msgs_msg_NavputFusionControl_cdrstream_desc.ops.ops);
		}
	}

	{
		navput_gnss_spoof_detector_s msg;

		if (_navput_gnss_spoof_detector_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_NAVPUT_GNSS_SPOOF_DETECTOR, &msg, sizeof(msg),
					      px4_msgs_msg_NavputGnssSpoofDetector_cdrstream_desc.ops.ops);
		}
	}

	{
		ranging_beacon_s msg;

		if (_ranging_beacon_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_RANGING_BEACON, &msg, sizeof(msg),
					      px4_msgs_msg_RangingBeacon_cdrstream_desc.ops.ops);
		}
	}

	{
		estimator_aid_source2d_s msg;

		if (_estimator_aid_src_aux_global_position_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_ESTIMATOR_AID_SOURCE_2D, &msg, sizeof(msg),
					      px4_msgs_msg_EstimatorAidSource2d_cdrstream_desc.ops.ops);
		}
	}

	{
		navput_aid_source2d_s msg;

		if (_navput_aid_src_aux_global_position_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_NAVPUT_AID_SOURCE_2D, &msg, sizeof(msg),
					      px4_msgs_msg_NavputAidSource2d_cdrstream_desc.ops.ops);
		}
	}

	{
		navput_aid_source1d_s msg;

		if (_navput_aid_src_ranging_beacon_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_NAVPUT_AID_SOURCE_1D, &msg, sizeof(msg),
					      px4_msgs_msg_NavputAidSource1d_cdrstream_desc.ops.ops);
		}
	}

	{
		navput_attitude_s msg;

		if (_navput_attitude_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_NAVPUT_ATTITUDE, &msg, sizeof(msg),
					      px4_msgs_msg_NavputAttitude_cdrstream_desc.ops.ops);
		}
	}

	{
		navput_status_flags_s msg;

		if (_navput_status_flags_sub.update(&msg)) {
			serialize_and_publish(PAYLOAD_TYPE_NAVPUT_STATUS_FLAGS, &msg, sizeof(msg),
					      px4_msgs_msg_NavputStatusFlags_cdrstream_desc.ops.ops);
		}
	}
}

int NavputTunnel::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Streams a fixed set of navput/estimator uORB topics to the GCS, CDR-encoded and
fragmented into MAVLink TUNNEL messages via the `mavlink_tunnel_out` uORB topic.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("navput_tunnel", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int navput_tunnel_main(int argc, char *argv[])
{
	return ModuleBase::main(NavputTunnel::desc, argc, argv);
}
