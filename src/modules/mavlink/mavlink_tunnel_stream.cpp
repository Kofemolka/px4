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

#include "mavlink_tunnel_stream.h"

#include <string.h>

void MavlinkTunnelStream::handle_update(mavlink_channel_t channel)
{
	// mavlink_tunnel_out has ORB_QUEUE_LENGTH set, so navput_tunnel may have
	// published several fragments since our last check - drain all of them,
	// not just the latest, or earlier fragments would be silently skipped.
	mavlink_tunnel_s tunnel_out;

	while (_tunnel_out_sub.update(&tunnel_out)) {
		mavlink_tunnel_t msg{};
		msg.target_system = tunnel_out.target_system;
		msg.target_component = tunnel_out.target_component;
		msg.payload_type = tunnel_out.payload_type;
		msg.payload_length = tunnel_out.payload_length;
		memcpy(msg.payload, tunnel_out.payload, sizeof(msg.payload));
		static_assert(sizeof(msg.payload) == sizeof(tunnel_out.payload), "mavlink_tunnel.payload size mismatch");

		mavlink_msg_tunnel_send_struct(channel, &msg);
	}
}
