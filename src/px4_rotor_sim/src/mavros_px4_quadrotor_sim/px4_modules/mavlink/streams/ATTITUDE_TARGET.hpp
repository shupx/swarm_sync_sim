/**
 * Copied from PX4-Autopilot/src/modules/mavlink/streams
 * Comment out useless defines.
 * @author Peixuan Shu
 */

/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef ATTITUDE_TARGET_HPP
#define ATTITUDE_TARGET_HPP

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

#include <matrix/matrix/math.hpp> // Added by Peixuan Shu
#include <drivers/drv_hrt.h>  // Added  by Peixuan Shu

#include <uORB/uORB_sim.hpp> // Added by Peixuan Shu
#include "../mavlink_msg_list.hpp"  // Added by Peixuan Shu

using namespace time_literals;

template <int N>  /* seperate static messages for UAV N */
class MavlinkStreamAttitudeTarget
{
private:

	uORB_sim::Subscription<vehicle_attitude_setpoint_s> _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB_sim::Subscription<vehicle_rates_setpoint_s> _att_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	hrt_abstime _last_att_sp_update{0};

public:
	bool send()
	{
		vehicle_attitude_setpoint_s att_sp;

		bool updated = false;

		if (_att_sp_sub.update(&att_sp)) {
			_last_att_sp_update = att_sp.timestamp;
			updated = true;

		} /* else if (hrt_elapsed_time(&_last_att_sp_update) > 500_ms) {
			if (!_att_sp_sub.copy(&att_sp)) {
				att_sp = {};
			}

			updated = _att_rates_sp_sub.updated();
		} */ // commented out by Peixuan Shu

		if (updated) {
			mavlink_attitude_target_t msg{};

			msg.time_boot_ms = att_sp.timestamp / 1000;
			matrix::Quatf(att_sp.q_d).copyTo(msg.q);

			vehicle_rates_setpoint_s att_rates_sp{};
			_att_rates_sp_sub.copy(&att_rates_sp);

			msg.body_roll_rate = att_rates_sp.roll;
			msg.body_pitch_rate = att_rates_sp.pitch;
			msg.body_yaw_rate = att_rates_sp.yaw;

			msg.thrust = matrix::Vector3f(att_sp.thrust_body).norm();

			// mavlink_msg_attitude_target_send_struct(_mavlink->get_channel(), &msg);

			/*  Added by Peixuan Shu. Write mavlink messages into "px4_modules/mavlink/mavlink_msg_list.hpp" */
			int handle = (int) px4::mavlink_stream_handle::ATTITUDE_TARGET;
			mavlink_msg_attitude_target_encode(1, 1, &px4::mavlink_stream_list[handle].msg, &msg); 
			px4::mavlink_stream_list[handle].updated = true;

			return true;
		}

		return false;
	}
};

#endif // ATTITUDE_TARGET_HPP
