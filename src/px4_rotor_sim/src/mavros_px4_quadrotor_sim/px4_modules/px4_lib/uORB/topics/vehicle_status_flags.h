/****************************************************************************
 *
 *   Copyright (C) 2013-2021 PX4 Development Team. All rights reserved.
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

/* Auto-generated by genmsg_cpp from file /home/spx/PX4-Autopilot/msg/vehicle_status_flags.msg */


#pragma once

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

struct vehicle_status_flags_s {

	uint64_t timestamp;
	bool calibration_enabled;
	bool system_sensors_initialized;
	bool system_hotplug_timeout;
	bool auto_mission_available;
	bool angular_velocity_valid;
	bool attitude_valid;
	bool local_altitude_valid;
	bool local_position_valid;
	bool local_velocity_valid;
	bool global_position_valid;
	bool gps_position_valid;
	bool home_position_valid;
	bool power_input_valid;
	bool battery_healthy;
	bool escs_error;
	bool escs_failure;
	bool position_reliant_on_gps;
	bool position_reliant_on_optical_flow;
	bool position_reliant_on_vision_position;
	bool dead_reckoning;
	bool flight_terminated;
	bool circuit_breaker_engaged_power_check;
	bool circuit_breaker_engaged_airspd_check;
	bool circuit_breaker_engaged_enginefailure_check;
	bool circuit_breaker_flight_termination_disabled;
	bool circuit_breaker_engaged_usb_check;
	bool circuit_breaker_engaged_posfailure_check;
	bool circuit_breaker_vtol_fw_arming_check;
	bool offboard_control_signal_lost;
	bool rc_signal_found_once;
	bool rc_calibration_in_progress;
	bool rc_calibration_valid;
	bool vtol_transition_failure;
	bool usb_connected;
	bool sd_card_detected_once;
	bool avoidance_system_required;
	bool avoidance_system_valid;
	bool parachute_system_present;
	bool parachute_system_healthy;
	uint8_t _padding0[1]; // required for logger
};