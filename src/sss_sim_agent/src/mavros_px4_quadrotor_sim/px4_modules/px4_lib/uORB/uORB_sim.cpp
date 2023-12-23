/**
 * @file px4_uorb_lists.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Store the PX4 global uorb messages to simulate the real PX4 uORB messages. 
 * 
 * Note: This program relies on px4_lib/uORB/
 * 
 * @version 1.0
 * @date 2023-12-14
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "uORB_sim.hpp"

namespace uORB_sim { 

// @TODO: Initialize if no publisher exists?
actuator_armed_s actuator_armed{};
autotune_attitude_control_status_s autotune_attitude_control_status{};
battery_status_s battery_status{};
commander_state_s commander_state{};
home_position_s home_position{};
manual_control_setpoint_s manual_control_setpoint{};
takeoff_status_s takeoff_status{};
offboard_control_mode_s offboard_control_mode{};
vehicle_angular_velocity_s vehicle_angular_velocity{};
vehicle_attitude_s vehicle_attitude{};
vehicle_command_s vehicle_command{};
vehicle_command_ack_s vehicle_command_ack{};
vehicle_attitude_setpoint_s vehicle_attitude_setpoint{};
vehicle_constraints_s vehicle_constraints{};
vehicle_control_mode_s vehicle_control_mode{};
vehicle_global_position_s vehicle_global_position{};
vehicle_gps_position_s vehicle_gps_position{};
vehicle_land_detected_s vehicle_land_detected{};
vehicle_local_position_s vehicle_local_position{};
vehicle_local_position_setpoint_s vehicle_local_position_setpoint{};
vehicle_local_position_setpoint_s trajectory_setpoint{};
vehicle_odometry_s vehicle_odometry{};
vehicle_rates_setpoint_s vehicle_rates_setpoint{};
vehicle_status_s vehicle_status{};
vehicle_status_flags_s vehicle_status_flags{};


}