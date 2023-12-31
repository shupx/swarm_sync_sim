/**
 * @file MavrosSim.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Simulated mavros that receives ROS topics and transfer to the
 * quadrotor dynamics and broadcast quadrotor states to ROS topics.
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-11-29
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */
 

#include "MavrosSim.hpp"

namespace mavros_sim
{

MavrosSim::MavrosSim(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    uas_ = std::make_shared<UAS>(); // uas_ stores some common data and functions
    uas_->set_tgt(1, 1); // set target_system_id, target_component_id
    uas_->set_timesync_mode(UAS::timesync_mode::PASSTHROUGH); // PASSTHROUGH: pass mavlink message time rather than ROS time into ROS message timestamp header
    // timesync_mode will be overwritten by the parameter "time/timesync_mode" in px4_config.yaml (default MAVLINK) if sys_time.cpp plugin is loaded!

    /* Load mavros_sim plugins(mavlink msg -> mavros ROS msg; mavros ROS msg -> mavlink msg)*/
    setpoint_raw_plugin_ = std::make_unique<std_plugins::SetpointRawPlugin>(uas_, nh_, nh_private_);
    local_position_plugin_ = std::make_unique<std_plugins::LocalPositionPlugin>(uas_, nh_, nh_private_);
    imu_plugin_ = std::make_unique<std_plugins::IMUPlugin>(uas_, nh_, nh_private_);
    sys_status_plugin_ = std::make_unique<std_plugins::SystemStatusPlugin>(uas_, nh_, nh_private_);
    command_plugin_ = std::make_unique<std_plugins::CommandPlugin>(uas_, nh_, nh_private_);
    global_position_plugin_ = std::make_unique<std_plugins::GlobalPositionPlugin>(uas_, nh_, nh_private_);

}

/* Publish all updated mavlink messages into ROS topics (Added by Peixuan Shu) */
void MavrosSim::PublishRosMessage()
{
	/* Search for mavlink streaming list and handle the updated messages */
	for (int i=0; i<MAVLINK_STREAM_NUM; ++i)
	{
		if (px4::mavlink_stream_list[i].updated)
		{
            // std::cout << "[MavrosSim::Publish] handle " << px4::mavlink_stream_list[i].msg.msgid << std::endl;
			handle_message(px4::mavlink_stream_list[i].msg);
			px4::mavlink_stream_list[i].updated = false; // waiting for the next update
		}
	}
}


//@TODO create a new thread to async handle IO messages (read and handle mavlink messages)

/* Publish mavlink messages into ROS topics (Added by Peixuan Shu) */
void MavrosSim::handle_message(const mavlink_message_t &msg)
{
	switch (msg.msgid) 
    {
        /* setpoint_raw_plugin_ */
        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
            setpoint_raw_plugin_->handle_position_target_local_ned(msg);
            break;
        case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
            setpoint_raw_plugin_->handle_position_target_global_int(msg);
            break;
        case MAVLINK_MSG_ID_ATTITUDE_TARGET:
            setpoint_raw_plugin_->handle_attitude_target(msg);
            break;
        
        /* local_position_plugin_ */
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            local_position_plugin_->handle_local_position_ned(msg);
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV:
            local_position_plugin_->handle_local_position_ned_cov(msg);
            break;
        
        /* imu_plugin_ */
        case MAVLINK_MSG_ID_ATTITUDE:
            imu_plugin_->handle_attitude(msg);
            break;
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            imu_plugin_->handle_attitude_quaternion(msg);
            break;
        case MAVLINK_MSG_ID_HIGHRES_IMU:
            imu_plugin_->handle_highres_imu(msg);
            break;
        case MAVLINK_MSG_ID_RAW_IMU:
            imu_plugin_->handle_raw_imu(msg);
            break;
        case MAVLINK_MSG_ID_SCALED_IMU:
            imu_plugin_->handle_scaled_imu(msg);
            break;
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
            imu_plugin_->handle_scaled_pressure(msg);
            break;
        
        /* sys_status_plugin_ */
        case MAVLINK_MSG_ID_HEARTBEAT:
            sys_status_plugin_->handle_heartbeat(msg);
            break;
        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
            sys_status_plugin_->handle_extended_sys_state(msg);
            break;
        case MAVLINK_MSG_ID_SYS_STATUS:
            sys_status_plugin_->handle_sys_status(msg);
            break;
        case MAVLINK_MSG_ID_STATUSTEXT:
            sys_status_plugin_->handle_statustext(msg);
            break;
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
            sys_status_plugin_->handle_estimator_status(msg);
            break;
        
        /* command_plugin_ */
        case MAVLINK_MSG_ID_COMMAND_ACK:
            // command_plugin_->handle_command_ack(msg); /* command ack machanism is banned in mavros_sim */
            break;
        
        /* global_position_plugin_ */
        case MAVLINK_MSG_ID_GPS_RAW_INT:
            global_position_plugin_->handle_gps_raw_int(msg);
            break;
        case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
            global_position_plugin_->handle_gps_global_origin(msg);
            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            global_position_plugin_->handle_global_position_int(msg);
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET:
            global_position_plugin_->handle_lpned_system_global_offset(msg);
            break;

        default:
		    std::cout << "[MavrosSim::handle_message] Unknown mavlink message id: " << msg.msgid << std::endl;
		    break;
    }
}

}
