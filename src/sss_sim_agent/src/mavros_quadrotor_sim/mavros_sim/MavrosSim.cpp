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

    /* Load mavros_sim plugins(mavlink msg -> mavros ROS msg; mavros ROS msg -> mavlink msg)*/
    setpoint_raw_plugin_ = std::make_unique<std_plugins::SetpointRawPlugin>();
    local_position_plugin_ = std::make_unique<std_plugins::LocalPositionPlugin>(uas_);
    imu_plugin_ = std::make_unique<std_plugins::IMUPlugin>(uas_);

}

/* Publish all updated mavlink messages into ROS topics (Added by Peixuan Shu) */
void MavrosSim::Publish()
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

        default:
		    std::cout << "[MavrosSim::handle_message] Unknown mavlink message id: " << msg.msgid << std::endl;
		    break;
    }
}

}
