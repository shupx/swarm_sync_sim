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
    /* Load mavros_sim plugins(mavlink msg -> mavros ROS msg; mavros ROS msg -> mavlink msg)*/

    // std_plugins::SetpointRawPlugin* setpoint_raw_plugin = new std_plugins::SetpointRawPlugin(); 
    setpoint_raw_plugin_ = std::make_unique<std_plugins::SetpointRawPlugin>();

}

}
