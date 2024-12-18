/**
 * @file MavrosSim.hpp
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
 

#include <ros/ros.h>

#include "plugins/setpoint_raw.cpp"
#include "plugins/local_position.cpp"
#include "plugins/imu.cpp"
#include "plugins/sys_status.cpp"
#include "plugins/command.cpp"
#include "plugins/global_position.cpp"

#include "lib/mavros_uas.h"

#include "px4_modules/mavlink/mavlink_msg_list.hpp" // store the simulated extern(global) mavlink messages (Created by Peixuan Shu)

namespace mavros_sim
{

class MavrosSim
{
    private:
        int agent_id_ = -1; // UAV id.
    
    public:
        /* Load mavros_sim plugins(mavlink msg -> mavros ROS msg; mavros ROS msg -> mavlink msg)*/
        MavrosSim(int agent_id, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

        /* Publish all updated mavlink messages into ROS topics (Added by Peixuan Shu) */
        void PublishRosMessage();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        std::shared_ptr<UAS> uas_; // store some common data and functions

        std::unique_ptr<std_plugins::SetpointRawPlugin> setpoint_raw_plugin_;
        std::unique_ptr<std_plugins::LocalPositionPlugin> local_position_plugin_;
        std::unique_ptr<std_plugins::IMUPlugin> imu_plugin_;
        std::unique_ptr<std_plugins::SystemStatusPlugin> sys_status_plugin_;
        std::unique_ptr<std_plugins::CommandPlugin> command_plugin_;
        std::unique_ptr<std_plugins::GlobalPositionPlugin> global_position_plugin_;

        /* Publish mavlink messages into ROS topics (Added by Peixuan Shu)*/
        void handle_message(const mavlink_message_t &msg);

};

}
