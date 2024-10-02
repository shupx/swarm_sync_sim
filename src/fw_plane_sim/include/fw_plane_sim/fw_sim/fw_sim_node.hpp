/**
 * @file fw_sim_node.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Fixed wing flight controller driver sim + quadrotor_dynamics. Main loop
 * 
 * Note: This program relies on fw_driver_sim, ss_utils
 * 
 * @version 1.0
 * @date 2024-10-02
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */



#pragma once

#include <ros/ros.h>
#include <sss_sim_env/sss_utils.hpp>

#include "BHDynamic.h"
#include "fw_plane_sim/fw_sim/fw_driver_sim.hpp"


namespace FwSimulator
{

class Agent
{
    public:
        Agent(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    
    private:

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        std::shared_ptr<FwDriverSim> fw_driver_sim_;

        double mainloop_period_;
        double mainloop_last_time_ = 0;
        sss_utils::Timer mainloop_timer_;

        void mainloop(const ros::TimerEvent &event);
};

}