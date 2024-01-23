/**
 * @file wheeltec_ugv_sim_node.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Tello driver sim + quadrotor_dynamics. Main loop
 * 
 * Note: This program relies on wheeltec_driver_sim, dynamics and sss_utils
 * 
 * @version 1.0
 * @date 2024-1-23
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */


#pragma once

#include <ros/ros.h>
#include <sss_sim_env/sss_utils.hpp>

#include "ugv_sim/ugv_dynamics.hpp"
#include "ugv_sim/wheeltec_driver_sim.hpp"


namespace WheeltecUgvSimulator
{

class Agent
{
    public:
        Agent(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    
    private:

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        std::shared_ptr<UgvSimulator::Dynamics> dynamics_;
        std::shared_ptr<WheeltecDriverSim> wheeltec_driver_sim_;

        double mainloop_period_;
        double mainloop_last_time_ = 0;
        sss_utils::Timer mainloop_timer_;

        void mainloop(const ros::TimerEvent &event);
};

}