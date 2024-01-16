/**
 * @file tello_quadrotor_sim.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Tello controller + quadrotor_dynamics. Main loop
 * 
 * Note: This program relies on tello driver sim, dynamics and sss_utils
 * 
 * @version 1.0
 * @date 2024-1-16
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */


#pragma once

#include <ros/ros.h>
#include <sss_sim_env/sss_utils.hpp>

#include "tello_sim/quadrotor_dynamics.hpp"


namespace TelloQuadSimulator
{

class Agent
{
    public:
        Agent(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    
    private:

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        // std::shared_ptr<TelloDriverSim> tello_driver_sim_(nh_, nh_private_);
        std::shared_ptr<Dynamics> dynamics_;
        // std::shared_ptr<Visualizer> visualizer_;

        double mainloop_period_;
        double mainloop_last_time_ = 0;
        sss_utils::Timer mainloop_timer_;

        void mainloop(const ros::TimerEvent &event);
};

}