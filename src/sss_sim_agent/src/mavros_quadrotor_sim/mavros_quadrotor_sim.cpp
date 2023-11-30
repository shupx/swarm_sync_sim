/**
 * @file mavros_quadrotor_sim.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Mavros(sim) + PX4 controller + quadrotor_dynamics. main loop
 * 
 * Note: This program relies on
 * 
 * @version 1.0
 * @date 2023-11-30
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */


#include "ros/ros.h"
#include "mavros_quadrotor_sim/quadrotor_dynamics.hpp"
#include "MavrosSim.hpp"

using namespace mavros_sim;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_quadrotor_sim");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    //create an object named time_server in heap rather than stack
    MavrosSim* mavros_sim_node = new MavrosSim(nh, nh_private); 

    ros::spin();
    return 0;
}