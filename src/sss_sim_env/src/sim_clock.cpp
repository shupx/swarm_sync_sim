/**
 * @file sim_clock.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief simulation clock. Determine the simulation time process.
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-11-19
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "ros/ros.h"
#include "TimeServer.hpp"
#include "TimeClient.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_clock");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    //create an object named time_server in heap rather than stack
    TimeServer* time_server = new TimeServer(nh, nh_private); 

    ros::spin();
    return 0;
}