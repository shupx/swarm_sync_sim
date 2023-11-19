/**
 * @file sim_clock.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief simulation clock. Determine the block of threads of each robot.
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

#include "sim_clock.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_clock");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    time_pub = nh.advertise<std_msgs::Time>("sim_time", 10);

    time_sub = nh.subscribe("sim_time_sub", 1, cb_time, ros::TransportHints().tcpNoDelay());

    ros::spin();
    return 0;
}

void cb_time(const std_msgs::Time::ConstPtr& msg)
{
    ros::Time time = msg->data;
    // ros::Time a = 
    if ((time - ros::Duration(1.2)).toNSec() > 0){
        std::cout << "big" << std::endl;
    }
    std::cout << (time - ros::Duration(1.2)).toNSec() << std::endl;
}