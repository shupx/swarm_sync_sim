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

    //create an object named time_server in heap rather than stack
    TimeServer* time_server = new TimeServer(nh, nh_private); 

    ros::spin();
    return 0;
}

TimeServer::TimeServer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),nh_private_(nh_private)
{
    time_pub_ = nh_.advertise<std_msgs::Time>("sim_time", 10);
    time_sub_ = nh_.subscribe("sim_time_sub", 1, &TimeServer::cb_time, this, ros::TransportHints().tcpNoDelay());

    next_client_id_ = 0;
}

void TimeServer::cb_time(const std_msgs::Time::ConstPtr& msg)
{
    ros::Time time = msg->data;
    // ros::Time a = 
    if ((time - ros::Duration(1.2)).toNSec() > 0){
        std::cout << "big" << std::endl;
    }
    std::cout << (time - ros::Duration(1.2)).toNSec() << std::endl;
}

TimeClient::TimeClient(const int &id){
    client_id_ = id;
    has_new_request_ = false;
    request_time_ = ros::Time(0.0); //0 seconds
}