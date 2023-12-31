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

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "sss_sim_env/TimeServer.hpp"


class SimClock :public nodelet::Nodelet
{
public:
    SimClock(){}

public:
    void onInit()
    {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle nh_private = getPrivateNodeHandle();
        // ros::NodeHandle nh = getMTNodeHandle();
        // ros::NodeHandle nh_private = getMTPrivateNodeHandle();

        ROS_INFO("SimClock Inited0");

        time_server = std::make_unique<TimeServer>(nh, nh_private);

        ROS_INFO("SimClock Inited");
        // NODELET_DEBUG("My debug statement");
        // NODELET_DEBUG_STREAM("my debug statement " << (double) 1.0)
        // NODELET_DEBUG_COND( 1 == 1, "my debug_statement")
        // NODELET_DEBUG_STREAM_COND( 1 == 1, "my debug statement " << (double) 1.0)
    }

private:
    std::unique_ptr<TimeServer> time_server;
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SimClock, nodelet::Nodelet);



// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "sim_clock");
//     ros::NodeHandle nh_private("~");
//     ros::NodeHandle nh;

//     // TimeServer time_server(nh, nh_private);

//     // //create an object named time_server in heap rather than stack
//     // TimeServer* time_server = new TimeServer(nh, nh_private); 

//     //Use unique_ptr to auto-destory the object when exiting.
//     std::unique_ptr<TimeServer> time_server(new TimeServer(nh, nh_private));

//     ros::spin();
//     return 0;
// }