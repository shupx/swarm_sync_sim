/**
 * @file fw_plane_visualizer_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Publish rotor propeller joint position and base_link tf states for the robot model visualization in rviz
 * 
 * Note: This program relies on mavros, px4 geo.h
 * 
 * @version 1.0
 * @date 2024-8-29
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */

#include <ros/ros.h>
#include "fw_plane_sim/fw_plane_visualizer.hpp"

using namespace FwPlaneSimulator;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fw_plane_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //Use unique_ptr to auto-destory the object when exiting.
    std::unique_ptr<Visualizer> visualizer(new Visualizer(nh, nh_private));

    ros::Rate loop_rate(50); // Hz
    while (ros::ok())
    {
        visualizer->Run();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}