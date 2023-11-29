/**
 * @file mavros_sim.hpp
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

#ifndef __MAVROS_SIM_H__
#define __MAVROS_SIM_H__
#include <ros/ros.h>


namespace MavrosQuadSimulator
{

class MavrosSim
{
    public:
        MavrosSim(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        
    private:

    


};


}

#endif