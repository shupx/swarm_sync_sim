/**
 * @file Sleep.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Replace ROS Duration and Rate sleep. Utilized by other sim/real agent nodes.
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2024-1-13
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __SSS_SLEEP__
#define __SSS_SLEEP__
#include <ros/ros.h>
#include <ros/time.h>
#include <thread>
#include "sss_sim_env/ClockUpdater.hpp"


namespace sss_utils
{

/**
 * \brief To replace ros::Duration(time).sleep() for swarm_sync_sim.
 * If use_sim_time is false, it is just same as ros::Duration(time).sleep()
 * If use_sim_time is true, it is ros::Duration(time).sleep() + requesting sim clock update in this thread.
 */
class Duration : public ros::Duration
{
    public:
        Duration() : ros::Duration() {}
        Duration(int32_t _sec, int32_t _nsec) : ros::Duration(_sec, _nsec) {}
        Duration(double t) : ros::Duration(t) {}
        Duration(const ros::Rate& rate) : ros::Duration(rate) {}
        bool sleep() const;
};


} //sss_utils
#endif