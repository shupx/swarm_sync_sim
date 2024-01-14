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
#include "sss_sim_env/Timer.hpp"


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

        /* Rewrite sleep() */
        bool sleep() const;
};

/**
 * \brief To replace ros::Rate for swarm_sync_sim.
 * If use_sim_time is false, it is just same as rate.sleep()
 * If use_sim_time is true, it is rate.sleep() + requesting sim clock update in this thread.
 */
class Rate
{
    public:
        /**
         * @brief  Constructor, creates a Rate
         * @param  frequency The desired rate to run at in Hz
         */
        Rate(double frequency);
        explicit Rate(const ros::Duration&);

        bool sleep();

        /**
         * @brief  Sets the start time for the rate to now
         */
        void reset();

        /**
         * @brief  Get the actual run time of a cycle from start to sleep
         * @return The runtime of the cycle
         */
        ros::Duration cycleTime() const;

        /**
         * @brief Get the expected cycle time -- one over the frequency passed in to the constructor
         */
        ros::Duration expectedCycleTime() const { return expected_cycle_time_; }
    
    private:
        ros::Time start_;
        ros::Duration expected_cycle_time_, actual_cycle_time_;
        std::shared_ptr<ros::Rate> rate_;
};


} //sss_utils
#endif