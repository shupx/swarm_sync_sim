/**
 * @file Timer.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Replace ROS timer. Utilized by other sim/real agent nodes.
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-12-7
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __SSSTIMER__
#define __SSSTIMER__
#include <ros/ros.h>
#include "sss_sim_env/ClockUpdater.hpp"


namespace sss_utils
{

class Timer
{
    private:
        ros::NodeHandle nh_;
        bool use_sim_time_;

        /* callback_ + clock update in every loop */
        void sim_timer_callback(const ros::TimerEvent &event);

    public:
        Timer() {} // for Timer object initialize without params
        ~Timer();

        Timer(const ros::Duration &period, const ros::TimerCallback& callback, bool oneshot, bool autostart);

        /* Start timer if autostart is false */
        void start();

        ros::Timer timer_;
        ros::Duration period_;
        bool oneshot_;
        bool autostart_;

        std::shared_ptr<ClockUpdater> clock_updater_;

        ros::TimerCallback callback_;
};


}
#endif