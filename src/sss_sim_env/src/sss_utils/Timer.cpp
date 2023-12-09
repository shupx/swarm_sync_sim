/**
 * @file Timer.cpp
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


#include "sss_sim_env/Timer.hpp"


namespace sss_utils
{

Timer::Timer(const ros::Duration &period, const ros::TimerCallback& callback, bool oneshot, bool autostart)
    :period_(period), oneshot_(oneshot), autostart_(autostart)
{
    nh_.param<bool>("/use_sim_time", use_sim_time_, false);

    callback_ = callback;

    /* If use_sim_time, create a clock_updater and ROS timer*/
    if (use_sim_time_)
    {
        ROS_INFO("[sssTimer] use_sim_time is true. Init");

        clock_updater_ = std::make_shared<ClockUpdater>();

        /*If use_sim_time, create a ROS timer with modified callback function */
        timer_ = nh_.createTimer(period_, &Timer::sim_timer_callback, this, oneshot_, autostart_); 

        if (autostart_)
        {
            start();
        }
    }
    else
    {
        /*If not use_sim_time, create a normal ROS timer */
        timer_ = nh_.createTimer(period_, callback_, oneshot_, autostart_); 
    }

}

Timer::~Timer()
{
}

void Timer::start()
{
    if (use_sim_time_)
    {
        /* request clock update to start the first loop */
        ros::Time start_time = ros::Time::now();
        // while (ros::Time::now() < start_time + period_)
        // {
        //     clock_updater_->request_clock_update(start_time + period_);
        //     ros::WallDuration(0.5).sleep();
        // }
        ros::WallDuration(0.5).sleep();
        clock_updater_->request_clock_update(start_time + period_);

    }

    timer_.start();
}

/* callback_ + clock update in every loop */
void Timer::sim_timer_callback(const ros::TimerEvent &event)
{
    ROS_INFO("sim_timer_callback");    

    // record the last loop real time
    static double last_time = 0.0;

    /* call the main callback function */
    callback_(event);

    ROS_INFO("sim_timer_callback2");    

    // /* request /clock to update for the next period*/
    clock_updater_->request_clock_update(event.current_expected + period_);

    ROS_INFO("sim_timer_callback3");    

    // print the real loop rate
    double time_now = ros::WallTime::now().toSec();
    double rate = 1.0 / (time_now - last_time);
    last_time = time_now;
    ROS_INFO("cmdloop_cb rate: %s Hz", std::to_string(rate).c_str());    
}


}