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

Timer::Impl::Impl(const ros::Duration &period, const ros::TimerCallback& callback, bool oneshot, bool autostart)
    :period_(period), callback_(callback), oneshot_(oneshot), autostart_(autostart), started_(false)
{
    nh_.param<bool>("/use_sim_time", use_sim_time_, false);

    /* If use_sim_time, create a clock_updater and ROS timer*/
    if (use_sim_time_)
    {
        ROS_INFO("[sss_utils::Timer] use_sim_time is true. Init");


        /* If use_sim_time, create a ROS timer with modified callback function 
        (add request_clock_update() each loop*/
        timer_ = nh_.createTimer(period_, &Timer::Impl::sim_timer_callback, this, oneshot_, autostart_); 

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

Timer::Impl::~Impl()
{
    stop();
}

void Timer::Impl::AccelerateTimerThreadFunc()
{
    /**
     * \brief Open a new thread to check ff /clock updates, call timer_.setPeriod(period) 
     * to release timers_cond_ in timer_manager.h for faster loop speed.
     * This is actually for fixing a bug in https://github.com/ros/ros_comm/blob/845f74602c7464e08ef5ac6fd9e26c97d0fe42c9/clients/roscpp/include/ros/timer_manager.h#L591 
     * , where if use_sim_time is true, the timmer manager will block at least 
     * for 1 ms even if the loop can be faster. This bug limits the timer loop 
     * speed to less than 1000 Hz. With this fix, the sim loop rate can be 
     * 20x faster than real speed.
     */
    if (use_sim_time_)
    {
        while (!kill_thread_)
        {
            static ros::Time last_time;
            ros::Time time_now = ros::Time::now();
            if (time_now != last_time)
            {
                /* Clock is updated. Try to update timer_manager for the next loop as well. 
                call timer_.setPeriod(period) to release timers_cond_ in timer_manager.h */
                timer_.setPeriod(period_, false);
                last_time = time_now;
            }
            /* This allows sim time to run up to 20x real-time even for very short timer periods.
             * Sleep for 1 ms or (period_ / 20) s
             * Inspired by https://github.com/ros/roscpp_core/blob/2951f0579a94955f5529d7f24bb1c8c7f0256451/rostime/src/time.cpp#L438 about ros::Duration::sleep() when use_sim_time is true
             */
            const uint32_t sleep_nsec = (period_.sec != 0) ? 1000000 : (std::min)(1000000, period_.nsec/20);
            ros::WallDuration(0,sleep_nsec).sleep();
        }
    }
}

/* callback_ + clock update in every loop */
void Timer::Impl::sim_timer_callback(const ros::TimerEvent &event)
{
    // record the last loop real time
    static double last_time = 0.0;

    /* call the main callback function */
    callback_(event);

    /* request /clock to update for the next period*/
    // TODO: repeat requesting in case of lose?
    clock_updater_->request_clock_update(event.current_expected + period_);

    // print the real loop rate
    double time_now = ros::WallTime::now().toSec();
    double rate = 1.0 / (time_now - last_time);
    last_time = time_now;
    // ROS_INFO("[sss_utils::Timer] timer_callback rate: %s Hz", std::to_string(rate).c_str());    
}

void Timer::Impl::start()
{
    if (!started_)
    {
        if (use_sim_time_)
        {
            clock_updater_ = std::make_shared<ClockUpdater>();

            /* request the first clock update to start the first loop */
            // TODO: repeat request clock update if sim_clock has not received it ?
            ros::Time start_time = ros::Time::now();
            ros::WallDuration(0.5).sleep();
            clock_updater_->request_clock_update(start_time + period_);

            // while (ros::Time::now() < start_time + period_)
            // {
            //     clock_updater_->request_clock_update(start_time + period_);
            //     ros::WallDuration(0.5).sleep();
            // }

            /* Launch a new thread to check /clock and update timer faster */
            kill_thread_ = false;
            boost::thread accelerate_timer_thread_ = boost::thread(&Timer::Impl::AccelerateTimerThreadFunc,this);
        }

        timer_.start();
    }
    started_ = true;
}

void Timer::Impl::stop()
{
    if (started_)
    {
        if (use_sim_time_)
        {
            clock_updater_->unregister();
            kill_thread_ = true;
            accelerate_timer_thread_.join();
        }

        timer_.stop();
    }
    started_ = false;
}


}