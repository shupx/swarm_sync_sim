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

Timer::Timer(const ros::NodeHandle &nh, const ros::Duration &period, const ros::TimerCallback& callback, bool oneshot, bool autostart)
{
    impl_ = std::make_shared<Impl>(nh, period, callback, oneshot, autostart);

    // int handle = TimerManagerExtra::global().add_timer(impl_);
    // impl_->set_handle(handle);
}

Timer::Impl::Impl(const ros::NodeHandle &nh, const ros::Duration &period, const ros::TimerCallback& callback, bool oneshot, bool autostart)
    :nh_(nh), period_(period), callback_(callback), oneshot_(oneshot), autostart_(autostart), started_(false), inited_(false)
{
    timer_handle_ = TimerManagerExtra::global().add_timer();

    nh_simclock.setCallbackQueue(&simclock_callback_queue_);
    async_spinner_.start();  // start a new thread to communicate with sim clock

    nh_.param<bool>("/use_sim_time", use_sim_time_, false);

    /* If use_sim_time, create a ROS timer with modified callback function*/
    if (use_sim_time_)
    {
        ROS_INFO("[sss_utils::Timer] use_sim_time is true. Init");

        /* If use_sim_time, create a ROS timer with modified callback function 
        (add request_clock_update() each loop*/
        timer_ = nh_.createTimer(period_, &Timer::Impl::sim_timer_callback, this, oneshot_, autostart_); 

        simclock_online_sub_ = nh_simclock.subscribe("/sss_clock_is_online", 1000, &Timer::Impl::cb_simclock_online, this);

        // /* Init callback_queue -> spinner_threads map*/
        // callback_queue_ = nh_.getCallbackQueue();
        // if (cbqueue_spinnerthreads_map.find(callback_queue_) == cbqueue_spinnerthreads_map.end())
        // {
        //     cbqueue_spinnerthreads_map[callback_queue_] = std::make_shared<ThreadIdVector>();
        // }

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

    //@TODO catch SIGINT error caused by ctrl+C, and destroy the parent before ros shutdown (for clock_updater_->unregister())

}

Timer::Impl::~Impl()
{
    stop();
    TimerManagerExtra::global().remove_timer_info(timer_handle_);
}

/* callback_() + clock update in each loop */
void Timer::Impl::sim_timer_callback(const ros::TimerEvent &event)
{
    std::thread::id thread_id = std::this_thread::get_id();

    ClockUpdaterPtr clock_updater = ThreadClockupdaters::global().get_clockupdater(thread_id);

    /* make clock waiting at now util the loop completes */
    while(!clock_updater->request_clock_update(event.current_expected))
    {
        // block until publishing successfully
        ros::WallDuration(0.1).sleep(); // 0.1 is fine

        std::cout << "[Timer::Impl::sim_timer_callback] Repeat clock_updater->request_clock_update " << event.current_expected.toSec() << std::endl;
    }
    
    /* Set next callback expected time as infinity as we are not sure the next expected time now */
    //@TODO check if other timers in timer_manager are blocked by this thread. If blocked, add its next time as infinity too.
    while(!TimerManagerExtra::global().add_next_cb_time(timer_handle_, ros::TIME_MAX))
    {
        // block until publishing successfully
        ros::WallDuration(0.1).sleep();

        std::cout << "[Timer::Impl::sim_timer_callback] Repeat add_next_cb_time " << ros::TIME_MAX.toSec() << " for timer " << timer_handle_ << std::endl;
    }


    /* call the main callback function */
    callback_(event);


    /* Request next triggered time */
    ros::Time next_time;
    if (oneshot_)
    {
        // request a large enough time
        next_time = ros::TIME_MAX - ros::Duration(2); // Do not set as ros::TIME_MAX as it may unblock the timer manager clock updater
    }
    else if(event.current_expected + period_ <= ros::Time::now())
    {
        next_time = ros::Time::now();

        ROS_WARN("[sss_utils::Timer::Impl::sim_timer_callback] Detect timer loop jumps. Set next expected time as now %ss with last time = %ss and period = %ss. This is caused by the callback execution time longer than the timer period. Check if a too long sleep is applied in a timer/subscriber callback or if enough multi-thread spinners are set.", std::to_string(next_time.toSec()).c_str(), std::to_string(event.current_expected.toSec()).c_str(), std::to_string(period_.toSec()).c_str());
    }
    else
    {
        next_time = event.current_expected + period_;
    }
    
    /* Set next callback expected time */
    TimerManagerExtra::global().add_next_cb_time(timer_handle_, next_time);

    /* Request infinite next time in this thread as we do not know whether there will be callbacks in this spinner thread in the future */
    clock_updater->request_clock_update(ros::TIME_MAX);

    /* print the real loop rate */
    // double time_now = ros::WallTime::now().toSec();
    // double rate = 1.0 / (time_now - last_sim_timer_cb_time_);
    // last_sim_timer_cb_time_ = time_now;
    // ROS_INFO("[sss_utils::Timer] timer_callback rate: %s Hz", std::to_string(rate).c_str()); 
}

void Timer::Impl::start()
{
    if (!started_)
    {
        timer_.start();
    }
    started_ = true;
}

/* Publish the first loop time request when sim clock is online */
void Timer::Impl::cb_simclock_online(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data == true)
    {
        /* request the first clock update to start the first loop */
        while(true)
        {
            if (TimerManagerExtra::global().add_next_cb_time(timer_handle_, ros::Time::now() + period_))
            {
                std::cout << "[Timer::Impl::cb_simclock_online] timer " << timer_handle_ << " add_next_cb_time " << (ros::Time::now() + period_).toSec() << std::endl;
                break;
            }
            // block until publishing successfully
            ros::WallDuration(0.2).sleep();
            ROS_INFO("[sss_timer] Repeat the first request_clock_update...");
        }

        inited_ = true;
        ROS_INFO("[sss_timer] inited_ = true");
    }
}

void Timer::Impl::stop()
{
    if (started_)
    {
        timer_.stop();
    }
    started_ = false;
}

}