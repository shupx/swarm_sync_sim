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
    impl_ = boost::make_shared<Impl>(nh, period, callback, oneshot, autostart);

    int handle = TimerManagerExtra::global().add_timer(impl_);
    impl_->set_handle(handle);
}

Timer::Impl::Impl(const ros::NodeHandle &nh, const ros::Duration &period, const ros::TimerCallback& callback, bool oneshot, bool autostart)
    :nh_(nh), period_(period), callback_(callback), oneshot_(oneshot), autostart_(autostart), started_(false), inited_(false), has_accelerate_timer_thread_(false)
{
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
}

/* callback_() + clock update in each loop */
void Timer::Impl::sim_timer_callback(const ros::TimerEvent &event)
{
    std::thread::id thread_id = std::this_thread::get_id();

    // /* Associate this spinner thread id with the callback queue of this timer */
    // ThreadIdVectorPtr spinner_threads = cbqueue_spinnerthreads_map[callback_queue_];
    // if (std::find(spinner_threads->begin(), spinner_threads->end(), thread_id) == spinner_threads->end())
    // {
    //     spinner_threads->emplace_back(thread_id);
    // }

    if(thread_clockupdater_map.find(thread_id) == thread_clockupdater_map.end())
    {
        /* If no clockupdater exists for this thread, make a clockupdater */
        thread_clockupdater_map[thread_id] = std::make_shared<ClockUpdater>(nh_simclock);
    }
    ClockUpdaterPtr clock_updater = thread_clockupdater_map[thread_id]; // Get the clockupdater of this thread

    /* make clock waiting at now util the loop completes */
    while(!clock_updater->request_clock_update(event.current_expected))
    {
        // block until publishing successfully
        ros::WallDuration(0.1).sleep();

        std::cout << "[Timer::Impl::sim_timer_callback] Repeat clock_updater->request_clock_update " << event.current_expected.toSec() << std::endl;
    }
    
    /* Set next callback expected time as infinity as we are not sure the next expected time now */
    while(!TimerManagerExtra::global().add_next_cb_time(timer_handle_, ros::Time{MAX_ROS_TIME}))
    {
        // block until publishing successfully
        ros::WallDuration(0.1).sleep();

        std::cout << "[Timer::Impl::sim_timer_callback] Repeat add_next_cb_time " << ros::Time{MAX_ROS_TIME}.toSec() << " for timer " << timer_handle_ << std::endl;
    }


    /* call the main callback function */
    callback_(event);


    /* Request next triggered time */
    ros::Time next_time;
    if (oneshot_)
    {
        next_time = ros::Time{MAX_ROS_TIME};
    }
    else if(event.current_expected + period_ <= ros::Time::now())
    {
        next_time = ros::Time::now();  // @TODO next time may be small than real next time
        std::cout << "[Timer::Impl::sim_timer_callback] Detect clock jumps, set next_time as now " << next_time.toSec() << " with event.current_expected = " << event.current_expected.toSec() << std::endl;
    }
    else
    {
        next_time = event.current_expected + period_;
    }
    
    /* Set next callback expected time */
    TimerManagerExtra::global().add_next_cb_time(timer_handle_, next_time);

    /* Request inifity next time in this thread as we do not know whether there will be callbacks in this spinner thread in the future)*/
    clock_updater->request_clock_update(ros::Time{MAX_ROS_TIME});

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
        if (use_sim_time_)
        {
            // clock_updater_ = std::make_shared<ClockUpdater>(nh_);

            // simclock_online_sub_ = nh_simclock.subscribe("/sss_clock_is_online", 1000, &Timer::Impl::cb_simclock_online, this);

            // /* Launch a new thread to check /clock and update timer faster */
            // kill_thread_ = false;
            // boost::thread accelerate_timer_thread_ = boost::thread(&Timer::Impl::AccelerateTimerThreadFunc,this);
            // has_accelerate_timer_thread_ = true;

            /* @TODO
             * Note that setPeriod() unblocks all timers in timer manager as timer manager is global. 
             * Maybe only one accelerate thread is needed for all timers?
             * But More timers bring more more callbacks in the callback queue. We need check clock update more frequently.
             */
        }

        timer_.start();
    }
    started_ = true;
}

/* Publish the first loop time request when sim clock is online */
void Timer::Impl::cb_simclock_online(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data == true)
    {
        // /* request the first clock update to start the first loop */
        // while(!clock_updater_->request_clock_update(ros::Time::now() + period_)) 
        // {
        //     // block until publishing successfully
        //     ros::WallDuration(0.2).sleep();
        //     ROS_INFO("[sss_timer] Repeat the first request_clock_update...");
        // }

        while(true)
        {
            if (TimerManagerExtra::global().add_next_cb_time(timer_handle_, ros::Time::now() + period_))
            {
                std::cout << "[Timer::Impl::cb_simclock_online] add_next_cb_time " << (ros::Time::now() + period_).toSec() << std::endl;
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

void Timer::Impl::AccelerateTimerThreadFunc()
{
    /**
     * \brief Open a new thread to check if /clock updates, call timer_.setPeriod(period) 
     * to release the condition variable "timers_cond_" in timer_manager.h for faster loop speed.
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
            // static ros::Time last_time;
            ros::Time time_now = ros::Time::now();
            if (time_now != last_clock_time_)
            {
                /* Clock is updated. Try to update timer_manager for the next loop as well. 
                call timer_.setPeriod(period) to release timers_cond_ in timer_manager.h */
                timer_.setPeriod(period_, false);
                last_clock_time_ = time_now;
            }
            /* This allows sim time to run up to 100 real-time ideally even for very short timer periods.
             * Sleep for 1 ms or (period_ / 100) s
             * Inspired by https://github.com/ros/roscpp_core/blob/2951f0579a94955f5529d7f24bb1c8c7f0256451/rostime/src/time.cpp#L438 about ros::Duration::sleep() when use_sim_time is true
            */
            const uint32_t sleep_nsec = (period_.sec != 0) ? 1000000 : (std::min)(1000000, period_.nsec/100);
            ros::WallDuration(0,sleep_nsec).sleep();

            /** @TODO
             * Note that setPeriod() unblocks all timers in timer_manager as timer manager is global. 
             * If there are multiple timers, their callbacks are queued in one thread by timermanager.h as ros::spin() is single threaded by default. 
             * Maybe we need just one accelerate thread for all timers but multiply its loop rate with timer multiples?
             */ 

        }
    }
}

void Timer::Impl::stop()
{
    if (started_)
    {
        if (use_sim_time_)
        {
            // clock_updater_->unregister();
            // kill_thread_ = true;
            // if (has_accelerate_timer_thread_)
            // {
            //     accelerate_timer_thread_.join();
            // }
        }

        timer_.stop();
    }
    started_ = false;
}


}