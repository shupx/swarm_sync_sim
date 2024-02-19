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
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Bool.h>
// #include <boost/thread.hpp>
#include <memory>
#include <thread>
#include <vector>
#include "sss_sim_env/ClockUpdater.hpp"


namespace sss_utils
{

/**
 * \brief To Replace ros::Timer for swarm_sync_sim.
 * If use_sim_time is true, it is a ros::Timer with request_clock_update in each loop.
 * If use_sim_time is false, it is just a normal ros::Timer.
 */
class Timer
{
    private:
        /**
         * \brief Implementation. Real function class. 
         * Use Impl just for the convenience of overloading assignment constructor
         * and copy constructor.
         */
        class Impl : public std::enable_shared_from_this<Impl>
        {
        public:
            /* If use_sim_time is true, it creates a ros::Timer with request_clock_update in each loop.
            * If use_sim_time is false, it just creates a normal ros::Timer.
            */
            Impl(const ros::NodeHandle &nh, const ros::Duration &period, const ros::TimerCallback& callback, bool oneshot, bool autostart);
            ~Impl();

            bool hasStarted() const {return timer_.hasStarted();}
            bool isValid() {return timer_.isValid();}
            bool isValid() const {return timer_.hasStarted();}
            bool hasPending() {return timer_.hasPending();}
            void setPeriod(const ros::Duration& period, bool reset=true)
            {timer_.setPeriod(period, reset);}

            /* start timer and send the first clock request */
            void start();
            /* stop timer */
            void stop();

            // added by Peixuan Shu
            ros::Duration get_period() {return period_;}

            void set_handle(int handle) {timer_handle_ = handle;}
            int get_handle() {return timer_handle_;}

        private:
            int timer_handle_; // added by Peixuan Shu

            bool use_sim_time_;
            ros::NodeHandle nh_;
            ros::Timer timer_;

            // std::shared_ptr<ClockUpdater> clock_updater_;

            ros::Duration period_;
            ros::TimerCallback callback_;
            bool oneshot_;
            bool autostart_;

            bool started_;

            bool inited_;

            // ros::CallbackQueueInterface* callback_queue_; // timer callback queue

            double last_sim_timer_cb_time_;
            /**
             * \brief callback_ + clock update in every loop 
             */
            void sim_timer_callback(const ros::TimerEvent &event);

            ros::NodeHandle nh_simclock;
            ros::CallbackQueue simclock_callback_queue_;
            ros::Subscriber simclock_online_sub_;
            ros::AsyncSpinner async_spinner_{1, &simclock_callback_queue_};
            /* Publish the first loop time request when sim clock is online */
            void cb_simclock_online(const std_msgs::Bool::ConstPtr& msg);
        };

        typedef std::shared_ptr<Impl> ImplPtr;
        typedef std::weak_ptr<Impl> ImplWPtr;
        ImplPtr impl_;

        friend class TimerManagerExtra;

    public:
        Timer() {} // Initialize timer object without params
        ~Timer() {}

        /* Overload copy constructor */
        Timer(const Timer& rhs)
        {
            // boost::shared_ptr copy
            impl_ = rhs.impl_;
        }

        /* Overload assignment constructor to copy constructor */
        // "default" means the default copy constructor
        Timer& operator=(const Timer& other) = default; 


        /* constructor */
        Timer(const ros::NodeHandle &nh, const ros::Duration &period, const ros::TimerCallback& callback, bool oneshot, bool autostart);

        /* Mananul start timer if autostart is false.  Does nothing if the timer is already started.*/
        void start()
        {
            impl_->start();
        }

        /**
         * \brief Stop the timer.  Once this call returns, no more callbacks will be called.  
         * Does nothing if the timer is already stopped.
         */
        void stop()      
        {
            impl_->stop();
        }

        /**
         * \brief Returns whether or not the timer has any pending events to call.
         */
        bool hasPending()
        {
            return impl_->hasPending();
        }

        /**
         * \brief Set the period of this timer
         * \param reset Whether to reset the timer. If true, timer ignores elapsed time and next cb occurs at now()+period
         */
        void setPeriod(const ros::Duration& period, bool reset=true)
        {
            impl_->setPeriod(period, reset);
        }
        
        bool isValid() { return impl_->isValid(); }
        bool isValid() const { return impl_->isValid(); }
};


/**
 * \brief timer_manager.h in roscpp governs all timers in the ROS space 
 * and adds the timer callback functions to the callback queue. This class 
 * serves as an extra part of timer_manager.h to request sim clock time.
 */
class TimerManagerExtra
{
    public:
        TimerManagerExtra()
        {
            nh_.param<bool>("/use_sim_time", use_sim_time_, false);

            if (use_sim_time_)
            {
                nh_.setCallbackQueue(&callback_queue_);
                async_spinner_.start(); // start a new thread to listen to clock updates

                ROS_INFO("[TimerManagerExtra] Create a new clock updater for ROS internal timer manager thread"); 
                clock_updater_ = std::make_shared<ClockUpdater>();

                clock_sub_ = nh_.subscribe("/clock", 10, &TimerManagerExtra::cb_clock, this);

                dummy_timer_ = nh_.createTimer(ros::DURATION_MAX, &TimerManagerExtra::dummy_timer_callback, this);
                // Call dummy_timer_.setPeriod() for faster loop speed in simulation.

                std::cout << "[sss_utils::TimerManagerExtra] Init" << std::endl;
            }
        }

        /* static(global) timer manager extra */
        static TimerManagerExtra& global()
        {
            static TimerManagerExtra global;
            return global;
        }

        /* add a timer to the timer_manager_extra */
        int add_timer()
        {
            std::lock_guard<std::mutex> lockGuard(timer_info_list_mutex_);

            static int handle = 0;
            timer_info_list_.emplace_back(
                TimerInfo
                {
                    .handle = handle,
                    // .timer_impl = timer_impl_ptr,
                    // .period = timer_impl_ptr->get_period(),
                    .has_next_expected_time = false,
                    .next_expected_time = ros::TIME_MAX
                }
            );
            handle ++;
            return (handle-1);
        }

        bool remove_timer_info(int handle)
        {
            /* find the time info that matches the handle id */
            auto it = std::find_if(timer_info_list_.begin(), timer_info_list_.end(), [handle](TimerInfo timer_info){return timer_info.handle == handle;});

            if (it == timer_info_list_.end())
            {
                std::cout << "[sss_utils::TimerManagerExtra] remove timer info failed. Handle " << handle << " not found." << std::endl;
                return false;
            }
            else
            {
                add_next_cb_time(handle, ros::TIME_MAX);
                timer_info_list_.erase(it);
                //@TODO reuse the handle id for new timer info?
                return true;
            }
        }

        /* add next expected triggerred time of a timer callback */
        bool add_next_cb_time(int handle, ros::Time next_time)
        {
            // std::cout << "[TimerManagerExtra::add_next_cb_time] timer info " << handle << " request " << next_time.toSec() << std::endl;

            std::lock_guard<std::mutex> lockGuard(timer_info_list_mutex_);

            /* Update the next_expected_time of timer[handle] and check if all timers having next expected times*/
            bool timer_handle_found = false;
            bool all_timers_have_next_expected_time = true;
            if (!timer_info_list_.empty())
            {
                for (auto it = timer_info_list_.begin(); it != timer_info_list_.end(); ++it)
                {
                    if ((*it).handle == handle)
                    {
                        (*it).has_next_expected_time = true;
                        (*it).next_expected_time = next_time;
                        timer_handle_found = true;
                    }
                    if ((*it).has_next_expected_time == false)
                    {
                        all_timers_have_next_expected_time = false;
                    }
                    if (!all_timers_have_next_expected_time && timer_handle_found)
                    {
                        break;
                    }
                }
            }
            if (!timer_handle_found)
            {
                std::cout << "[TimerManagerExtra::add_next_cb_time] No timer with handle " << handle << std::endl;
                return false;
            }
            
            /* If this timer request inifite time, publish infinite time to unblock this time client */
            /* @TODO may not be necessary */
            // @TODO check if other timers are blocked by this uncertain timer
            // if (next_time == ros::TIME_MAX)
            // {
            //     bool ret;
            //     ret = clock_updater_->request_clock_update(next_time);

            //     // std::cout << "[TimerManagerExtra::add_next_cb_time] clock_updater_->request_clock_update " << next_time.toSec() << std::endl;

            //     return ret;
            // }

            /* If all timers have next expected times, request the smallest one */
            if (all_timers_have_next_expected_time)
            {
                std::sort(timer_info_list_.begin(), timer_info_list_.end(), compare_next_cb_time);
                ros::Time request_time = timer_info_list_[0].next_expected_time;

                bool ret;
                
                std::lock_guard<std::mutex> lockGuard(last_request_time_mutex_);
                ret = clock_updater_->request_clock_update(request_time);
                last_request_time_ = request_time;

                // std::cout << "[TimerManagerExtra::add_next_cb_time] clock_updater_->request_clock_update " << request_time.toSec() << std::endl;

                return ret;
            }
            else
            {
                // std::cout << "[TimerManagerExtra::add_next_cb_time] Some other timers do not have next expected time. Timer " << handle << " said." << std::endl;

                bool ret = true;
                
                /* Publish now time request for this clockupdater (recover from the last infinite reqeust to avoid all thread requesting TIME_MAX) */
                if (clock_updater_->get_request_time() == ros::TIME_MAX)
                {
                    std::lock_guard<std::mutex> lockGuard(last_request_time_mutex_);
                    ret = clock_updater_->request_clock_update(ros::Time::now());
                    last_request_time_ = ros::Time::now();

                    // std::cout << "[TimerManagerExtra::add_next_cb_time] clock_updater_->request_clock_update " << ros::Time::now().toSec() << std::endl;
                }
                
                return ret;
            }
        }

        /* clock updater requests last requesting time */
        bool request_last_time()
        {
            std::lock_guard<std::mutex> lockGuard(last_request_time_mutex_);
            if (last_request_time_ == ros::TIME_MAX)
            {
                last_request_time_ = ros::Time::now();
            }
            return clock_updater_->request_clock_update(last_request_time_);
        }

        ClockUpdaterPtr clock_updater_;

    private:
        bool use_sim_time_;

        ros::Time last_request_time_ = ros::TIME_MAX;
        std::mutex last_request_time_mutex_;

        ros::NodeHandle nh_;
        ros::CallbackQueue callback_queue_;
        ros::Subscriber clock_sub_;
        ros::AsyncSpinner async_spinner_{1, &callback_queue_};

        ros::Timer dummy_timer_;

        struct TimerInfo
        {
            int handle;
            // sss_utils::Timer::ImplPtr timer_impl;
            // ros::Duration period;
            bool has_next_expected_time;
            ros::Time next_expected_time;
        };

        std::vector<TimerInfo> timer_info_list_; // next expected triggerred time of all timers
        std::mutex timer_info_list_mutex_;

        static bool compare_next_cb_time(TimerInfo a, TimerInfo b)
        {
            return a.next_expected_time < b.next_expected_time;
        }

        void cb_clock(const rosgraph_msgs::Clock::ConstPtr& msg)
        {
            ros::Time now = msg->clock;

            // std::cout << "[TimerManagerExtra::cb_clock] receive clock time: " << now.toSec() << std::endl;
        
            {
                std::lock_guard<std::mutex> lockGuard(timer_info_list_mutex_);

                if(!timer_info_list_.empty())
                {
                    // timer_info_list_[0].timer_impl->setPeriod(timer_info_list_[0].period, false); // setPeriod(period, false) does not affect the period and next expected time. 

                    // dummy_timer_.setPeriod(ros::DURATION_MAX, false);

                    // sort next_time_list_ from small to large
                    std::sort(timer_info_list_.begin(), timer_info_list_.end(), compare_next_cb_time);

                    /* Clear the outdated next expected time */
                    for (auto it = timer_info_list_.begin(); it != timer_info_list_.end(); ++it)
                    {
                        if ((*it).next_expected_time <= now)
                        {
                            (*it).has_next_expected_time = false;

                            /**
                             * \brief The purpose of calling timer.setPeriod(period, false) is to release 
                             * the condition variable "timers_cond_" in timer_manager.h for faster loop speed.
                             * This is actually for fixing a bug in https://github.com/ros/ros_comm/blob/845f74602c7464e08ef5ac6fd9e26c97d0fe42c9/clients/roscpp/include/ros/timer_manager.h#L591 
                             * , where if use_sim_time is true, the timmer manager will block at least for 1 ms 
                             * at each timer callback even if the loop can be faster. This bug limits the real timer loop 
                             * speed to less than 1000 Hz. With this fix, the sim loop rate can be as fast as it can be (6000Hz+ max in practice).
                             */
                            // if ((*it).timer_impl->hasPending())
                            // {
                            //     //setPeriod(period, false) may change the next time of this timer based on the last real time. So I use a dummy timer.
                            //     // (*it).timer_impl->setPeriod((*it).period, false);
                            //     dummy_timer_.setPeriod(ros::DURATION_MAX, false);
                            // }
                            dummy_timer_.setPeriod(ros::DURATION_MAX, false);
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
        }

        void dummy_timer_callback(const ros::TimerEvent &event) {}
};


// typedef std::vector<std::thread::id> ThreadIdVector;
// typedef std::shared_ptr<ThreadIdVector> ThreadIdVectorPtr;

// /* global variables */

// static std::map<ros::CallbackQueueInterface*, ThreadIdVectorPtr>  cbqueue_spinnerthreads_map; // callback queue -> spinner threads that works for this callback queue

}
#endif