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
#include <std_msgs/Bool.h>
#include <boost/thread.hpp>
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

            bool kill_thread_;
            bool has_accelerate_timer_thread_;
            ros::Time last_clock_time_;
            boost::thread accelerate_timer_thread_;
            /**
             * \brief Open a new thread to check if /clock updates, call timer_.setPeriod(period) 
             * to release timers_cond_ in timer_manager.h for faster loop speed.
             * This is actually for fixing a bug in https://github.com/ros/ros_comm/blob/845f74602c7464e08ef5ac6fd9e26c97d0fe42c9/clients/roscpp/include/ros/timer_manager.h#L591 
             * , where if use_sim_time is true, the timmer manager will block at least 
             * for 1 ms even if the loop can be faster. This bug limits the timer loop 
             * speed to less than 1000 Hz. With this fix, the real loop speed can be 
             * 20x faster than real speed.
             */
            void AccelerateTimerThreadFunc();
        };

        typedef boost::shared_ptr<Impl> ImplPtr;
        typedef boost::weak_ptr<Impl> ImplWPtr;
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
 * serves as an extra part of timer_manager.h to request clock time.
 */
class TimerManagerExtra
{
    public:
        TimerManagerExtra()
        {
            nh_.setCallbackQueue(&callback_queue_);
            async_spinner_.start(); // start a new thread to listen to clock updates

            clock_updater_ = std::make_shared<ClockUpdater>(nh_);

            clock_sub_ = nh_.subscribe("/clock", 10, &TimerManagerExtra::cb_clock, this);

            std::cout << "[sss_utils::TimerManagerExtra] Init" << std::endl;
        }

        /* static(global) timer manager extra */
        static TimerManagerExtra& global()
        {
            static TimerManagerExtra global;
            return global;
        }

        /* add a timer to the timer_manager_extra */
        int add_timer(sss_utils::Timer::ImplPtr timer_impl_ptr)
        {
            static int handle = 0;
            timer_info_list_.emplace_back(
                TimerInfo
                {
                    .handle = handle,
                    .timer_impl = timer_impl_ptr,
                    .period = timer_impl_ptr->get_period(),
                    .has_next_expected_time = false,
                    .next_expected_time = ros::Time{MAX_ROS_TIME}
                }
            );
            handle ++;
            return (handle-1);
        }

        /* add next expected triggerred time of a timer callback */
        bool add_next_cb_time(int handle, ros::Time next_time)
        {
            std::lock_guard<std::mutex> lockGuard(timer_info_list_mutex_);

            /* Update the next_expected_time of timer[handle] and check if all timers having next expected times*/
            bool timer_handle_found = false;
            bool all_timer_has_next_expected_time = true;
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
                    all_timer_has_next_expected_time = false;
                }
                if (!all_timer_has_next_expected_time && timer_handle_found)
                {
                    break;
                }
            }
            if (!timer_handle_found)
            {
                std::cout << "[TimerManagerExtra::add_next_cb_time] No timer with handle " << handle << std::endl;
                return false;
            }
            
            /* If all timers have next expected times, request the smallest one */
            if (all_timer_has_next_expected_time)
            {
                std::sort(timer_info_list_.begin(), timer_info_list_.end(), compare_next_cb_time);
                ros::Time request_time = timer_info_list_[0].next_expected_time;
                return clock_updater_->request_clock_update(request_time);
            }

            return true;

            // /* If the next_cb_time is larger than now, push it to the timer_info_list_ and request clock update */
            // if (next_time > ros::Time::now())
            // {
            //     next_time_list_.emplace_back(next_time);
            //     // sort next_time_list_ from small to large
            //     std::sort(next_time_list_.begin(), next_time_list_.end());
            //     return clock_updater_->request_clock_update(next_time_list_[0]);
            // }
            // else
            // {
            //     if (!next_time_list_.empty())
            //     {
            //         std::sort(next_time_list_.begin(), next_time_list_.end());
            //         return clock_updater_->request_clock_update(next_time_list_[0]);
            //     }
            //     else
            //     {
            //         return false;
            //     }
            // }
        }

    private:
        ClockUpdaterPtr clock_updater_;

        ros::NodeHandle nh_;
        ros::CallbackQueue callback_queue_;
        ros::Subscriber clock_sub_;
        ros::AsyncSpinner async_spinner_{1, &callback_queue_};

        struct TimerInfo
        {
            int handle;
            sss_utils::Timer::ImplPtr timer_impl;
            ros::Duration period;
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
                    timer_info_list_[0].timer_impl->setPeriod(timer_info_list_[0].period, false); // setPeriod(period, false) does not affect the period and next expected time. 

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
                             * , where if use_sim_time is true, the timmer manager will block at least 
                             * for 1 ms even if the loop can be faster. This bug limits the real timer loop 
                             * speed to less than 1000 Hz. With this fix, the sim loop rate can be as fast 
                             * as it can be.
                             */
                            (*it).timer_impl->setPeriod((*it).period, false); // setPeriod(period, false) does not affect the period and next expected time. 
                        }
                        else
                        {
                            break;
                        }
                    }

                    // /* erase next expected time that smaller than now */
                    // int erase_cout = 0;
                    // for (auto it = next_time_list_.begin(); it != next_time_list_.end();)
                    // {
                    //     // std::cout << "[TimerManagerExtra::cb_clock] next_time_list_(i) = " << *it << std::endl;

                    //     if(*it <= now)
                    //     {
                    //         // std::cout << "[TimerManagerExtra::cb_clock] erase " << *it << std::endl;
                    //         it = next_time_list_.erase(it);
                    //         erase_cout ++;
                    //     }
                    //     else
                    //     {
                    //         // ++it;
                    //         break;
                    //     }
                    // }

                    // /* Request clock update */
                    // if (!next_time_list_.empty() && erase_cout != 0)
                    // {
                    //     clock_updater_->request_clock_update(next_time_list_[0]);
                    // }
                    // else
                    // {
                    //     //@TODO What if no next_time exists in the list?
                    //     // clock_updater_->request_clock_update(ros::Time{MAX_ROS_TIME});
                    // }
                }
            }

            // /**
            //  * \brief The purpose of calling timer.setPeriod(period, false) is to release 
            //  * the condition variable "timers_cond_" in timer_manager.h for faster loop speed.
            //  * This is actually for fixing a bug in https://github.com/ros/ros_comm/blob/845f74602c7464e08ef5ac6fd9e26c97d0fe42c9/clients/roscpp/include/ros/timer_manager.h#L591 
            //  * , where if use_sim_time is true, the timmer manager will block at least 
            //  * for 1 ms even if the loop can be faster. This bug limits the real timer loop 
            //  * speed to less than 1000 Hz. With this fix, the sim loop rate can be as fast 
            //  * as it can be.
            //  */
            // if(!timer_list_.empty())
            // {
            //     timer_list_[0]->setPeriod(timer_list_[0]->get_period(), false);
            //     // No affecting the period and next expected time. 
            // }
            // else
            // {
            //     std::cout << "[sss_utils::TimerManagerExtra] Warn! No timer added in TimerManagerExtra" << std::endl;
            // }
        }
};


// typedef std::vector<std::thread::id> ThreadIdVector;
// typedef std::shared_ptr<ThreadIdVector> ThreadIdVectorPtr;

// /* global variables */

// static std::map<ros::CallbackQueueInterface*, ThreadIdVectorPtr>  cbqueue_spinnerthreads_map; // callback queue -> spinner threads that works for this callback queue

}
#endif