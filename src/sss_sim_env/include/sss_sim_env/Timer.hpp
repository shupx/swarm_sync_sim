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
#include <std_msgs/Bool.h>
#include <boost/thread.hpp>
#include "sss_sim_env/ClockUpdater.hpp"


namespace sss_utils
{

/**
 * \brief To Replace ros::Timer for swarm_sync_sim.
 * If use_sim_time is true, it is a ros::Timer with request_clock_update every loop.
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
        class Impl
        {
        public:
            /* If use_sim_time is true, it creates a ros::Timer with request_clock_update every loop.
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
            /* stop timer and unregister clock_updater_ */
            void stop();

        private:
            bool use_sim_time_;
            ros::NodeHandle nh_;
            ros::Timer timer_;
            ros::Subscriber simclock_online_sub_;

            std::shared_ptr<ClockUpdater> clock_updater_;

            ros::Duration period_;
            ros::TimerCallback callback_;
            bool oneshot_;
            bool autostart_;

            bool started_;

            bool inited_;

            double last_sim_timer_cb_time_;
            /**
             * \brief callback_ + clock update in every loop 
             */
            void sim_timer_callback(const ros::TimerEvent &event);

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
        Timer(const ros::NodeHandle &nh, const ros::Duration &period, const ros::TimerCallback& callback, bool oneshot, bool autostart)
        {
            impl_ = boost::make_shared<Impl>(nh, period, callback, oneshot, autostart);
        }

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

}
#endif