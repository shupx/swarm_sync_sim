/**
 * @file ClockUpdater.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Update sim clock. Utilized by other sim/real agent nodes.
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

#ifndef __CLOCK_UPDATER__
#define __CLOCK_UPDATER__
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>
#include <thread>
#include <map>
#include "sss_sim_env/TimeRequest.h"
#include "sss_sim_env/ClientRegister.h"
#include "sss_sim_env/ClientUnregister.h"

// #ifndef MAX_ROS_TIME
// #define MAX_ROS_TIME UINT32_MAX, 999999999UL  // {sec, nsec}
// #endif

namespace sss_utils
{

/* Static (global) update clock pulisher used by all threads */
class UpdateClockPublisher
{
    public:
        UpdateClockPublisher()
        {
            update_clock_pub_ = nh_.advertise<sss_sim_env::TimeRequest>("/sss_time_client/update_clock_request", 1000, true); //latched
        }

        /* static(global) publisher */
        static UpdateClockPublisher& global()
        {
            static UpdateClockPublisher global;
            return global;
        }

        /* publish time request for time client i */
        bool publish(const int& time_client_id, const ros::Time &new_time)
        {
            sss_sim_env::TimeRequest::Ptr msg(new sss_sim_env::TimeRequest);
            msg->time_client_id = time_client_id;
            msg->request_time = new_time;
            update_clock_pub_.publish(msg);

            return update_clock_pub_.getNumSubscribers() > 0;
        }
    private:
        ros::NodeHandle nh_;
        ros::Publisher update_clock_pub_;
};


class ClockUpdater /*: public std::enable_shared_from_this<ClockUpdater> */
{
    public:
        ClockUpdater();
        ~ClockUpdater();

        /* Publish new time request */
        bool request_clock_update(const ros::Time &new_time);

        /* unregister from time server. Stop time request in the future */
        bool unregister();

        /* get the latest request clock time */
        ros::Time get_request_time() {return request_time_;}
    
    private:
        bool use_sim_time;

        ros::NodeHandle nh_async_;
        ros::CallbackQueue callback_queue_;
        ros::AsyncSpinner async_spinner_{1, &callback_queue_};

        ros::Subscriber simclock_online_sub_;
        ros::ServiceClient register_client_;
        ros::ServiceClient unregister_client_;

        int time_client_id_;

        // ros::Publisher update_clock_pub_;

        bool inited_; // if ClockUpdater is inited

        ros::Time request_time_; // requested clock time

        /* Register when sim clock is online */
        void cb_simclock_online(const std_msgs::Bool::ConstPtr& msg);  
};


typedef std::shared_ptr<ClockUpdater> ClockUpdaterPtr;

/* global variables */
// static std::map<std::thread::id, ClockUpdaterPtr>  thread_clockupdater_map;  

/* static(global) thread->clockupdaters map */
class ThreadClockupdaters
{
public:
    ThreadClockupdaters() {}

    /* static(global) thread->clockupdaters map */
    static ThreadClockupdaters& global()
    {
        static ThreadClockupdaters global;
        return global;
    }

    ClockUpdaterPtr get_clockupdater(const std::thread::id& thread_id)
    {
        /* If no clockupdater exists on this thread, make a clockupdater */
        if(thread_clockupdater_map_.find(thread_id) == thread_clockupdater_map_.end())
        {
            // std::cout << "[ThreadClockupdaters::get_clockupdater] Create a new clock updater on thread " << thread_id << std::endl;
            ROS_INFO("[ThreadClockupdaters::get_clockupdater] Create a new clock updater on thread %s", std::to_string(*(unsigned int*)&thread_id).c_str()); 
            thread_clockupdater_map_[thread_id] = std::make_shared<ClockUpdater>();
        }

        return thread_clockupdater_map_[thread_id];
    }

private:
    // One clockupdater serves for one thread
    std::map<std::thread::id, ClockUpdaterPtr> thread_clockupdater_map_;
};

} //sss_utils

#endif