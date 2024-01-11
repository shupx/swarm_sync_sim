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
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Bool.h>
#include <thread>
#include <map>
#include "sss_sim_env/ClientRegister.h"
#include "sss_sim_env/ClientUnregister.h"

namespace sss_utils
{

class ClockUpdater : public std::enable_shared_from_this<ClockUpdater> 
{
    public:
        ClockUpdater(const ros::NodeHandle &nh);
        ~ClockUpdater();

        /* Publish new time request */
        bool request_clock_update(const ros::Time &new_time);

        /* unregister from time server. Stop time request in the future */
        bool unregister();

        /* get the latest request clock time */
        ros::Time get_request_time() {return request_time_;}
    
    private:
        bool use_sim_time;

        ros::NodeHandle nh_;

        ros::Subscriber simclock_online_sub_;
        ros::ServiceClient register_client_;
        ros::ServiceClient unregister_client_;

        int time_client_id_;

        ros::Publisher update_clock_pub_;

        bool inited_; // if ClockUpdater is inited

        ros::Time request_time_{DBL_MAX}; // requested clock time (default infinity)

        /* Register when sim clock is online */
        void cb_simclock_online(const std_msgs::Bool::ConstPtr& msg);  
};



typedef std::shared_ptr<ClockUpdater> ClockUpdaterPtr;

/* global variables */
static std::map<std::thread::id, ClockUpdaterPtr>  thread_clockupdater_map;  // One clockupdater serves for one thread

}

#endif