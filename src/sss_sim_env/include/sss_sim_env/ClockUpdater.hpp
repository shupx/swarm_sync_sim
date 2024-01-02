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
#include "sss_sim_env/ClientRegister.h"
#include "sss_sim_env/ClientUnregister.h"

namespace sss_utils
{

class ClockUpdater
{
    private:
        bool use_sim_time;

        ros::NodeHandle nh_;

        ros::Subscriber simclock_online_sub_;
        ros::ServiceClient register_client_;
        ros::ServiceClient unregister_client_;

        int time_client_id_;

        ros::Publisher update_clock_pub_;

        bool inited_; // if ClockUpdater is inited

        /* Register when sim clock is online */
        void cb_simclock_online(const std_msgs::Bool::ConstPtr& msg);
        
    public:
        ClockUpdater(const ros::NodeHandle &nh);
        ~ClockUpdater();



        /* Publish new time request */
        bool request_clock_update(ros::Time new_time);

        /* unregister from time server. Stop time request in the future */
        bool unregister();
        
};

}
#endif