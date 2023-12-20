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


#include "sss_sim_env/ClockUpdater.hpp"

namespace sss_utils
{

ClockUpdater::ClockUpdater()
{
    nh_.param<bool>("/use_sim_time", use_sim_time, false);

    if (use_sim_time){
        ROS_INFO("[ClockUpdater] use_sim_time == true. Init. Waiting for service /sss_timeclient_register");
        init();
    }
    else{
        ROS_INFO("[ClockUpdater] use_sim_time == false. Skip!");
    }
}

ClockUpdater::~ClockUpdater()
{
    // Unregister from /sss_timeclient_unregister
    unregister();
}

/* Init sim clock updater */
void ClockUpdater::init()
{
    register_client_ = nh_.serviceClient<sss_sim_env::ClientRegister>("/sss_timeclient_register");
    unregister_client_ = nh_.serviceClient<sss_sim_env::ClientUnregister>("/sss_timeclient_unregister");

    //register a time client
    // block until registered successfully
    sss_sim_env::ClientRegister srv;
    while(!register_client_.call(srv))
    {
        ros::WallDuration(0.5).sleep();
    }

    time_client_id_ = srv.response.client_id;
    ROS_INFO("[ClockUpdater] register to /sss_timeclient_register with response id = %s", std::to_string(time_client_id_).c_str());

    update_clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/sss_time_client"+std::to_string(time_client_id_)+"/update_clock_request", 10);
}

/* Publish new time request */
void ClockUpdater::request_clock_update(ros::Time new_time)
{
    if (use_sim_time){
        rosgraph_msgs::Clock msg;
        msg.clock = new_time;
        update_clock_pub_.publish(msg);
        // ROS_INFO("[ClockUpdater%s] publish %ss to topic update_clock_request", std::to_string(time_client_id_).c_str(), std::to_string(new_time.toSec()).c_str());
    }
}

/* unregister from time server. Stop time request in the future */
bool ClockUpdater::unregister()
{
    if (use_sim_time){
        sss_sim_env::ClientUnregister srv;
        srv.request.client_id = time_client_id_;
        if(unregister_client_.call(srv))
        {
            ROS_INFO("[ClockUpdater%s] Unregister!", std::to_string(time_client_id_).c_str());
            std::cout << "[ClockUpdater" << time_client_id_ << "] Unregister" << std::endl;
            return true;
        }
        else
        {
            ROS_INFO("[ClockUpdater%s] Fail to unregister!", std::to_string(time_client_id_).c_str());
            std::cout << "[ClockUpdater" << time_client_id_ << "] Fail to unregister" << std::endl;
            return false;
        }
    }
    return true;
}

}