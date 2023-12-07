/**
 * @file ClockUpdater.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Update sim clock. Utilized by other sim agent nodes.
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


#include "ClockUpdater/ClockUpdater.h"

ClockUpdater::ClockUpdater(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),nh_private_(nh_private)
{
    nh_.param<bool>("/use_sim_time", use_sim_time, false);

    if (use_sim_time){
        init();
    }
    else{
        ROS_INFO("[ClockUpdater] use_sim_time == false, skip");
    }
}

ClockUpdater::~ClockUpdater()
{
    // Unregister from /sss_timeclient_unregister
    if (use_sim_time){
        sss_sim_env::ClientUnregister srv;
        srv.request.client_id = time_client_id_;
        unregister_client_.call(srv);
    }
}

/* Init sim clock updater */
void ClockUpdater::init()
{
    register_client_ = nh_.serviceClient<sss_sim_env::ClientRegister>("/sss_timeclient_register");
    unregister_client_ = nh_.serviceClient<sss_sim_env::ClientUnregister>("/sss_timeclient_unregister");

    //register a time client
    sss_sim_env::ClientRegister srv;
    if(register_client_.call(srv)){
        time_client_id_ = srv.response.client_id;
    }
    else{
        ROS_ERROR("[ClockUpdater] Fail to register with /sss_timeclient_register");
    }

    update_clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/sss_time_client"+std::to_string(time_client_id_)+"/update_clock_request", 10);
}

/* Publish new time request */
void ClockUpdater::request_clock_update(ros::Time new_time)
{
    rosgraph_msgs::Clock msg;
    msg.clock = new_time;
    update_clock_pub_.publish(msg);
}