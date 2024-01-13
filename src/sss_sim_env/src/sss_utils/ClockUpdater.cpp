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
    : inited_(false)
{
    nh_async_.param<bool>("/use_sim_time", use_sim_time, false);

    if (use_sim_time){
        // ROS_INFO("[ClockUpdater] use_sim_time == true. Init. Waiting for sim_clock online.");

        nh_async_.setCallbackQueue(&callback_queue_);
        async_spinner_.start(); // start a new thread to listen to sim clock online

        simclock_online_sub_ = nh_async_.subscribe("/sss_clock_is_online", 1000, &ClockUpdater::cb_simclock_online, this);
        register_client_ = nh_async_.serviceClient<sss_sim_env::ClientRegister>("/sss_timeclient_register");
        unregister_client_ = nh_async_.serviceClient<sss_sim_env::ClientUnregister>("/sss_timeclient_unregister");

        UpdateClockPublisher::global(); // Create a global update_clock_publisher if has not been created
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

/* Register when sim clock is online */
void ClockUpdater::cb_simclock_online(const std_msgs::Bool::ConstPtr& msg)
{
    // std::cout << "[ClockUpdater::cb_simclock_online] Detect sim_clock online!" << std::endl;

    if (msg->data == true)
    {
        sss_sim_env::ClientRegister srv;
        while(!register_client_.call(srv))  // block until registered successfully
        {
            ros::WallDuration(0.5).sleep();
            ROS_INFO("[ClockUpdater] Waiting for service /sss_timeclient_register...");
        }

        time_client_id_ = srv.response.client_id;
        ROS_INFO("[ClockUpdater] register to /sss_timeclient_register with response id = %s", std::to_string(time_client_id_).c_str());
        
        // update_clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/sss_time_client"+std::to_string(time_client_id_)+"/update_clock_request", 1000, true); //latched

        inited_ = true;

        // std::cout << "[ClockUpdater::cb_simclock_online] inited_ = true" << std::endl;

        // request_clock_update(ros::TIME_MAX); // Request infinity time on init
    }
}

/* Publish new time request */
bool ClockUpdater::request_clock_update(const ros::Time &new_time)
{
    if (use_sim_time){
        if (inited_)
        {
            request_time_ = new_time;

            // rosgraph_msgs::ClockPtr msg(new rosgraph_msgs::Clock);
            // msg->clock = new_time;
            // update_clock_pub_.publish(msg);

            // ROS_INFO("[ClockUpdater%s] publish %ss to topic update_clock_request", std::to_string(time_client_id_).c_str(), std::to_string(new_time.toSec()).c_str());

            return UpdateClockPublisher::global().publish(time_client_id_, request_time_);
        }
        else
        {
            ROS_ERROR("[ClockUpdater%s] Not inited.", std::to_string(time_client_id_).c_str());
            return false;
        }
    }
    return true;
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