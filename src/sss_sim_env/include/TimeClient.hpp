/**
 * @file TimeClient.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Subscribe the time-updating request from each client and 
 * update their status.
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-11-19
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __TIMECLIENT__
#define __TIMECLIENT__
#include "ros/ros.h"
#include <std_msgs/Time.h>

class TimeClient{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber update_time_request_sub_;
        void cb_update_time_request(const std_msgs::Time::ConstPtr& msg);
    public:
        int client_id_;
        bool has_new_request;
        ros::Time request_time;    
        TimeClient(const int &id, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
};

#endif