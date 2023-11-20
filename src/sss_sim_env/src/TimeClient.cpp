/**
 * @file TimeClient.cpp
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

#include "TimeClient.hpp"

TimeClient::TimeClient(const int &id, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): client_id_(id),nh_(nh),nh_private_(nh_private)
{
    update_time_request_sub_ = nh_.subscribe("/sss_time_client"+std::to_string(client_id_)+"/update_time_request", 1, &TimeClient::cb_update_time_request, this, ros::TransportHints().tcpNoDelay());

    has_new_request = false;
    request_time = ros::Time(0.0); //0 seconds
}

void TimeClient::cb_update_time_request(const std_msgs::Time::ConstPtr& msg)
{
    request_time = msg->data;
}