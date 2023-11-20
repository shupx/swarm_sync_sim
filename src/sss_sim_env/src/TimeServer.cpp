/**
 * @file TimeServer.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief simulation clock time server. Process the sim_time update request from 
 * time_client and determine the simulation time.
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


#include "TimeServer.hpp"

TimeServer::TimeServer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),nh_private_(nh_private)
{
    time_pub_ = nh_.advertise<std_msgs::Time>("/sss_sim_time", 10);
    time_sub_ = nh_.subscribe("/sss_sim_time_sub", 1, &TimeServer::cb_time, this, ros::TransportHints().tcpNoDelay());
    timeclient_register_service_ = nh_.advertiseService("/sss_timeclient_register",  &TimeServer::timeclient_register, this);

    next_client_id_ = 0;

    std::unique_ptr<TimeClient> real_time_client(new TimeClient(next_client_id_,nh_,nh_private_));
    clients_vector_.emplace_back(std::move(real_time_client));

    std::cout << clients_vector_[0]->has_new_request << std::endl;
}

void TimeServer::cb_time(const std_msgs::Time::ConstPtr& msg)
{
    ros::Time time = msg->data;
    // ros::Time a = 
    if ((time - ros::Duration(1.2)).toNSec() > 0){
        std::cout << "big" << std::endl;
    }
    std::cout << (time - ros::Duration(1.2)).toNSec() << std::endl;
}

bool TimeServer::timeclient_register(sss_sim_env::ClientRegister::Request& req,
                                     sss_sim_env::ClientRegister::Response& res)
{
    //register a time client
    next_client_id_ ++;
    std::unique_ptr<TimeClient> client(new TimeClient(next_client_id_,nh_,nh_private_));
    clients_vector_.emplace_back(std::move(client));   
    res.client_id = next_client_id_;
    res.success = true; 

    return true;
}