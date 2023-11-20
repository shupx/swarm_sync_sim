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
    sim_time_pub_ = nh_.advertise<std_msgs::Time>("/sss_sim_time", 10);
    timeclient_register_service_ = nh_.advertiseService("/sss_timeclient_register",  &TimeServer::timeclient_register, this);
    timeclient_unregister_service_ = nh_.advertiseService("/sss_timeclient_unregister",  &TimeServer::timeclient_unregister, this);

    sim_time = ros::Time(0.0); //0 seconds

    next_client_id_ = 0;

    std::unique_ptr<TimeClient> real_time_client(new TimeClient(next_client_id_, this, nh_, nh_private_));
    clients_vector_.emplace_back(std::move(real_time_client));

    ROS_INFO("[TimeServer] Register time client %s",std::to_string(next_client_id_).c_str());
}


bool TimeServer::timeclient_register(sss_sim_env::ClientRegister::Request& req,
                                     sss_sim_env::ClientRegister::Response& res)
{
    //register a time client
    next_client_id_ ++;
    std::unique_ptr<TimeClient> client(new TimeClient(next_client_id_, this, nh_,nh_private_));
    clients_vector_.emplace_back(std::move(client));   
    res.client_id = next_client_id_;
    res.success = true; 

    return true;
}


bool TimeServer::timeclient_unregister(sss_sim_env::ClientUnregister::Request& req,
                                     sss_sim_env::ClientUnregister::Response& res)
{
    //unregister a time client
    int id =  req.client_id;
    res.success = false; 
    if (id == 0)
    {
        ROS_WARN("[TimeServer] Time client 0 is reserved and can not be unregistered!");
    }
    for (int i=1; i<clients_vector_.size(); ++i)
    {
        if (id == clients_vector_[i]->client_id_){
            clients_vector_.erase(clients_vector_.begin() + i);
            ROS_INFO("[TimeServer] Unregister time client %s",std::to_string(id).c_str());
            res.success = true; 
            break;
        }
    }
    return true;
}


void TimeServer::update_time(int client_id)
{    
    ROS_INFO("[TimeServer] update_time called by time_client%s",std::to_string(next_client_id_).c_str());
    bool all_client_has_new_request = true;
    ros::Time min_time_request = ros::Time(0.0);
    // search for minimum time request when all clients have new request
    for (int i=0; i<clients_vector_.size(); ++i)
    {
        if (clients_vector_[i]->has_new_request == false){
            all_client_has_new_request = false;
            break;
        }
        else{
            if (min_time_request == ros::Time(0.0) || clients_vector_[i]->request_time < min_time_request){
                min_time_request = clients_vector_[i]->request_time;
            }
        }
    }
    // update and publish sim_time if all clients have new request
    if (all_client_has_new_request == true){
        if (min_time_request > sim_time){
            sim_time = min_time_request;
            std_msgs::Time msg;
            msg.data = min_time_request;
            sim_time_pub_.publish(msg);
        }
        // set has_new_request = false for clients whose request_time is satisfied.
        for (int i=0; i<clients_vector_.size(); ++i)
        {
            if (clients_vector_[i]->request_time <= min_time_request){
                clients_vector_[i]->has_new_request = false;
            }
        }        
    }
    std::cout << sim_time << std::endl;
}


TimeServer::TimeClient::TimeClient(const int &id, TimeServer *obj, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): client_id_(id),nh_(nh),nh_private_(nh_private)
{
    has_new_request = false;
    request_time = ros::Time(0.0); //0 seconds
    update_time_request_sub_ = nh_.subscribe("/sss_time_client"+std::to_string(client_id_)+"/update_time_request", 1000, &TimeClient::cb_update_time_request, this, ros::TransportHints().tcpNoDelay());
    time_server = obj;
}


void TimeServer::TimeClient::cb_update_time_request(const std_msgs::Time::ConstPtr& msg)
{
    // ROS_INFO("[TimeClient %s] Receive time request %ss %sns", std::to_string(client_id_).c_str(), std::to_string(request_time.sec).c_str(), std::to_string(request_time.nsec).c_str());
    ros::Time new_request_time = msg->data;
    if (new_request_time > request_time){
        request_time = msg->data;
        has_new_request = true;
        time_server->update_time(client_id_);
    }
    else{
        ROS_ERROR("[TimeClient %s] new time request %ss %sns can not be smaller than last request %ss %sns", std::to_string(client_id_).c_str(), std::to_string(new_request_time.sec).c_str(), std::to_string(new_request_time.nsec).c_str(), std::to_string(request_time.sec).c_str(), std::to_string(request_time.nsec).c_str());
    }

}