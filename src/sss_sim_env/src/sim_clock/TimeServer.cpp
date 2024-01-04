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


#include "sss_sim_env/TimeServer.hpp"

TimeServer::TimeServer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    nh_.param<bool>("/use_sim_time", is_sim_time_, false);
    if (!is_sim_time_)
    {
        ROS_WARN("[TimeServer] /use_sim_time is false! Force to set /use_sim_time to true");
        nh_.setParam("/use_sim_time", true);
    }

    nh_private_.param<float>("max_speed_ratio", max_speed_ratio_, 100);
    nh_private_.param<bool>("auto_start", auto_start_, false);

    if (max_speed_ratio_ <= 0)
        { ROS_ERROR("[TimeServer] Invalid max_speed_ratio. It shoule be positive!");}
    is_paused_ = !auto_start_;

    sim_clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10, true); // latched
    online_pub_ = nh_.advertise<std_msgs::Bool>("/sss_clock_is_online", 10, true); // latched
    clock_control_service_ = nh_.advertiseService("/sss_clock_control", &TimeServer::cb_clock_control, this);
    timeclient_register_service_ = nh_.advertiseService("/sss_timeclient_register",  &TimeServer::cb_timeclient_register, this);
    timeclient_unregister_service_ = nh_.advertiseService("/sss_timeclient_unregister",  &TimeServer::cb_timeclient_unregister, this);

    init_time_ = ros::Time(0.0); //0 seconds
    sim_time_ = init_time_;

    next_client_id_ = 0;

    // client 0 is the simulation speed regulator
    std::unique_ptr<TimeClient> real_time_client(new TimeClient(next_client_id_, this, nh_, nh_private_));
    clients_vector_.emplace_back(std::move(real_time_client));
    ROS_INFO("[TimeServer] Register time client %s as simulation speed regulator",std::to_string(next_client_id_).c_str());
    speed_regulator_period_ = 0.01; // s
    speed_regulator_timer_ = nh_.createWallTimer(ros::WallDuration(speed_regulator_period_), &TimeServer::cb_speed_regulator_timer, this);

    ROS_INFO("[TimeServer] Initialized at max %sx speed!", std::to_string(max_speed_ratio_).c_str());

    /* Publish true to /sss_clock_is_online */
    std_msgs::Bool::Ptr msg(new std_msgs::Bool);
    msg->data = true;
    online_pub_.publish(msg);
}

void TimeServer::cb_speed_regulator_timer(const ros::WallTimerEvent &event)
{
    // try to update clock according to max_speed_ratio_
    static ros::Time expected_time = init_time_;
    if (!is_paused_)
    { 
        if (sim_time_ >= expected_time)
        {
            expected_time = expected_time + ros::Duration(speed_regulator_period_ * max_speed_ratio_);
            clients_vector_[0]->request_time = expected_time;
            clients_vector_[0]->has_new_request = true;
            try_update_clock();
        }
    }
}

bool TimeServer::cb_clock_control(sss_sim_env::SimClockControl::Request& req,
                                     sss_sim_env::SimClockControl::Response& res)
{
    // Control the clock to pause / proceed / speed up
    is_paused_ = !req.proceed; 
    if (req.max_sim_speed != 0) { max_speed_ratio_ = req.max_sim_speed;}

    res.success = true;

    return true;
}

bool TimeServer::cb_timeclient_register(sss_sim_env::ClientRegister::Request& req,
                                     sss_sim_env::ClientRegister::Response& res)
{
    //register a time client
    next_client_id_ ++;
    std::unique_ptr<TimeClient> client(new TimeClient(next_client_id_, this, nh_,nh_private_));
    clients_vector_.emplace_back(std::move(client));   

    ROS_INFO("[TimeServer] Register time client %s",std::to_string(next_client_id_).c_str());

    res.client_id = next_client_id_;
    res.success = true; 

    //update time immediately
    try_update_clock();

    return true;
}


bool TimeServer::cb_timeclient_unregister(sss_sim_env::ClientUnregister::Request& req,
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
            //update time immediately
            try_update_clock();
            break;
        }
    }
    return true;
}

/** Check all timeclients and decide whether or not updating clock time **/
void TimeServer::try_update_clock()
{   
    ros::WallTime begin=ros::WallTime::now();  

    if (clients_vector_.empty())
    {
        ROS_INFO("[TimeServer] try_update_clock() called. But clients_vector_ is empty, which means that there is no TimeClient");
        return;
    }

    bool all_client_has_new_request = true;
    ros::Time min_time_request = ros::Time(0.0);
    // search for minimum time request when all clients have new request
    for (int i=0; i<clients_vector_.size(); ++i)
    {
        if (clients_vector_[i]->has_new_request == false){
            all_client_has_new_request = false;
            break;

            // ROS_INFO("[TimeServer] try_update_clock() refuses to update clock because clients_vector_[%s] has no new request", std::to_string(i).c_str());
        }
        else{
            if (min_time_request == ros::Time(0.0) || clients_vector_[i]->request_time < min_time_request){
                min_time_request = clients_vector_[i]->request_time;
            }
        }
    }
    // update and publish sim_time if all clients have new request
    if (all_client_has_new_request == true){
        if (min_time_request > sim_time_){
            sim_time_ = min_time_request;
            rosgraph_msgs::ClockPtr msg(new rosgraph_msgs::Clock);
            msg->clock = min_time_request;
            sim_clock_pub_.publish(msg);

            // ROS_INFO("[TimeServer] try_update_clock() publish %ss to /clock", std::to_string(sim_time_.toSec()).c_str());
        }
        // set has_new_request = false for clients whose request_time is satisfied.
        for (int i=0; i<clients_vector_.size(); ++i)
        {
            if (clients_vector_[i]->request_time <= min_time_request){
                clients_vector_[i]->has_new_request = false;
            }
        }        
    }

    ros::WallDuration duration=ros::WallTime::now() - begin;  
    // ROS_INFO("[TimeServer] Function try_update_clock() called using %ss %sns", std::to_string(duration.sec).c_str(), std::to_string(duration.nsec).c_str());

}

/** Create a timeclient for each node thread that requires clock updating**/
TimeServer::TimeClient::TimeClient(const int &id, TimeServer *obj, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): client_id_(id), time_server(obj), nh_(nh), nh_private_(nh_private)
{
    has_new_request = false;
    request_time = ros::Time(0.0); //0 seconds
    update_clock_request_sub_ = nh_.subscribe("/sss_time_client"+std::to_string(client_id_)+"/update_clock_request", 1000, &TimeClient::cb_update_clock_request, this);
}


void TimeServer::TimeClient::cb_update_clock_request(const rosgraph_msgs::Clock::ConstPtr& msg)
{
    // ROS_INFO("[TimeClient %s] Receive time request %ss", std::to_string(client_id_).c_str(), std::to_string(msg->clock.toSec()).c_str());

    ros::Time new_request_time = msg->clock;
    if (new_request_time > request_time)
    {
        request_time = new_request_time;
        has_new_request = true;
        time_server->try_update_clock();
    }
    else if (new_request_time < request_time)
    {
        ROS_WARN("[TimeClient %s] new time request %ss is smaller than last request %ss", std::to_string(client_id_).c_str(), std::to_string(new_request_time.toSec()).c_str(), std::to_string(request_time.toSec()).c_str());

        request_time = new_request_time;
        has_new_request = true;
        time_server->try_update_clock();
    }

}