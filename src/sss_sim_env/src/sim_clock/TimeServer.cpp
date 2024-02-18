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
    nh_async_.setCallbackQueue(&async_callback_queue_);
    async_spinner_.start(); // start a new thread to listen to clock updates

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
    speed_regulator_expected_time_ = init_time_;

    /* Publish the first clock time */
    rosgraph_msgs::ClockPtr clock_msg(new rosgraph_msgs::Clock);
    clock_msg->clock = init_time_;
    sim_clock_pub_.publish(clock_msg);

    next_client_id_ = 0;

    /* client 0 is the simulation speed regulator */
    // TimeClientPtr real_time_client(new TimeClient(next_client_id_, this, nh_async_, nh_private_)); // use a single thread spinner of nh_async_
    TimeClientPtr real_time_client(new TimeClient(next_client_id_));
    clients_vector_.emplace_back(std::move(real_time_client));
    ROS_INFO("[TimeServer] Register time client %s as simulation speed regulator",std::to_string(next_client_id_).c_str());
    // clients_vector_[0]->has_new_request = true;
    // clients_vector_[0]->request_time = ros::TIME_MAX;

    // update client 0 time in a wall timer with a seperate spinner thread
    speed_regulator_period_ = 0.01; // (s) maximum sim step (crucial for timers on multi threads)
    speed_regulator_timer_ = nh_async_.createWallTimer(ros::WallDuration(speed_regulator_period_ / max_speed_ratio_), &TimeServer::cb_speed_regulator_timer, this);
    // speed_regulator_timer_ = nh_.createWallTimer(ros::WallDuration(speed_regulator_period_), &TimeServer::cb_speed_regulator_timer, this);

    update_clock_request_sub_ = nh_async_.subscribe("/sss_time_client/update_clock_request", 10000, &TimeServer::cb_update_clock_request, this);

    /* Publish true to /sss_clock_is_online */
    std_msgs::Bool::Ptr online_msg(new std_msgs::Bool);
    online_msg->data = true;
    online_pub_.publish(online_msg);

    ROS_INFO("[TimeServer] Initialized at max %sx speed!", std::to_string(max_speed_ratio_).c_str());
}

void TimeServer::cb_speed_regulator_timer(const ros::WallTimerEvent &event)
{
    /* try to update clock according to max_speed_ratio_ */
    // static ros::Time expected_time = init_time_;
    if (!is_paused_)
    {
        if (sim_time_ >= speed_regulator_expected_time_)        
        {
            speed_regulator_expected_time_ = speed_regulator_expected_time_ + ros::Duration(speed_regulator_period_);
            {
                std::lock_guard<std::recursive_mutex> LockGuard(clients_vector_[0]->client_mutex_);
                clients_vector_[0]->request_time = speed_regulator_expected_time_;
                clients_vector_[0]->has_new_request = true;
            }
            // try_update_clock();
        }
        try_update_clock();
    }
}

bool TimeServer::cb_clock_control(sss_sim_env::SimClockControl::Request& req,
                                     sss_sim_env::SimClockControl::Response& res)
{
    /* Control the clock to pause / proceed*/
    is_paused_ = !req.proceed; 

    /* Reset speed regulator */
    if (req.max_sim_speed != 0) 
    { 
        max_speed_ratio_ = req.max_sim_speed;
        speed_regulator_timer_.setPeriod(ros::WallDuration(speed_regulator_period_ / max_speed_ratio_), false);
    }

    res.success = true;

    return true;
}

bool TimeServer::cb_timeclient_register(sss_sim_env::ClientRegister::Request& req,
                                     sss_sim_env::ClientRegister::Response& res)
{
    //register a time client
    next_client_id_ ++;
    // TimeClientPtr client(new TimeClient(next_client_id_, this, nh_async_,nh_private_)); // use a single thread spinner of nh_async_
    TimeClientPtr client(new TimeClient(next_client_id_));
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

void TimeServer::cb_update_clock_request(const sss_sim_env::TimeRequest::ConstPtr& msg)
{
    // ROS_WARN("[TimeClient %s] Receive time request %ss", std::to_string(msg->time_client_id).c_str(), std::to_string(msg->request_time.toSec()).c_str());

    uint32_t id = msg->time_client_id;
    ros::Time new_request_time = msg->request_time;

    /* find the time client that matches the id */
    auto it = std::find_if(clients_vector_.begin(), clients_vector_.end(), [id](TimeClientPtr client){return client->client_id_ == id;});

    if(it == clients_vector_.end())
    {
        ROS_WARN("[TimeServer::cb_update_clock_request] time client id %s not found.", std::to_string(id).c_str());
        return;
    }

    {
        std::lock_guard<std::recursive_mutex> LockGuard((*it)->client_mutex_);

        if (new_request_time < (*it)->request_time)
        {
            // ROS_WARN("[TimeClient %s] new time request %ss is smaller than last request %ss", std::to_string(client_id_).c_str(), std::to_string(new_request_time.toSec()).c_str(), std::to_string(request_time.toSec()).c_str());
        }

        (*it)->request_time = new_request_time;
        (*it)->has_new_request = true;
    }

    try_update_clock();
}


/** Check all timeclients and decide whether or not to update the sim clock time **/
bool TimeServer::try_update_clock()
{   
    // std::lock_guard<std::recursive_mutex> LockGuard(try_update_clock_mutex_);

    ros::WallTime begin=ros::WallTime::now();  

    /* Check whether there exists a time client */
    if (clients_vector_.empty())
    {
        ROS_INFO("[TimeServer] try_update_clock() called. But clients_vector_ is empty, which means that there is no TimeClient");
        return false;
    }

    /* Check whether there exists a time client other than the speed regulator*/
    if (clients_vector_.size() == 1)
    {
        // ROS_INFO("[TimeServer] try_update_clock() called. But no time client > 0 registered");
        return false;
    }

    /* Check whether all time clients except client 0 have infinite time requests (Abnormal) */
    if (clients_vector_.size() > 1)
    {
        bool all_clients_has_infinity_request = true;
        for (int i=1; i<clients_vector_.size(); ++i)
        {
            if (clients_vector_[i]->has_new_request == false ||  
                    clients_vector_[i]->request_time < ros::Time{UINT32_MAX,0})
            {
                all_clients_has_infinity_request = false;
                break;
            }
        }
        if (all_clients_has_infinity_request)
        {
            ROS_WARN("[TimeServer::try_update_clock] All time clients >=1 have new infinite request time, which means they are inactive. Stop clock.");
            return false;
        }
    }

    bool ret = false;

    /* Search for the minimum time request when all clients have new request */
    bool all_client_has_new_request = true;
    ros::Time min_time_request = ros::Time(0.0);
    for (int i=0; i<clients_vector_.size(); ++i)
    {
        std::lock_guard<std::recursive_mutex> LockGuard(clients_vector_[i]->client_mutex_);

        if (clients_vector_[i]->has_new_request == false){
            all_client_has_new_request = false;

            // ROS_INFO("[TimeServer] try_update_clock() refuses to update clock because time client %s has no new request. Its last request is %ss", std::to_string(clients_vector_[i]->client_id_).c_str(), std::to_string(clients_vector_[i]->request_time.toSec()).c_str());

            break;
        }
        else{
            if (min_time_request == ros::Time(0.0) || clients_vector_[i]->request_time < min_time_request){
                min_time_request = clients_vector_[i]->request_time;
            }
        }
    }

    // std::cout << "[TimeServer::try_update_clock] all_client_has_new_request = " << all_client_has_new_request << ", min_time_request = " << min_time_request.toSec() << std::endl;

    /* Publish the minimum time request (must be larger than now) as sim_time if all clients have new request */
    if (all_client_has_new_request)
    {
        if (min_time_request > ros::Time::now()){
        // if (min_time_request > sim_time_){
            sim_time_ = min_time_request;
            rosgraph_msgs::ClockPtr msg(new rosgraph_msgs::Clock);
            msg->clock = min_time_request;
            sim_clock_pub_.publish(msg);

            // ROS_INFO("[TimeServer] try_update_clock() publish %ss to /clock", std::to_string(sim_time_.toSec()).c_str());

            ret = true;
        }

        // set has_new_request = false for clients whose request_time is satisfied.
        for (int i=0; i<clients_vector_.size(); ++i)
        {
            std::lock_guard<std::recursive_mutex> LockGuard(clients_vector_[i]->client_mutex_);

            // if (clients_vector_[i]->request_time <= min_time_request)
            if (clients_vector_[i]->request_time <= sim_time_)
            // if (clients_vector_[i]->request_time <= ros::Time::now())
            {
                clients_vector_[i]->has_new_request = false;
            }
        }        
    }

    // ros::WallDuration duration=ros::WallTime::now() - begin;  
    // ROS_INFO("[TimeServer] Function try_update_clock() called using %ss %sns", std::to_string(duration.sec).c_str(), std::to_string(duration.nsec).c_str());

    return ret;
}

/** Create a timeclient for each node thread that requires clock updating**/
TimeServer::TimeClient::TimeClient(const int &id): client_id_(id)
// TimeServer::TimeClient::TimeClient(const int &id, TimeServer *obj, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): client_id_(id), time_server(obj), nh_(nh), nh_private_(nh_private)
{
    has_new_request = false;
    request_time = ros::Time(0.0); //0 seconds

    // update_clock_request_sub_ = nh_.subscribe("/sss_time_client"+std::to_string(client_id_)+"/update_clock_request", 1000, &TimeClient::cb_update_clock_request, this);

    // nh_async.setCallbackQueue(&callback_queue_);
    // update_clock_request_sub_ = nh_async.subscribe("/sss_time_client"+std::to_string(client_id_)+"/update_clock_request", 1000, &TimeClient::cb_update_clock_request, this);
    // async_spinner_.start(); // start a single spinner thread for this subscriber
}


// void TimeServer::TimeClient::cb_update_clock_request(const rosgraph_msgs::Clock::ConstPtr& msg)
// {
//     // ROS_INFO("[TimeClient %s] Receive time request %ss", std::to_string(client_id_).c_str(), std::to_string(msg->clock.toSec()).c_str());

//     ros::Time new_request_time = msg->clock;
//     if (new_request_time < request_time)
//     {
//         // ROS_WARN("[TimeClient %s] new time request %ss is smaller than last request %ss", std::to_string(client_id_).c_str(), std::to_string(new_request_time.toSec()).c_str(), std::to_string(request_time.toSec()).c_str());
//     }

//     {
//         // std::lock_guard<std::recursive_mutex> LockGuard(client_mutex_);
//         request_time = new_request_time;
//         has_new_request = true;
//     }
    
//     time_server->try_update_clock();
// }