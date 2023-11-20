/**
 * @file TimeServer.hpp
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

#ifndef __TIMESERVER__
#define __TIMESERVER__
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include "sss_sim_env/ClientRegister.h"
#include "TimeClient.hpp"

class TimeServer
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher time_pub_;
        ros::Subscriber time_sub_;
        ros::ServiceServer timeclient_register_service_;

        int next_client_id_;

        std::map<int, std::unique_ptr<TimeClient>> clients_; // map ID and timeclient
        std::vector<std::unique_ptr<TimeClient>> clients_vector_;


        void cb_time(const std_msgs::Time::ConstPtr& msg);
        bool timeclient_register(sss_sim_env::ClientRegister::Request& req,
                                sss_sim_env::ClientRegister::Response& res);
    public:
        TimeServer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);


};


#endif