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
#include "sss_sim_env/ClientUnregister.h"

class TimeServer
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher sim_time_pub_;
        ros::ServiceServer timeclient_register_service_;
        ros::ServiceServer timeclient_unregister_service_;

        int next_client_id_;

        class TimeClient{
            private:
                ros::NodeHandle nh_;
                ros::NodeHandle nh_private_;
                ros::Subscriber update_time_request_sub_;
                TimeServer* time_server;
                void cb_update_time_request(const std_msgs::Time::ConstPtr& msg);
            public:
                int client_id_;
                bool has_new_request;
                ros::Time request_time;    
                TimeClient(const int &id, TimeServer *obj, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
                // TimeClient(const int &id, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        };

        std::vector<std::unique_ptr<TimeClient>> clients_vector_;

        bool timeclient_register(sss_sim_env::ClientRegister::Request& req,
                                sss_sim_env::ClientRegister::Response& res);
        bool timeclient_unregister(sss_sim_env::ClientUnregister::Request& req,
                                sss_sim_env::ClientUnregister::Response& res);
    
    public:
        TimeServer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        void update_time(int client_id);
        ros::Time sim_time;
};


#endif