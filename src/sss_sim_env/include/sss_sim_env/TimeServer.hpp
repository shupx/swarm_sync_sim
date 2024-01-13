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
#include <ros/callback_queue.h>
// #include <std_msgs/Time.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Bool.h>
#include "sss_sim_env/TimeRequest.h"
#include "sss_sim_env/ClientRegister.h"
#include "sss_sim_env/ClientUnregister.h"
#include "sss_sim_env/SimClockControl.h"

class TimeServer
{
    private:
        bool is_sim_time_;
        bool is_paused_;
        bool auto_start_;
        ros::Time init_time_;
        ros::Time speed_regulator_expected_time_;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher sim_clock_pub_;
        ros::Publisher online_pub_;
        ros::Subscriber update_clock_request_sub_;
        ros::ServiceServer clock_control_service_;
        ros::ServiceServer timeclient_register_service_;
        ros::ServiceServer timeclient_unregister_service_;

        ros::CallbackQueue async_callback_queue_;
        ros::NodeHandle nh_async_;
        ros::AsyncSpinner async_spinner_{1, &async_callback_queue_};

        ros::CallbackQueue speed_regulator_callback_queue_;
        ros::NodeHandle nh_speed_regulator_;
        ros::AsyncSpinner speed_regulator_spinner_{1, &speed_regulator_callback_queue_};
        ros::WallTimer speed_regulator_timer_;

        int next_client_id_;
        double speed_regulator_period_;
        float max_speed_ratio_;

        // std::recursive_mutex try_update_clock_mutex_;
        // std::recursive_mutex sim_time_mutex_;

        class TimeClient
        {
            private:
                ros::NodeHandle nh_;
                ros::NodeHandle nh_private_;

                // ros::NodeHandle nh_async;
                // ros::CallbackQueue callback_queue_;
                // ros::AsyncSpinner async_spinner_{1, &callback_queue_};
                // ros::Subscriber update_clock_request_sub_;

                TimeServer* time_server;
                void cb_update_clock_request(const rosgraph_msgs::Clock::ConstPtr& msg);
            public:
                int client_id_;
                bool has_new_request;
                ros::Time request_time;    
                std::recursive_mutex client_mutex_;
                TimeClient(const int &id, TimeServer *obj, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        };

        typedef std::shared_ptr<TimeClient> TimeClientPtr;
        std::vector<TimeClientPtr> clients_vector_;

        bool cb_clock_control(sss_sim_env::SimClockControl::Request& req,
                                sss_sim_env::SimClockControl::Response& res);        
        bool cb_timeclient_register(sss_sim_env::ClientRegister::Request& req,
                                sss_sim_env::ClientRegister::Response& res);
        bool cb_timeclient_unregister(sss_sim_env::ClientUnregister::Request& req,
                                sss_sim_env::ClientUnregister::Response& res);
        
        void cb_speed_regulator_timer(const ros::WallTimerEvent &event);

        void cb_update_clock_request(const sss_sim_env::TimeRequest::ConstPtr& msg);
    
    public:
        TimeServer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        bool try_update_clock();
        ros::Time sim_time_;
};


#endif