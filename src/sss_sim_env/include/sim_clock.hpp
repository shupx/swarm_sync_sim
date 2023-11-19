/**
 * @file sim_clock.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Header files of simulation clock. Determine the block of threads of each robot.
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

#ifndef __SIM_CLOCK__
#define __SIM_CLOCK__
#include "ros/ros.h"
#include <std_msgs/Time.h>

class TimeClient{
    private:
        int client_id_;
        bool has_new_request_;
        ros::Time request_time_;     
    public:
        TimeClient(const int &id);
};


class TimeServer{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher time_pub_;
        ros::Subscriber time_sub_;

        int next_client_id_;

        void cb_time(const std_msgs::Time::ConstPtr& msg);
    public:
        TimeServer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

};



#endif