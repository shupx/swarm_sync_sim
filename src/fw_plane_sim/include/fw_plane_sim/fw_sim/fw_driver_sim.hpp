/**
 * @file fw_driver_sim.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Fixed wing flight controller driver ROS node sim
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2024-10-1
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */


#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/State.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "BHDynamic.h"
#include <geo/geo.h> // for px4 gps->NED utils

namespace FwSimulator
{

typedef struct
{
    double High_input; // m
    double Vel_input; // m/s
    double Roll_input; // deg
    double Pitch_input; // deg
    double Yaw_input; // deg
} Input; 

class FwDriverSim
{
    public:
        FwDriverSim(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

        /* Publish states to ROS topics */
        void PublishState();

        void UpdateDynamicState(State_Output state);

        /* Get control inputs */
        Input GetControlInputs();

        /* Get arming state */
        bool GetArmed();

        /* Get stage_ */
        int GetStage();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher stage_pub_;
        ros::Publisher nav_pub_;
        ros::Publisher vel_pub_;
        ros::Publisher angle_pub_;
        ros::Publisher ias_pub_;
        ros::Publisher pos_pub_;
        ros::Publisher baro_pub_;
        ros::Publisher local_pose_pub_;
        ros::Publisher global_pos_pub_;
        ros::Publisher mavros_state_pub_;

        ros::Subscriber cmd_vel_sub_;

        State_Output dynamic_state_;

        bool is_armed_;
        int stage_; 
        std::string frame_id_; // frame id name

        double world_origin_lat_;
        double world_origin_lon_;
        float world_origin_asml_alt_;
        bool enable_mavros_topic_;

        Input input_;

        void cb_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg);
};

}