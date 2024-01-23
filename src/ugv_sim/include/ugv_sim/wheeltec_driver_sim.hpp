/**
 * @file wheeltec_driver_sim.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Wheeltec UGV driver ROS node sim
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2024-1-23
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */


#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <algorithm>

#include "ugv_sim/ugv_dynamics.hpp"

using namespace UgvSimulator;

namespace WheeltecUgvSimulator
{

/* UGV type */
enum class UgvType : uint32_t 
{
    Mecanum,
    Unicycle,
    Ackermann
};

class WheeltecDriverSim
{
    public:
        /**
         * @brief Wheeltec ROS driver sim
         * @param ugv_type UgvType::Mecanum/Unicycle/Ackermann
         */
        WheeltecDriverSim(const UgvType& ugv_type, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, const std::shared_ptr<Dynamics> &dynamics);

        /**
         * @brief Set the imu zero yaw.
         * @param yaw yaw angle (rad)
         */ 
        void SetImuYawBase(const double& yaw);

        /**
         * @brief Decide the orientaion source of the /pose topic
         * @param enable true to use imu, false to not
         */ 
        void UseImuOrientation(const bool& enable);

        /* Set cmd_vel to real velocity (m/s) and angular velocity (m/s) scale */
        void SetVelCmdScale(const double& linear_scale, const double& angular_scale);

        /**
         * @brief Set linear and yaw rate limit (m/s) (rad/s)
         */
        void SetVelLimit(const double& v_front_limit, const double& v_left_limit, const double& omega_limit);

        /**
         * @brief Set minimum turning radius (meter) for ackermann ugv
         */
        void SetMinTurnRadius(const double& min_turning_radius);

        /* Publish states to ROS topics */
        void PublishState();

        /* Get control inputs */
        Dynamics::Input GetControlInputs();


    private:
        UgvType ugv_type_;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        std::shared_ptr<Dynamics> dynamics_;

        ros::Publisher imu_pub_;
        ros::Publisher pose_pub_;
        ros::Publisher twist_pub_; // velocity

        ros::Subscriber cmd_vel_sub_;

        double linear_vel_scale_; // scale from cmd to velocity (m/s)
        double angular_vel_scale_; // scale from cmd to angular elocity (rad/s)

        double imu_yaw_base_; //imu zero yaw (Physical East Maybe?) ->  world yaw (Mocap/UWB X axis) 
        Eigen::Quaterniond imu_quat_offset_; // imu use the initial heading as zero yaw
        bool use_imu_orientation_; // If true, use imu orientation in /pose topic. If false, use mocap orientation

        double cmd_v_front_body_; // cmd_vel front velocity (m/s)
        double cmd_v_left_body_; // cmd_vel left velocity (m/s)
        double cmd_w_up_body_; // cmd_vel omega velocity (rad/s)

        double v_front_limit_;
        double v_left_limit_;
        double omega_limit_;
        double min_turning_radius_;

        /* Limit x to [-abs(limit_value), abs(limit_value)] */
        double _limit(const double& x, const double& limit_value) {return std::max(std::min(x, std::abs(limit_value)), -std::abs(limit_value));}

        /**
         * @brief cmd_vel command
         * msg.linear.x (front) m/s
         * msg.linear.y (left for mecanum wheel) m/s
         * msg.linear.z (unused)
         * msg.angular.x (unused)
         * msg.angular.y (unused)
         * msg.angular.z (turn left) rad/s
         */
        void cb_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg);

};

}