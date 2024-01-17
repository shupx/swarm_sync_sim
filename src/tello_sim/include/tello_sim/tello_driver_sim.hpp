/**
 * @file tello_driver_sim.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Tello driver ROS node sim
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2024-1-17
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
// #include <tello_sim/TelloStatus.h>
#include <algorithm>

#include "tello_sim/quadrotor_dynamics.hpp"


namespace TelloQuadSimulator
{

class TelloDriverSim
{
    public:
        TelloDriverSim(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,  const std::shared_ptr<Dynamics> &dynamics);

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

        /* Set cmd[-1,1] to velocity (m/s) and angular velocity (m/s) scale */
        void SetVelCmdScale(const double& linear_scale, const double& angular_scale);

        /* Set frame id name*/
        void SetFrameId(const std::string& frame_id);

        /* Publish states to ROS topics */
        void PublishState();

        /* Get control inputs */
        Dynamics::Input GetControlInputs();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        std::shared_ptr<Dynamics> dynamics_;

        ros::Publisher imu_pub_;
        ros::Publisher status_pub_;
        ros::Publisher pose_pub_;

        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber takeoff_sub_;
        ros::Subscriber land_sub_;

        bool is_armed_;
        double linear_vel_scale_; // scale from [-1,1] cmd to velocity (m/s)
        double angular_vel_scale_; // scale from [-1,1] cmd to angular elocity (rad/s)

        double init_yaw_; // initial yaw for imu initialization
        Eigen::Quaterniond imu_quat_offset_; // imu use the initial heading as zero yaw
        bool use_imu_orientation_; // If true, use imu orientation in /pose topic. If false, use mocap orientation

        std::string frame_id_;

        Eigen::Vector3d linear_vel_cmd_; // velocity commands (m/s) in world frame
        double yaw_rate_cmd_; // yaw rate commands (rad/s) in world frame

        double _limit(const double& x) {return std::max(std::min(x, 1.0), -1.0);}

        void cb_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg);
        void cb_takeoff(const std_msgs::Empty::ConstPtr& msg);
        void cb_land(const std_msgs::Empty::ConstPtr& msg);

};

}