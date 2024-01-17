
/**
 * @file tello_driver_sim.cpp
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

#include "tello_sim/tello_driver_sim.hpp"

namespace TelloQuadSimulator
{

TelloDriverSim::TelloDriverSim(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, const std::shared_ptr<Dynamics> &dynamics)
    : nh_(nh), nh_private_(nh_private), dynamics_(dynamics)
{
    using namespace Eigen;

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10, false);
    // status_pub_ = nh_.advertise<tello_sim::TelloStatus>("status", 10, false);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10, false);

    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1000, &TelloDriverSim::cb_cmd_vel, this);
    takeoff_sub_ = nh_.subscribe("takeoff", 1000, &TelloDriverSim::cb_takeoff, this);
    land_sub_ = nh_.subscribe("land", 1000, &TelloDriverSim::cb_land, this);

    /* Init */
    is_armed_ = false;
    linear_vel_scale_ = 2.0;
    angular_vel_scale_ = 3.14;

    init_yaw_ = dynamics_->getRPY()[2];
    imu_quat_offset_ = AngleAxisd{init_yaw_, Vector3d::UnitZ()};
    use_imu_orientation_ = false;

    linear_vel_cmd_ = Vector3d::Zero();
    yaw_rate_cmd_ = 0.0;
}

void TelloDriverSim::PublishState()
{
    Eigen::Quaterniond quat = dynamics_->getQuat();
    Eigen::Quaterniond quat_imu = imu_quat_offset_.conjugate() * quat; // imu use the initial heading as zero yaw
    Eigen::Vector3d angular_vel = dynamics_->getAngVel();
    Eigen::Vector3d pos = dynamics_->getState().pos;
    ros::Time time_cb = ros::Time::now();

    sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);
    imu_msg->header.stamp = time_cb;
    imu_msg->header.frame_id = frame_id_ + "/init_map";
    imu_msg->orientation.w = quat_imu.w();
    imu_msg->orientation.x = quat_imu.x();
    imu_msg->orientation.y = quat_imu.y();
    imu_msg->orientation.z = quat_imu.z(); //@TODO relative to the initial heading.
    imu_msg->angular_velocity.x = angular_vel[0];
    imu_msg->angular_velocity.y = angular_vel[1];
    imu_msg->angular_velocity.z = angular_vel[2];
    imu_pub_.publish(imu_msg);

    geometry_msgs::PoseStamped::Ptr pose_msg(new geometry_msgs::PoseStamped);
    pose_msg->header.stamp = time_cb;
    pose_msg->header.frame_id = "map";
    if (use_imu_orientation_)
    {
        pose_msg->pose.orientation.w = quat_imu.w();
        pose_msg->pose.orientation.x = quat_imu.x();
        pose_msg->pose.orientation.y = quat_imu.y();
        pose_msg->pose.orientation.z = quat_imu.z();
    }
    else
    {
        pose_msg->pose.orientation.w = quat.w();
        pose_msg->pose.orientation.x = quat.x();
        pose_msg->pose.orientation.y = quat.y();
        pose_msg->pose.orientation.z = quat.z();
    }
    pose_msg->pose.position.x = pos[0];
    pose_msg->pose.position.y = pos[1];
    pose_msg->pose.position.z = pos[2];
    pose_pub_.publish(pose_msg);
};

Dynamics::Input TelloDriverSim::GetControlInputs()
{
    Dynamics::Input input;
    if (is_armed_)
    {
        input.linear_vel = linear_vel_cmd_;
        input.yaw_rate = yaw_rate_cmd_;
    }
    else
    {
        input.linear_vel = Eigen::Vector3d::Zero();
        input.yaw_rate = 0.0;
    }

    // std::cout << "Input: \n" << input.linear_vel << ", " << input.yaw_rate << std::endl;

    return input;
}

void TelloDriverSim::SetImuYawBase(const double& yaw)
{
    init_yaw_ = yaw;
    imu_quat_offset_ = Eigen::AngleAxisd{init_yaw_, Eigen::Vector3d::UnitZ()};
}

void TelloDriverSim::SetVelCmdScale(const double& linear_scale, const double& angular_scale)
{
    linear_vel_scale_ = linear_scale;
    angular_vel_scale_ = angular_scale;
}

 void TelloDriverSim::SetFrameId(const std::string& frame_id)
 {
    frame_id_ = frame_id;
 }

void TelloDriverSim::UseImuOrientation(const bool& enable)
{
    use_imu_orientation_ = enable;
}

void TelloDriverSim::cb_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    // 0.5 is the default scale in tello_driver_node
    double v_right_body = linear_vel_scale_ * _limit(msg->linear.x * 0.5); // limit to [-1, 1] body right
    double v_front_body = linear_vel_scale_ * _limit(msg->linear.y * 0.5); // limit to [-1, 1] body front
    double v_z_body = linear_vel_scale_ * _limit(msg->linear.z * 0.5); // limit to [-1, 1] body up
    double w_down_body = angular_vel_scale_ * _limit(msg->angular.z * 0.5); // limit to [-1, 1] body down (turn right)

    double yaw = dynamics_->getRPY()[2] - M_PI/2; // get real yaw angle relative to the world frame

    linear_vel_cmd_[0] = v_right_body * cos(yaw) - v_front_body * sin(yaw); // world x
    linear_vel_cmd_[1] = v_right_body * sin(yaw) + v_front_body * cos(yaw); // world y
    linear_vel_cmd_[2] = v_z_body; // world z
    yaw_rate_cmd_ = -w_down_body;  // in dynamics positive yaw rate is up
}

void TelloDriverSim::cb_takeoff(const std_msgs::Empty::ConstPtr& msg)
{
    Eigen::Vector3d pos = dynamics_->getState().pos;
    if (!is_armed_ && pos[2] <= 0.0)
    {
        dynamics_->setPos(pos[0], pos[1], 1.0); // fly to 1.0 m
        is_armed_ = true;
    }
}

void TelloDriverSim::cb_land(const std_msgs::Empty::ConstPtr& msg)
{
    Eigen::Vector3d pos = dynamics_->getState().pos;
    if (is_armed_)
    {
        dynamics_->setPos(pos[0], pos[1], 0.0); // fly to 1.0 m
        is_armed_ = false;
    }
}

}
