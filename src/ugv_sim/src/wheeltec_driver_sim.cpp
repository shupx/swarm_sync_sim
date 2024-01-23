/**
 * @file wheeltec_driver_sim.cpp
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
#include "ugv_sim/wheeltec_driver_sim.hpp"

namespace WheeltecUgvSimulator
{

WheeltecDriverSim::WheeltecDriverSim(const UgvType& ugv_type, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, const std::shared_ptr<Dynamics> &dynamics)
    : ugv_type_(ugv_type), nh_(nh), nh_private_(nh_private), dynamics_(dynamics)
{
    using namespace Eigen;

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10, false);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10, false);
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("twist", 10, false); 

    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1000, &WheeltecDriverSim::cb_cmd_vel, this);

    /* Init */
    linear_vel_scale_ = 1.0;
    angular_vel_scale_ = 1.0;
    v_front_limit_ = 3.0; // m/s
    v_left_limit_ = 3.0; // m/s
    omega_limit_ = M_PI; // rad/s
    min_turning_radius_ = 0.1; // m

    /* imu yaw is aligned with world East and not aligned with mocap/UWB X axis */
    // imu_yaw_base_ = dynamics_->getRPY()[2];
    imu_yaw_base_ = M_PI_4; // pi/4 is randomly chosen in order to distinguish imu zero yaw from world X
    imu_quat_offset_ = AngleAxisd{imu_yaw_base_, Vector3d::UnitZ()};
    use_imu_orientation_ = false;

    cmd_v_front_body_ = 0.0;
    cmd_v_left_body_ = 0.0;
    cmd_w_up_body_ = 0.0;
}

void WheeltecDriverSim::PublishState()
{
    Eigen::Quaterniond quat = dynamics_->getQuat();
    Eigen::Quaterniond quat_imu = imu_quat_offset_.conjugate() * quat; // imu use the World east? as zero yaw
    Eigen::Vector3d angular_vel = dynamics_->getAngVel();
    Eigen::Vector3d linear_vel = dynamics_->getLinearVel();
    Eigen::Vector3d pos = dynamics_->getState().pos;
    ros::Time time_cb = ros::Time::now();

    sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);
    imu_msg->header.stamp = time_cb;
    imu_msg->header.frame_id = "world";
    imu_msg->orientation.w = quat_imu.w();
    imu_msg->orientation.x = quat_imu.x();
    imu_msg->orientation.y = quat_imu.y();
    imu_msg->orientation.z = quat_imu.z();
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

    geometry_msgs::TwistStamped::Ptr twist_msg(new geometry_msgs::TwistStamped);
    twist_msg->header.stamp = time_cb;
    twist_msg->header.frame_id = "map";
    twist_msg->twist.linear.x = linear_vel[0];
    twist_msg->twist.linear.y = linear_vel[1];
    twist_msg->twist.linear.z = linear_vel[2];
    twist_msg->twist.angular.x = angular_vel[0];
    twist_msg->twist.angular.y = angular_vel[1];
    twist_msg->twist.angular.z = angular_vel[2];
    twist_pub_.publish(twist_msg);
};

Dynamics::Input WheeltecDriverSim::GetControlInputs()
{

    double yaw = dynamics_->getRPY()[2]; // get real yaw angle relative to the world frame

    Dynamics::Input input;
    input.linear_vel[0] = cmd_v_front_body_ * cos(yaw) - cmd_v_left_body_ * sin(yaw); // world x
    input.linear_vel[1] = cmd_v_front_body_ * sin(yaw) + cmd_v_left_body_ * cos(yaw); // world y
    input.linear_vel[2] = 0.0; // world z
    input.yaw_rate = cmd_w_up_body_;  // in dynamics positive yaw rate is up

    // std::cout << "Input: \n" << input.linear_vel << ", " << input.yaw_rate << std::endl;

    return input;
}

void WheeltecDriverSim::SetImuYawBase(const double& yaw)
{
    imu_yaw_base_ = yaw;
    imu_quat_offset_ = Eigen::AngleAxisd{imu_yaw_base_, Eigen::Vector3d::UnitZ()};
}

void WheeltecDriverSim::SetVelCmdScale(const double& linear_scale, const double& angular_scale)
{
    linear_vel_scale_ = linear_scale;
    angular_vel_scale_ = angular_scale;
}

void WheeltecDriverSim::UseImuOrientation(const bool& enable)
{
    use_imu_orientation_ = enable;
}

void WheeltecDriverSim::SetVelLimit(const double& v_front_limit, const double& v_left_limit, const double& omega_limit)
{
    v_front_limit_ = v_front_limit;
    v_left_limit_ = v_left_limit;
    omega_limit_ = omega_limit;
}

void WheeltecDriverSim::SetMinTurnRadius(const double& min_turning_radius)
{
    min_turning_radius_ = min_turning_radius;
}

void WheeltecDriverSim::cb_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_v_front_body_ = linear_vel_scale_ * _limit(msg->linear.x, v_front_limit_); // body front

    if (ugv_type_ == UgvType::Mecanum)
    {
        cmd_v_left_body_ = linear_vel_scale_ * _limit(msg->linear.y, v_left_limit_); // body left
    }
    else
    {
        cmd_v_left_body_ = 0.0;
    }

    if (ugv_type_ == UgvType::Ackermann)
    {
        cmd_w_up_body_ = angular_vel_scale_ * _limit(msg->angular.z, cmd_v_front_body_ / min_turning_radius_); // body up (turn left)
    }
    else
    {
        cmd_w_up_body_ = angular_vel_scale_ * _limit(msg->angular.z, omega_limit_); // body up (turn left)
    }
}

}
