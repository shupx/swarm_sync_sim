/**
 * @file fw_driver_sim.cpp
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

#include "fw_plane_sim/fw_sim/fw_driver_sim.hpp"

namespace FwSimulator
{

FwDriverSim::FwDriverSim(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    // using namespace Eigen;

    stage_pub_ = nh_.advertise<std_msgs::Int32>("stage", 10, false);
    nav_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("nav", 10, false);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("nav", 10, false);
    angle_pub_ = nh_.advertise<geometry_msgs::Vector3>("angle", 10, false);
    ias_pub_ = nh_.advertise<std_msgs::Float32>("ias", 10, false);
    pos_pub_ = nh_.advertise<geometry_msgs::Vector3>("pos", 10, false);
    baro_pub_ = nh_.advertise<std_msgs::Float32>("baro", 10, false);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1000, &FwDriverSim::cb_cmd_vel, this);

    nh_private_.param<double>("world_origin_latitude_deg", world_origin_lat_, 39.978861);
    nh_private_.param<double>("world_origin_longitude_deg", world_origin_lon_, 116.339803);
    nh_private_.param<float>("world_origin_AMSL_alt_metre", world_origin_asml_alt_, 53.0);

    // ROS_WARN("[FwDriverSim] world_origin_latitude_deg %f", world_origin_lat_);
    // ROS_WARN("[FwDriverSim] world_origin_longitude_deg %f", world_origin_lon_);

    /* Init */
    is_armed_ = false;
    stage_ =  4; // 4 for offboard

    input_.High_input = 0.0;
    input_.Vel_input = 0.0;
    input_.Roll_input = 0.0;
    input_.Yaw_input = 0.0;
}

void FwDriverSim::UpdateDynamicState(State_Output state)
{
    dynamic_state_ = state;
}

void FwDriverSim::PublishState()
{
    ros::Time time_cb = ros::Time::now();

    /* get states */
    double pos_x = dynamic_state_.posiEState;
    double pos_y = dynamic_state_.posiNState;
    double pos_z = -dynamic_state_.posiDState;
    // double vel_alpha = dynamic_state_.velalphaState; // 速度倾角，输出暂为0
    // double vel_beta = dynamic_state_.velbeteState; // 速度偏角，输出暂为0
    double vel_ground = dynamic_state_.velState;
    double vel_x = dynamic_state_.velState * cos(dynamic_state_.pitchState) * cos(dynamic_state_.yawState);
    double vel_y = dynamic_state_.velState * cos(dynamic_state_.pitchState) * sin(dynamic_state_.yawState);
    double vel_z = dynamic_state_.velState * sin(dynamic_state_.pitchState);
    double angle_x = dynamic_state_.rollState;
    double angle_y = dynamic_state_.pitchState;
    double angle_z = dynamic_state_.yawState;
    // double angle_alpha = dynamic_state_.alphaState; // 攻角，输出暂为0
    // double angle_beta = dynamic_state_.beteState; // 侧滑角，输出暂为0

    /* publish stage */
    std_msgs::Int32 stage_msg;
    stage_msg.data = stage_; // 4 for offboard
    stage_pub_.publish(stage_msg);

    /* Calculate lat/lon according to x(North), y(East) and origin lat/lon */
    double lat, lon;
    MapProjection global_local_proj_ref{world_origin_lat_, world_origin_lon_, 0}; // MapProjection from PX4 geo.h
    global_local_proj_ref.reproject(pos_y, pos_x, lat, lon);
    /* publish nav */
    sensor_msgs::NavSatFix nav_msg;
    nav_msg.header.stamp = time_cb;
    // nav_msg.header.frame_id = frame_id_;
    nav_msg.latitude = lat;
    nav_msg.longitude = lon;
    nav_msg.altitude = pos_z; //TODO: check the altitude
    nav_pub_.publish(nav_msg);

    /* publish velocity */
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = vel_x;
    vel_msg.linear.y = vel_y;
    vel_msg.linear.z = vel_z;
    vel_pub_.publish(vel_msg);

    /* publish IAS */
    std_msgs::Float32 ias_msg;
    ias_msg.data = vel_ground; // TODO: check the ground speed
    ias_pub_.publish(ias_msg);

    /* publish pos */
    geometry_msgs::Vector3 pos_msg;
    pos_msg.x = pos_x;
    pos_msg.y = pos_y;
    pos_msg.z = pos_z;
    pos_pub_.publish(pos_msg);

    /* publish baro */
    std_msgs::Float32 baro_msg;
    baro_msg.data = pos_z; // TODO: check the altitude
    baro_pub_.publish(baro_msg);

    /* publish angle */
    geometry_msgs::Vector3 angle_msg;
    angle_msg.x = angle_x;
    angle_msg.y = angle_y;
    angle_msg.z = angle_z;
    angle_pub_.publish(angle_msg);

};

void FwDriverSim::cb_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    is_armed_ = true;
    input_.Vel_input = msg->linear.x;
    input_.High_input = msg->linear.y;
    input_.Roll_input = msg->linear.z;
    // Pitch_input = msg->angular.x;
    // input_.Yaw_input = 0.0;
    
    stage_ = int(msg->angular.y);
}

Input FwDriverSim::GetControlInputs()
{
    return input_;
}

bool FwDriverSim::GetArmed()
{ 
    return is_armed_; 
}

int FwDriverSim::GetStage()
{
    return stage_;
}

} // namespace FwSimulator
