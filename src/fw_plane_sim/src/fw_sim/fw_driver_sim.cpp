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
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("vel", 10, false);
    angle_pub_ = nh_.advertise<geometry_msgs::Vector3>("angle", 10, false);
    ias_pub_ = nh_.advertise<std_msgs::Float32>("ias", 10, false);
    pos_pub_ = nh_.advertise<geometry_msgs::Vector3>("pos", 10, false);
    baro_pub_ = nh_.advertise<std_msgs::Float32>("baro", 10, false);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1000, &FwDriverSim::cb_cmd_vel, this);

    /* for visualization */
    local_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
                                        "mavros/local_position/pose", 5);
    global_pos_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(
                                        "mavros/global_position/global", 5);
    mavros_state_pub_ = nh_.advertise<mavros_msgs::State>(
                                        "mavros/state", 5);

    nh_private_.param<double>("world_origin_latitude_deg", world_origin_lat_, 39.978861);
    nh_private_.param<double>("world_origin_longitude_deg", world_origin_lon_, 116.339803);
    nh_private_.param<float>("world_origin_AMSL_alt_metre", world_origin_asml_alt_, 53.0);
    nh_private_.param<bool>("enable_mavros_topic", enable_mavros_topic_, true);

    bool use_sss_sim_time;
    nh_.param<bool>("/use_sss_sim_time", use_sss_sim_time, false);
    if (!use_sss_sim_time)
    {
        ROS_WARN("[FwDriverSim] /use_sss_sim_time is false, the multiple UAVs "
                    "may not be synchronized. "
                    "It is recommended to set /use_sss_sim_time and /use_sim_time "
                    "to true in the launch file.");
    }

    /* Init */
    is_armed_ = false;
    stage_ = 4; // 4 for offboard

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

    /* get states (NED/FRD -> ENU/FLU)*/
    double pos_x = dynamic_state_.posiNState; // north (m)
    double pos_y = dynamic_state_.posiEState; // east (m)
    double pos_z = dynamic_state_.posiDState; // down (m)
    double angle_x = dynamic_state_.rollState; // roll (Forward)(rad)
    double angle_y = dynamic_state_.pitchState; // pitch (Right)(rad)
    double angle_z = dynamic_state_.yawState; // yaw (Down)(rad)
    // double vel_alpha = dynamic_state_.velalphaState; // 速度倾角，输出暂为0
    // double vel_beta = dynamic_state_.velbeteState; // 速度偏角，输出暂为0
    // double angle_alpha = dynamic_state_.alphaState; // 攻角，输出暂为0
    // double angle_beta = dynamic_state_.beteState; // 侧滑角，输出暂为0
    double vel_ground = dynamic_state_.velState;
    /* TODO: vel is corresponding to the vel_alpha and vel_beta, not body attitude */
    double vel_x = vel_ground * cos(-angle_y) * cos(angle_z); // north (m)
    double vel_y = vel_ground * cos(-angle_y) * sin(angle_z); // east (m)
    double vel_z = vel_ground * sin(-angle_y); // down (m)

    /* publish stage */
    std_msgs::Int32 stage_msg;
    stage_msg.data = stage_; // 4 for offboard
    stage_pub_.publish(stage_msg);

    /* Calculate lat/lon according to x(North), y(East) and origin lat/lon */
    double lat, lon;
    MapProjection global_local_proj_ref{world_origin_lat_, world_origin_lon_, 0}; // MapProjection from PX4 geo.h
    global_local_proj_ref.reproject(pos_x, pos_y, lat, lon);
    /* publish nav */
    sensor_msgs::NavSatFix nav_msg;
    nav_msg.header.stamp = time_cb;
    // nav_msg.header.frame_id = frame_id_;
    nav_msg.latitude = lat;
    nav_msg.longitude = lon;
    nav_msg.altitude = -pos_z; //TODO: check the altitude
    nav_pub_.publish(nav_msg);

    /* publish velocity */
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = vel_x; // north (m/s)
    vel_msg.linear.y = vel_y; // east (m/s)
    vel_msg.linear.z = vel_z; // down (m/s)
    vel_pub_.publish(vel_msg);

    /* publish IAS */
    std_msgs::Float32 ias_msg;
    ias_msg.data = vel_ground; // TODO: check the ground speed
    ias_pub_.publish(ias_msg);

    /* publish pos */
    geometry_msgs::Vector3 pos_msg;
    pos_msg.x = pos_x; // north (m)
    pos_msg.y = pos_y; // east (m)
    pos_msg.z = pos_z; // down (m)
    pos_pub_.publish(pos_msg);

    /* publish baro */
    std_msgs::Float32 baro_msg;
    baro_msg.data = -pos_z; // TODO: check the altitude
    baro_pub_.publish(baro_msg);

    /* publish angle */
    geometry_msgs::Vector3 angle_msg;
    // (NED/FRD -> ENU/FLU)
    angle_msg.x = angle_x * 180 / M_PI; // roll (Forward) (deg)
    angle_msg.y = angle_y * 180 / M_PI; // pitch (Right) (deg)
    angle_msg.z = angle_z * 180 / M_PI; // yaw (Down) (deg)
    angle_pub_.publish(angle_msg);

    /* publish mavros topics for visualization */
    if (enable_mavros_topic_)
    {
        geometry_msgs::PoseStamped local_pose;
        local_pose.header.stamp = time_cb;
        local_pose.header.frame_id = "map";
        local_pose.pose.position.x = pos_y; // East (m)
        local_pose.pose.position.y = pos_x; // North (m)
        local_pose.pose.position.z = -pos_z; // Up (m)
        tf2::Quaternion quaternion;
        double roll_f_rad = angle_x;
        double pitch_l_rad = -angle_y;
        double yaw_u_rad = -angle_z + M_PI_2;
        quaternion.setRPY(roll_f_rad, pitch_l_rad, yaw_u_rad);
        local_pose.pose.orientation = tf2::toMsg(quaternion);
        local_pose_pub_.publish(local_pose);

        sensor_msgs::NavSatFix global_pos;
        global_pos.header.stamp = time_cb;
        global_pos.header.frame_id = "map";
        global_pos.latitude = lat;
        global_pos.longitude = lon;
        global_pos.altitude = -pos_z; // Up (m)
        global_pos_pub_.publish(global_pos);

        mavros_msgs::State mavros_state;
        mavros_state.header.stamp = time_cb;
        mavros_state.armed = is_armed_;
        mavros_state_pub_.publish(mavros_state);
    }
};

void FwDriverSim::cb_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    int stage_cmd = int(msg->angular.y);
    if (stage_cmd == 1) // setpoints
    {
        is_armed_ = true;
        input_.Vel_input = msg->linear.x; // m/s
        input_.High_input = msg->linear.y; // m
        input_.Roll_input = msg->linear.z; // deg
        // Pitch_input = msg->angular.x; // deg
        // input_.Yaw_input = 0.0; // deg
    }
    else if (stage_cmd == 2) // 航线
    {
        stage_ = 6; // 航线
    }
    else if (stage_cmd == 3) // 目标盘旋
    {
        stage_ = 8; // 航点
    }
    else if (stage_cmd == 4) // 返回盘旋
    {
        stage_ = 6; // 航线
    }
    else if (stage_cmd == 5) // 就地盘旋
    {
        stage_ = 3; // 盘旋
    }
    else if (stage_cmd == 1999) // 进入编队模式（只用于仿真）
    {
        stage_ = 4; // 盘旋
    }
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
