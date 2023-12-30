/**
 * @file drone_visualizer.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Publish rotor propeller joint position and base_link tf states for the robot model visualization in rviz
 * 
 * Note: This program relies on mavros, px4 geo.h
 * 
 * @version 1.0
 * @date 2023-12-29
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */


#include "mavros_px4_quadrotor_sim/drone_visualizer.hpp"

namespace MavrosQuadSimulator
{

Visualizer::Visualizer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : 
    nh_(nh), 
    nh_private_(nh_private),
    armed_(false)
{
    nh_private_.param<float>("visualize_max_freq", max_freq_, 0);
    nh_private_.param<float>("visualize_path_time", history_path_time_, 5.0);
    nh_private_.param<std::string>("visualize_tf_frame", tf_frame_, "map");
    nh_private_.param<std::string>("base_link_name", tf_child_frame_, "base_link");
    nh_private_.param<std::string>("rotor_0_joint_name", rotor_joints_name_[0], "rotor_0_joint");
    nh_private_.param<std::string>("rotor_1_joint_name", rotor_joints_name_[1], "rotor_1_joint");
    nh_private_.param<std::string>("rotor_2_joint_name", rotor_joints_name_[2], "rotor_2_joint");
    nh_private_.param<std::string>("rotor_3_joint_name", rotor_joints_name_[3], "rotor_3_joint");

    int source;
    nh_private_.param<int>("local_pos_source", source, 0);
    local_pos_source_ = (enum position_mode)source;
    
    nh_private_.param<double>("world_origin_latitude_deg", world_origin_lat_, 39.978861);
    nh_private_.param<double>("world_origin_longitude_deg", world_origin_lon_, 116.339803);
    nh_private_.param<float>("world_origin_AMSL_alt_metre", world_origin_asml_alt_, 53.0);

    mavros_state_sub_ = nh_.subscribe("mavros/state", 1, &Visualizer::cb_mavros_state, this);
    mavros_local_pose_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &Visualizer::cb_mavros_local_pose, this);
    mavros_global_pose_sub_ = nh_.subscribe("mavros/global_position/global", 1, &Visualizer::cb_mavros_global_pose, this);

    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("history_path", 1);

    quat_.w = 1.0;
    quat_.x = 0.0;
    quat_.y = 0.0;
    quat_.z = 0.0;
}

void Visualizer::PublishRotorJointState()
{
    static double last_time = 0.0;
    double time_now = ros::Time::now().toSec();
    float rotor_joint_update_freq = 10.0; // 10Hz fixed
    if (time_now - last_time > 1.0 / rotor_joint_update_freq)
    {
        double dt = time_now - last_time;

        static float joint_pos_[4] = {0.0, 0.5, 2.6, 1.4};
        float RPM = 1000; // revolutions per minute
        float omega = RPM * 2 * M_PI / 60; // rad/s
        if (!armed_) {omega = 0.0;}

        sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState);
        msg->header.stamp = ros::Time::now();
        int joint_num = sizeof(rotor_joints_name_) / sizeof(rotor_joints_name_[0]);
        msg->name.resize(joint_num);
        msg->position.resize(joint_num);
        for (int i=0; i<joint_num; ++i)
        {
            joint_pos_[i] = WrapToPi1(joint_pos_[i] + omega * dt);
            msg->name[i] = rotor_joints_name_[i];
            msg->position[i] = joint_pos_[i];
        }
        joint_pub_.publish(msg);

        last_time = time_now;
    }
}

void Visualizer::PublishBaseLinkTF()
{
    static double last_time = 0.0;
    double time_now = ros::Time::now().toSec();
    if (time_now - last_time > 1.0 / max_freq_)
    {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.frame_id = tf_frame_;
        odom_trans.child_frame_id = tf_child_frame_;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = pos_x_;
        odom_trans.transform.translation.y = pos_y_;
        odom_trans.transform.translation.z = pos_z_;
        odom_trans.transform.rotation = quat_;
        tf2_broadcaster.sendTransform(odom_trans);

        last_time = time_now;
    }
}

void Visualizer::PublishPath()
{
    static double last_time = 0.0;
    double time_now = ros::Time::now().toSec();
    if (time_now - last_time > 1.0 / max_freq_)
    {
        geometry_msgs::PoseStamped TrajPose_;
        TrajPose_.header.stamp = ros::Time::now();
        TrajPose_.header.frame_id = tf_frame_;
        TrajPose_.pose.position.x = pos_x_;
        TrajPose_.pose.position.y = pos_y_;
        TrajPose_.pose.position.z = pos_z_;          
        TrajPose_.pose.orientation = quat_;

        static std::vector<geometry_msgs::PoseStamped> TrajPoseHistory_vector;
        TrajPoseHistory_vector.insert(TrajPoseHistory_vector.begin(), TrajPose_);
        if (TrajPoseHistory_vector.size() > history_path_time_ * max_freq_)
        {
            TrajPoseHistory_vector.pop_back();
        }

        nav_msgs::Path::Ptr path_msg(new nav_msgs::Path);
        path_msg->header.stamp = ros::Time::now();
        path_msg->header.frame_id = tf_frame_;
        path_msg->poses = TrajPoseHistory_vector;
        path_pub_.publish(path_msg);

        last_time = time_now;
    }
}

void Visualizer::cb_mavros_state(const mavros_msgs::State::ConstPtr& msg)
{
    armed_ = msg->armed;
}

void Visualizer::cb_mavros_local_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    switch (local_pos_source_)
    {
        case position_mode::MOCAP:
        {
            pos_x_ = msg->pose.position.x;
            pos_y_ = msg->pose.position.y;
            pos_z_ = msg->pose.position.z;
            quat_ = msg->pose.orientation;
            break;
        }
        case position_mode::GPS:
        {
            pos_z_ = msg->pose.position.z;
            quat_ = msg->pose.orientation;            
            break;
        }
        default:
        {
            ROS_ERROR("[Visualizer] invalid local_pos_source.");
            break;
        }
    }
}

void Visualizer::cb_mavros_global_pose(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    switch (local_pos_source_)
    {
        case position_mode::MOCAP:
            break;
        case position_mode::GPS:
        {
            double lat = msg->latitude;
            double lon = msg->longitude;
            MapProjection global_local_proj_ref{world_origin_lat_, world_origin_lon_, 0}; // MapProjection from PX4 geo.h
            // calculate pos x,y based on the lat/lon and world origin ref lat/lon
            global_local_proj_ref.project(lat, lon, pos_y_, pos_x_); // pos_x_: East; pos_y_: North
            break;
        }
        default:
        {
            ROS_ERROR("[Visualizer] invalid local_pos_source.");
            break;
        }
    }
}



}
