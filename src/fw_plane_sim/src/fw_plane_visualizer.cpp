/**
 * @file fw_plane_visualizer.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Publish rotor propeller joint position and base_link tf states for the robot model visualization in rviz
 * 
 * Note: This program relies on mavros, px4 geo.h
 * 
 * @version 1.0
 * @date 2024-8-29
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */


#include "fw_plane_sim/fw_plane_visualizer.hpp"

namespace FwPlaneSimulator
{

Visualizer::Visualizer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : 
    nh_(nh), 
    nh_private_(nh_private),
    armed_(false),
    pose_valid_(false)
{
    nh_private_.param<float>("visualize_max_freq", max_freq_, 100);
    nh_private_.param<float>("visualize_path_time", history_path_time_, 5.0);
    nh_private_.param<std::string>("visualize_marker_name", marker_name_, "uav");
    nh_private_.param<std::string>("visualize_tf_frame", tf_frame_, "map");
    nh_private_.param<std::string>("base_link_name", tf_child_frame_, "base_link");
    nh_private_.param<std::string>("rotor_0_joint_name", rotor_joints_name_[0], "rotor_0_joint");
    // nh_private_.param<std::string>("rotor_1_joint_name", rotor_joints_name_[1], "rotor_1_joint");
    // nh_private_.param<std::string>("rotor_2_joint_name", rotor_joints_name_[2], "rotor_2_joint");
    // nh_private_.param<std::string>("rotor_3_joint_name", rotor_joints_name_[3], "rotor_3_joint");

    int source;
    nh_private_.param<int>("local_pos_source", source, 0);
    local_pos_source_ = (enum position_mode)source;

    std::string tf_prefix;
    if (nh_private_.getParam("base_link_tf_prefix", tf_prefix))
    {
        tf_child_frame_ = tf_prefix + "/" + tf_child_frame_;
    }

    nh_private_.param<bool>("enable_history_path", enable_history_path_, true);
    
    nh_private_.param<double>("world_origin_latitude_deg", world_origin_lat_, 39.978861);
    nh_private_.param<double>("world_origin_longitude_deg", world_origin_lon_, 116.339803);
    nh_private_.param<float>("world_origin_AMSL_alt_metre", world_origin_asml_alt_, 53.0);

    ROS_WARN("[visualizer] world_origin_latitude_deg %f", world_origin_lat_);
    ROS_WARN("[visualizer] world_origin_longitude_deg %f", world_origin_lon_);

    /* Setting tcpNoNelay tells the subscriber to ask publishers that connect
        to set TCP_NODELAY on their side. This prevents some state messages
        from being bundled together, increasing the latency of one of the messages. */
    ros::TransportHints transport_hints;
    transport_hints.tcpNoDelay(true);
    mavros_state_sub_ = nh_.subscribe("mavros/state", 1, &Visualizer::cb_mavros_state, this, transport_hints);
    mavros_local_pose_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &Visualizer::cb_mavros_local_pose, this, transport_hints);
    mavros_global_pose_sub_ = nh_.subscribe("mavros/global_position/global", 1, &Visualizer::cb_mavros_global_pose, this, transport_hints);

    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>("history_path", 1, true);
    marker_name_pub_ = nh_.advertise<visualization_msgs::Marker>("marker_name", 1, true);

    quat_.w = 1.0;
    quat_.x = 0.0;
    quat_.y = 0.0;
    quat_.z = 0.0;
}

void Visualizer::Run()
{
    PublishRotorJointState();
    PublishBaseLinkTF();
    PublishMarkerName();
    if (pose_valid_)  // avoid jumping from (0,0) to the initial points
    {
        PublishPath();
    }
}

void Visualizer::PublishRotorJointState()
{
    // static double last_time = 0.0;
    double time_now = ros::Time::now().toSec();
    float rotor_joint_update_freq = 100.0; // 100Hz max
    if (time_now - last_time_PublishRotorJointState_ > 1.0 / rotor_joint_update_freq)
    {
        double dt = time_now - last_time_PublishRotorJointState_;

        // static float joint_pos_[4] = {0.0, 0.5, 2.6, 1.4};
        float RPM = 100; // revolutions per minute
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

        last_time_PublishRotorJointState_ = time_now;
    }
}

void Visualizer::PublishBaseLinkTF()
{
    // static double last_time = 0.0;
    double time_now = ros::Time::now().toSec();
    if (time_now - last_time_PublishBaseLinkTF_ > 1.0 / max_freq_)
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

        last_time_PublishBaseLinkTF_ = time_now;
    }
}

void Visualizer::PublishPath()
{
    if (enable_history_path_)
    {
        /* Publishing a long path to rviz is heavily time consuming */
        //@TODO do not publish new path if the uav is static?
        if (path_pub_.getNumSubscribers() > 0)
        {
            // static double last_time = 0.0;
            double time_now = ros::Time::now().toSec();
            float history_path_update_freq = 10.0; // 10Hz fixed
            if (time_now - last_time_PublishPath_ > 1.0 / history_path_update_freq)
            {
                geometry_msgs::PoseStamped TrajPose_;
                TrajPose_.header.stamp = ros::Time::now();
                TrajPose_.header.frame_id = tf_frame_;
                TrajPose_.pose.position.x = pos_x_;
                TrajPose_.pose.position.y = pos_y_;
                TrajPose_.pose.position.z = pos_z_;          
                TrajPose_.pose.orientation = quat_;

                /* heavily time consuming */
                TrajPoseHistory_vector_.insert(TrajPoseHistory_vector_.begin(), TrajPose_);
                if (TrajPoseHistory_vector_.size() > history_path_time_ * history_path_update_freq)
                {
                    TrajPoseHistory_vector_.pop_back();
                }

                // std::vector<geometry_msgs::PoseStamped> old_vector{TrajPoseHistory_vector_};
                // TrajPoseHistory_vector_[0] = TrajPose_;
                // for (int i=1; i< old_vector.size(); ++i)
                // {
                //     TrajPoseHistory_vector_[i] = old_vector[i-1];
                // }
                
                nav_msgs::Path::Ptr path_msg(new nav_msgs::Path);
                path_msg->header.stamp = ros::Time::now();
                path_msg->header.frame_id = tf_frame_;
                path_msg->poses = TrajPoseHistory_vector_;
                path_pub_.publish(path_msg); /* heavily time consuming */

                last_time_PublishPath_ = time_now;
            }
        }
    }
}

void Visualizer::PublishMarkerName()
{
    // static double last_time = 0.0;
    double time_now = ros::Time::now().toSec();
    float marker_name_freq = 10;
    if (time_now - last_time_PublishMarkerName_ > 1.0 / marker_name_freq)
    {
        visualization_msgs::Marker::Ptr marker(new visualization_msgs::Marker);
        // marker->header.frame_id = tf_frame_;
        marker->header.frame_id = tf_child_frame_;
        marker->header.stamp = ros::Time();
        marker->ns = "my_namespace";
        marker->id = 0;
        marker->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker->action = visualization_msgs::Marker::ADD;
        marker->pose.position.x = 5.7;
        marker->pose.position.y = 4.7;
        marker->pose.position.z = 1.0;
        marker->pose.orientation = quat_;
        marker->scale.x = 6;
        marker->scale.y = 6;
        marker->scale.z = 6;
        marker->color.a = 1.0; // Don't forget to set the alpha!
        marker->color.r = 0; // [0,1]
        marker->color.g = 0; // [0,1]
        marker->color.b = 0; // [0,1]
        marker->frame_locked = true;
        //only if using a MESH_RESOURCE marker type:
        // marker->mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        //only if using a TEXT_VIEW_FACING marker type:
        marker->text = marker_name_;
        marker_name_pub_.publish( marker );

        last_time_PublishMarkerName_ = time_now;
    }
}

void Visualizer::cb_mavros_state(const mavros_msgs::State::ConstPtr& msg)
{
    armed_ = msg->armed;
}

void Visualizer::cb_mavros_local_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // pose_valid_ = true;
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
    // ros::Time tic = ros::Time::now();

    pose_valid_ = true;
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
            // ROS_WARN("lat %f", lat);
            // ROS_WARN("lon %f", lon);
            break;
        }
        default:
        {
            ROS_ERROR("[Visualizer] invalid local_pos_source.");
            break;
        }
    }

    // double toc = (ros::Time::now() - tic).toSec();
    // ROS_INFO("toc: %f", toc);
}


}
