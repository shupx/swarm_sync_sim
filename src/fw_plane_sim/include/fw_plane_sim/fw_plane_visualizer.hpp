/**
 * @file fw_plane_visualizer.hpp
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


#pragma once

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/CommandAck.h>

#include <geo/geo.h> // for px4 gps->NED utils

#include <cmath>
// Wrap to [-pi, pi)
inline double WrapToPi1(double theta) { return theta - 2 * M_PI * std::floor((theta + M_PI)/(2 * M_PI)); }
// Wrap to (-pi, pi]
inline double WrapToPi2(double theta) { return theta - 2 * M_PI * std::ceil((theta - M_PI)/(2 * M_PI)); }


namespace FwPlaneSimulator
{

/**
 * \brief A class for publishing rotor propeller joint position and base_link tf for the robot model visualization in rviz
 */
class Visualizer
{
public:
    /** \brief A class for publishing rotor propeller joint position and base_link tf for the robot model visualization in rviz
     * @param nh public ROS nodehandle
     * @param nh_private private ROS nodehandle
     */ 
    Visualizer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    /* Publish rotor joint states, base link TF and history path for displaying in rviz */
    void Run();

    /* Publish rotor/propeller joint states according to the arming state */
    void PublishRotorJointState();

    /* Publish the robot base link TF for displaying in the rviz */
    void PublishBaseLinkTF();

    /* Publish the robot base link history path for displaying in rviz */
    void PublishPath();

    /* Publish marker name */
    void PublishMarkerName();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber mavros_state_sub_;
    ros::Subscriber mavros_local_pose_sub_;
    ros::Subscriber mavros_global_pose_sub_;

    ros::Publisher joint_pub_;
    ros::Publisher path_pub_;
    ros::Publisher marker_name_pub_;

    ros::ServiceServer switch_pos_service_;

    tf2_ros::TransformBroadcaster tf2_broadcaster;

    enum class position_mode : uint32_t {
        MOCAP,
        GPS
    };
    /* local_pos_source_: default 0 for mocap(use local_position/pose for visualization directly), 1 for GPS (use global_position/global relative to the world_origin_gps for x,y visualization, use local_position/pose for z and attitude visualization) */
    position_mode local_pos_source_; 

    double world_origin_lat_; // Reference point latitude (degrees). The world (0,0,0) point
    double world_origin_lon_; // Reference point longitude (degrees). The world (0,0,0) point
    float world_origin_asml_alt_; // Reference altitude AMSL (metres). The world (0,0,0) point

    float max_freq_; /* default 10Hz. Maximum base_link tf publishing rate */
    float history_path_time_; /* (s) displaying history path time */
    bool armed_;
    float pos_x_, pos_y_, pos_z_;
    geometry_msgs::Quaternion quat_;

    std::string marker_name_; // visualize_marker_name
    std::string tf_frame_; // visualize_tf_frame
    std::string tf_child_frame_; // visualize_tf_child_frame
    std::string rotor_joints_name_[1]; // rotor joints name

    bool enable_history_path_;
    bool pose_valid_;

    double last_time_PublishRotorJointState_ = 0.0;
    double last_time_PublishBaseLinkTF_ = 0.0;
    double last_time_PublishPath_ = 0.0;
    double last_time_PublishMarkerName_ = 0.0;

    float joint_pos_[4] = {0.0, 0.5, 2.6, 1.4};
    std::vector<geometry_msgs::PoseStamped> TrajPoseHistory_vector_;


    void cb_mavros_state(const mavros_msgs::State::ConstPtr& msg);
    void cb_mavros_local_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void cb_mavros_global_pose(const sensor_msgs::NavSatFix::ConstPtr& msg);

    bool SwitchVisualizePoseSource(mavros_msgs::CommandAck::Request &req, mavros_msgs::CommandAck::Response &res);
    
};


}