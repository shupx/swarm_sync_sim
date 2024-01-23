/**
 * @file drone_visualizer.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Publish rotor propeller joint position and base_link tf states for the robot model visualization in rviz
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
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>


#include <cmath>
// Wrap to [-pi, pi)
inline double WrapToPi1(double theta) { return theta - 2 * M_PI * std::floor((theta + M_PI)/(2 * M_PI)); }
// Wrap to (-pi, pi]
inline double WrapToPi2(double theta) { return theta - 2 * M_PI * std::ceil((theta - M_PI)/(2 * M_PI)); }


namespace TelloQuadSimulator
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

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber tello_pose_sub_;

    ros::Publisher joint_pub_;
    ros::Publisher path_pub_;

    tf2_ros::TransformBroadcaster tf2_broadcaster_;

    float max_freq_; /* default 10Hz. Maximum base_link tf publishing rate */
    float history_path_time_; /* (s) displaying history path time */
    bool armed_;
    double pos_x_, pos_y_, pos_z_;
    geometry_msgs::Quaternion quat_;

    std::string tf_frame_; // visualize_tf_frame
    std::string tf_child_frame_; // visualize_tf_child_frame
    std::string rotor_joints_name_[4]; // rotor joints name

    bool enable_history_path_;

    double last_time_PublishRotorJointState_ = 0.0;
    double last_time_PublishBaseLinkTF_ = 0.0;
    double last_time_PublishPath_ = 0.0;

    float joint_pos_[4] = {0.0, 0.5, 2.6, 1.4};
    std::vector<geometry_msgs::PoseStamped> TrajPoseHistory_vector_;


    void cb_tello_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
};


}