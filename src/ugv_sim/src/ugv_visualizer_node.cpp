/**
 * @file ugv_visualizer_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Publish wheel joint position and base_link tf states for the robot model visualization in rviz
 * 
 * Note: This program relies on geometry_msgs
 * 
 * @version 1.0
 * @date 2024-1-23
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */


#include "ugv_sim/ugv_visualizer.hpp"

using namespace UgvSimulator;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //Use unique_ptr to auto-destory the object when exiting.
    std::unique_ptr<Visualizer> visualizer(new Visualizer(nh, nh_private));

    ros::Rate loop_rate(50); // Hz
    while (ros::ok())
    {
        visualizer->Run();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

namespace UgvSimulator
{

Visualizer::Visualizer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : 
    nh_(nh), 
    nh_private_(nh_private),
    pose_valid_(false)
{
    nh_private_.param<float>("visualize_max_freq", max_freq_, 10);
    nh_private_.param<float>("visualize_path_time", history_path_time_, 5.0);
    nh_private_.param<std::string>("visualize_marker_name", marker_name_, "ugv");
    nh_private_.param<std::string>("visualize_tf_frame", tf_frame_, "map");
    nh_private_.param<std::string>("base_link_name", tf_child_frame_, "base_link");
    nh_private_.param<std::string>("rotor_0_joint_name", rotor_joints_name_[0], "rotor_0_joint");
    nh_private_.param<std::string>("rotor_1_joint_name", rotor_joints_name_[1], "rotor_1_joint");
    nh_private_.param<std::string>("rotor_2_joint_name", rotor_joints_name_[2], "rotor_2_joint");
    nh_private_.param<std::string>("rotor_3_joint_name", rotor_joints_name_[3], "rotor_3_joint");

    std::string tf_prefix;
    if (nh_private_.getParam("base_link_tf_prefix", tf_prefix))
    {
        tf_child_frame_ = tf_prefix + "/" + tf_child_frame_;
    }

    nh_private_.param<bool>("enable_history_path", enable_history_path_, true);

    pose_sub_ = nh_.subscribe("pose", 1, &Visualizer::cb_pose, this);
    twist_sub_ = nh_.subscribe("twist", 1, &Visualizer::cb_twist, this);

    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("history_path", 1, true);
    marker_name_pub_ = nh_.advertise<visualization_msgs::Marker>("marker_name", 1, true);

    pos_x_ = 0.0; pos_y_ = 0.0; pos_z_ = 0.0;
    v_front_ = 0.0; v_left_ = 0.0; omega_ = 0.0;
    quat_.w = 1.0;
    quat_.x = 0.0;
    quat_.y = 0.0;
    quat_.z = 0.0;
    yaw_ = 0.0;
}

void Visualizer::Run()
{
    PublishRotorJointState();
    // PublishBaseLinkTF();
    PublishMarkerName();
    if (pose_valid_) // avoid jumping from (0,0) to the initial points
    {
        PublishPath();
    }
}

void Visualizer::PublishRotorJointState()
{
    // static double last_time = 0.0;
    double time_now = ros::Time::now().toSec();
    float rotor_joint_update_freq = 15.0; // 15Hz max
    if (time_now - last_time_PublishRotorJointState_ > 1.0 / rotor_joint_update_freq)
    {
        PublishBaseLinkTF(); // keep base link tf and rotor joint state synchronized

        // double dt = time_now - last_time_PublishRotorJointState_;
        double dt = 1.0 / rotor_joint_update_freq;

        float wheel_radius = 0.04;
        // float length = 0.3; // front to rear wheel
        // float width = 0.216; // left to right wheel
        float length = 0.4; // front to rear wheel
        float width = 0.3; // left to right wheel

        /* Get the mecanum wheel angular speed */
        float wheel_v[4];
        wheel_v[0] = v_front_ - v_left_ - omega_ * (length/2+width/2); // upper left
        wheel_v[1] = v_front_ + v_left_ + omega_ * (length/2+width/2); // upper right
        wheel_v[2] = v_front_ + v_left_ - omega_ * (length/2+width/2); // lower left
        wheel_v[3] = v_front_ - v_left_ + omega_ * (length/2+width/2); // lower right
        float wheel_omega[4];
        wheel_omega[0] = - wheel_v[0] / wheel_radius;
        wheel_omega[1] = wheel_v[1] / wheel_radius;
        wheel_omega[2] = - wheel_v[2] / wheel_radius;
        wheel_omega[3] = wheel_v[3] / wheel_radius;

        sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState);
        msg->header.stamp = ros::Time::now();
        int joint_num = sizeof(rotor_joints_name_) / sizeof(rotor_joints_name_[0]);
        msg->name.resize(joint_num);
        msg->position.resize(joint_num);
        for (int i=0; i<joint_num; ++i)
        {
            joint_pos_[i] = WrapToPi1(joint_pos_[i] + wheel_omega[i] * dt);
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
        tf2_broadcaster_.sendTransform(odom_trans);

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
            float history_path_update_freq = 5.0; // 5Hz fixed
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
    if (time_now - last_time_PublishMarkerName_ > 1.0 / max_freq_)
    {
        visualization_msgs::Marker::Ptr marker(new visualization_msgs::Marker);
        marker->header.frame_id = tf_child_frame_;
        marker->header.stamp = ros::Time();
        marker->ns = "my_namespace";
        marker->id = 0;
        marker->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker->action = visualization_msgs::Marker::ADD;
        marker->pose.position.x = 0.25;
        marker->pose.position.y = 0.25;
        marker->pose.position.z = 0.25;
        marker->pose.orientation = quat_;
        marker->scale.x = 0.17; // metre
        marker->scale.y = 0.17; // metre
        marker->scale.z = 0.17; // metre
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


void Visualizer::cb_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose_valid_ = true;
    pos_x_ = msg->pose.position.x;
    pos_y_ = msg->pose.position.y;
    pos_z_ = msg->pose.position.z;
    quat_ = msg->pose.orientation;

    /* The Eigen conversion to euler angle is incorrect since the range of the output is always
     * [0, pi], [-pi, pi], [-pi, pi] for the first, second, and third elements
     */
    // Eigen::Vector3d euler_angle = state_.R.eulerAngles(2,1,0);

    // Z-Y-X order，RPY, roll[-pi, pi], pitch[-pi/2, pi/2], yaw[-pi, pi]
    // Correct conversion to ensure the correct range of roll, pitch and roll
    Eigen::Vector3d eulerAngle_rpy; // Z-Y-X RPY
    Eigen::Quaterniond q{quat_.w, quat_.x, quat_.y, quat_.z};
    Eigen::Matrix3d rot = Eigen::Matrix3d{q};
    eulerAngle_rpy(0) = std::atan2(rot(2, 1), rot(2, 2)); // roll
    eulerAngle_rpy(1) = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2))); //pitch
    eulerAngle_rpy(2) = std::atan2(rot(1, 0), rot(0, 0)); //yaw
    yaw_ = eulerAngle_rpy(2);
}

void Visualizer::cb_twist(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    double vx = msg->twist.linear.x; // world x
    double vy = msg->twist.linear.y; // world y

    /* World to body frame */
    v_front_ = vx * cos(-yaw_) - vy * sin(-yaw_);
    v_left_ = vx * sin(-yaw_) + vy * cos(-yaw_);

    omega_ = msg->twist.angular.z; // rad/s
}


}
