/**
 * @file local_position.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Simulated mavros plugins. Change from mavlink cpp headers to c headers in aligned with PX4
 * 
 * Similar to https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/local_position.cpp
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-12-19
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

/**
 * @brief LocalPosition plugin
 * @file local_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

// macros for mavlink cpp to c headers conversions by Peixuan Shu
#ifndef FLOAT4_TO_ARRAY
#include <array>
#define FLOAT4_TO_ARRAY(x) std::array<float, 4>{x[0], x[1], x[2], x[3]}
#endif
#ifndef ARRAY_TO_FLOAT4
#include <iostream>
#include <algorithm>
#define ARRAY_TO_FLOAT4(arr, x) std::copy(arr.begin(), arr.end(), x)
#endif

// #include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h> // for tf::

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

// header files added by Peixuan Shu
#include <ros/ros.h>
#include <mavros/frame_tf.h> // Added by Peixuan Shu for ftf::
#include <mavlink/v2.0/common/mavlink.h> // Added by Peixuan Shu to use mavlink c headers
#include <mavlink/v2.0/minimal/mavlink.h> // Added by Peixuan Shu to use mavlink c headers
#include "../lib/mavros_uas.h" // Added by Peixuan Shu for mavros_sim::UAS


using namespace mavros; // for mavros::ftf, added by Peixuan Shu


namespace mavros_sim {  // namespace modified from mavros to mavros_sim by Peixuan Shu
namespace std_plugins {
/**
 * @brief Local position plugin.
 * Publish local position to TF, PositionStamped, TwistStamped
 * and Odometry
 */
class LocalPositionPlugin
{
public:
	LocalPositionPlugin(const std::shared_ptr<UAS> &uas, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
		lp_nh(nh, "mavros/local_position"),  // nodehandle modified by Peixuan Shu
		lp_nh_private(nh_private, "local_position"),   // nodehandle added by Peixuan Shu
		tf_send(false),
		has_local_position_ned(false),
		has_local_position_ned_cov(false),
		m_uas(uas) // added by Peixuan Shu
	{
		// PluginBase::initialize(uas_);

		// header frame_id.
		// default to map (world-fixed,ENU as per REP-105).
		lp_nh_private.param<std::string>("frame_id", frame_id, "map");  // nodehandle modified by Peixuan Shu
		// Important tf subsection
		// Report the transform from world to base_link here.
		lp_nh_private.param("tf/send", tf_send, false);  // nodehandle modified by Peixuan Shu
		lp_nh_private.param<std::string>("tf/frame_id", tf_frame_id, "map");  // nodehandle modified by Peixuan Shu
		lp_nh_private.param<std::string>("tf/child_frame_id", tf_child_frame_id, "base_link");  // nodehandle modified by Peixuan Shu

		local_position = lp_nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
		local_position_cov = lp_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_cov", 10);
		local_velocity_local = lp_nh.advertise<geometry_msgs::TwistStamped>("velocity_local", 10);
		local_velocity_body = lp_nh.advertise<geometry_msgs::TwistStamped>("velocity_body", 10);
		local_velocity_cov = lp_nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity_body_cov", 10);
		local_accel = lp_nh.advertise<geometry_msgs::AccelWithCovarianceStamped>("accel", 10);
		local_odom = lp_nh.advertise<nav_msgs::Odometry>("odom",10);
	}

private:
	std::shared_ptr<UAS> m_uas; // store some common data and functions. Added by Peixuan Shu

	ros::NodeHandle lp_nh;
	ros::NodeHandle lp_nh_private; // nodehandle added by Peixuan Shu

	ros::Publisher local_position;
	ros::Publisher local_position_cov;
	ros::Publisher local_velocity_local;
	ros::Publisher local_velocity_body;
	ros::Publisher local_velocity_cov;
	ros::Publisher local_accel;
	ros::Publisher local_odom;

	std::string frame_id;		//!< frame for Pose
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_child_frame_id;	//!< frame for TF
	bool tf_send;
	bool has_local_position_ned;
	bool has_local_position_ned_cov;

	void publish_tf(boost::shared_ptr<nav_msgs::Odometry> &odom)
	{
		if (tf_send) {
			geometry_msgs::TransformStamped transform;
			transform.header.stamp = odom->header.stamp;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;
			transform.transform.translation.x = odom->pose.pose.position.x;
			transform.transform.translation.y = odom->pose.pose.position.y;
			transform.transform.translation.z = odom->pose.pose.position.z;
			transform.transform.rotation = odom->pose.pose.orientation;
			m_uas->tf2_broadcaster.sendTransform(transform);
		}
	}

public: // public by Peixuan Shu
	void handle_local_position_ned(const mavlink_message_t& msg)  // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
	{
		// Decode mavlink message. Added by Peixuan Shu
		mavlink_local_position_ned_t pos_ned;
		mavlink_msg_local_position_ned_decode(&msg, &pos_ned); // mavlink c lib		

		has_local_position_ned = true;

		//--------------- Transform FCU position and Velocity Data ---------------//
		auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.x, pos_ned.y, pos_ned.z));
		auto enu_velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz));

		//--------------- Get Odom Information ---------------//
		// Note this orientation describes baselink->ENU transform
		auto enu_orientation_msg = m_uas->get_attitude_orientation_enu();
		auto baselink_angular_msg = m_uas->get_attitude_angular_velocity_enu();
		Eigen::Quaterniond enu_orientation;
		tf::quaternionMsgToEigen(enu_orientation_msg, enu_orientation);
		auto baselink_linear = ftf::transform_frame_enu_baselink(enu_velocity, enu_orientation.inverse());

		auto odom = boost::make_shared<nav_msgs::Odometry>();
		odom->header = m_uas->synchronized_header(frame_id, pos_ned.time_boot_ms);
		odom->child_frame_id = tf_child_frame_id;

		tf::pointEigenToMsg(enu_position, odom->pose.pose.position);
		odom->pose.pose.orientation = enu_orientation_msg;
		tf::vectorEigenToMsg(baselink_linear, odom->twist.twist.linear);
		odom->twist.twist.angular = baselink_angular_msg;

		// publish odom if we don't have LOCAL_POSITION_NED_COV
		if (!has_local_position_ned_cov) {
			local_odom.publish(odom);
		}

		// publish pose always
		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
		pose->header = odom->header;
		pose->pose = odom->pose.pose;
		local_position.publish(pose);

		// publish velocity always
		// velocity in the body frame
		auto twist_body = boost::make_shared<geometry_msgs::TwistStamped>();
		twist_body->header.stamp = odom->header.stamp;
		twist_body->header.frame_id = tf_child_frame_id;
		twist_body->twist.linear = odom->twist.twist.linear;
		twist_body->twist.angular = baselink_angular_msg;
		local_velocity_body.publish(twist_body);

		// velocity in the local frame
		auto twist_local = boost::make_shared<geometry_msgs::TwistStamped>();
		twist_local->header.stamp = twist_body->header.stamp;
		twist_local->header.frame_id = tf_child_frame_id;
		tf::vectorEigenToMsg(enu_velocity, twist_local->twist.linear);
		tf::vectorEigenToMsg(ftf::transform_frame_baselink_enu(ftf::to_eigen(baselink_angular_msg), enu_orientation),
						twist_local->twist.angular);

		local_velocity_local.publish(twist_local);

		// publish tf
		publish_tf(odom);
	}

	void handle_local_position_ned_cov(const mavlink_message_t& msg)  // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
	{
		// Decode mavlink message. Added by Peixuan Shu
		mavlink_local_position_ned_cov_t pos_ned;
		mavlink_msg_local_position_ned_cov_decode(&msg, &pos_ned); // mavlink c lib		

		has_local_position_ned_cov = true;

		auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.x, pos_ned.y, pos_ned.z));
		auto enu_velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz));

		auto enu_orientation_msg = m_uas->get_attitude_orientation_enu();
		auto baselink_angular_msg = m_uas->get_attitude_angular_velocity_enu();
		Eigen::Quaterniond enu_orientation;
		tf::quaternionMsgToEigen(enu_orientation_msg, enu_orientation);
		auto baselink_linear = ftf::transform_frame_enu_baselink(enu_velocity, enu_orientation.inverse());

		auto odom = boost::make_shared<nav_msgs::Odometry>();
		odom->header = m_uas->synchronized_header(frame_id, pos_ned.time_usec);
		odom->child_frame_id = tf_child_frame_id;

		tf::pointEigenToMsg(enu_position, odom->pose.pose.position);
		odom->pose.pose.orientation = enu_orientation_msg;
		tf::vectorEigenToMsg(baselink_linear, odom->twist.twist.linear);
		odom->twist.twist.angular = baselink_angular_msg;

		odom->pose.covariance[0] = pos_ned.covariance[0];	// x
		odom->pose.covariance[7] = pos_ned.covariance[9];	// y
		odom->pose.covariance[14] = pos_ned.covariance[17];	// z

		odom->twist.covariance[0] = pos_ned.covariance[24];	// vx
		odom->twist.covariance[7] = pos_ned.covariance[30];	// vy
		odom->twist.covariance[14] = pos_ned.covariance[35];	// vz
		// TODO: orientation + angular velocity covariances from ATTITUDE_QUATERION_COV

		// publish odom always
		local_odom.publish(odom);

		// publish pose_cov always
		auto pose_cov = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
		pose_cov->header = odom->header;
		pose_cov->pose = odom->pose;
		local_position_cov.publish(pose_cov);

		// publish velocity_cov always
		auto twist_cov = boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>();
		twist_cov->header.stamp = odom->header.stamp;
		twist_cov->header.frame_id = odom->child_frame_id;
		twist_cov->twist = odom->twist;
		local_velocity_cov.publish(twist_cov);

		// publish pose, velocity, tf if we don't have LOCAL_POSITION_NED
		if (!has_local_position_ned) {
			auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
			pose->header = odom->header;
			pose->pose = odom->pose.pose;
			local_position.publish(pose);

			auto twist = boost::make_shared<geometry_msgs::TwistStamped>();
			twist->header.stamp = odom->header.stamp;
			twist->header.frame_id = odom->child_frame_id;
			twist->twist = odom->twist.twist;
			local_velocity_body.publish(twist);

			// publish tf
			publish_tf(odom);
		}

		// publish accelerations
		auto accel = boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
		accel->header = odom->header;

		auto enu_accel = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.ax, pos_ned.ay, pos_ned.az));
		tf::vectorEigenToMsg(enu_accel, accel->accel.accel.linear);

		accel->accel.covariance[0] = pos_ned.covariance[39];	// ax
		accel->accel.covariance[7] = pos_ned.covariance[42];	// ay
		accel->accel.covariance[14] = pos_ned.covariance[44];	// az

		local_accel.publish(accel);
	}
};
}	// namespace std_plugins
}	// namespace mavros_sim