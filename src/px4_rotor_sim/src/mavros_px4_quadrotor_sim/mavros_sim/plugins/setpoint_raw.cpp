/**
 * @file setpoint_raw.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Simulated mavros plugins.  Change from mavlink cpp headers to c headers in aligned with PX4
 * 
 * Similar to https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/setpoint_raw.cpp
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-11-29
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

/**
 * @brief SetpointRAW plugin
 * @file setpoint_raw.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015,2016 Vladimir Ermakov.
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

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

// header files added by Peixuan Shu
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h> // for tf::
#include <tf_conversions/tf_eigen.h>
#include <mavros/frame_tf.h> // Added by Peixuan Shu for ftf::
#include <mavlink/v2.0/common/mavlink.h> // Added by Peixuan Shu to use mavlink c headers
#include "../lib/mavros_uas.h" // Added by Peixuan Shu for mavros_sim::UAS
#include "setpoint_mixin.h" // modified in mavros_sim namespace


using namespace mavros; // for mavros::ftf, added by Peixuan Shu

namespace mavros_sim {  // namespace modified from mavros to mavros_sim by Peixuan Shu
namespace std_plugins {

/**
 * @brief Setpoint RAW plugin
 *
 * Send position setpoints and publish current state (return loop).
 * User can decide what set of filed needed for operation via IGNORE bits.
 */

class SetpointRawPlugin :
	public plugin::SetPositionTargetLocalNEDMixin<SetpointRawPlugin>,
	public plugin::SetPositionTargetGlobalIntMixin<SetpointRawPlugin>,
	public plugin::SetAttitudeTargetMixin<SetpointRawPlugin>
{
    public:
        SetpointRawPlugin(int agent_id, const std::shared_ptr<UAS> &uas, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : 
            sp_nh(nh, "mavros/setpoint_raw"),
            sp_nh_private(nh_private, "setpoint_raw"), // nodehandle modified by Peixuan Shu
            m_uas(uas), // added by Peixuan Shu
		    agent_id_(agent_id) // added by Peixuan Shu
        {
            bool tf_listen;

            local_sub = sp_nh.subscribe("local", 10, &SetpointRawPlugin::local_cb, this);
            global_sub = sp_nh.subscribe("global", 10, &SetpointRawPlugin::global_cb, this);
            attitude_sub = sp_nh.subscribe("attitude", 10, &SetpointRawPlugin::attitude_cb, this);
            target_local_pub = sp_nh.advertise<mavros_msgs::PositionTarget>("target_local", 10);
            target_global_pub = sp_nh.advertise<mavros_msgs::GlobalPositionTarget>("target_global", 10);
            target_attitude_pub = sp_nh.advertise<mavros_msgs::AttitudeTarget>("target_attitude", 10);

            // Set Thrust scaling in px4_config.yaml, setpoint_raw block.
            if (!sp_nh_private.getParam("thrust_scaling", thrust_scaling))  //sp_nh_private modified by Peixuan Shu
            {   
                // ROS_WARN_THROTTLE_NAMED(5, "setpoint_raw", "thrust_scaling parameter is unset. Attitude (and angular rate/thrust) setpoints will be ignored.");
                // thrust_scaling = -1.0;
                
                thrust_scaling = 1.0; // modified by Peixuan Shu (default value in px4_config.yaml)
            }
        }

        /* -*- message handlers (publish mavlink messages to ROS topics) -*- */

	    void handle_position_target_local_ned(const mavlink_message_t& msg) // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
        {
            // Decode mavlink message. Added by Peixuan Shu
            mavlink_position_target_local_ned_t tgt;
            mavlink_msg_position_target_local_ned_decode(&msg, &tgt); // mavlink c lib

            // Transform desired position,velocities,and accels from ENU to NED frame
            auto position = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.x, tgt.y, tgt.z));
            auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
            auto af = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
            float yaw = ftf::quaternion_get_yaw(
                        ftf::transform_orientation_aircraft_baselink(
                            ftf::transform_orientation_ned_enu(
                                ftf::quaternion_from_rpy(0.0, 0.0, tgt.yaw))));
            Eigen::Vector3d ang_vel_ned(0.0, 0.0, tgt.yaw_rate);
            auto ang_vel_enu = ftf::transform_frame_ned_enu(ang_vel_ned);
            float yaw_rate = ang_vel_enu.z();

            auto target = boost::make_shared<mavros_msgs::PositionTarget>();

            target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
            target->coordinate_frame = tgt.coordinate_frame;
            target->type_mask = tgt.type_mask;
            tf::pointEigenToMsg(position, target->position);
            tf::vectorEigenToMsg(velocity, target->velocity);
            tf::vectorEigenToMsg(af, target->acceleration_or_force);
            target->yaw = yaw;
            target->yaw_rate = yaw_rate;

            target_local_pub.publish(target);
        }

        void handle_position_target_global_int(const mavlink_message_t& msg) // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
        {
            // Decode mavlink message. Added by Peixuan Shu
            mavlink_position_target_global_int_t tgt;
            mavlink_msg_position_target_global_int_decode(&msg, &tgt); // mavlink c lib

            // Transform desired velocities from ENU to NED frame
            auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
            auto af = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
            float yaw = ftf::quaternion_get_yaw(
                        ftf::transform_orientation_aircraft_baselink(
                            ftf::transform_orientation_ned_enu(
                                ftf::quaternion_from_rpy(0.0, 0.0, tgt.yaw))));
            Eigen::Vector3d ang_vel_ned(0.0, 0.0, tgt.yaw_rate);
            auto ang_vel_enu = ftf::transform_frame_ned_enu(ang_vel_ned);
            float yaw_rate = ang_vel_enu.z();

            auto target = boost::make_shared<mavros_msgs::GlobalPositionTarget>();

            target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
            target->coordinate_frame = tgt.coordinate_frame;
            target->type_mask = tgt.type_mask;
            target->latitude = tgt.lat_int / 1e7;
            target->longitude = tgt.lon_int / 1e7;
            target->altitude = tgt.alt;
            tf::vectorEigenToMsg(velocity, target->velocity);
            tf::vectorEigenToMsg(af, target->acceleration_or_force);
            target->yaw = yaw;
            target->yaw_rate = yaw_rate;

            target_global_pub.publish(target);
        }

        void handle_attitude_target(const mavlink_message_t& msg) // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
        {
            // Decode mavlink message. Added by Peixuan Shu
            mavlink_attitude_target_t tgt;
            mavlink_msg_attitude_target_decode(&msg, &tgt); // mavlink c lib

            // Transform orientation from baselink -> ENU
            // to aircraft -> NED
            auto orientation = ftf::transform_orientation_ned_enu(
                        ftf::transform_orientation_baselink_aircraft(
                            ftf::mavlink_to_quaternion(FLOAT4_TO_ARRAY(tgt.q)))); // Change float[4] into std::array by Peixuan Shu

            auto body_rate = ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(tgt.body_roll_rate, tgt.body_pitch_rate, tgt.body_yaw_rate));

            auto target = boost::make_shared<mavros_msgs::AttitudeTarget>();

            target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
            target->type_mask = tgt.type_mask;
            tf::quaternionEigenToMsg(orientation, target->orientation);
            tf::vectorEigenToMsg(body_rate, target->body_rate);
            target->thrust = tgt.thrust;

            target_attitude_pub.publish(target);
        }

    	std::shared_ptr<UAS> m_uas; // store some common data and functions. Added by Peixuan Shu
	    int agent_id_ = -1; // global UAV mavlink id

private:	
        ros::NodeHandle sp_nh;
        ros::NodeHandle sp_nh_private;  // nodehandle added by Peixuan Shu
        ros::Subscriber local_sub, global_sub, attitude_sub;
        ros::Publisher target_local_pub, target_global_pub, target_attitude_pub;

        double thrust_scaling;

	    /* -*- callbacks -*- */

        void local_cb(const mavros_msgs::PositionTarget::ConstPtr &req)
        {
            Eigen::Vector3d position, velocity, af;
            float yaw, yaw_rate;

            tf::pointMsgToEigen(req->position, position);
            tf::vectorMsgToEigen(req->velocity, velocity);
            tf::vectorMsgToEigen(req->acceleration_or_force, af);

            // Transform frame ENU->NED
            if (req->coordinate_frame == mavros_msgs::PositionTarget::FRAME_BODY_NED || req->coordinate_frame == mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED) {
                position = ftf::transform_frame_baselink_aircraft(position);
                velocity = ftf::transform_frame_baselink_aircraft(velocity);
                af = ftf::transform_frame_baselink_aircraft(af);
                yaw = ftf::quaternion_get_yaw(
                        ftf::transform_orientation_absolute_frame_aircraft_baselink(
                                ftf::quaternion_from_rpy(0.0, 0.0, req->yaw)));
            } else {
                position = ftf::transform_frame_enu_ned(position);
                velocity = ftf::transform_frame_enu_ned(velocity);
                af = ftf::transform_frame_enu_ned(af);
                yaw = ftf::quaternion_get_yaw(
                        ftf::transform_orientation_aircraft_baselink(
                            ftf::transform_orientation_ned_enu(
                                ftf::quaternion_from_rpy(0.0, 0.0, req->yaw))));
            }

            Eigen::Vector3d ang_vel_enu(0.0, 0.0, req->yaw_rate);
            auto ang_vel_ned = ftf::transform_frame_ned_enu(ang_vel_enu);
            yaw_rate = ang_vel_ned.z();

            // update sp_SET_POSITION_TARGET_LOCAL_NED
            set_position_target_local_ned(
                        req->header.stamp.toNSec() / 1000000,
                        req->coordinate_frame,
                        req->type_mask,
                        position,
                        velocity,
                        af,
                        yaw, yaw_rate);
        }

        void global_cb(const mavros_msgs::GlobalPositionTarget::ConstPtr &req)
        {
            Eigen::Vector3d velocity, af;
            float yaw, yaw_rate;

            tf::vectorMsgToEigen(req->velocity, velocity);
            tf::vectorMsgToEigen(req->acceleration_or_force, af);

            // Transform frame ENU->NED
            velocity = ftf::transform_frame_enu_ned(velocity);
            af = ftf::transform_frame_enu_ned(af);
            yaw = ftf::quaternion_get_yaw(
                        ftf::transform_orientation_aircraft_baselink(
                            ftf::transform_orientation_ned_enu(
                                ftf::quaternion_from_rpy(0.0, 0.0, req->yaw))));
            Eigen::Vector3d ang_vel_enu(0.0, 0.0, req->yaw_rate);
            auto ang_vel_ned = ftf::transform_frame_ned_enu(ang_vel_enu);
            yaw_rate = ang_vel_ned.z();

            // update sp_SET_POSITION_TARGET_GLOBAL_INT
            set_position_target_global_int(
                        req->header.stamp.toNSec() / 1000000,
                        req->coordinate_frame,
                        req->type_mask,
                        req->latitude * 1e7,
                        req->longitude * 1e7,
                        req->altitude,
                        velocity,
                        af,
                        yaw, yaw_rate);
        }

        void attitude_cb(const mavros_msgs::AttitudeTarget::ConstPtr &req)
        {
            Eigen::Quaterniond desired_orientation;
            Eigen::Vector3d baselink_angular_rate;
            Eigen::Vector3d body_rate;
            double thrust;

            // ignore thrust is false by default, unless no thrust scaling is set or thrust is zero
            auto ignore_thrust = req->thrust != 0.0 && thrust_scaling < 0.0;

            if (ignore_thrust) {
                // I believe it's safer without sending zero thrust, but actually ignoring the actuation.
                ROS_FATAL_THROTTLE_NAMED(5, "setpoint_raw", "Recieved thrust, but ignore_thrust is true: "
                    "the most likely cause of this is a failure to specify the thrust_scaling parameters "
                    "on px4/apm_config.yaml. Actuation will be ignored.");
                return;
            } else {
                if (thrust_scaling == 0.0) {
                    ROS_WARN_THROTTLE_NAMED(5, "setpoint_raw", "thrust_scaling parameter is set to zero.");
                }
                thrust = std::min(1.0, std::max(0.0, req->thrust * thrust_scaling));
            }

            // Take care of attitude setpoint
            desired_orientation = ftf::to_eigen(req->orientation);

            // Transform desired orientation to represent aircraft->NED,
            // MAVROS operates on orientation of base_link->ENU
            auto ned_desired_orientation = ftf::transform_orientation_enu_ned(
                ftf::transform_orientation_baselink_aircraft(desired_orientation));

            body_rate = ftf::transform_frame_baselink_aircraft(
                ftf::to_eigen(req->body_rate));

            // update sp_SET_ATTITUDE_TARGET
            set_attitude_target(
                        req->header.stamp.toNSec() / 1000000,
                        req->type_mask,
                        ned_desired_orientation,
                        body_rate,
                        thrust);

        }        



};

}	// namespace std_plugins
}	// namespace mavros_sim