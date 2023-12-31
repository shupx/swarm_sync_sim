/**
 * @file command.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Simulated mavros plugins. Change from mavlink cpp headers to c headers in aligned with PX4
 * 
 * Similar to https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/command.cpp
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-12-22
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

/**
 * @brief Command plugin
 * @file command.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
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

#include <chrono>
#include <condition_variable>
// #include <mavros/mavros_plugin.h>

#include <mavros_msgs/CommandAck.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandInt.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandTriggerControl.h>
#include <mavros_msgs/CommandTriggerInterval.h>
#include <mavros_msgs/CommandVtolTransition.h>

// header files added by Peixuan Shu
#include <ros/ros.h>
#include <mavlink/v2.0/common/mavlink.h> // Added by Peixuan Shu to use mavlink c headers
#include <mavlink/v2.0/minimal/mavlink.h> // Added by Peixuan Shu to use mavlink c headers
#include "../lib/mavros_uas.h" // Added by Peixuan Shu for mavros_sim::UAS
#include "px4_modules/mavlink/mavlink_msg_list.hpp" // Added by Peixuan Shu store the simulated static(global) mavlink messages

namespace mavros_sim {  // namespace modified from mavros to mavros_sim by Peixuan Shu
namespace std_plugins {
static constexpr double ACK_TIMEOUT_DEFAULT = 5.0;
// using utils::enum_value;
using lock_guard = std::lock_guard<std::mutex>;
using unique_lock = std::unique_lock<std::mutex>;

class CommandTransaction {
public:
	std::mutex cond_mutex;
	std::condition_variable ack;
	uint16_t expected_command;
	uint8_t result;

	explicit CommandTransaction(uint16_t command) :
		ack(),
		expected_command(command),
		// Default result if wait ack timeout
		result(MAV_RESULT_FAILED)
	{ }
};

/**
 * @brief Command plugin.
 *
 * Send any command via COMMAND_LONG
 */
class CommandPlugin
{
public:
	CommandPlugin(const std::shared_ptr<UAS> &uas, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
		cmd_nh(nh, "mavros/cmd"),   // nodehandle modified by Peixuan Shu
		cmd_nh_private(nh_private, "cmd"),   // nodehandle added by Peixuan Shu
		use_comp_id_system_control(false),
		m_uas(uas) // added by Peixuan Shu
	{
		// PluginBase::initialize(uas_);

		double command_ack_timeout;

		cmd_nh_private.param("command_ack_timeout", command_ack_timeout, ACK_TIMEOUT_DEFAULT);   // nodehandle modified by Peixuan Shu
		cmd_nh_private.param("use_comp_id_system_control", use_comp_id_system_control, false);   // nodehandle modified by Peixuan Shu

		command_ack_timeout_dt = ros::Duration(command_ack_timeout);

		command_long_srv = cmd_nh.advertiseService("command", &CommandPlugin::command_long_cb, this);
		// command_ack_srv = cmd_nh.advertiseService("command_ack", &CommandPlugin::command_ack_cb, this);
		command_int_srv = cmd_nh.advertiseService("command_int", &CommandPlugin::command_int_cb, this);
		arming_srv = cmd_nh.advertiseService("arming", &CommandPlugin::arming_cb, this);
		// set_home_srv = cmd_nh.advertiseService("set_home", &CommandPlugin::set_home_cb, this);
		takeoff_srv = cmd_nh.advertiseService("takeoff", &CommandPlugin::takeoff_cb, this);
		land_srv = cmd_nh.advertiseService("land", &CommandPlugin::land_cb, this);
		// trigger_control_srv = cmd_nh.advertiseService("trigger_control", &CommandPlugin::trigger_control_cb, this);
		// trigger_interval_srv = cmd_nh.advertiseService("trigger_interval", &CommandPlugin::trigger_interval_cb, this);
		// vtol_transition_srv = cmd_nh.advertiseService("vtol_transition", &CommandPlugin::vtol_transition_cb, this);
	}

private:
	using L_CommandTransaction = std::list<CommandTransaction>;

	std::mutex mutex;

	std::shared_ptr<UAS> m_uas; // store some common data and functions. Added by Peixuan Shu

	ros::NodeHandle cmd_nh;
	ros::NodeHandle cmd_nh_private; // nodehandle added by Peixuan Shu
	ros::ServiceServer command_long_srv;
	ros::ServiceServer command_int_srv;
	ros::ServiceServer command_ack_srv;
	ros::ServiceServer arming_srv;
	ros::ServiceServer set_home_srv;
	ros::ServiceServer takeoff_srv;
	ros::ServiceServer land_srv;
	ros::ServiceServer trigger_control_srv;
	ros::ServiceServer trigger_interval_srv;
	ros::ServiceServer vtol_transition_srv;

	bool use_comp_id_system_control;

	L_CommandTransaction ack_waiting_list;
	ros::Duration command_ack_timeout_dt;


	/* -*- message handlers -*- */

public: // public by Peixuan Shu
	void handle_command_ack(const mavlink_message_t& msg)  // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
	{
		// Decode mavlink message. Added by Peixuan Shu
		mavlink_command_ack_t ack;
		mavlink_msg_command_ack_decode(&msg, &ack); // mavlink c lib		

		lock_guard lock(mutex);

		// XXX(vooon): place here source ids check

		for (auto &tr : ack_waiting_list) {
			if (tr.expected_command == ack.command) {
				tr.result = ack.result;
				tr.ack.notify_all();
				return;
			}
		}

		ROS_WARN_THROTTLE_NAMED(10, "cmd", "CMD: Unexpected command %u, result %u",
			ack.command, ack.result);
	}

private:
	/* -*- mid-level functions -*- */

	bool wait_ack_for(CommandTransaction &tr) {
		unique_lock lock(tr.cond_mutex);
		if (tr.ack.wait_for(lock, std::chrono::nanoseconds(command_ack_timeout_dt.toNSec())) == std::cv_status::timeout) {
			ROS_WARN_NAMED("cmd", "CMD: Command %u -- wait ack timeout", tr.expected_command);
			return false;
		} else {
			return true;
		}
	}

	/**
	 * Common function for command service callbacks.
	 *
	 * NOTE: success is bool in messages, but has unsigned char type in C++
	 */
	bool send_command_long_and_wait(bool broadcast,
		uint16_t command, uint8_t confirmation,
		float param1, float param2,
		float param3, float param4,
		float param5, float param6,
		float param7,
		unsigned char &success, uint8_t &result)
	{
		// using mavlink::common::MAV_RESULT;

		unique_lock lock(mutex);

		L_CommandTransaction::iterator ack_it;

		/* check transactions */
		for (const auto &tr : ack_waiting_list) {
			if (tr.expected_command == command) {
				ROS_WARN_THROTTLE_NAMED(10, "cmd", "CMD: Command %u already in progress", command);
				return false;
			}
		}

		/**
		 * @note APM & PX4 master always send COMMAND_ACK. Old PX4 never.
		 * Don't expect any ACK in broadcast mode.
		 */
		bool is_ack_required = (confirmation != 0 || m_uas->is_ardupilotmega() || m_uas->is_px4()) && !broadcast;

		/** Added by Peixuan Shu
		 * Disable ack mechanism (no wait anymore) since it will block the ROS callback queue, 
		 * which leads to the block of the mainloop timer!
		 */ 
		is_ack_required = false; // added by Peixuan Shu 
		std::cout << "[mavros_sim::CommandPlugin] Caution! Vehicle command ack is not simulated by mavros_sim, so the service response is always true!" << std::endl;

		if (is_ack_required)
			ack_it = ack_waiting_list.emplace(ack_waiting_list.end(), command);

		command_long(broadcast,
			command, confirmation,
			param1, param2,
			param3, param4,
			param5, param6,
			param7);

		if (is_ack_required) {
			lock.unlock();
			bool is_not_timeout = wait_ack_for(*ack_it);
			lock.lock();

			success = is_not_timeout && ack_it->result == MAV_RESULT_ACCEPTED;
			result = ack_it->result;

			ack_waiting_list.erase(ack_it);
		}
		else {
			success = true;
			result = MAV_RESULT_ACCEPTED;
		}

		return true;
	}

	/**
	 * Common function for COMMAND_INT service callbacks.
	 */
	bool send_command_int(bool broadcast,
		uint8_t frame, uint16_t command,
		uint8_t current, uint8_t autocontinue,
		float param1, float param2,
		float param3, float param4,
		int32_t x, int32_t y,
		float z,
		unsigned char &success)
	{
		/* Note: seems that COMMAND_INT don't produce COMMAND_ACK
		 * so wait don't needed.
		 */
		command_int(broadcast,
			frame, command, current, autocontinue,
			param1, param2,
			param3, param4,
			x, y, z);

		success = true;
		return true;
	}

	bool send_command_ack( uint16_t command, uint8_t req_result, uint8_t progress, int32_t result_param2,
		unsigned char &success, uint8_t &res_result)
	{
		// using mavlink::common::MAV_RESULT;

		command_ack(command,
			req_result, progress,
			result_param2);


		success = true;
		res_result = MAV_RESULT_ACCEPTED;

		return true;
	}


	/* -*- low-level send -*- */

	template<typename MsgT>
	inline void set_target(MsgT &cmd, bool broadcast)
	{
		// using mavlink::minimal::MAV_COMPONENT;

		const uint8_t tgt_sys_id = (broadcast) ? 0 : m_uas->get_tgt_system();
		const uint8_t tgt_comp_id = (broadcast) ? 0 :
			(use_comp_id_system_control) ?
			MAV_COMP_ID_SYSTEM_CONTROL : m_uas->get_tgt_component();

		cmd.target_system = tgt_sys_id;
		cmd.target_component = tgt_comp_id;
	}

	void command_long(bool broadcast,
		uint16_t command, uint8_t confirmation,
		float param1, float param2,
		float param3, float param4,
		float param5, float param6,
		float param7)
	{
		const uint8_t confirmation_fixed = (broadcast) ? 0 : confirmation;

		// mavlink::common::msg::COMMAND_LONG cmd {};
		mavlink_command_long_t cmd {}; // modified to mavlink c headers by Peixuan Shu
		set_target(cmd, broadcast);

		cmd.command = command;
		cmd.confirmation = confirmation_fixed;
		cmd.param1 = param1;
		cmd.param2 = param2;
		cmd.param3 = param3;
		cmd.param4 = param4;
		cmd.param5 = param5;
		cmd.param6 = param6;
		cmd.param7 = param7;

		// UAS_FCU(m_uas)->send_message_ignore_drop(cmd);

		/*  Added by Peixuan Shu. Write mavlink messages into "px4_modules/mavlink/mavlink_msg_list.hpp" */
		int handle = (int) px4::mavlink_receive_handle::COMMAND_LONG;
		mavlink_msg_command_long_encode(1, 1, &px4::mavlink_receive_list[handle].msg, &cmd); 
		px4::mavlink_receive_list[handle].updated = true;
	}

	void command_int(bool broadcast,
		uint8_t frame, uint16_t command,
		uint8_t current, uint8_t autocontinue,
		float param1, float param2,
		float param3, float param4,
		int32_t x, int32_t y,
		float z)
	{
		// mavlink::common::msg::COMMAND_INT cmd {};
		mavlink_command_int_t cmd {}; // modified to mavlink c headers by Peixuan Shu
		set_target(cmd, broadcast);

		cmd.frame = frame;
		cmd.command = command;
		cmd.current = current;
		cmd.autocontinue = autocontinue;
		cmd.param1 = param1;
		cmd.param2 = param2;
		cmd.param3 = param3;
		cmd.param4 = param4;
		cmd.x = x;
		cmd.y = y;
		cmd.z = z;

		// UAS_FCU(m_uas)->send_message_ignore_drop(cmd);

		/*  Added by Peixuan Shu. Write mavlink messages into "px4_modules/mavlink/mavlink_msg_list.hpp" */
		int handle = (int) px4::mavlink_receive_handle::COMMAND_INT;
		mavlink_msg_command_int_encode(1, 1, &px4::mavlink_receive_list[handle].msg, &cmd); 
		px4::mavlink_receive_list[handle].updated = true;		
	}

	void command_ack( uint16_t command, uint8_t result,
		uint8_t progress, int32_t result_param2)
	{
		// // mavlink::common::msg::COMMAND_ACK cmd {};
		// mavlink_command_ack_t cmd {}; // modified to mavlink c headers by Peixuan Shu
		// set_target(cmd, false);

		// cmd.command = command;
		// cmd.result = result;
		// cmd.progress = progress;
		// cmd.result_param2 = result_param2;

		// // UAS_FCU(m_uas)->send_message_ignore_drop(cmd);

		// /*  Added by Peixuan Shu. Write mavlink messages into "px4_modules/mavlink/mavlink_msg_list.hpp" */
		// int handle = (int) px4::mavlink_receive_handle::COMMAND_ACK;
		// mavlink_msg_command_ack_encode(1, 1, &px4::mavlink_receive_list[handle].msg, &cmd); 
		// px4::mavlink_receive_list[handle].updated = true;		
	}


	/* -*- callbacks -*- */

	bool command_long_cb(mavros_msgs::CommandLong::Request &req,
		mavros_msgs::CommandLong::Response &res)
	{
		return send_command_long_and_wait(req.broadcast,
			req.command, req.confirmation,
			req.param1, req.param2,
			req.param3, req.param4,
			req.param5, req.param6,
			req.param7,
			res.success, res.result);
	}

	bool command_int_cb(mavros_msgs::CommandInt::Request &req,
		mavros_msgs::CommandInt::Response &res)
	{
		return send_command_int(req.broadcast,
			req.frame, req.command,
			req.current, req.autocontinue,
			req.param1, req.param2,
			req.param3, req.param4,
			req.x, req.y, req.z,
			res.success);
	}

	bool command_ack_cb(mavros_msgs::CommandAck::Request &req,
		mavros_msgs::CommandAck::Response &res)
	{
		return send_command_ack(req.command, req.result,
			req.progress, req.result_param2,
			res.success, res.result);
	}


	bool arming_cb(mavros_msgs::CommandBool::Request &req,
		mavros_msgs::CommandBool::Response &res)
	{
		// using mavlink::common::MAV_CMD;
		return send_command_long_and_wait(false,
			MAV_CMD_COMPONENT_ARM_DISARM, 1,
			(req.value) ? 1.0 : 0.0,
			0, 0, 0, 0, 0, 0,
			res.success, res.result);
	}

	bool set_home_cb(mavros_msgs::CommandHome::Request &req,
		mavros_msgs::CommandHome::Response &res)
	{
		// using mavlink::common::MAV_CMD;
		return send_command_long_and_wait(false,
			MAV_CMD_DO_SET_HOME, 1,
			(req.current_gps) ? 1.0 : 0.0,
			0, 0, req.yaw, req.latitude, req.longitude, req.altitude,
			res.success, res.result);
	}

	bool takeoff_cb(mavros_msgs::CommandTOL::Request &req,
		mavros_msgs::CommandTOL::Response &res)
	{
		// using mavlink::common::MAV_CMD;
		return send_command_long_and_wait(false,
			MAV_CMD_NAV_TAKEOFF, 1,
			req.min_pitch,
			0, 0,
			req.yaw,
			req.latitude, req.longitude, req.altitude,
			res.success, res.result);
	}

	bool land_cb(mavros_msgs::CommandTOL::Request &req,
		mavros_msgs::CommandTOL::Response &res)
	{
		// using mavlink::common::MAV_CMD;
		return send_command_long_and_wait(false,
			MAV_CMD_NAV_LAND, 1,
			0, 0, 0,
			req.yaw,
			req.latitude, req.longitude, req.altitude,
			res.success, res.result);
	}

	bool trigger_control_cb(mavros_msgs::CommandTriggerControl::Request &req,
		mavros_msgs::CommandTriggerControl::Response &res)
	{
		// using mavlink::common::MAV_CMD;
		return send_command_long_and_wait(false,
			MAV_CMD_DO_TRIGGER_CONTROL, 1,
			(req.trigger_enable) ? 1.0 : 0.0,
			(req.sequence_reset) ? 1.0 : 0.0,
			(req.trigger_pause) ? 1.0 : 0.0,
			0, 0, 0, 0,
			res.success, res.result);
	}

	bool trigger_interval_cb(mavros_msgs::CommandTriggerInterval::Request &req,
		mavros_msgs::CommandTriggerInterval::Response &res)
	{
		// using mavlink::common::MAV_CMD;

		// trigger interval can only be set when triggering is disabled
		return send_command_long_and_wait(false,
			MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL, 1,
			req.cycle_time,
			req.integration_time,
			0, 0, 0, 0, 0,
			res.success, res.result);
	}

	bool vtol_transition_cb(mavros_msgs::CommandVtolTransition::Request &req,
		mavros_msgs::CommandVtolTransition::Response &res)
	{
		// using mavlink::common::MAV_CMD;
		return send_command_long_and_wait(false,
			MAV_CMD_DO_VTOL_TRANSITION, false,
			req.state,
			0, 0, 0, 0, 0, 0,
			res.success, res.result);
	}
};
}	// namespace std_plugins
}	// namespace mavros

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::CommandPlugin, mavros::plugin::PluginBase)
