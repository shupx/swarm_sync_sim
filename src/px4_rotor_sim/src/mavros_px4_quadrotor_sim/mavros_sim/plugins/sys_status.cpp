/**
 * @file sys_status.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Simulated mavros plugins. Change from mavlink cpp headers to c headers in aligned with PX4
 * 
 * Similar to https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/sys_status.cpp
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-12-20
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

/**
 * @brief System Status plugin
 * @file sys_status.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov.
 * Copyright 2022 Dr.-Ing. Amilcar do Carmo Lucas, IAV GmbH.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

// #include <mavros/mavros_plugin.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/VehicleInfo.h>
#include <mavros_msgs/VehicleInfoGet.h>
#include <mavros_msgs/MessageInterval.h>


#ifdef HAVE_SENSOR_MSGS_BATTERYSTATE_MSG
#include <sensor_msgs/BatteryState.h>
using BatteryMsg = sensor_msgs::BatteryState;
#else
#include <mavros_msgs/BatteryStatus.h>
using BatteryMsg = mavros_msgs::BatteryStatus;
#endif

// header files added by Peixuan Shu
#include <ros/ros.h>
#include <mavros/frame_tf.h> // Added by Peixuan Shu for ftf::
#include <mavlink/v2.0/common/mavlink.h> // Added by Peixuan Shu to use mavlink c headers
#include <mavlink/v2.0/minimal/mavlink.h> // Added by Peixuan Shu to use mavlink c headers
#include "../lib/mavros_uas.h" // Added by Peixuan Shu for mavros_sim::UAS
#include "px4_modules/mavlink/mavlink_msg_list.hpp" // Added by Peixuan Shu store the simulated static(global) mavlink messages
#include "unordered_map" // Added by Peixuan Shu for std::unordered_map


using namespace mavros; // for mavros::ftf, added by Peixuan Shu

namespace mavros_sim {  // namespace modified from mavros to mavros_sim by Peixuan Shu

/**
 * @brief Make printf-formatted std::string (added by Peixuan Shu from libmavconn/include/mavconn/thread_utils.h)
 *
 */
namespace utils  // namespace added by Peixuan Shu
{
template<typename ... Args>
std::string format(const std::string &fmt, Args ... args)
{
	// C++11 specify that string store elements continously
	std::string ret;

	auto sz = std::snprintf(nullptr, 0, fmt.c_str(), args...);
	ret.reserve(sz + 1); ret.resize(sz);	// to be sure there have room for \0
	std::snprintf(&ret.front(), ret.capacity() + 1, fmt.c_str(), args...);
	return ret;
}
}

namespace std_plugins {


#define MAX_NR_BATTERY_STATUS 10

// Delete all diagnostic_updater by Peixuan Shu

/**
 * @brief System status plugin.
 *
 * Required by all plugins.
 */
class SystemStatusPlugin
{
public:
	SystemStatusPlugin(const std::shared_ptr<UAS> &uas, const ros::NodeHandle &nh_input, const ros::NodeHandle &nh_private_input) : // uas added by Peixuan Shu
		nh(nh_input, "mavros"),  // nodehandle modified by Peixuan Shu
		nh_private(nh_private_input),  // nodehandle modified by Peixuan Shu
		// hb_diag("Heartbeat", 10),
		// mem_diag("APM Memory"),
		// hwst_diag("APM Hardware"),
		// sys_diag("System"),
		conn_heartbeat_mav_type(MAV_TYPE_ONBOARD_CONTROLLER),
		version_retries(RETRIES_COUNT),
		disable_diag(false),
		has_battery_status0(false),
		m_uas(uas) // added by Peixuan Shu
	{
		// PluginBase::initialize(uas_);

		ros::WallDuration conn_heartbeat;

		double conn_timeout_d;
		double conn_heartbeat_d;
		std::vector<double> min_voltage;
		std::string conn_heartbeat_mav_type_str;

		nh_private.param("conn/timeout", conn_timeout_d, 10.0);
		nh_private.param("sys/min_voltage", min_voltage, {10.0});
		nh_private.param("sys/disable_diag", disable_diag, false);

		// heartbeat rate parameter
		if (nh_private.getParam("conn/heartbeat_rate", conn_heartbeat_d) && conn_heartbeat_d != 0.0) {
			conn_heartbeat = ros::WallDuration(ros::Rate(conn_heartbeat_d));
		}

		// // heartbeat mav type parameter
		// if (nh_private.getParam("conn/heartbeat_mav_type", conn_heartbeat_mav_type_str)) {
		// 	conn_heartbeat_mav_type = utils::mav_type_from_str(conn_heartbeat_mav_type_str);
		// }

		// // heartbeat diag always enabled
		// UAS_DIAG(m_uas).add(hb_diag);
		// if (!disable_diag) {
		// 	UAS_DIAG(m_uas).add(sys_diag);
		// 	for (size_t i = 0; i < MAX_NR_BATTERY_STATUS && i < min_voltage.size(); ++i) {
		// 		batt_diag[i].set_min_voltage(min_voltage[i]);
		// 		UAS_DIAG(m_uas).add(batt_diag[i]);
		// 	}
		// }


		// // one-shot timeout timer
		// timeout_timer = nh.createWallTimer(ros::WallDuration(conn_timeout_d),
		// 		&SystemStatusPlugin::timeout_cb, this, true);
		// //timeout_timer.start();

		// if (!conn_heartbeat.isZero()) {
		// 	heartbeat_timer = nh.createWallTimer(conn_heartbeat,
		// 			&SystemStatusPlugin::heartbeat_cb, this);
		// 	//heartbeat_timer.start();
		// }

		// // version request timer
		// autopilot_version_timer = nh.createWallTimer(ros::WallDuration(1.0),
		// 		&SystemStatusPlugin::autopilot_version_cb, this);
		// autopilot_version_timer.stop();

		state_pub = nh.advertise<mavros_msgs::State>("state", 10, true);
		extended_state_pub = nh.advertise<mavros_msgs::ExtendedState>("extended_state", 10);
		batt_pub = nh.advertise<BatteryMsg>("battery", 10);
		estimator_status_pub = nh.advertise<mavros_msgs::EstimatorStatus>("estimator_status", 10);
		statustext_pub = nh.advertise<mavros_msgs::StatusText>("statustext/recv", 10);
		// statustext_sub = nh.subscribe("statustext/send", 10, &SystemStatusPlugin::statustext_cb, this);
		// rate_srv = nh.advertiseService("set_stream_rate", &SystemStatusPlugin::set_rate_cb, this);
		mode_srv = nh.advertiseService("set_mode", &SystemStatusPlugin::set_mode_cb, this);
		// vehicle_info_get_srv = nh.advertiseService("vehicle_info_get", &SystemStatusPlugin::vehicle_info_get_cb, this);
		// message_interval_srv = nh.advertiseService("set_message_interval", &SystemStatusPlugin::set_message_interval_cb, this);

		// init state topic
		publish_disconnection();
		// enable_connection_cb();
	}

private:
	std::shared_ptr<UAS> m_uas; // store some common data and functions. Added by Peixuan Shu

	ros::NodeHandle nh;
	ros::NodeHandle nh_private; // nodehandle added by Peixuan Shu

	// HeartbeatStatus hb_diag;
	// MemInfo mem_diag;
	// HwStatus hwst_diag;
	// SystemStatusDiag sys_diag;
	// std::vector<BatteryStatusDiag> batt_diag;
	ros::WallTimer timeout_timer;
	ros::WallTimer heartbeat_timer;
	ros::WallTimer autopilot_version_timer;

	ros::Publisher state_pub;
	ros::Publisher extended_state_pub;
	ros::Publisher batt_pub;
	ros::Publisher estimator_status_pub;
	ros::Publisher statustext_pub;
	ros::Subscriber statustext_sub;
	ros::ServiceServer rate_srv;
	ros::ServiceServer mode_srv;
	ros::ServiceServer vehicle_info_get_srv;
	ros::ServiceServer message_interval_srv;

	MAV_TYPE conn_heartbeat_mav_type;
	static constexpr int RETRIES_COUNT = 6;
	int version_retries;
	bool disable_diag;
	bool has_battery_status0;

	using M_VehicleInfo = std::unordered_map<uint16_t, mavros_msgs::VehicleInfo>;
	M_VehicleInfo vehicles;

	/* -*- mid-level helpers -*- */

	// Get vehicle key for the unordered map containing all vehicles
	inline uint16_t get_vehicle_key(uint8_t sysid,uint8_t compid) {
		return sysid << 8 | compid;
	}

	// Find or create vehicle info
	inline M_VehicleInfo::iterator find_or_create_vehicle_info(uint8_t sysid, uint8_t compid) {
		auto key = get_vehicle_key(sysid, compid);
		M_VehicleInfo::iterator ret = vehicles.find(key);

		if (ret == vehicles.end()) {
			// Not found
			mavros_msgs::VehicleInfo v;
			v.sysid = sysid;
			v.compid = compid;
			v.available_info = 0;

			auto res = vehicles.emplace(key, v);	//-> pair<iterator, bool>
			ret = res.first;
		}

		ROS_ASSERT(ret != vehicles.end());
		return ret;
	}

	/**
	 * Sent STATUSTEXT message to rosout
	 *
	 * @param[in] severity  Levels defined in common.xml
	 */
	void process_statustext_normal(uint8_t severity, std::string &text)
	{
		// using mavlink::common::MAV_SEVERITY;

		switch (severity) {
		// [[[cog:
		// for l1, l2 in (
		//     (('EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR'), 'ERROR'),
		//     (('WARNING', 'NOTICE'), 'WARN'),
		//     (('INFO', ), 'INFO'),
		//     (('DEBUG', ), 'DEBUG')
		//     ):
		//     for v in l1:
		//         cog.outl("case enum_value(MAV_SEVERITY::%s):" % v)
		//     cog.outl("\tROS_%s_STREAM_NAMED(\"fcu\", \"FCU: \" << text);" % l2)
		//     cog.outl("\tbreak;")
		// ]]]
		case MAV_SEVERITY_EMERGENCY:
		case MAV_SEVERITY_ALERT:
		case MAV_SEVERITY_CRITICAL:
		case MAV_SEVERITY_ERROR:
			ROS_ERROR_STREAM_NAMED("fcu", "FCU: " << text);
			break;
		case MAV_SEVERITY_WARNING:
		case MAV_SEVERITY_NOTICE:
			ROS_WARN_STREAM_NAMED("fcu", "FCU: " << text);
			break;
		case MAV_SEVERITY_INFO:
			ROS_INFO_STREAM_NAMED("fcu", "FCU: " << text);
			break;
		case MAV_SEVERITY_DEBUG:
			ROS_DEBUG_STREAM_NAMED("fcu", "FCU: " << text);
			break;
		// [[[end]]] (checksum: 315aa363b5ecb4dda66cc8e1e3d3aa48)
		default:
			ROS_WARN_STREAM_NAMED("fcu", "FCU: UNK(" << +severity << "): " << text);
			break;
		};
	}

	static std::string custom_version_to_hex_string(std::array<uint8_t, 8> &array)
	{
		// should be little-endian
		uint64_t b;
		memcpy(&b, array.data(), sizeof(uint64_t));
		b = le64toh(b);

		return utils::format("%016llx", b);
	}

	// void process_autopilot_version_normal(mavlink::common::msg::AUTOPILOT_VERSION &apv, uint8_t sysid, uint8_t compid)
	// {
	// 	char prefix[16];
	// 	std::snprintf(prefix, sizeof(prefix), "VER: %d.%d", sysid, compid);

	// 	ROS_INFO_NAMED("sys", "%s: Capabilities         0x%016llx", prefix, (long long int)apv.capabilities);
	// 	ROS_INFO_NAMED("sys", "%s: Flight software:     %08x (%s)",
	// 			prefix,
	// 			apv.flight_sw_version,
	// 			custom_version_to_hex_string(apv.flight_custom_version).c_str());
	// 	ROS_INFO_NAMED("sys", "%s: Middleware software: %08x (%s)",
	// 			prefix,
	// 			apv.middleware_sw_version,
	// 			custom_version_to_hex_string(apv.middleware_custom_version).c_str());
	// 	ROS_INFO_NAMED("sys", "%s: OS software:         %08x (%s)",
	// 			prefix,
	// 			apv.os_sw_version,
	// 			custom_version_to_hex_string(apv.os_custom_version).c_str());
	// 	ROS_INFO_NAMED("sys", "%s: Board hardware:      %08x", prefix, apv.board_version);
	// 	ROS_INFO_NAMED("sys", "%s: VID/PID:             %04x:%04x", prefix, apv.vendor_id, apv.product_id);
	// 	ROS_INFO_NAMED("sys", "%s: UID:                 %016llx", prefix, (long long int)apv.uid);
	// }

	// void process_autopilot_version_apm_quirk(mavlink::common::msg::AUTOPILOT_VERSION &apv, uint8_t sysid, uint8_t compid)
	// {
	// 	char prefix[16];
	// 	std::snprintf(prefix, sizeof(prefix), "VER: %d.%d", sysid, compid);

	// 	// Note based on current APM's impl.
	// 	// APM uses custom version array[8] as a string
	// 	ROS_INFO_NAMED("sys", "%s: Capabilities         0x%016llx", prefix, (long long int)apv.capabilities);
	// 	ROS_INFO_NAMED("sys", "%s: Flight software:     %08x (%*s)",
	// 			prefix,
	// 			apv.flight_sw_version,
	// 			8, apv.flight_custom_version.data());
	// 	ROS_INFO_NAMED("sys", "%s: Middleware software: %08x (%*s)",
	// 			prefix,
	// 			apv.middleware_sw_version,
	// 			8, apv.middleware_custom_version.data());
	// 	ROS_INFO_NAMED("sys", "%s: OS software:         %08x (%*s)",
	// 			prefix,
	// 			apv.os_sw_version,
	// 			8, apv.os_custom_version.data());
	// 	ROS_INFO_NAMED("sys", "%s: Board hardware:      %08x", prefix, apv.board_version);
	// 	ROS_INFO_NAMED("sys", "%s: VID/PID:             %04x:%04x", prefix, apv.vendor_id, apv.product_id);
	// 	ROS_INFO_NAMED("sys", "%s: UID:                 %016llx", prefix, (long long int)apv.uid);
	// }

	void publish_disconnection() {
		auto state_msg = boost::make_shared<mavros_msgs::State>();
		state_msg->header.stamp = ros::Time::now();
		state_msg->connected = false;
		state_msg->armed = false;
		state_msg->guided = false;
		state_msg->mode = "";
		state_msg->system_status = MAV_STATE_UNINIT;

		state_pub.publish(state_msg);
	}

public: // public by Peixuan Shu

	/* -*- message handlers -*- */

	void handle_heartbeat(const mavlink_message_t& msg)  // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
	{
		// Decode mavlink message. Added by Peixuan Shu
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(&msg, &hb); // mavlink c lib	

		// using mavlink::minimal::MAV_MODE_FLAG;

		// Store generic info of all heartbeats seen
		auto it = find_or_create_vehicle_info(msg.sysid, msg.compid);

		auto vehicle_mode = m_uas->str_mode_v10(hb.base_mode, hb.custom_mode);
		auto stamp = ros::Time::now();

		// Update vehicle data
		it->second.header.stamp = stamp;
		it->second.available_info |= mavros_msgs::VehicleInfo::HAVE_INFO_HEARTBEAT;
		it->second.autopilot = hb.autopilot;
		it->second.type = hb.type;
		it->second.system_status = hb.system_status;
		it->second.base_mode = hb.base_mode;
		it->second.custom_mode = hb.custom_mode;
		it->second.mode = vehicle_mode;

		if (!(hb.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
			it->second.mode_id = hb.base_mode;
		} else {
			it->second.mode_id = hb.custom_mode;
		}

		// Continue from here only if vehicle is my target
		if (!m_uas->is_my_target(msg.sysid, msg.compid)) {
			ROS_DEBUG_NAMED("sys", "HEARTBEAT from [%d, %d] dropped.", msg.sysid, msg.compid);
			return;
		}

		// // update context && setup connection timeout
		m_uas->update_heartbeat(hb.type, hb.autopilot, hb.base_mode);
		m_uas->update_connection_status(true);
		// timeout_timer.stop();
		// timeout_timer.start();

		// build state message after updating uas
		auto state_msg = boost::make_shared<mavros_msgs::State>();
		state_msg->header.stamp = stamp;
		state_msg->connected = true;
		state_msg->armed = !!(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
		state_msg->guided = !!(hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED);
		state_msg->manual_input = !!(hb.base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
		state_msg->mode = vehicle_mode;
		state_msg->system_status = hb.system_status;

		state_pub.publish(state_msg);
		// hb_diag.tick(hb.type, hb.autopilot, state_msg->mode, hb.system_status);
	}

	void handle_extended_sys_state(const mavlink_message_t& msg)  // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
	{
		// Decode mavlink message. Added by Peixuan Shu
		mavlink_extended_sys_state_t state;
		mavlink_msg_extended_sys_state_decode(&msg, &state); // mavlink c lib	

		auto state_msg = boost::make_shared<mavros_msgs::ExtendedState>();
		state_msg->header.stamp = ros::Time::now();
		state_msg->vtol_state = state.vtol_state;
		state_msg->landed_state = state.landed_state;

		extended_state_pub.publish(state_msg);
	}

	void handle_sys_status(const mavlink_message_t& msg)  // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
	{
		// Decode mavlink message. Added by Peixuan Shu
		mavlink_sys_status_t stat;
		mavlink_msg_sys_status_decode(&msg, &stat); // mavlink c lib		

		// using MC = mavlink::minimal::MAV_COMPONENT;
		if (static_cast<MAV_COMPONENT>(msg.compid) == MAV_COMP_ID_GIMBAL) {
			return;
		}

		float volt = stat.voltage_battery * 0.001f;	// mV
		float curr = stat.current_battery * 0.01f;	// 10 mA or -1
		float rem = stat.battery_remaining * 0.01f;	// or -1

		// sys_diag.set(stat);

		if (has_battery_status0)
			return;

		// batt_diag[0].set(volt, curr, rem);
		auto batt_msg = boost::make_shared<BatteryMsg>();
		batt_msg->header.stamp = ros::Time::now();

#ifdef HAVE_SENSOR_MSGS_BATTERYSTATE_MSG
		batt_msg->voltage = volt;
		batt_msg->current = -curr;
		batt_msg->charge = NAN;
		batt_msg->capacity = NAN;
		batt_msg->design_capacity = NAN;
		batt_msg->percentage = rem;
		batt_msg->power_supply_status = BatteryMsg::POWER_SUPPLY_STATUS_DISCHARGING;
		batt_msg->power_supply_health = BatteryMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
		batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		batt_msg->present = true;
		batt_msg->cell_voltage.clear();	// not necessary. Cell count and Voltage unknown.
		batt_msg->location = "";
		batt_msg->serial_number = "";
#else	// mavros_msgs::BatteryStatus
		batt_msg->voltage = volt;
		batt_msg->current = curr;
		batt_msg->remaining = rem;
#endif

		batt_pub.publish(batt_msg);
	}

	void handle_statustext(const mavlink_message_t& msg)  // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
	{
		// Decode mavlink message. Added by Peixuan Shu
		mavlink_statustext_t textm;
		mavlink_msg_statustext_decode(&msg, &textm); // mavlink c lib	

		// auto text = mavlink::to_string(textm.text);
		std::string text = textm.text; // modified by Peixuan Shu
		process_statustext_normal(textm.severity, text);

		auto st_msg = boost::make_shared<mavros_msgs::StatusText>();
		st_msg->header.stamp = ros::Time::now();
		st_msg->severity = textm.severity;
		st_msg->text = text;
		statustext_pub.publish(st_msg);
	}

	// void handle_meminfo(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::MEMINFO &mem)
	// {
	// 	mem_diag.set(std::max(static_cast<uint32_t>(mem.freemem), mem.freemem32), mem.brkval);
	// }

	// void handle_hwstatus(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::HWSTATUS &hwst)
	// {
	// 	hwst_diag.set(hwst.Vcc, hwst.I2Cerr);
	// }

	// void handle_autopilot_version(const mavlink_message_t& msg)  // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
	// {
	// 	// Decode mavlink message. Added by Peixuan Shu
	// 	mavlink_autopilot_version_t apv;
	// 	mavlink_msg_autopilot_version_decode(&msg, &apv); // mavlink c lib		

	// 	// we want to store only FCU caps
	// 	if (m_uas->is_my_target(msg.sysid, msg.compid)) {
	// 		autopilot_version_timer.stop();
	// 		m_uas->update_capabilities(true, apv.capabilities);
	// 	}

	// 	// but print all version responses
	// 	if (m_uas->is_ardupilotmega())
	// 		process_autopilot_version_apm_quirk(apv, msg.sysid, msg.compid);
	// 	else
	// 		process_autopilot_version_normal(apv, msg.sysid, msg.compid);

	// 	// Store generic info of all autopilot seen
	// 	auto it = find_or_create_vehicle_info(msg.sysid, msg.compid);

	// 	// Update vehicle data
	// 	it->second.header.stamp = ros::Time::now();
	// 	it->second.available_info |= mavros_msgs::VehicleInfo::HAVE_INFO_AUTOPILOT_VERSION;
	// 	it->second.capabilities = apv.capabilities;
	// 	it->second.flight_sw_version = apv.flight_sw_version;
	// 	it->second.middleware_sw_version = apv.middleware_sw_version;
	// 	it->second.os_sw_version = apv.os_sw_version;
	// 	it->second.board_version = apv.board_version;
	// 	it->second.flight_custom_version = custom_version_to_hex_string(apv.flight_custom_version);
	// 	it->second.vendor_id = apv.vendor_id;
	// 	it->second.product_id = apv.product_id;
	// 	it->second.uid = apv.uid;
	// }

// 	void handle_battery_status(const mavlink::mavlink_message_t *msg, mavlink::common::msg::BATTERY_STATUS &bs)
// 	{
// #ifdef HAVE_SENSOR_MSGS_BATTERYSTATE_MSG
// 		using BT = mavlink::common::MAV_BATTERY_TYPE;
// 		auto batt_msg = boost::make_shared<BatteryMsg>();
// 		batt_msg->header.stamp = ros::Time::now();

// 		batt_msg->current = bs.current_battery==-1?NAN:-(bs.current_battery * 0.01f);	// 10 mA
// 		batt_msg->charge = NAN;
// 		batt_msg->capacity = NAN;
// 		batt_msg->design_capacity = NAN;
// 		batt_msg->percentage = bs.battery_remaining * 0.01f;
// 		batt_msg->power_supply_status = BatteryMsg::POWER_SUPPLY_STATUS_DISCHARGING;
// 		batt_msg->power_supply_health = BatteryMsg::POWER_SUPPLY_HEALTH_UNKNOWN;

// 		switch (bs.type) {
// 		// [[[cog:
// 		// for f in (
// 		//     'LIPO',
// 		//     'LIFE',
// 		//     'LION',
// 		//     'NIMH',
// 		//     'UNKNOWN'):
// 		//     cog.outl("case enum_value(BT::%s):" % f)
// 		//     if f == 'UNKNOWN':
// 		//         cog.outl("default:")
// 		//     cog.outl("\tbatt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_%s;" % f)
// 		//     cog.outl("\tbreak;")
// 		// ]]]
// 		case enum_value(BT::LIPO):
// 			batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_LIPO;
// 			break;
// 		case enum_value(BT::LIFE):
// 			batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_LIFE;
// 			break;
// 		case enum_value(BT::LION):
// 			batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_LION;
// 			break;
// 		case enum_value(BT::NIMH):
// 			batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_NIMH;
// 			break;
// 		case enum_value(BT::UNKNOWN):
// 		default:
// 			batt_msg->power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
// 			break;
// 		// [[[end]]] (checksum: 2bf14a81b3027f14ba1dd9b4c086a41d)
// 		}

// 		batt_msg->present = true;

// 		batt_msg->cell_voltage.clear();
// 		batt_msg->cell_voltage.reserve(bs.voltages.size() + bs.voltages_ext.size());
// 		float cell_voltage;
// 		float voltage_acc = 0.0f;
// 		float total_voltage = 0.0f;
// 		constexpr float coalesce_voltage = (UINT16_MAX-1) * 0.001f; // 65,534V cell voltage means that the next element in the array must be added to this one
// 		for (auto v : bs.voltages) {
// 			if (v == UINT16_MAX)
// 				break;

// 			if (v == UINT16_MAX-1) // cell voltage is above 65,534V
// 			{
// 				voltage_acc += coalesce_voltage;
// 				continue;          // add to the next array element to get the correct voltage
// 			}
// 			cell_voltage = voltage_acc + (v * 0.001f);	// 1 mV
// 			voltage_acc = 0.0f;
// 			batt_msg->cell_voltage.push_back(cell_voltage);
// 			total_voltage += cell_voltage;
// 		}
// 		for (auto v : bs.voltages_ext) {
// 			if (v == UINT16_MAX || v == 0)  // this one is different from the for loop above to support mavlink2 message truncation
// 				break;

// 			if (v == UINT16_MAX-1) // cell voltage is above 65,534V
// 			{
// 				voltage_acc += coalesce_voltage;
// 				continue;          // add to the next array element to get the correct voltage
// 			}
// 			cell_voltage = voltage_acc + (v * 0.001f);	// 1 mV
// 			voltage_acc = 0.0f;
// 			batt_msg->cell_voltage.push_back(cell_voltage);
// 			total_voltage += cell_voltage;
// 		}
// 		batt_msg->voltage = total_voltage;

// 		batt_msg->location = utils::format("id%u", bs.id);
// 		batt_msg->serial_number = "";
// 		batt_pub.publish(batt_msg);

// 		if (bs.id == 0) {
// 			has_battery_status0 = true;
// 		}

// 		if (!disable_diag && bs.id >= 0 && bs.id < MAX_NR_BATTERY_STATUS) {
// 			batt_diag[bs.id].set(total_voltage, batt_msg->current, batt_msg->percentage);
// 			batt_diag[bs.id].setcell_v(batt_msg->cell_voltage);
// 		}
// #endif
// 	}

	void handle_estimator_status(const mavlink_message_t& msg)  // Change from mavlink cpp headers to mavlink c headers by Peixuan Shu
	{
		// Decode mavlink message. Added by Peixuan Shu
		mavlink_estimator_status_t status;
		mavlink_msg_estimator_status_decode(&msg, &status); // mavlink c lib		

		// using ESF = mavlink::common::ESTIMATOR_STATUS_FLAGS;

		auto est_status_msg = boost::make_shared<mavros_msgs::EstimatorStatus>();
		est_status_msg->header.stamp = ros::Time::now();

		// [[[cog:
		// import pymavlink.dialects.v20.common as common
		// ename = 'ESTIMATOR_STATUS_FLAGS'
		// ename_pfx2 = 'ESTIMATOR_'
		//
		// enum = sorted(common.enums[ename].items())
		// enum.pop() # -> remove ENUM_END
		//
		// for k, e in enum:
		//     desc = e.description.split(' ', 1)[1] if e.description.startswith('0x') else e.description
		//     esf = e.name
		//
		//     if esf.startswith(ename + '_'):
		//         esf = esf[len(ename) + 1:]
		//     if esf.startswith(ename_pfx2):
		//         esf = esf[len(ename_pfx2):]
		//     if esf[0].isdigit():
		//         esf = 'SENSOR_' + esf
		//     cog.outl("est_status_msg->%s_status_flag = !!(status.flags & enum_value(ESF::%s));" % (esf.lower(), esf))
		// ]]]
		est_status_msg->attitude_status_flag = !!(status.flags & ESTIMATOR_ATTITUDE);
		est_status_msg->velocity_horiz_status_flag = !!(status.flags & ESTIMATOR_VELOCITY_HORIZ);
		est_status_msg->velocity_vert_status_flag = !!(status.flags & ESTIMATOR_VELOCITY_VERT);
		est_status_msg->pos_horiz_rel_status_flag = !!(status.flags & ESTIMATOR_POS_HORIZ_REL);
		est_status_msg->pos_horiz_abs_status_flag = !!(status.flags & ESTIMATOR_POS_HORIZ_ABS);
		est_status_msg->pos_vert_abs_status_flag = !!(status.flags & ESTIMATOR_POS_VERT_ABS);
		est_status_msg->pos_vert_agl_status_flag = !!(status.flags & ESTIMATOR_POS_VERT_AGL);
		est_status_msg->const_pos_mode_status_flag = !!(status.flags & ESTIMATOR_CONST_POS_MODE);
		est_status_msg->pred_pos_horiz_rel_status_flag = !!(status.flags & ESTIMATOR_PRED_POS_HORIZ_REL);
		est_status_msg->pred_pos_horiz_abs_status_flag = !!(status.flags & ESTIMATOR_PRED_POS_HORIZ_ABS);
		est_status_msg->gps_glitch_status_flag = !!(status.flags & ESTIMATOR_GPS_GLITCH);
		est_status_msg->accel_error_status_flag = !!(status.flags & ESTIMATOR_ACCEL_ERROR);
		// [[[end]]] (checksum: da59238f4d4337aeb395f7205db08237)

		estimator_status_pub.publish(est_status_msg);
	}

private:
	/* -*- timer callbacks -*- */

	// void timeout_cb(const ros::WallTimerEvent &event)
	// {
	// 	m_uas->update_connection_status(false);
	// }

	// void heartbeat_cb(const ros::WallTimerEvent &event)
	// {
	// 	using mavlink::common::MAV_MODE;

	// 	mavlink::minimal::msg::HEARTBEAT hb {};

	// 	hb.type = enum_value(conn_heartbeat_mav_type); //! @todo patch PX4 so it can also handle this type as datalink
	// 	hb.autopilot = enum_value(MAV_AUTOPILOT::INVALID);
	// 	hb.base_mode = enum_value(MAV_MODE::MANUAL_ARMED);
	// 	hb.custom_mode = 0;
	// 	hb.system_status = enum_value(MAV_STATE::ACTIVE);

	// 	UAS_FCU(m_uas)->send_message_ignore_drop(hb);
	// }

	// void autopilot_version_cb(const ros::WallTimerEvent &event)
	// {
	// 	using mavlink::common::MAV_CMD;

	// 	bool ret = false;

	// 	// Request from all first 3 times, then fallback to unicast
	// 	bool do_broadcast = version_retries > RETRIES_COUNT / 2;

	// 	try {
	// 		auto client = nh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

	// 		mavros_msgs::CommandLong cmd{};

	// 		cmd.request.broadcast = do_broadcast;
	// 		cmd.request.command = enum_value(MAV_CMD::REQUEST_AUTOPILOT_CAPABILITIES);
	// 		cmd.request.confirmation = false;
	// 		cmd.request.param1 = 1.0;

	// 		ROS_DEBUG_NAMED("sys", "VER: Sending %s request.",
	// 				(do_broadcast) ? "broadcast" : "unicast");
	// 		ret = client.call(cmd);
	// 	}
	// 	catch (ros::InvalidNameException &ex) {
	// 		ROS_ERROR_NAMED("sys", "VER: %s", ex.what());
	// 	}

	// 	ROS_ERROR_COND_NAMED(!ret, "sys", "VER: command plugin service call failed!");

	// 	if (version_retries > 0) {
	// 		version_retries--;
	// 		ROS_WARN_COND_NAMED(version_retries != RETRIES_COUNT - 1, "sys",
	// 				"VER: %s request timeout, retries left %d",
	// 				(do_broadcast) ? "broadcast" : "unicast",
	// 				version_retries);
	// 	}
	// 	else {
	// 		m_uas->update_capabilities(false);
	// 		autopilot_version_timer.stop();
	// 		ROS_WARN_NAMED("sys", "VER: your FCU don't support AUTOPILOT_VERSION, "
	// 				"switched to default capabilities");
	// 	}
	// }

	// void connection_cb(bool connected) override
	// {
	// 	has_battery_status0 = false;

	// 	// if connection changes, start delayed version request
	// 	version_retries = RETRIES_COUNT;
	// 	if (connected)
	// 		autopilot_version_timer.start();
	// 	else
	// 		autopilot_version_timer.stop();

	// 	// add/remove APM diag tasks
	// 	if (connected && !disable_diag && m_uas->is_ardupilotmega()) {
	// 		UAS_DIAG(m_uas).add(mem_diag);
	// 		UAS_DIAG(m_uas).add(hwst_diag);
	// 	}
	// 	else {
	// 		UAS_DIAG(m_uas).removeByName(mem_diag.getName());
	// 		UAS_DIAG(m_uas).removeByName(hwst_diag.getName());
	// 	}

	// 	if (!connected) {
	// 		// publish connection change
	// 		publish_disconnection();

	// 		// Clear known vehicles
	// 		vehicles.clear();
	// 	}
	// }

	/* -*- subscription callbacks -*- */

	// void statustext_cb(const mavros_msgs::StatusText::ConstPtr &req) {
	// 	mavlink::common::msg::STATUSTEXT statustext {};
	// 	statustext.severity = req->severity;

	// 	// Limit the length of the string by null-terminating at the 50-th character
	// 	ROS_WARN_COND_NAMED(req->text.length() >= statustext.text.size(), "sys",
	// 			"Status text too long: truncating...");
	// 	mavlink::set_string_z(statustext.text, req->text);

	// 	UAS_FCU(m_uas)->send_message_ignore_drop(statustext);
	// }

	/* -*- ros callbacks -*- */

	// bool set_rate_cb(mavros_msgs::StreamRate::Request &req,
	// 		mavros_msgs::StreamRate::Response &res)
	// {
	// 	mavlink::common::msg::REQUEST_DATA_STREAM rq = {};

	// 	rq.target_system = m_uas->get_tgt_system();
	// 	rq.target_component = m_uas->get_tgt_component();
	// 	rq.req_stream_id = req.stream_id;
	// 	rq.req_message_rate = req.message_rate;
	// 	rq.start_stop = (req.on_off) ? 1 : 0;

	// 	UAS_FCU(m_uas)->send_message_ignore_drop(rq);
	// 	return true;
	// }

	bool set_mode_cb(mavros_msgs::SetMode::Request &req,
			mavros_msgs::SetMode::Response &res)
	{
		// using mavlink::minimal::MAV_MODE_FLAG;

		uint8_t base_mode = req.base_mode;
		uint32_t custom_mode = 0;

		if (req.custom_mode != "") {
			if (!m_uas->cmode_from_str(req.custom_mode, custom_mode)) {
				res.mode_sent = false;
				return true;
			}

			/**
			 * @note That call may trigger unexpected arming change because
			 *       base_mode arming flag state based on previous HEARTBEAT
			 *       message value.
			 */
			base_mode |= (m_uas->get_armed()) ? MAV_MODE_FLAG_SAFETY_ARMED : 0;
			base_mode |= (m_uas->get_hil_state()) ? MAV_MODE_FLAG_HIL_ENABLED : 0;
			base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
		}

		// mavlink::common::msg::SET_MODE sm = {};
		mavlink_set_mode_t sm {}; // modified to mavlink c headers by Peixuan Shu
		sm.target_system = m_uas->get_tgt_system();
		sm.base_mode = base_mode;
		sm.custom_mode = custom_mode;

		// UAS_FCU(m_uas)->send_message_ignore_drop(sm);

		/*  Added by Peixuan Shu. Write mavlink messages into "px4_modules/mavlink/mavlink_msg_list.hpp" */
		int handle = (int) px4::mavlink_receive_handle::SET_MODE;
		mavlink_msg_set_mode_encode(1, 1, &px4::mavlink_receive_list[handle].msg, &sm); 
		px4::mavlink_receive_list[handle].updated = true;

		res.mode_sent = true;
		return true;
	}

	// bool vehicle_info_get_cb(mavros_msgs::VehicleInfoGet::Request &req,
	// 		mavros_msgs::VehicleInfoGet::Response &res)
	// {
	// 	if (req.get_all) {
	// 		// Send all vehicles
	// 		for (const auto &got : vehicles) {
	// 			res.vehicles.emplace_back(got.second);
	// 		}

	// 		res.success = true;
	// 		return res.success;
	// 	}

	// 	uint8_t req_sysid = req.sysid;
	// 	uint8_t req_compid = req.compid;

	// 	if (req.sysid == 0 && req.compid == 0) {
	// 		// use target
	// 		req_sysid = m_uas->get_tgt_system();
	// 		req_compid = m_uas->get_tgt_component();
	// 	}

	// 	uint16_t key = get_vehicle_key(req_sysid, req_compid);
	// 	auto it = vehicles.find(key);

	// 	if (it == vehicles.end()) {
	// 		// Vehicle not found
	// 		res.success = false;
	// 		return res.success;
	// 	}

	// 	res.vehicles.emplace_back(it->second);
	// 	res.success = true;
	// 	return res.success;
	// }

    // bool set_message_interval_cb(mavros_msgs::MessageInterval::Request &req,
    //         mavros_msgs::MessageInterval::Response &res)
    // {
    //     using mavlink::common::MAV_CMD;

    //     try {
    //         auto client = nh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

    //         // calculate interval
    //         float interval_us;
    //         if (req.message_rate < 0) {
    //             interval_us = -1.0f;
    //         } else if (req.message_rate == 0) {
    //             interval_us = 0.0f;
    //         } else {
    //             interval_us = 1000000.0f / req.message_rate;
    //         }

    //         mavros_msgs::CommandLong cmd{};

    //         cmd.request.broadcast = false;
    //         cmd.request.command = enum_value(MAV_CMD::SET_MESSAGE_INTERVAL);
    //         cmd.request.confirmation = false;
    //         cmd.request.param1 = req.message_id;
    //         cmd.request.param2 = interval_us;

    //         ROS_DEBUG_NAMED("sys", "SetMessageInterval: Request msgid %u at %f hz",
    //                 req.message_id, req.message_rate);
    //         res.success = client.call(cmd);
    //     }
    //     catch (ros::InvalidNameException &ex) {
    //         ROS_ERROR_NAMED("sys", "SetMessageInterval: %s", ex.what());
    //     }

    //     ROS_ERROR_COND_NAMED(!res.success, "sys", "SetMessageInterval: command plugin service call failed!");

    //     return res.success;
    // }
};
}	// namespace std_plugins
}	// namespace mavros

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SystemStatusPlugin, mavros::plugin::PluginBase)
