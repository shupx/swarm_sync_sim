/**
 * @file px4_uorb_lists.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Store the PX4 global uorb messages to simulate the real PX4 uORB messages. 
 * 
 * Note: This program relies on px4_lib/uORB/
 * 
 * @version 1.0
 * @date 2023-12-14
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __UORB_SIM_HPP__
#define __UORB_SIM_HPP__

#include <px4_platform_common/defines.h>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

#include <iostream> // for std::cout, std::endl

#ifdef ORB_ID
    #error [uORB_sim.hpp] Wrong include order: do not include original uORB.h of PX4 as it was rewritten in uORB_sim.hpp.
#endif

#ifndef ORB_ID
#define ORB_ID(x) uORB_sim::x 
#endif

namespace uORB_sim { 

// store all px4 uORB messages (declaring global)
extern actuator_armed_s actuator_armed;
extern autotune_attitude_control_status_s autotune_attitude_control_status;
extern battery_status_s battery_status;
extern commander_state_s commander_state;
extern cpuload_s cpuload;
extern home_position_s home_position;
extern manual_control_setpoint_s manual_control_setpoint;
extern takeoff_status_s takeoff_status;
extern offboard_control_mode_s offboard_control_mode;
extern vehicle_angular_velocity_s vehicle_angular_velocity;
extern vehicle_attitude_s vehicle_attitude;
extern vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
extern vehicle_command_s vehicle_command;
extern vehicle_command_ack_s vehicle_command_ack;
extern vehicle_constraints_s vehicle_constraints;
extern vehicle_control_mode_s vehicle_control_mode;
extern vehicle_global_position_s vehicle_global_position;
extern vehicle_gps_position_s vehicle_gps_position;
extern vehicle_land_detected_s vehicle_land_detected;
extern vehicle_local_position_s vehicle_local_position;
extern vehicle_local_position_setpoint_s vehicle_local_position_setpoint; // real target setpoints
extern vehicle_local_position_setpoint_s trajectory_setpoint; // mavlink setpoints
extern vehicle_odometry_s vehicle_odometry;
extern vehicle_rates_setpoint_s vehicle_rates_setpoint;
extern vehicle_status_s vehicle_status;
extern vehicle_status_flags_s vehicle_status_flags;

// Base uORB subscription wrapper class
template<typename T>
class Subscription
{
    public:
        Subscription(T& uorb_msg): global_uorb_msg_(&uorb_msg) 
        {
            last_uorb_msg_ = uorb_msg;
        }

        //@TODO: check if global_uorb_msg_ is updated and not null
         // compare it with \0\0\0 is invalid actually
        bool valid() {return !(memcmp(global_uorb_msg_, "\0\0\0", sizeof(*global_uorb_msg_)) == 0);}

	    bool update(void *dst)
        {
            if (valid()){
                memcpy(dst, global_uorb_msg_, sizeof(*global_uorb_msg_));
                return true;
            }
            else{
                std::cout << "uorb not initialized" << std::endl;
                return false;
            }
        }
        /**
         * Check if there is a new update. (Modified by Peixuan Shu)
         */
	    bool updated() 
        {
            T now_uorb_msg_;
            update(&now_uorb_msg_);
            /* compare to decide if the uorb message is updated */
            //@TODO Using memcmp to compare struct may cause problems due to random struct byte alignance
            if (memcmp(&now_uorb_msg_, &last_uorb_msg_, sizeof(last_uorb_msg_)) != 0)
            {
                last_uorb_msg_ = now_uorb_msg_;
                return true;
            }
            else
            {
                return false;
            }
            // return valid();
        }

        /**
         * Copy the struct
         * @param dst The uORB message struct we are updating.
         */
        bool copy(void *dst) { return update(dst);}

    private:
        T* global_uorb_msg_; // the pointer of the global uorb message in this file
        T last_uorb_msg_; // to decide whether data updates
};

// Subscription wrapper class with data
template<class T>
class SubscriptionData : public Subscription<T>
{
public:
	/**
	 * Constructor
	 *
	 * @param id The uORB metadata ORB_ID enum for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionData(T& uorb_msg) : Subscription<T>(uorb_msg)
	{
		Subscription<T>::copy(&_data);
	}

	~SubscriptionData() = default;

	// no copy, assignment, move, move assignment
	SubscriptionData(const SubscriptionData &) = delete;
	SubscriptionData &operator=(const SubscriptionData &) = delete;
	SubscriptionData(SubscriptionData &&) = delete;
	SubscriptionData &operator=(SubscriptionData &&) = delete;

	// update the embedded struct.
	bool update() { return Subscription<T>::update((void *)(&_data)); }

	const T &get() const { return _data; }

private:

	T _data{};
};


/**
 * uORB publication wrapper class
 */
template<typename T>
class Publication
{
    public:
        Publication(T& uorb_msg): global_uorb_msg_(uorb_msg) {}
        bool advertised() const { return true; }
        bool advertise() { return advertised(); }
        /**
         * Publish the struct
         * @param data The uORB message struct we are updating.
         */
        bool publish(const T &data)
        {
            if (!advertised()) {
                advertise();
            }
            global_uorb_msg_ = data;
            return (PX4_OK);
        }
    private:
        T &global_uorb_msg_; // & denotes Reference passing
};

/**
 * The publication class with data embedded.
 */
template<typename T>
class PublicationData : public Publication<T>
{
public:
	PublicationData(T& uorb_msg) : Publication<T>(uorb_msg) {}

	T	&get() { return _data; }
	void	set(const T &data) { _data = data; }

	// Publishes the embedded struct.
	bool	update() { return Publication<T>::publish(_data); }
	bool	update(const T &data)
	{
		_data = data;
		return Publication<T>::publish(_data);
	}

private:
	T _data{};
};

}

/* Diverse uORB header defines */ //XXX: move to better location
#define ORB_ID_VEHICLE_ATTITUDE_CONTROLS    ORB_ID(actuator_controls_0)
typedef uint8_t arming_state_t;
typedef uint8_t main_state_t;
typedef uint8_t hil_state_t;
typedef uint8_t navigation_state_t;
typedef uint8_t switch_pos_t;

#endif
