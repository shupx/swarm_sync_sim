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

#pragma once

#include <px4_platform_common/defines.h>

#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <iostream> // for std::cout, std::endl

#ifndef ORB_ID
#define ORB_ID(x) uORB_sim::x 
#endif

namespace uORB_sim { 

// TODO: initialize if no publisher exists?
// store all px4 uORB messages
static autotune_attitude_control_status_s autotune_attitude_control_status;
static home_position_s home_position;
static manual_control_setpoint_s manual_control_setpoint;
static takeoff_status_s takeoff_status;
static offboard_control_mode_s offboard_control_mode;
static vehicle_angular_velocity_s vehicle_angular_velocity;
static vehicle_attitude_s vehicle_attitude;
static vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
static vehicle_constraints_s vehicle_constraints;
static vehicle_control_mode_s vehicle_control_mode;
static vehicle_global_position_s vehicle_global_position;
static vehicle_land_detected_s vehicle_land_detected;
static vehicle_local_position_s vehicle_local_position;
static vehicle_local_position_setpoint_s vehicle_local_position_setpoint;
static vehicle_local_position_setpoint_s trajectory_setpoint;
static vehicle_odometry_s vehicle_odometry;
static vehicle_rates_setpoint_s vehicle_rates_setpoint;
static vehicle_status_s vehicle_status;

// Base uORB subscription wrapper class
template<typename T>
class Subscription
{
    public:
        Subscription(T& uorb_msg): global_uorb_msg_(&uorb_msg) {}
        bool valid() {return !memcmp(&global_uorb_msg_, "\0\0\0", sizeof(global_uorb_msg_)) == 0;}
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
         * Check if there is a new update.
         */
	    bool updated() {return valid();}

        /**
         * Copy the struct
         * @param dst The uORB message struct we are updating.
         */
        bool copy(void *dst) { return update(dst);}

    private:
        T* global_uorb_msg_; // the pointer of the global uorb message in this file
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