/**
 * @file px4_uorb_lists.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Store the PX4 SITL global uorb messages.
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


#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>


namespace uORB_sim { 

static takeoff_status_s takeoff_status;
static vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
static vehicle_constraints_s vehicle_constraints;
static vehicle_control_mode_s vehicle_control_mode;
static vehicle_land_detected_s vehicle_land_detected;
static vehicle_local_position_s vehicle_local_position;
static vehicle_local_position_setpoint_s vehicle_local_position_setpoint;
static vehicle_local_position_setpoint_s trajectory_setpoint;

template<typename T>
class Subscription
{
    public:
        Subscription(const T& uorb_msg): global_uorb_msg_(uorb_msg)
        {
        }

	    bool update(void *dst)
        {
            memcpy(dst, &global_uorb_msg_, sizeof(global_uorb_msg_));
			return true;
        }

    private:
        T global_uorb_msg_;
};

}