/**
 * @file px4_sitl.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief PX4 main function. Communicate with mavros and dynamics.
 * Mavros <---> PX4_SITL <---> Quadrotor Dynamics
 * 
 * Note: This program relies on mavlink, px4_modules, quadrotor_dynamics
 * 
 * @version 1.0
 * @date 2023-12-11
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */


#ifndef __PX4_SITL_H__
#define __PX4_SITL_H__

#include <ros/ros.h>
#include <iostream> // for std::cout, std::endl
#include <memory>  // for std::shared_ptr

#include <mavlink/v2.0/common/mavlink.h> // from ros-noetic-mavlink or include/ folder

#include <parameters/px4_parameters.hpp> // store all extern(global) px4 parameters
#include <matrix/matrix/math.hpp> // for px4 geometry utils
#include <geo/geo.h> // for px4 geo utils (gps to ENU)
#include <hysteresis/hysteresis.h> // for px4 hysteresis utils
#include <mavros/frame_tf.h> // for mavros::ftf frame conversion
#include <uORB/uORB_sim.hpp> // simulate uORB publication and subscription. Store the extern(global) simulated uORB messages

#include "px4_modules/mavlink/mavlink_receiver.h"
#include "px4_modules/mavlink/mavlink_streamer.hpp"
#include "px4_modules/commander/Commander.hpp"
#include "px4_modules/mc_pos_control/MulticopterPositionControl.hpp"
#include "px4_modules/mc_att_control/mc_att_control.hpp"
#include "px4_modules/px4_lib/drivers/drv_hrt.h"  // time utils, store the extern(global) simulated time
#include "px4_modules/mavlink/mavlink_msg_list.hpp" // store the simulated extern(global) mavlink messages

#include "mavros_px4_quadrotor_sim/quadrotor_dynamics.hpp"


namespace MavrosQuadSimulator
{

/**
 * \brief PX4 flight stack simulation. Communicate with mavros and dynamics.
 * Mavros <---> PX4_SITL <---> Quadrotor Dynamics
 */
template <int N>  /* seperate static messages for UAV N */
class PX4SITL
{
public:
    PX4SITL(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, const std::shared_ptr<Dynamics> &dynamics);

    /* Load px4 parameters from ROS parameter space to override the default values from <parameters/px4_parameters.hpp>*/    
    void load_px4_params_from_ros_params();

    /**
     * \brief Run px4 modules for one step. Run pos and att controller to calculate control output
     * @param time_us The microseconds (us) now.
     */
    void Run(const uint64_t &time_us);

    /* Get px4 params from px4::parameters */
    template <px4::params p>
    void get_px4_param(float& output);

    /* Get px4 params from px4::parameters */
    template <px4::params p>
    void get_px4_param(int32_t& output);

    /* Get px4 params from px4::parameters */
    template <px4::params p>
    void get_px4_param(bool& output);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    enum class position_mode : uint32_t {
        MOCAP,
        GPS
    };
    position_mode local_pos_source_;

    float init_x_East_metre_;
    float init_y_North_metre_;
    float init_z_Up_metre_;

    double world_origin_lat_; // Reference point latitude (degrees). The world (0,0,0) point
    double world_origin_lon_; // Reference point longitude (degrees). The world (0,0,0) point
    float world_origin_asml_alt_; // Reference altitude AMSL (metres). The world (0,0,0) point

    double init_lat_; // latitude at initial point
    double init_lon_; // longitude at initial point

    bool pos_inited_; // if the LoadInitPos() is called

	// publications with topic
	uORB_sim::Publication<vehicle_attitude_s>           _attitude_pub {ORB_ID(vehicle_attitude)};
	uORB_sim::Publication<vehicle_local_position_s>     _local_position_pub{ORB_ID(vehicle_local_position)};
    uORB_sim::Publication<vehicle_angular_velocity_s> _vehicle_angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};
    uORB_sim::Publication<battery_status_s> _battery_status_pub{ORB_ID(battery_status)};
	uORB_sim::Publication<vehicle_global_position_s>    _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB_sim::Publication<vehicle_odometry_s>           _odometry_pub{ORB_ID(vehicle_odometry)};
    uORB_sim::Publication<vehicle_land_detected_s>      _vehicle_land_detected_pub{ORB_ID(vehicle_land_detected)};

    uORB_sim::Subscription<vehicle_command_s>           _vehicle_command_sub{ORB_ID(vehicle_command)};
    uORB_sim::Subscription<vehicle_rates_setpoint_s>           _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
    uORB_sim::Subscription<vehicle_local_position_s>           _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB_sim::Subscription<vehicle_control_mode_s>           _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
    

    std::shared_ptr<MavlinkReceiver<N> > mavlink_receiver_;
    std::shared_ptr<MavlinkStreamer<N> > mavlink_streamer_;
    std::shared_ptr<Commander<N> > commander_;
    std::shared_ptr<MulticopterPositionControl<N> > mc_pos_control_; 
    std::shared_ptr<MulticopterAttitudeControl<N> > mc_att_control_; 

    std::shared_ptr<Dynamics> uav_dynamics_;

    /* load initial position and local_pos_source */
    void update_init_pos_from_dynamics();

    /**
     * \brief Update px4 estimator uorb states (pos/vel/acc/att, etc.) from UAV dynamical model 
     * @param time_us The microseconds (us) now.
     */
    void UpdateDroneStates(const uint64_t &time_us);

    /* Land detector module. Update vehicle_land_detected uORB message*/
    void DectectLand(const uint64_t &time_us);

	/* Search for mavlink receiving list and handle the updated messages (transfer into PX4 uORB messages) */
	void ReceiveMavlink();

    /* Stream mavlink messages into "px4_modules/mavlink/mavlink_msg_list.hpp" at a given frequency */
    void StreamMavlink(const uint64_t &time_us);

    /* Send control input calculated by the controller to the quadrotor dynamics */
    void SendControlInput();


};



}

// #include "px4_sitl.cpp"

#endif