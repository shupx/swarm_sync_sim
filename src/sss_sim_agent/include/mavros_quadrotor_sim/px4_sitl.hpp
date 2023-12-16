/**
 * @file px4_sitl.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief PX4 main function. Communicate with mavros and dynamics.
 * Mavros <---> PX4_SITL <---> Quadrotor Dynamics
 * 
 * Note: This program relies on mavlink, px4_modules
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

#include <mavlink/v2.0/common/mavlink.h> // from ros-noetic-mavlink
#include <parameters/px4_parameters.hpp> // store all px4 parameters

#include "px4_modules/mavlink/mavlink_receiver.h"
#include "px4_modules/mc_pos_control/MulticopterPositionControl.hpp"
#include "px4_modules/mc_att_control/mc_att_control.hpp"


namespace MavrosQuadSimulator
{

/**
 * \brief PX4 flight stack simulation. Communicate with mavros and dynamics.
 * Mavros <---> PX4_SITL <---> Quadrotor Dynamics
 */
class PX4SITL
{
public:
    PX4SITL(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    /* Load px4 parameters from ROS parameter space to override the default values from <parameters/px4_parameters.hpp>*/    
    void load_px4_params_from_ros_params();

    /* Run pos and att controller to calculate control output */
    void RunController();

    /* Get px4 params from px4::parameters */
    template <px4::params p>
    void get_px4_param(float& output);

    /* Get px4 params from px4::parameters */
    template <px4::params p>
    void get_px4_param(int& output);

    /* Get px4 params from px4::parameters */
    template <px4::params p>
    void get_px4_param(bool& output);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    std::shared_ptr<MavlinkReceiver> mavlink_receiver_;
    std::shared_ptr<MulticopterPositionControl> mc_pos_control_; 
    std::shared_ptr<MulticopterAttitudeControl> mc_att_control_; 


    // void set_position_target_local_ned(const mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED& sp);
    // void set_position_target_global_int(const mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT& sp);
    // void set_attitude_target(const mavlink::common::msg::SET_ATTITUDE_TARGET& sp);

};



}



#endif