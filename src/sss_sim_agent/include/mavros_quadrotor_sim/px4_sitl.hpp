/**
 * @file px4_sitl.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief PX4 main function. Communicate with mavros and dynamics.
 * Mavros <---> PX4 <---> Quadrotor Dynamics
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
#include <mavlink/v2.0/common/common.hpp>
#include <iostream>
#include <stdexcept>

#include "px4_modules/px4_lib/px4_platform_common/param.h"

#include "px4_modules/AttitudeControl/AttitudeControl.hpp"
#include "px4_modules/PositionControl/PositionControl.hpp"

#include "mavros_quadrotor_sim/quadrotor_dynamics.hpp"



namespace MavrosQuadSimulator
{

/**
 * \brief PX4 flight stack simulation. Communicate with mavros and dynamics.
 * Mavros <---> PX4 <---> Quadrotor Dynamics
 */
class PX4SITL
{
public:
    PX4SITL();

    void get_px4_param(float& output, const px4::params& param);
    void get_px4_param(int& output, const px4::params& param);

private:
    std::shared_ptr<PositionControl> pos_ctrl_; 
    std::shared_ptr<AttitudeControl> Att_ctrl_; 

    void set_position_target_local_ned(const mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED& sp);
    void set_position_target_global_int(const mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT& sp);
    void set_attitude_target(const mavlink::common::msg::SET_ATTITUDE_TARGET& sp);

    // the default parameters are stored in <parameters/px4_parameters.hpp>
	DEFINE_PARAMETERS(
		// Position Control
		(ParamFloat<px4::params::MPC_XY_P>)        _param_mpc_xy_p,
		(ParamFloat<px4::params::MPC_Z_P>)          _param_mpc_z_p,
		(ParamFloat<px4::params::MPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_I_ACC>) _param_mpc_xy_vel_i_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_D_ACC>) _param_mpc_xy_vel_d_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>)  _param_mpc_z_vel_p_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_I_ACC>)  _param_mpc_z_vel_i_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_D_ACC>)  _param_mpc_z_vel_d_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>)   _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>)  _param_mpc_z_v_auto_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>)  _param_mpc_z_v_auto_dn,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>)  _param_mpc_tiltmax_air,
		(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,
		(ParamBool<px4::params::MPC_USE_HTE>)       _param_mpc_use_hte,

		// Takeoff / Land
		(ParamFloat<px4::params::MPC_SPOOLUP_TIME>) _param_mpc_spoolup_time, /**< time to let motors spool up after arming */
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>)   _param_mpc_tko_ramp_t,   /**< time constant for smooth takeoff ramp */
		(ParamFloat<px4::params::MPC_TKO_SPEED>)    _param_mpc_tko_speed,
		(ParamFloat<px4::params::MPC_LAND_SPEED>)   _param_mpc_land_speed,

		(ParamFloat<px4::params::MPC_VEL_MANUAL>)   _param_mpc_vel_manual,
		(ParamFloat<px4::params::MPC_XY_CRUISE>)    _param_mpc_xy_cruise,
		(ParamFloat<px4::params::MPC_LAND_ALT2>)    _param_mpc_land_alt2,    /**< downwards speed limited below this altitude */
		(ParamInt<px4::params::MPC_POS_MODE>)       _param_mpc_pos_mode,
		(ParamInt<px4::params::MPC_ALT_MODE>)       _param_mpc_alt_mode,
		(ParamFloat<px4::params::MPC_TILTMAX_LND>)  _param_mpc_tiltmax_lnd,  /**< maximum tilt for landing and smooth takeoff */
		(ParamFloat<px4::params::MPC_THR_MIN>)      _param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>)      _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_XY_MARG>)  _param_mpc_thr_xy_marg,

		(ParamFloat<px4::params::SYS_VEHICLE_RESP>) _param_sys_vehicle_resp,
		(ParamFloat<px4::params::MPC_ACC_HOR>)      _param_mpc_acc_hor,
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>)   _param_mpc_acc_up_max,
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>)  _param_mpc_acc_hor_max,
		(ParamFloat<px4::params::MPC_JERK_AUTO>)    _param_mpc_jerk_auto,
		(ParamFloat<px4::params::MPC_JERK_MAX>)     _param_mpc_jerk_max,
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>)    _param_mpc_man_y_max,
		(ParamFloat<px4::params::MPC_MAN_Y_TAU>)    _param_mpc_man_y_tau,

		(ParamFloat<px4::params::MPC_XY_VEL_ALL>)   _param_mpc_xy_vel_all,
		(ParamFloat<px4::params::MPC_Z_VEL_ALL>)    _param_mpc_z_vel_all
	);
    float a = _param_mpc_xy_p.get();

};



}



#endif