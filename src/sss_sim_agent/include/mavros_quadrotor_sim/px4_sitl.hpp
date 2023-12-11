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

// #include "px4_modules/px4_lib/px4_platform_common/defines.h"
#include "px4_modules/px4_lib/parameters/px4_parameters.hpp"
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
        PX4SITL()
        {
            px4::params param = px4::params::MPC_XY_P;
            px4::param_info_s param_info = px4::parameters[int(param)];
            switch (px4::parameters_type[int(param)])
            {
                case 0:
                    break; 
                case 1:
                {
                    int32_t a = param_info.val.i;
                    break; 
                }
                case 2:
                {
                    float a = param_info.val.f;
                    break; 
                }
                default:
                    break;
            };
        };

    private:
        std::shared_ptr<PositionControl> pos_ctrl_; 
        std::shared_ptr<AttitudeControl> Att_ctrl_; 




};

}



#endif