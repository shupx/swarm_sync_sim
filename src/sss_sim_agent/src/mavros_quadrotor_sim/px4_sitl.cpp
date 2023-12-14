/**
 * @file px4_sitl.cpp
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


#include "mavros_quadrotor_sim/px4_sitl.hpp"

namespace MavrosQuadSimulator
{

PX4SITL::PX4SITL()
{
    
}


void PX4SITL::get_px4_param(float& output, const px4::params& param)
{
        param_info_s param_info = px4::parameters[int(param)];
        switch (px4::parameters_type[(int) param])
        {
            case PARAM_TYPE_UNKNOWN:
            {
                throw std::invalid_argument("[PX4SITL::get_px4_param] param type can not be unknown (0)");
                break; 
            }
            case PARAM_TYPE_INT32:
            {
                throw std::invalid_argument("[PX4SITL::get_px4_param] param type is int. But the output is float");
                break; 
            }
            case PARAM_TYPE_FLOAT:
            {
                output = param_info.val.f;
                break; 
            }
            default:
                throw std::invalid_argument("[PX4SITL::get_px4_param] param type is unknown");
                break;
        };
}

void PX4SITL::get_px4_param(int& output, const px4::params& param)
{
        param_info_s param_info = px4::parameters[int(param)];
        switch (px4::parameters_type[(int) param])
        {
            case PARAM_TYPE_UNKNOWN:
            {
                throw std::invalid_argument("[PX4SITL::get_px4_param] param type can not be unknown (0)");
                break; 
            }
            case PARAM_TYPE_INT32:
            {
                output = param_info.val.i;
                break; 
            }
            case PARAM_TYPE_FLOAT:
            {
                throw std::invalid_argument("[PX4SITL::get_px4_param] param type is float. But the output is int");
                break; 
            }
            default:
                throw std::invalid_argument("[PX4SITL::get_px4_param] param type is unknown");
                break;
        };
}
}
