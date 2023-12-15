/**
 * @file px4_sitl.cpp
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


#include "mavros_quadrotor_sim/px4_sitl.hpp"

namespace MavrosQuadSimulator
{

PX4SITL::PX4SITL(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    /* Load px4 parameters from ROS parameter space to override the default values from <parameters/px4_parameters.hpp>*/
    load_px4_params_from_ros_params();

    mc_pos_control_ = std::make_shared<MulticopterPositionControl>(false);
    mc_att_control_ = std::make_shared<MulticopterAttitudeControl>(false);

    mc_pos_control_->init();
    mc_att_control_->init();
}

void PX4SITL::load_px4_params_from_ros_params()
{
    /* Load px4 parameters from ROS parameter space to override the default values from <parameters/px4_parameters.hpp>*/
    for (int i=0; i<sizeof(px4::parameters)/sizeof(px4::parameters[0]); ++i)
    {
        switch (px4::parameters_type[i])
        {
            case PARAM_TYPE_INT32:
            {
                int default_value = px4::parameters[i].val.i;
                nh_private_.getParam(px4::parameters[i].name, px4::parameters[i].val.i);
                if (px4::parameters[i].val.i != default_value)
                {
                    std::cout << "[PX4SITL] Reset " << px4::parameters[i].name << " from " << default_value << " to " << px4::parameters[i].val.i << std::endl;
                }
                break;
            }
            case PARAM_TYPE_FLOAT:
            {
                float default_value = px4::parameters[i].val.f;
                nh_private_.getParam(px4::parameters[i].name, px4::parameters[i].val.f);
                if (px4::parameters[i].val.f != default_value)
                {
                    std::cout << "[PX4SITL] Reset " << px4::parameters[i].name << " from " << default_value << " to " << px4::parameters[i].val.f << std::endl;
                }
                break; 
            }   
            default:
            {
                throw std::invalid_argument("[PX4SITL::load_px4_param_from_ros_params] param type is unknown.");
                break;  
            }      
        }
    }
}

void PX4SITL::RunController()
{
    /* Run pos and att controller to calculate control output */
    mc_pos_control_->Run();
    mc_att_control_->Run();
}


template <px4::params p>
void PX4SITL::get_px4_param(float& output)
{
	    // static type-check
	    static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_FLOAT, "parameter type must be float");

        output = px4::parameters[(int) p].val.f;
}

template <px4::params p>
void PX4SITL::get_px4_param(int32_t& output)
{
	    // static type-check
	    static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

        output = px4::parameters[(int) p].val.i;
}

template <px4::params p>
void PX4SITL::get_px4_param(bool& output)
{
	    // static type-check
	    static_assert(px4::parameters_type[(int)p] == PARAM_TYPE_INT32, "parameter type must be int32_t");

        output = px4::parameters[(int) p].val.i != 0;
}


}
