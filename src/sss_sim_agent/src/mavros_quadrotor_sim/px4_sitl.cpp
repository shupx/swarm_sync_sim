/**
 * @file px4_sitl.cpp
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


#include "mavros_quadrotor_sim/px4_sitl.hpp"

namespace MavrosQuadSimulator
{

PX4SITL::PX4SITL(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, const std::shared_ptr<Dynamics> &dynamics)
    : nh_(nh), nh_private_(nh_private), uav_dynamics_(dynamics)
{
    /* Load px4 parameters from ROS parameter space to override the default values from <parameters/px4_parameters.hpp>*/
    load_px4_params_from_ros_params(); // Before loading px4 modules!

    /* Load px4 modules */
    mavlink_receiver_ = std::make_shared<MavlinkReceiver>();
    mc_pos_control_ = std::make_shared<MulticopterPositionControl>(false);
    mc_att_control_ = std::make_shared<MulticopterAttitudeControl>(false);

    /* Init px4 modules */
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

void PX4SITL::Run(const uint64_t &time_us)
{
    /* Update px4 uorb states from UAV dynamical model */
    UpdateUorbStates(time_us);

    /* Run mavlink receiver to update command uorb messages */
    // mavlink_message_t msg; 
    // mavlink_receiver_->handle_message(&msg);

    /* Run pos and att controller to calculate control output */
    mc_pos_control_->Run(); // calling period should between [0.002f, 0.04f] 25Hz-500Hz
    mc_att_control_->Run(); // calling should between [0.0002f, 0.02f] 50Hz-5000Hz

}

void PX4SITL::StreamMavlink(const uint64_t &time_us)
{

}

void PX4SITL::UpdateUorbStates(const uint64_t &time_us)
{
    /* Read true values from uav dynamic models */
    Dynamics::State state = uav_dynamics_->getState();
    Eigen::Vector3d acc = uav_dynamics_->getAcc();
    Eigen::Quaterniond q = uav_dynamics_->getQuat();
    Eigen::Vector3d omega = uav_dynamics_->getAngVel();

    // Refer to https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/msg/vehicle_attitude.msg
    vehicle_attitude_s vehicle_attitude_msg{};
    vehicle_attitude_msg.timestamp = time_us;
    vehicle_attitude_msg.timestamp_sample = time_us;
    vehicle_attitude_msg.q[0] = q.w();
    vehicle_attitude_msg.q[1] = q.x();
    vehicle_attitude_msg.q[2] = q.y();
    vehicle_attitude_msg.q[3] = q.z();
    _attitude_pub.publish(vehicle_attitude_msg);

    // Refer to https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/msg/vehicle_local_position.msg
    vehicle_local_position_s vehicle_local_position_msg{};
    vehicle_local_position_msg.timestamp = time_us;
    vehicle_local_position_msg.timestamp_sample = time_us;
    vehicle_local_position_msg.xy_valid = true;
    vehicle_local_position_msg.z_valid = true;
    vehicle_local_position_msg.v_xy_valid = true;
    vehicle_local_position_msg.v_z_valid = true;
    vehicle_local_position_msg.x = state.pos[0];
    vehicle_local_position_msg.y = state.pos[1];
    vehicle_local_position_msg.z = state.pos[2];
    vehicle_local_position_msg.vx = state.vel[0];
    vehicle_local_position_msg.vy = state.vel[1];
    vehicle_local_position_msg.vz = state.vel[2];   
    vehicle_local_position_msg.z_deriv = state.vel[2];   
    vehicle_local_position_msg.ax = acc[0];
    vehicle_local_position_msg.ay = acc[1];
    vehicle_local_position_msg.az = acc[2]; 
    // vehicle_local_position_msg.heading = q.toRotationMatrix().eulerAngles(2,1,0);
    /* Instance is set from a quaternion representing transformation
	 * from frame 2 to frame 1.
	 * This instance will hold the angles defining the 3-2-1 intrinsic
	 * Tait-Bryan rotation sequence from frame 1 to frame 2.
     */
    double q_vec[] = {q.w(),q.x(),q.y(),q.z()};
    vehicle_local_position_msg.heading = matrix::Eulerd{matrix::Quatd{q_vec}}(2); 
    vehicle_local_position_msg.heading_good_for_control = true;
    _local_position_pub.publish(vehicle_local_position_msg);

    // vehicle_angular_velocity
    // vehicle_status

    //TODO: vehicle_global_position_msg
    // vehicle_global_position_s vehicle_global_position_msg{};
    // _global_position_pub.publish(vehicle_global_position_msg);


    // // Refer to https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/msg/vehicle_odometry.msg
    // vehicle_odometry_s vehicle_odometry_msg{};
    // vehicle_odometry_msg.timestamp = time_us;
    // vehicle_odometry_msg.timestamp_sample = time_us;
    // vehicle_odometry_msg.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;
    // vehicle_odometry_msg.x = state.pos[0];
    // vehicle_odometry_msg.y = state.pos[1];
    // vehicle_odometry_msg.z = state.pos[2];
    // vehicle_odometry_msg.q[0] = q.w();
    // vehicle_odometry_msg.q[1] = q.x();
    // vehicle_odometry_msg.q[2] = q.y();
    // vehicle_odometry_msg.q[3] = q.z();
    // vehicle_odometry_msg.q_offset[0] = 1.0;
    // vehicle_odometry_msg.q_offset[1] = 0.0;
    // vehicle_odometry_msg.q_offset[2] = 0.0;
    // vehicle_odometry_msg.q_offset[3] = 0.0;
    // vehicle_odometry_msg.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_NED;
    // vehicle_odometry_msg.vx = state.vel[0];
    // vehicle_odometry_msg.vy = state.vel[1];
    // vehicle_odometry_msg.vz = state.vel[2];   
    // vehicle_odometry_msg.rollspeed = omega[0];
    // vehicle_odometry_msg.pitchspeed = omega[1];
    // vehicle_odometry_msg.yawspeed = omega[2];
    // _odometry_pub.publish(vehicle_odometry_msg);

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
