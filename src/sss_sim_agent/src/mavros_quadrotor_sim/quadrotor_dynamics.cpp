/**
 * @file quadrotor_ode.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Ordinary Differential Equations of quadrotor dynamics 
 * solved by odeint.
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-11-27
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "mavros_quadrotor_sim/quadrotor_dynamics.hpp"

namespace odeint = boost::numeric::odeint;

namespace MavrosQuadSimulator
{

/* Initialize */
Quadrotor::Quadrotor()
{
    state_.pos = Eigen::Vector3d::Zero();
    state_.vel = Eigen::Vector3d::Zero();
    state_.R = Eigen::Matrix3d::Identity();
    updateStateVector(); // Initialize state_vec_
}

void Quadrotor::updateStateVector()
{
    if (state_vec_.empty()){
        /* Initialize state vecotr */
        for (int i = 0; i < 3; i++)
            {state_vec_.emplace_back(state_.pos(i)); }
        for (int i = 0; i < 3; i++)
            {state_vec_.emplace_back(state_.vel(i)); }
        for (int i = 0; i < 3; i++)
            {state_vec_.emplace_back(state_.R(i, 0)); }
        for (int i = 0; i < 3; i++)
            {state_vec_.emplace_back(state_.R(i, 1)); }
        for (int i = 0; i < 3; i++)
            {state_vec_.emplace_back(state_.R(i, 2)); }
    }
    else{
        /* Update state vector */
        for (int i = 0; i < 3; i++)
        {
            state_vec_[0 + i]  = state_.pos(i);
            state_vec_[3 + i]  = state_.vel(i);
            state_vec_[6 + i]  = state_.R(i, 0);
            state_vec_[9 + i]  = state_.R(i, 1);
            state_vec_[12 + i] = state_.R(i, 2);        
        }
    }

}

void Quadrotor::setState(const Quadrotor::State &state)
{
    state_.pos = state.pos;
    state_.vel = state.vel;
    state_.R   = state.R;
    updateStateVector();
}

void Quadrotor::setOmegaThrust(const Eigen::Vector3d& omega, const double& thrust)
{
    omega_ = omega;
    thrust_ = thrust;
}

Quadrotor::State Quadrotor::getState()
{
    return state_;
}

Eigen::Vector3d Quadrotor::getAcc()
{
    return acc_;
}

double Quadrotor::getMass()
{
    return mass_;
}

double Quadrotor::getGravityAcc()
{
    return g_;
}



}

