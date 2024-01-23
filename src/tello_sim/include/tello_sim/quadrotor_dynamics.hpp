/**
 * @file quadrotor_dynamics.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Ordinary Differential Equations of quadrotor dynamics 
 * solved by odeint.
 * 
 * Note: This program relies on <boost/numeric/odeint.hpp>
 * 
 * @version 1.0
 * @date 2024-1-16
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */

#pragma once

#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace TelloQuadSimulator
{

class Dynamics
{
  public:
    // For internal use, but needs to be public for odeint
    typedef std::vector<double> StateVector;    

    struct State
    {
      Eigen::Vector3d pos;
      // Eigen::Vector3d vel;
      Eigen::Matrix3d R;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    
    struct Input
    {
      Eigen::Vector3d linear_vel; // linear 3-axis velocity (relative to the World frame)
      double yaw_rate; //  CCW is positive
    };

    Dynamics();

    void setState(const Dynamics::State &state);
    void setPos(const double& pos_x, const double& pos_y, const double& pos_z);
    void setRPY(const double& roll, const double& pitch, const double& yaw);
    void setInput(const Dynamics::Input &input);
    void setSimStep(const double& dt);
    void setMass(const double& m);
    void setGravityAcc(const double& g);

    Dynamics::State getState();
    double getSimStep();
    double getMass();
    double getGravityAcc();
    // Eigen::Vector3d getAcc();
    Eigen::Quaterniond getQuat();
    /* angular velocity (Only yaw rate is valid) */
    Eigen::Vector3d getAngVel();
    /* Z-Y-X order，RPY, roll[-pi, pi], pitch[-pi/2, pi/2], yaw[-pi, pi] */
    Eigen::Vector3d getRPY();

    void differentialEquation(const StateVector& x, StateVector& dxdt, double t);

    // Run the actual dynamics during [start_time, end_time] with a time step of sim_step_
    void step(const double& start_time, const double& end_time);

  private:
    double sim_step_;
    double mass_;
    double g_;
    // double thrust_;
    Eigen::Vector3d vel_; // linear velocity relative to the World frame (ENU)
    Eigen::Vector3d omega_; // angular velocity (Only yaw rate is valid)
    // Eigen::Vector3d acc_;
    // Eigen::Quaterniond q_;

    Dynamics::State state_;
    StateVector state_vec_;
    StateVector vectorizeState(const Dynamics::State& state);
    Dynamics::State devectorizeState(const StateVector& vec);

};


}