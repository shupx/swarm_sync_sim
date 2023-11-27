/**
 * @file quadrotor_dynamics.hpp
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

#ifndef __QUADROTOR_DYNAMICS_H__
#define __QUADROTOR_DYNAMICS_H__

#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>

namespace MavrosQuadSimulator
{

class Quadrotor
{
  public:
    // For internal use, but needs to be public for odeint
    typedef std::vector<double> StateVector;    

    struct State
    {
      Eigen::Vector3d pos;
      Eigen::Vector3d vel;
      Eigen::Matrix3d R;
      // Eigen::Vector3d omega;
      // Eigen::Array4d  motor_rpm;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    Quadrotor();

    // Runs the actual dynamics simulation with a time step of dt
    void step(double dt);

    
    void setState(const Quadrotor::State &state);
    void setOmegaThrust(const Eigen::Vector3d& omega, const double& thrust);

    Quadrotor::State getState();
    Eigen::Vector3d getAcc();
    double getMass();
    double getGravityAcc();

  private:
    double mass_;
    double g_;
    double thrust_;
    Eigen::Vector3d acc_;
    Eigen::Vector3d omega_;

    double sim_step_;
    Quadrotor::State state_;
    StateVector state_vec_;
    void updateStateVector();



};


}

#endif