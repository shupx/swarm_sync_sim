/**
 * @file mavros_px4_quadrotor_sim_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Mavros(sim) + PX4 controller + quadrotor_dynamics. main loop
 * 
 * Note: This program relies on mavros_sim, px4_sitl, dynamics and sss_utils
 * 
 * @version 1.0
 * @date 2023-12-10
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */


#include "mavros_px4_quadrotor_sim/mavros_px4_quadrotor_sim_node.hpp"

using namespace MavrosQuadSimulator;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_px4_quadrotor_sim_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //Use unique_ptr to auto-destory the object when exiting.
    std::unique_ptr<Agent> agent(new Agent(nh, nh_private));

    ros::spin();
    return 0;
}

namespace MavrosQuadSimulator
{

Agent::Agent(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    /* Check if sim time is used */

    nh_.param<bool>("/use_sim_time", is_sim_time_, false);
    if (!is_sim_time_)
    {
        // ROS_WARN("[Agent] /use_sim_time is false! Force to set /use_sim_time to true");
        // nh_.setParam("/use_sim_time", true);
        ROS_WARN("[Agent] /use_sim_time is false! Maybe real time is used.");
    }


    /* init sim modules */

    dynamics_ = std::make_shared<Dynamics>();
    dynamics_->setSimStep(0.01); // set odeint integration step

    px4sitl_ = std::make_shared<PX4SITL>(nh_, nh_private_, dynamics_);

    mavros_sim_ = std::make_shared<mavros_sim::MavrosSim>(nh_, nh_private_);


    /* Set main loop */

    mainloop_period_ = 0.01; // should be multiples of the dynamics_->getSimStep()
    float times = mainloop_period_ / dynamics_->getSimStep();
    ROS_ASSERT_MSG(times >= 1 && times == int(times), "[MavrosQuadSimulator::Agent] mainloop_period_ should be multiples of dynamic sim step!");
    mainloop_timer_ = sss_utils::createTimer(ros::Duration(mainloop_period_), &Agent::mainloop, this);
}

void Agent::mainloop(const ros::TimerEvent &event)
{
    /*  Validate time  */
    static double last_time = ros::Time::now().toSec(); // use sim time
    double next_time = ros::Time::now().toSec() + mainloop_period_;
    if (next_time < last_time)
    {
        ROS_ERROR("[sim Agent] The next_time %ss is smaller than the last_time %ss. Does the clock steps back?", std::to_string(next_time).c_str(), std::to_string(last_time).c_str());
    }

    /* Run PX4 SITL for one loop */
    uint64_t time_us = ros::Time::now().toNSec() / uint64_t(1e3); // do not use 1e3 alone as it is double type!
    px4sitl_->Run(time_us);

    /* Dynamics step forward (ode integrate the numerical model) */
    dynamics_->step(last_time, next_time);

    /* Mavros publishing mavlink messages to ROS topics */
    mavros_sim_->PublishRosMessage();

    last_time = next_time;
}

}
