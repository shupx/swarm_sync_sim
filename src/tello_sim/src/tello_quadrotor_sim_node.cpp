/**
 * @file tello_quadrotor_sim.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Tello driver sim + quadrotor_dynamics. Main loop
 * 
 * Note: This program relies on tello driver sim, dynamics and sss_utils
 * 
 * @version 1.0
 * @date 2024-1-16
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "tello_sim/tello_quadrotor_sim_node.hpp"

using namespace TelloQuadSimulator;

/* For compile of node */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tello_quadrotor_sim_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //Use unique_ptr to auto-destory the object when exiting.
    std::unique_ptr<Agent> agent(new Agent(nh, nh_private));

    ros::spin();
    return 0;
}


/* For compile of nodelet */
#include <nodelet/nodelet.h>
namespace TelloQuadSimulator
{
class SimAgent :public nodelet::Nodelet
{
public:
    SimAgent(){}
public:
    void onInit()
    {
        ros::NodeHandle nh = getNodeHandle(); // get the seperate nodehandle of this node
        ros::NodeHandle nh_private = getPrivateNodeHandle(); // get the seperate private nodehandle of this node
        // ros::NodeHandle nh = getMTNodeHandle();
        // ros::NodeHandle nh_private = getMTPrivateNodeHandle();

        agent_ = std::make_unique<Agent>(nh, nh_private);
    }
private:
    std::unique_ptr<Agent> agent_;
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TelloQuadSimulator::SimAgent, nodelet::Nodelet);



namespace TelloQuadSimulator
{

Agent::Agent(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    /* Check if sim time is used */
    if (!ros::Time::isSimTime())
    {
        // ROS_WARN("[Agent] /use_sim_time is false! Force to set /use_sim_time to true");
        // nh_.setParam("/use_sim_time", true);
        ROS_WARN("[Agent] /use_sim_time is false! Maybe real time is used.");
    }

    /* Init params */
    float init_x, init_y, init_z, init_roll, init_pitch, init_yaw;
    float linear_vel_scale, angular_vel_scale;
    bool use_imu_orientation;
    std::string frame_id;
    nh_private_.param<float>("init_x_East_metre", init_x, 0.0);
    nh_private_.param<float>("init_y_North_metre", init_y, 0.0);
    nh_private_.param<float>("init_z_Up_metre", init_z, 0.0);
    nh_private_.param<float>("init_roll_deg", init_roll, 0.0);
    nh_private_.param<float>("init_pitch_deg", init_pitch, 0.0);
    nh_private_.param<float>("init_yaw_deg", init_yaw, 0.0);
    nh_private_.param<float>("linear_vel_scale", linear_vel_scale, 2.0);
    nh_private_.param<float>("angular_vel_scale", angular_vel_scale, 3.14);
    nh_private_.param<bool>("use_imu_orientation", use_imu_orientation, false);
    nh_private_.param<std::string>("frame_id", frame_id, "");

    /* init sim modules */
    dynamics_ = std::make_shared<Dynamics>();
    dynamics_->setSimStep(0.01); // set odeint integration step
    dynamics_->setPos(init_x, init_y, init_z);
    dynamics_->setRPY(init_roll*M_PI/180, init_pitch*M_PI/180, init_yaw*M_PI/180);

    tello_driver_sim_ = std::make_shared<TelloDriverSim>(nh_, nh_private_, dynamics_);
    tello_driver_sim_->SetImuYawBase(dynamics_->getRPY()[2]); // initial yaw
    tello_driver_sim_->UseImuOrientation(use_imu_orientation);
    tello_driver_sim_->SetVelCmdScale(linear_vel_scale, angular_vel_scale);
    tello_driver_sim_->SetFrameId(frame_id);

    /* Create the main loop timer */
    mainloop_period_ = 0.01; // should be multiples of the dynamics_->getSimStep()
    float times = mainloop_period_ / dynamics_->getSimStep();
    ROS_ASSERT_MSG(times >= 1 && times == int(times), "[TelloQuadSimulator::Agent] mainloop_period_ should be multiples of dynamic sim step!");
    mainloop_timer_ = sss_utils::createTimer(nh_, ros::Duration(mainloop_period_), &Agent::mainloop, this);
}

void Agent::mainloop(const ros::TimerEvent &event)
{
    /*  Validate time  */
    if (mainloop_last_time_ == 0) { mainloop_last_time_ = ros::Time::now().toSec(); } // init mainloop_last_time_
    double next_time = ros::Time::now().toSec() + mainloop_period_;
    if (next_time < mainloop_last_time_)
    {
        ROS_ERROR("[TelloQuadSimulator::Agent::mainloo] The next_time %ss is smaller than the last_time %ss. Does the clock steps back?", std::to_string(next_time).c_str(), std::to_string(mainloop_last_time_).c_str());
    }

    /* Publish the newest states to ROS topics */
    tello_driver_sim_->PublishState();

    /* Set control inputs */
    dynamics_->setInput(tello_driver_sim_->GetControlInputs());

    /* Dynamics step forward (ode integrate the numerical model) */
    dynamics_->step(mainloop_last_time_, next_time);

    mainloop_last_time_ = next_time;
}

}
