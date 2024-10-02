/**
 * @file fw_sim_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Fixed wing flight controller driver sim + quadrotor_dynamics. Main loop
 * 
 * Note: This program relies on fw_driver_sim, ss_utils
 * 
 * @version 1.0
 * @date 2024-10-02
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "fw_plane_sim/fw_sim/fw_sim_node.hpp"

using namespace FwSimulator;

/* For compile of node */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "fw_sim_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //Use unique_ptr to auto-destory the object when exiting.
    std::unique_ptr<Agent> agent(new Agent(nh, nh_private));

    ros::spin();
    return 0;
}


/* For compile of nodelet */
#include <nodelet/nodelet.h>
namespace FwSimulator
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
PLUGINLIB_EXPORT_CLASS(FwSimulator::SimAgent, nodelet::Nodelet);



namespace FwSimulator
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
    float init_vel;
    float dynamics_period_ms;
    std::string frame_id;
    nh_private_.param<float>("init_x_East_metre", init_x, 0.0);
    nh_private_.param<float>("init_y_North_metre", init_y, 0.0);
    nh_private_.param<float>("init_z_Up_metre", init_z, 100.0);
    nh_private_.param<float>("init_roll_deg", init_roll, 0.0);
    nh_private_.param<float>("init_pitch_deg", init_pitch, 0.0);
    nh_private_.param<float>("init_yaw_deg", init_yaw, 0.0);
    nh_private_.param<float>("init_vel_m_per_s", init_vel, 30.0);
    nh_private_.param<float>("dynamics_period_ms", dynamics_period_ms, 25.0);
    // nh_private_.param<std::string>("frame_id", frame_id, "");

    /* init dynamics */
    State_Init baseState;
    baseState.posiNInit = init_y;  // 北向初始位置，m
    baseState.posiEInit = init_x;  // 东向初始位置，m
    baseState.posiDInit = -init_z; // 地向初始位置，m
    baseState.velInit = init_vel;  // 初始速度，m/s
    baseState.pitchInit = init_pitch*M_PI/180; // 初始俯仰角，rad
    baseState.yawInit = init_yaw*M_PI/180;   // 初始偏航角，rad
    baseState.rollInit = init_roll*M_PI/180;  // 初始滚转角，rad
    InitState(baseState); // from BHDynamics.h

    /* init driver */
    fw_driver_sim_ = std::make_shared<FwDriverSim>(nh_, nh_private_);
    State_Output init_dynamic_state;
    init_dynamic_state.posiNState = baseState.posiNInit;
    init_dynamic_state.posiEState = baseState.posiEInit;
    init_dynamic_state.posiDState = baseState.posiDInit;
    init_dynamic_state.velState = baseState.velInit;
    init_dynamic_state.pitchState = baseState.pitchInit;
    init_dynamic_state.yawState = baseState.yawInit;
    init_dynamic_state.rollState = baseState.rollInit;
    // init_dynamic_state.velalphaState = 0;
    // init_dynamic_state.velbeteState = 0;
    // init_dynamic_state.alphaState = 0;
    // init_dynamic_state.beteState = 0;
    fw_driver_sim_->UpdateDynamicState(init_dynamic_state);

    /* Create the main loop timer */
    mainloop_period_ = dynamics_period_ms; // 
    mainloop_timer_ = sss_utils::createTimer(nh_, ros::Duration(mainloop_period_), &Agent::mainloop, this);
}

void Agent::mainloop(const ros::TimerEvent &event)
{
    /*  Validate time  */
    if (mainloop_last_time_ == 0) { mainloop_last_time_ = ros::Time::now().toSec(); } // init mainloop_last_time_
    double next_time = ros::Time::now().toSec() + mainloop_period_;
    if (next_time < mainloop_last_time_)
    {
        ROS_ERROR("[FwSimulator::Agent::mainloop] The next_time %ss is smaller than the last_time %ss. Does the clock steps back?", std::to_string(next_time).c_str(), std::to_string(mainloop_last_time_).c_str());
    }

    /* Publish the newest states to ROS topics */
    fw_driver_sim_->PublishState();

    /* Set control inputs and step forward (ode integrate the numerical model) */
    if (fw_driver_sim_->GetStage() == 4 || fw_driver_sim_->GetArmed()) // 4 for offboard
    {
        Input input = fw_driver_sim_->GetControlInputs();
        State_Output new_dynamic_state = OutLoopCtrl_1(input.High_input, input.Vel_input, input.Roll_input);
        fw_driver_sim_->UpdateDynamicState(new_dynamic_state);
    }

    mainloop_last_time_ = next_time;
}

}
