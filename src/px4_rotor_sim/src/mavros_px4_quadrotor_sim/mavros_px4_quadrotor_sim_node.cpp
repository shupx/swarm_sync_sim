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

/* For compile of node */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_px4_quadrotor_sim_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    /* allocate global storage for messages of the agent */
    int expected_agent_num = 1; // each agent runs in seperate process. No need to distinguish the messages.
    px4::allocate_mavlink_message_storage(expected_agent_num);
    uORB_sim::allocate_uorb_message_storage(expected_agent_num);

    //Use unique_ptr to auto-destory the object when exiting.
    std::unique_ptr<Agent> agent(new Agent(agent_index, nh, nh_private));

    ros::spin();
    return 0;
}


/* For compile of nodelet */
#include <nodelet/nodelet.h>
namespace MavrosQuadSimulator
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

        /* allocate global storage for messages of agent i */
        int expected_agent_num = agent_index + 1;
        px4::allocate_mavlink_message_storage(expected_agent_num);
        uORB_sim::allocate_uorb_message_storage(expected_agent_num);

        // std::cout << "expected_agent_num: " << expected_agent_num << std::endl;

        agent_ = std::make_unique<Agent>(agent_index, nh, nh_private);

        agent_index ++;

        // NODELET_DEBUG("My debug statement")
        // NODELET_DEBUG_STREAM("my debug statement " << (double) 1.0)
        // NODELET_DEBUG_COND( 1 == 1, "my debug_statement")
        // NODELET_DEBUG_STREAM_COND( 1 == 1, "my debug statement " << (double) 1.0)
    }
private:
    std::unique_ptr<Agent> agent_;
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(MavrosQuadSimulator::SimAgent, nodelet::Nodelet);



namespace MavrosQuadSimulator
{

Agent::Agent(int agent_id, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : agent_id_(agent_id), nh_(nh), nh_private_(nh_private)
{
    /* Check if sim time is used */

    nh_.param<bool>("/use_sim_time", is_sim_time_, false);
    if (!is_sim_time_)
    {
        // ROS_WARN("[Agent] /use_sim_time is false! Force to set /use_sim_time to true");
        // nh_.setParam("/use_sim_time", true);
        ROS_WARN("[Agent] /use_sim_time is false! Maybe real time is used.");
    }

    float init_x, init_y, init_z;
    nh_private_.param<float>("init_x_East_metre", init_x, 0.0);
    nh_private_.param<float>("init_y_North_metre", init_y, 0.0);
    nh_private_.param<float>("init_z_Up_metre", init_z, 0.0);


    /* init sim modules */

    dynamics_ = std::make_shared<Dynamics>();
    dynamics_->setSimStep(0.01); // set odeint integration step
    dynamics_->setPos(init_x, init_y, init_z);

    px4sitl_ = std::make_shared<PX4SITL>(agent_id_, nh_, nh_private_, dynamics_);

    mavros_sim_ = std::make_shared<mavros_sim::MavrosSim>(agent_id_, nh_, nh_private_);

    visualizer_ = std::make_shared<Visualizer>(nh_, nh_private_);


    /* Set main loop */

    mainloop_period_ = 0.01; // should be multiples of the dynamics_->getSimStep()
    float times = mainloop_period_ / dynamics_->getSimStep();
    ROS_ASSERT_MSG(times >= 1 && times == int(times), "[MavrosQuadSimulator::Agent] mainloop_period_ should be multiples of dynamic sim step!");
    mainloop_timer_ = sss_utils::createTimer(nh_, ros::Duration(mainloop_period_), &Agent::mainloop, this);
}

void Agent::mainloop(const ros::TimerEvent &event)
{
    /*  Validate time  */
    if (mainloop_last_time_ == 0) { mainloop_last_time_ = ros::Time::now().toSec(); } // init mainloop_last_time_
    double next_time = ros::Time::now().toSec() + mainloop_period_;
    if (next_time < mainloop_last_time_)
    {
        ROS_ERROR("[sim Agent] The next_time %ss is smaller than the last_time %ss. Does the clock steps back?", std::to_string(next_time).c_str(), std::to_string(mainloop_last_time_).c_str());
    }

    /* Run PX4 SITL for one loop */
    uint64_t time_us = ros::Time::now().toNSec() / uint64_t(1e3); // do not use 1e3 alone as it is double type!
    px4sitl_->Run(time_us);

    /* Dynamics step forward (ode integrate the numerical model) */
    dynamics_->step(mainloop_last_time_, next_time);

    /* Mavros publishing mavlink messages to ROS topics */
    mavros_sim_->PublishRosMessage();

    /* Publish rotor propeller joint positions, base_link tf and history path for the robot model visualization in rviz */
    visualizer_->Run();
    //@TODO use a param to determine whether to publish history path? (time-consuming)

    mainloop_last_time_ = next_time;
}

}
