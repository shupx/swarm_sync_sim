/**
 * @file mavros_px4_quadrotor_sim_node.hpp
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


#ifndef __MAVROS_PX4_QUADROTOR_SIM_NODE_H__
#define __MAVROS_PX4_QUADROTOR_SIM_NODE_H__

#include <ros/ros.h>
#include <sss_sim_env/sss_utils.hpp>

#include "mavros_sim/MavrosSim.hpp"
#include "mavros_px4_quadrotor_sim/px4_sitl.hpp"
#include "mavros_px4_quadrotor_sim/quadrotor_dynamics.hpp"
#include "mavros_px4_quadrotor_sim/drone_visualizer.hpp"


namespace MavrosQuadSimulator
{

extern int agent_index; // index of MavrosQuadSimulator agents (this variable is global across nodelets)
int agent_index = 0;

/* allocate global storage for messages of agent i */
void allocate_message_storage(int expected_agent_num)
{
    while(px4::mavlink_stream_lists.size() < expected_agent_num)
    {
        px4::mavlink_stream_lists.emplace_back(std::array<mavlink_info_s, MAVLINK_STREAM_NUM>{});
    }
    while(px4::mavlink_receive_lists.size() < expected_agent_num)
    {
        px4::mavlink_receive_lists.emplace_back(std::array<mavlink_info_s, MAVLINK_RECEIVE_NUM>{});
    }
    
}


class Agent
{
    public:
        Agent(int agent_id, const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    
    private:
        bool is_sim_time_;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        std::shared_ptr<Dynamics> dynamics_;
        std::shared_ptr<PX4SITL> px4sitl_;
        std::shared_ptr<mavros_sim::MavrosSim> mavros_sim_;
        std::shared_ptr<Visualizer> visualizer_;

        double mainloop_period_;
        sss_utils::Timer mainloop_timer_;

        void mainloop(const ros::TimerEvent &event);

        int agent_id_=-1; // agent id

};

}

#endif