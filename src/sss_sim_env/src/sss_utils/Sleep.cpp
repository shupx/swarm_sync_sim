/**
 * @file Sleep.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Replace ROS Duration and Rate sleep. Utilized by other sim/real agent nodes.
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2024-1-13
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Peixuan Shu
 * All rights reserved.
 * 
 */


#include "sss_sim_env/Sleep.hpp"


namespace sss_utils
{

/**
 * \brief To replace ros::Duration(time).sleep() for swarm_sync_sim.
 * If use_sim_time is false, it is just same as ros::Duration(time).sleep()
 * If use_sim_time is true, it is ros::Duration(time).sleep() + requesting sim clock update in this thread.
 */
bool Duration::sleep() const
{
    if (ros::Time::useSystemTime())
    {
        /* Normal ros::Duration().sleep() */
        return ros::Duration(sec, nsec).sleep();
    }
    else // if use sim time
    {
        /* Requesting sim clock update in this thread */

        std::thread::id thread_id = std::this_thread::get_id();

        // if(thread_clockupdater_map.find(thread_id) == thread_clockupdater_map.end())
        // {
        //     /* If no clockupdater exists on this thread, make a clockupdater */
        //     thread_clockupdater_map[thread_id] = std::make_shared<ClockUpdater>(); 
        //     std::cout << "[sss_utils::Duration::sleep] Create a new clock updater on thread_id " << thread_id << std::endl;
        // }
        // ClockUpdaterPtr clock_updater = thread_clockupdater_map[thread_id]; // Get the clockupdater of this thread

        ClockUpdaterPtr clock_updater = ThreadClockupdaters::global().get_clockupdater(thread_id);

        ros::Time start = ros::Time::now();
        ros::Time end = start + ros::Duration(sec, nsec);

        while(!clock_updater->request_clock_update(end))
        {
            // block until publishing successfully
            ros::WallDuration(0.1).sleep();

            std::cout << "[sss_utils::Duration::sleep] Repeat clock_updater->request_clock_update " << (ros::Time::now() + ros::Duration(sec, nsec)).toSec() << std::endl;
        }

        /* Block until the duration is satisfied */
        {
            // This allows sim time to run up to 100x real-time even for very short sleep durations.
            const uint32_t sleep_nsec = (sec != 0) ? 1000000 : (std::min)(1000000, nsec/100);
            while (ros::Time::now() < end)
            {
                ros::WallDuration(0, sleep_nsec).sleep();

                // If time jumped backwards from when we started sleeping, return immediately
                if (ros::Time::now() < start)
                {
                    return false;
                }
            }
            return true;
        }

        // return ros::Duration(sec, nsec).sleep();
    }
}

} //sss_utils