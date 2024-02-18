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
        std::thread::id thread_id = std::this_thread::get_id();
        ClockUpdaterPtr clock_updater = ThreadClockupdaters::global().get_clockupdater(thread_id);

        /* Request sim clock update on this thread */
        ros::Time start = ros::Time::now();
        ros::Time end = start + ros::Duration(sec, nsec);
        while(!clock_updater->request_clock_update(end))
        {
            // block until publishing successfully
            ros::WallDuration(0.5).sleep();

            std::cout << "[sss_utils::Duration::sleep] Sim clock not online. Repeat clock_updater->request_clock_update " << (ros::Time::now() + ros::Duration(sec, nsec)).toSec() << std::endl;
        }

        /* Release the time client of the TimerManagerExtra */
        while(!TimerManagerExtra::global().clock_updater_->request_clock_update(ros::TIME_MAX))
        {
            // block until publishing successfully
            ros::WallDuration(0.5).sleep();

            std::cout << "[sss_utils::Duration::sleep] Sim clock not online. Repeat clock_updater->request_clock_update " << ros::TIME_MAX.toSec() << std::endl;
        }

        /* Block until the duration is satisfied */
        {
            // This allows sim time to run up to 200x real-time even for very short sleep durations.
            const uint32_t sleep_nsec = (sec != 0) ? 1000000 : (std::min)(1000000, nsec/200);
            while (ros::Time::now() < end)
            {
                ros::WallDuration(0, sleep_nsec).sleep();

                // If time jumped backwards from when we started sleeping, return immediately
                if (ros::Time::now() < start)
                {
                    return false;
                }
            }
            // Release the time client on this thread (especically for topic subscriber callbacks, may be no more new callbacks in this thread)
            //@TODO This release is unsafe as all clockupdaters may request infinity time, so clock will update at the maximum speed which may jump some timer callbacks?
            clock_updater->request_clock_update(ros::TIME_MAX);

            return true;
        }
        // return ros::Duration(sec, nsec).sleep();

    }
}


Rate::Rate(double frequency)
: start_(ros::Time::now())
, expected_cycle_time_(1.0 / frequency)
, actual_cycle_time_(0.0)
{ 
    if (ros::Time::useSystemTime())
    {
        /* Normal rate.sleep() */
        rate_ = std::make_shared<ros::Rate>(expected_cycle_time_);
    }
}

Rate::Rate(const ros::Duration& d)
  : start_(ros::Time::now())
  , expected_cycle_time_(d.sec, d.nsec)
  , actual_cycle_time_(0.0)
{
    if (ros::Time::useSystemTime())
    {
        /* Normal rate.sleep() */
        rate_ = std::make_shared<ros::Rate>(expected_cycle_time_);
    }
}

/**
 * \brief To replace rate.sleep() for swarm_sync_sim.
 * If use_sim_time is false, it is just same as rate.sleep()
 * If use_sim_time is true, it is rate.sleep() + requesting sim clock update in this thread.
 */
bool Rate::sleep()
{
    if (ros::Time::useSystemTime())
    {
        /* Normal rate.sleep() */
        return rate_->sleep();
    }
    else  // if use sim time
    {
        ros::Time expected_end = start_ + expected_cycle_time_;

        ros::Time actual_end = ros::Time::now();

        // detect backward jumps in time
        if (actual_end < start_)
        {
            expected_end = actual_end + expected_cycle_time_;
        }

        //calculate the time we'll sleep for
        ros::Duration sleep_time = expected_end - actual_end;

        //set the actual amount of time the loop took in case the user wants to know
        actual_cycle_time_ = actual_end - start_;

        //make sure to reset our start time
        start_ = expected_end;

        //if we've taken too much time we won't sleep
        if(sleep_time <= ros::Duration(0.0))
        {
            // if we've jumped forward in time, or the loop has taken more than a full extra
            // cycle, reset our cycle
            if (actual_end > expected_end + expected_cycle_time_)
            {
            start_ = actual_end;
            }
            // return false to show that the desired rate was not met
            return false;
        }

        // return sleep_time.sleep();

        /* use sss_utils::Duration for sleep */
        return sss_utils::Duration(sleep_time.sec, sleep_time.nsec).sleep();
    }
}

void Rate::reset()
{
  start_ = ros::Time::now();
}

ros::Duration Rate::cycleTime() const
{
  return actual_cycle_time_;
}

} //sss_utils