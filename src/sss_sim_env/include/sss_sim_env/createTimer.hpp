/**
 * @file createTimer.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Some functions. Utilized by other sim/real agent nodes.
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-12-9
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __CREATE_TIMER__
#define __CREATE_TIMER__
#include <ros/ros.h>
#include <sss_sim_env/Timer.hpp>

using namespace ros;

namespace sss_utils
{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Versions of createTimer()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Create a timer which will call a callback at the specified rate.  This variant takes
 * anything that can be bound to a Boost.Function, including a bare function
 *
 * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
 * be stopped, and the callback will no longer be called.
 *
 * \param period The period at which to call the callback
 * \param callback The function to call
 * \param oneshot If true, this timer will only fire once
 * \param autostart If true (default), return timer that is already started
 */
Timer createTimer(Duration period, const TimerCallback& callback, bool oneshot = false,
                bool autostart = true)
{
    //@TODO specify nodehandle
    Timer sss_timer(period, callback, oneshot, autostart);
    return sss_timer;
}

/**
 * \brief Create a timer which will call a callback at the specified rate.  This variant takes
 * a class member function, and a bare pointer to the object to call the method on.
 *
 * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
 * be stopped, and the callback will no longer be called.
 *
 * \param r The rate at which to call the callback
 * \param callback The method to call
 * \param obj The object to call the method on
 * \param oneshot If true, this timer will only fire once
 * \param autostart If true (default), return timer that is already started
 */
template<class Handler, class Obj>
Timer createTimer(Rate r, Handler h, Obj o, bool oneshot = false, bool autostart = true)
{
    return createTimer(r.expectedCycleTime(), h, o, oneshot, autostart);
}

/**
 * \brief Create a timer which will call a callback at the specified rate.  This variant takes
 * a class member function, and a bare pointer to the object to call the method on.
 *
 * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
 * be stopped, and the callback will no longer be called.
 *
 * \param period The period at which to call the callback
 * \param callback The method to call
 * \param obj The object to call the method on
 * \param oneshot If true, this timer will only fire once
 * \param autostart If true (default), return timer that is already started
 */
template<class T>
Timer createTimer(Duration period, void(T::*callback)(const TimerEvent&) const, T* obj, 
                bool oneshot = false, bool autostart = true)
{
    return createTimer(period, boost::bind(callback, obj, boost::placeholders::_1), oneshot, autostart);
}

/**
 * \brief Create a timer which will call a callback at the specified rate.  This variant takes
 * a class member function, and a bare pointer to the object to call the method on.
 *
 * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
 * be stopped, and the callback will no longer be called.
 *
 * \param period The period at which to call the callback
 * \param callback The method to call
 * \param obj The object to call the method on
 * \param oneshot If true, this timer will only fire once
 * \param autostart If true (default), return timer that is already started
 */
template<class T>
Timer createTimer(Duration period, void(T::*callback)(const TimerEvent&), T* obj, 
                bool oneshot = false, bool autostart = true)
{
    return createTimer(period, boost::bind(callback, obj, boost::placeholders::_1), oneshot, autostart);
}

/**
 * \brief Create a timer which will call a callback at the specified rate.  This variant takes
 * a class member function, and a shared pointer to the object to call the method on.
 *
 * When the Timer (and all copies of it) returned goes out of scope, the timer will automatically
 * be stopped, and the callback will no longer be called.
 *
 * \param period The period at which to call the callback
 * \param callback The method to call
 * \param obj The object to call the method on.  Since this is a shared pointer, the object will
 * automatically be tracked with a weak_ptr so that if it is deleted before the Timer goes out of
 * scope the callback will no longer be called (and therefore will not crash).
 * \param oneshot If true, this timer will only fire once
 * \param autostart If true (default), return timer that is already started
 */
template<class T>
Timer createTimer(Duration period, void(T::*callback)(const TimerEvent&), 
        const boost::shared_ptr<T>& obj, bool oneshot = false, bool autostart = true)
{
    return createTimer(period, boost::bind(callback, obj.get(), boost::placeholders::_1), oneshot, autostart);
}

}
#endif