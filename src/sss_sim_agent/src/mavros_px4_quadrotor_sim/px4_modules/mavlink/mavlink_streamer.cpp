/**
 * @file mavlink_streamer.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Simulate the streaming of mavlink messages in PX4. 
 * 
 * Note: This program relies on mavlink
 * 
 * @version 1.0
 * @date 2023-12-17
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "mavlink_streamer.hpp"

MavlinkStreamer::MavlinkStreamer()
{
    mavlink_stream1_ = STREAM_MAKE_PTR(MavlinkStreamAttitudeQuaternion)(50, this); // set streaming rate
    mavlink_stream2_ = STREAM_MAKE_PTR(MavlinkStreamLocalPositionNED)(30, this); // set streaming rate
    mavlink_stream3_ = STREAM_MAKE_PTR(MavlinkStreamAttitudeTarget)(50, this); // set streaming rate
    mavlink_stream4_ = STREAM_MAKE_PTR(MavlinkStreamPositionTargetLocalNed)(30, this); // set streaming rate
    mavlink_stream5_ = STREAM_MAKE_PTR(MavlinkStreamHeartbeat)(10, this); // set streaming rate
    mavlink_stream6_ = STREAM_MAKE_PTR(MavlinkStreamSysStatus)(2, this); // set streaming rate
    mavlink_stream7_ = STREAM_MAKE_PTR(MavlinkStreamGlobalPositionInt)(30, this); // set streaming rate
    mavlink_stream7_ = STREAM_MAKE_PTR(MavlinkStreamGpsGlobalOrigin)(100, this); // set streaming rate (Actually it will only send the mavlink stream when the gps origin is updated. Refer to streams/GPS_GLOBAL_ORIGIN.hpp)

    //@TODO SysStatus for battery
    //@TODO extended_sys_state for extended_state

}


MavlinkStreamer::~MavlinkStreamer()
{
    stream_signal_.disconnect_all_slots();
}

void MavlinkStreamer::Stream(const uint64_t &time_us)
{
    // stream all mavlink messages
    stream_signal_(time_us); 
}