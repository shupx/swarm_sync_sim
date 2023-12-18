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
    mavlink_stream1 = STREAM_MAKE_PTR(MavlinkStreamAttitudeQuaternion)(50, this);
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