/**
 * @file mavlink_streamer.hpp
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

#pragma once

#include <iostream> // for std::cout, std::endl
#include <memory>  // for std::shared_ptr

#include "mavlink_msg_list.hpp"  // store the simulated static(global) mavlink messages

class MavlinkStreamer
{
public:
	MavlinkStreamer();

	~MavlinkStreamer() {}

	void Stream(const uint64_t &time_us);

private:

};


