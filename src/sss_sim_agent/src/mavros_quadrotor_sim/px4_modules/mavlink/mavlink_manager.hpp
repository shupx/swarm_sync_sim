/**
 * @file mavlink_manager.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Simulate the sending and receiving of mavlink messages in PX4. 
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

#include "mavlink_msg_list.hpp"
#include "mavlink_receiver.h"


class MavlinkManager
{
public:
	MavlinkManager()
	{
   		mavlink_receiver_ = std::make_shared<MavlinkReceiver>();
	}

	~MavlinkManager() {}

	/* Search for mavlink receiving list and handle the updated messages (transfer into PX4 uORB messages) */
	void Receive();

private:
	std::shared_ptr<MavlinkReceiver> mavlink_receiver_;

};


void MavlinkManager::Receive()
{
	/* Search for mavlink receiving list and handle the updated messages */
	for (int i=0; i<MAVLINK_RECEIVE_NUM; ++i)
	{
		mavlink_info_s mavlink_info = px4::mavlink_receive_list[i];
		
		if (mavlink_info.updated)
		{
			mavlink_receiver_->handle_message(&mavlink_info.msg);
			mavlink_info.updated = false; // waiting for the next update
		}
	}


}


