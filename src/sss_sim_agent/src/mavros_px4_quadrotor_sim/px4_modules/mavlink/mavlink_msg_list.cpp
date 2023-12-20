/**
 * @file mavlink_msg_list.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Store the mavlink messages to simulate the send and receive of mavlink messages. 
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-12-17
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */


#include "mavlink_msg_list.hpp"


namespace px4 
{ 

// Init mavlink lists here (they are global/extern!)

mavlink_info_s mavlink_stream_list[MAVLINK_STREAM_NUM]; // define (allocate storage)

mavlink_info_s mavlink_receive_list[MAVLINK_RECEIVE_NUM]; // define (allocate 


}