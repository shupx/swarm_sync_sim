/**
 * @file uas_timesync.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Change from mavros to mavros_sim
 * 
 * Similar to https://github.com/mavlink/mavros/blob/master/mavros/src/lib/uas_timesync.cpp
 * 
 * Note: This program relies on 
 * 
 * @version 1.0
 * @date 2023-12-19
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

/**
 * @brief MAVROS UAS manager
 * @file uas.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014,2017 Vladimir Ermakov, M.H.Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <array>
#include <unordered_map>
#include <stdexcept>
#include "mavros_uas.h"
// #include <mavros/utils.h>
#include <mavros/px4_custom_mode.h>

using namespace mavros_sim;


/* -*- time syncronise functions -*- */

static inline ros::Time ros_time_from_ns(const uint64_t stamp_ns) {
	return ros::Time(
		stamp_ns / 1000000000UL,		// t_sec
		stamp_ns % 1000000000UL);		// t_nsec
}

ros::Time UAS::synchronise_stamp(uint32_t time_boot_ms) {
	// copy offset from atomic var
	uint64_t offset_ns = time_offset;

	if (offset_ns > 0 || tsync_mode == timesync_mode::PASSTHROUGH) {
		uint64_t stamp_ns = static_cast<uint64_t>(time_boot_ms) * 1000000UL + offset_ns;
		return ros_time_from_ns(stamp_ns);
	}
	else
		return ros::Time::now();
}

ros::Time UAS::synchronise_stamp(uint64_t time_usec) {
	uint64_t offset_ns = time_offset;

	if (offset_ns > 0 || tsync_mode == timesync_mode::PASSTHROUGH) {
		uint64_t stamp_ns = time_usec * 1000UL + offset_ns;
		return ros_time_from_ns(stamp_ns);
	}
	else
		return ros::Time::now();
}

