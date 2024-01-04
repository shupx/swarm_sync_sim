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

template <int N>  /* seperate static messages for UAV N */
MavlinkStreamer<N>::MavlinkStreamer()
{
    mavlink_stream_AttitudeQuaternion_ = STREAM_MAKE_PTR(MavlinkStreamAttitudeQuaternion)(50, this); // set streaming rate
    mavlink_stream_LocalPositionNED_ = STREAM_MAKE_PTR(MavlinkStreamLocalPositionNED)(30, this); // set streaming rate
    mavlink_stream_AttitudeTarget_ = STREAM_MAKE_PTR(MavlinkStreamAttitudeTarget)(50, this); // set streaming rate
    mavlink_stream_PositionTargetLocalNed_ = STREAM_MAKE_PTR(MavlinkStreamPositionTargetLocalNed)(30, this); // set streaming rate
    mavlink_stream_Heartbeat_ = STREAM_MAKE_PTR(MavlinkStreamHeartbeat)(10, this); // set streaming rate
    mavlink_stream_SysStatus_ = STREAM_MAKE_PTR(MavlinkStreamSysStatus)(2, this); // set streaming rate
    mavlink_stream_GlobalPositionInt_ = STREAM_MAKE_PTR(MavlinkStreamGlobalPositionInt)(30, this); // set streaming rate
    mavlink_stream_GpsGlobalOrigin_ = STREAM_MAKE_PTR(MavlinkStreamGpsGlobalOrigin)(100, this); // set streaming rate (Actually it will only send the mavlink stream when the gps origin is updated. Refer to streams/GPS_GLOBAL_ORIGIN.hpp)

    //@TODO SysStatus for battery
    //@TODO extended_sys_state for extended_state

}


template <int N>  /* seperate static messages for UAV N */
MavlinkStreamer<N>::~MavlinkStreamer()
{
    stream_signal_.disconnect_all_slots();
}

template <int N>  /* seperate static messages for UAV N */
void MavlinkStreamer<N>::Stream(const uint64_t &time_us)
{
    // stream all mavlink messages
    stream_signal_(time_us); 
}

template class MavlinkStreamer<1>; template class MavlinkStreamer<2>; template class MavlinkStreamer<3>; template class MavlinkStreamer<4>; template class MavlinkStreamer<5>; template class MavlinkStreamer<6>; template class MavlinkStreamer<7>; template class MavlinkStreamer<8>; template class MavlinkStreamer<9>; template class MavlinkStreamer<10>; 
template class MavlinkStreamer<11>; template class MavlinkStreamer<12>; template class MavlinkStreamer<13>; template class MavlinkStreamer<14>; template class MavlinkStreamer<15>; template class MavlinkStreamer<16>; template class MavlinkStreamer<17>; template class MavlinkStreamer<18>; template class MavlinkStreamer<19>; template class MavlinkStreamer<20>; 
template class MavlinkStreamer<21>; template class MavlinkStreamer<22>; template class MavlinkStreamer<23>; template class MavlinkStreamer<24>; template class MavlinkStreamer<25>; template class MavlinkStreamer<26>; template class MavlinkStreamer<27>; template class MavlinkStreamer<28>; template class MavlinkStreamer<29>; template class MavlinkStreamer<30>; 
template class MavlinkStreamer<31>; template class MavlinkStreamer<32>; template class MavlinkStreamer<33>; template class MavlinkStreamer<34>; template class MavlinkStreamer<35>; template class MavlinkStreamer<36>; template class MavlinkStreamer<37>; template class MavlinkStreamer<38>; template class MavlinkStreamer<39>; template class MavlinkStreamer<40>; 
template class MavlinkStreamer<41>; template class MavlinkStreamer<42>; template class MavlinkStreamer<43>; template class MavlinkStreamer<44>; template class MavlinkStreamer<45>; template class MavlinkStreamer<46>; template class MavlinkStreamer<47>; template class MavlinkStreamer<48>; template class MavlinkStreamer<49>; template class MavlinkStreamer<50>; 
template class MavlinkStreamer<51>; template class MavlinkStreamer<52>; template class MavlinkStreamer<53>; template class MavlinkStreamer<54>; template class MavlinkStreamer<55>; template class MavlinkStreamer<56>; template class MavlinkStreamer<57>; template class MavlinkStreamer<58>; template class MavlinkStreamer<59>; template class MavlinkStreamer<60>; 
template class MavlinkStreamer<61>; template class MavlinkStreamer<62>; template class MavlinkStreamer<63>; template class MavlinkStreamer<64>; template class MavlinkStreamer<65>; template class MavlinkStreamer<66>; template class MavlinkStreamer<67>; template class MavlinkStreamer<68>; template class MavlinkStreamer<69>; template class MavlinkStreamer<70>; 
template class MavlinkStreamer<71>; template class MavlinkStreamer<72>; template class MavlinkStreamer<73>; template class MavlinkStreamer<74>; template class MavlinkStreamer<75>; template class MavlinkStreamer<76>; template class MavlinkStreamer<77>; template class MavlinkStreamer<78>; template class MavlinkStreamer<79>; template class MavlinkStreamer<80>; 
template class MavlinkStreamer<81>; template class MavlinkStreamer<82>; template class MavlinkStreamer<83>; template class MavlinkStreamer<84>; template class MavlinkStreamer<85>; template class MavlinkStreamer<86>; template class MavlinkStreamer<87>; template class MavlinkStreamer<88>; template class MavlinkStreamer<89>; template class MavlinkStreamer<90>; 
template class MavlinkStreamer<91>; template class MavlinkStreamer<92>; template class MavlinkStreamer<93>; template class MavlinkStreamer<94>; template class MavlinkStreamer<95>; template class MavlinkStreamer<96>; template class MavlinkStreamer<97>; template class MavlinkStreamer<98>; template class MavlinkStreamer<99>; template class MavlinkStreamer<100>; 


/* The above explicit template instantiation declartions are 
 * auto-generated by the following python script:

#! /bin/python
import sys
# generate explicit template instantiation declartions

def output(s):
    sys.stdout.write(s)

def main(class_name, count):
    for i in range(int(count)):
        num = i+1
        output("template class {}<{}>; ".format(class_name, num))
        if num%10 == 0:
            output("\n")

if __name__ == '__main__':
    if len(sys.argv) > 2:
        main(sys.argv[1], sys.argv[2])
    else:
        print("[Error] Please input your class name and count after python xxx.py. For example: python xxx.py class_name 10")

# python generate_template.py class_name 100

*/