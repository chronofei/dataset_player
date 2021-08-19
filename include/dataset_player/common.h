#pragma once

// C/C++
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>

namespace dataset_player
{
	
template <typename T>
void initHeader(T & msg, const ros::Time& stamp, const std::string frameID, const uint seq)
{
	msg.header.stamp    = stamp;
	msg.header.frame_id = frameID;
	msg.header.seq      = seq;
	return;
}

} // end namespace dataset_player