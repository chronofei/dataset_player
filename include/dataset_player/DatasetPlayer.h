#pragma once

// dataset_player
#include "dataset_player/KITTIDatasetPlayer.h"
#include "dataset_player/TUMDatasetPlayer.h"

// ros
#include <ros/ros.h>

namespace dataset_player
{

class DatasetPlayer
{
public:
	DatasetPlayer();
	bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);
	void spin();
private:
	BasicDatasetPlayer *_basicDatasetPlayer;
}; // end class DatasetPlayer

} // end namespace dataset_player