#pragma once

// C/C++
#include <thread>

// dataset_player
#include "dataset_player/BasicDatasetPlayer.h"

namespace dataset_player
{

class TUMDatasetPlayer : public BasicDatasetPlayer
{
public:
	TUMDatasetPlayer(ros::NodeHandle & node, ros::NodeHandle & privateNode);
	bool setup();
private:
	ros::NodeHandle _node;
	ros::NodeHandle _privateNode;
}; // end class TUMDataset

} // end namespace dataset_player