#pragma once

// dataset_player
#include "dataset_player/BasicDatasetPlayer.h"

namespace dataset_player
{

class KITTIDatasetPlayer : public BasicDatasetPlayer
{
public:
	KITTIDatasetPlayer(ros::NodeHandle node);
	bool process();
private:
	// TODO
}; // end class KITTIDataset

} // end namespace dataset_player