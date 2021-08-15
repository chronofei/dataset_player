#include "dataset_player/TUMDatasetPlayer.h"

namespace dataset_player
{

TUMDatasetPlayer::TUMDatasetPlayer(ros::NodeHandle node)
: BasicDatasetPlayer(node)
{
	// TODO
}

bool TUMDatasetPlayer::process()
{
	if (getConfigureParam()._pubImageColor)
	{
		std::thread threadImageColor(&TUMDatasetPlayer::processImageColor, this);
		threadImageColor.detach();
	}

	if (getConfigureParam()._pubImageDepth)
	{
		std::thread threadImageDepth(&TUMDatasetPlayer::processImageDepth, this);
		threadImageDepth.detach();
	}

	if (getConfigureParam()._pubGroundTruth)
	{
		std::thread threadGroundTruth(&TUMDatasetPlayer::processGroundTruth, this);
		threadGroundTruth.detach();
	}

	return true;
}

} // end namesapce dataset_player