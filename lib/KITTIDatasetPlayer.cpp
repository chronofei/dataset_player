#include "dataset_player/KITTIDatasetPlayer.h"

namespace dataset_player
{

KITTIDatasetPlayer::KITTIDatasetPlayer(ros::NodeHandle node) 
: BasicDatasetPlayer(node)
{
	// TODO
}

bool KITTIDatasetPlayer::process()
{
	if (getConfigureParam()._pubPointCloud)
	{
		std::thread threadPointCloud(&KITTIDatasetPlayer::processPointCloud, this);
		threadPointCloud.detach();
	}

	if (getConfigureParam()._pubImageColor)
	{
		std::thread threadImageColor(&KITTIDatasetPlayer::processImageColor, this);
		threadImageColor.detach();
	}

	if (getConfigureParam()._pubImageGry)
	{
		std::thread threadImageGry(&KITTIDatasetPlayer::processImageGry, this);
		threadImageGry.detach();
	}

	if (getConfigureParam()._pubGroundTruth)
	{
		std::thread threadGroundTruth(&KITTIDatasetPlayer::processGroundTruth, this);
		threadGroundTruth.detach();
	}
	
	return true;
}

} // end namespace dataset_player