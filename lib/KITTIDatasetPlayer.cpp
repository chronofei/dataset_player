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
		ROS_INFO_STREAM("Begin point cloud thread!");
		std::thread threadPointCloud(&KITTIDatasetPlayer::processPointCloud, this);
		threadPointCloud.detach();
	}

	if (getConfigureParam()._pubImageColor)
	{
		ROS_INFO_STREAM("Begin image color thread!");
		std::thread threadImageColor(&KITTIDatasetPlayer::processImageColor, this);
		threadImageColor.detach();
	}

	if (getConfigureParam()._pubImageGry)
	{
		ROS_INFO_STREAM("Begin image gry thread!");
		std::thread threadImageGry(&KITTIDatasetPlayer::processImageGry, this);
		threadImageGry.detach();
	}

	if (getConfigureParam()._pubGroundTruth)
	{
		ROS_INFO_STREAM("Begin ground truth thread!");
		std::thread threadGroundTruth(&KITTIDatasetPlayer::processGroundTruth, this);
		threadGroundTruth.detach();
	}
	
	return true;
}

} // end namespace dataset_player