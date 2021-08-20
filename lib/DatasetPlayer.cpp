#include "dataset_player/DatasetPlayer.h"

namespace dataset_player
{

DatasetPlayer::DatasetPlayer()
{
	_basicDatasetPlayer = NULL;
}

bool DatasetPlayer::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
	std::string sParam;

	if (privateNode.getParam("datasetType", sParam))
	{
		if (sParam == "KITTI")
		{
			ROS_INFO_STREAM("Set datasetType: KITTI");
			_basicDatasetPlayer = new KITTIDatasetPlayer(node, privateNode);
		}
		else if (sParam == "TUM")
		{
			ROS_INFO_STREAM("Set datasetType: TUM");
			_basicDatasetPlayer = new TUMDatasetPlayer(node, privateNode);
		}
		else
		{
			ROS_ERROR_STREAM("Invalid datasetType parameter: " << sParam);
			return false;
		}
	}
	else
	{
		ROS_ERROR_STREAM("Unable to get the type of dataset");
		return false;
	}

	return true;
}

void DatasetPlayer::spin()
{
	if (!_basicDatasetPlayer->setup())
	{
		ROS_INFO_STREAM("Dataset Player setup fail! Please check if some necessary parameters are provided.");
		return;
	}

	_basicDatasetPlayer->process();

	return;
}

} // end namespace dataset_player