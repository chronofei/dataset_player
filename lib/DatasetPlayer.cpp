#include "dataset_player/DatasetPlayer.h"

namespace dataset_player
{

DatasetPlayer::DatasetPlayer()
{
	// TODO
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
			_basicDatasetPlayer->setup();
		}
		else if (sParam == "TUM")
		{
			ROS_INFO_STREAM("Set datasetType: TUM");
			_basicDatasetPlayer = new TUMDatasetPlayer(node, privateNode);
			_basicDatasetPlayer->setup();
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
	_basicDatasetPlayer->process();
}

} // end namespace dataset_player