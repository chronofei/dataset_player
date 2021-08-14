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
	bool bParam;

	if (privateNode.getParam("datasetType", sParam))
	{
		if (sParam == "KITTI")
		{
			ROS_INFO("Set datasetType: KITTI");
			_datasetType = DatasetType::KITTI;
			_basicDatasetPlayer = new KITTIDatasetPlayer;
		}
		else if (sParam == "TUM")
		{
			ROS_INFO("Set datasetType: TUM");
			_datasetType = DatasetType::TUM;
			_basicDatasetPlayer = new TUMDatasetPlayer;
		}
		else
		{
			ROS_ERROR("Invalid datasetType parameter: %s", sParam);
			return false;
		}
	}
	else
	{
		ROS_ERROR("Unable to get the type of dataset");
		return false;
	}

	if(_datasetType == DatasetType::TUM)
	{
		// if (privateNode.getParam("pubRGB", bParam))
		// {
		// 	if(bParam)
		// 	{
		// 		_pubImageColor = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
		// 	}
		// 	_basicDatasetPlayer->setPubRGB(bParam);
		// }

		// if (parivateNode.getParam("pubDepth", bParam))
		// {
		// 	if (bParam)
		// 	{
		// 		_pubImageColor = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
		// 	}
		// 	_basicDatasetPlayer->setPubDepth(bParam);
		// }

		// if (privateNode.getParam("pubGroundTruth", bParam))
		// {
		// 	if (bParam)
		// 	{
		// 		_pubImageColor = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
		// 	}
		// 	_basicDatasetPlayer->setPubGroundTruth(bParam);
		// }
	}
	else if (_datasetType == DatasetType::KITTI)
	{
		if (privateNode.getParam("pubPointCloud", bParam))
		{
			if (bParam)
			{
				_pubPointCloud = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1);
			}
			_basicDatasetPlayer->setPubPointCloud(bParam);

		}
		// if (privateNode.getParam("pubColor", bParam))
		// {
		// 	if (bParam)
		// 	{
		// 		_pubImageColor = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1);
		// 	}
		// 	_basicDatasetPlayer->setPubColor(bParam);
		// }
		// if (privateNode.getParam("pubGry", bParam))
		// {
		// 	if (bParam)
		// 	{
		// 		_pubImageGry = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1);
		// 	}
		// 	_basicDatasetPlayer->setPubGry(bParam);
		// }
		// if (privateNode.getParam("pubGroundTruth", bParam))
		// {
		// 	if (bParam)
		// 	{
		// 		_pubGroundTruth = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1);
		// 	}
		// 	_basicDatasetPlayer->setPubGroundTruth(bParam);
		// }
	}
	
	return true;
}


void DatasetPlayer::spin()
{
	// TODO
}

} // end namespace dataset_player