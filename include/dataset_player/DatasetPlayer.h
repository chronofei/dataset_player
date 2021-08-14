#pragma once

// dataset_player
#include "dataset_player/KITTIDatasetPlayer.h"
#include "dataset_player/TUMDatasetPlayer.h"

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace dataset_player
{

enum DatasetType
{
	KITTI = 0,
	TUM = 1
};

class DatasetPlayer
{
public:
	DatasetPlayer();
	bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);
	void spin();
private:
	BasicDatasetPlayer *_basicDatasetPlayer;

	DatasetType _datasetType;

	ros::Publisher _pubPointCloud;

	ros::Publisher _pubImageColor;
	ros::Publisher _pubImageGry;
	ros::Publisher _pubImageDepth;

	ros::Publisher _pubGroundTruth;
}; // end class DatasetPlayer

} // end namespace dataset_player