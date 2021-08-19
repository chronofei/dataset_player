#pragma once

// dataset_player
#include "dataset_player/BasicDatasetPlayer.h"

namespace dataset_player
{

class KITTIDatasetPlayer : public BasicDatasetPlayer
{
public:
	KITTIDatasetPlayer(ros::NodeHandle & node, ros::NodeHandle & privateNode);
	bool setup();

	bool initFilename(std::string & filename, const TopicType topicType, 
					  const std::string & pathOfDataset, const std::string & subDirectory, uint seq, bool left = true);

	bool readPointCloud(const std::string & filename, pcl::PointCloud<pcl::PointXYZI> & pointcloud);

	bool readGroundTruth(const std::string & filename, pcl::PointCloud<pcl::PointXYZI> & pointcloud);

private:
	ros::NodeHandle _privateNode;
}; // end class KITTIDataset

} // end namespace dataset_player