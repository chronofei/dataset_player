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

	bool initFilename(std::string & filename, const TopicType topicType, 
					  const std::string & pathOfDataset, const std::string & subDirectory, uint seq, bool left = true);

	bool readGroundTruth(const std::string & filename, pcl::PointCloud<pcl::PointXYZI> & pointcloud);
}; // end class TUMDataset

} // end namespace dataset_player