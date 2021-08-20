#include "dataset_player/TUMDatasetPlayer.h"

namespace dataset_player
{

TUMDatasetPlayer::TUMDatasetPlayer(ros::NodeHandle & node, ros::NodeHandle & privateNode)
: BasicDatasetPlayer(node, privateNode)
{
	// TODO
}

bool TUMDatasetPlayer::initFilename(std::string & filename, const TopicType topicType, 
				  const std::string & pathOfDataset, const std::string & subDirectory, uint seq, bool left)
{
	// TODO
}

bool TUMDatasetPlayer::readGroundTruth(const std::string & filename, pcl::PointCloud<pcl::PointXYZI> & pointcloud)
{
	// TODO
}

} // end namesapce dataset_player