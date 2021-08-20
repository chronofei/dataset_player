#include "dataset_player/KITTIDatasetPlayer.h"

namespace dataset_player
{

KITTIDatasetPlayer::KITTIDatasetPlayer(ros::NodeHandle & node, ros::NodeHandle & privateNode) 
: BasicDatasetPlayer(node, privateNode)
{
	// TODO
}

bool KITTIDatasetPlayer::initFilename(std::string & filename, const TopicType topicType, 
									  const std::string & pathOfDataset, const std::string & subDirectory, uint seq, bool left)
{
	std::stringstream ss;
	ss << std::setfill('0') << std::setw(6) << seq;

	if (topicType == TopicType::PointCloud)
		filename = pathOfDataset + "/data_odometry_velodyne/dataset/sequences/" + subDirectory + "/velodyne/" + ss.str() + ".bin";
	else if (topicType == TopicType::ImageColor)
		if (left)
			filename = pathOfDataset + "/data_odometry_color/dataset/sequences/" + subDirectory + "/image_2/" + ss.str() + ".png";
		else
			filename = pathOfDataset + "/data_odometry_color/dataset/sequences/" + subDirectory + "/image_3/" + ss.str() + ".png";
	else if (topicType == TopicType::ImageGry)
		if (left)
			filename = pathOfDataset + "/data_odometry_gry/" + subDirectory + "/image_0/" + ss.str() + ".png";
		else
			filename = pathOfDataset + "/data_odometry_gry/" + subDirectory + "/image_1/" + ss.str() + ".png";
	else if (topicType == TopicType::GroundTruth)
		filename = pathOfDataset + "/data_odometry_poses/dataset/poses/" + subDirectory + ".txt";

	return true;
}

bool KITTIDatasetPlayer::readPointCloud(const std::string & filename, pcl::PointCloud<pcl::PointXYZI> & pointcloud)
{
	std::ifstream fin(filename, std::ios::binary | std::ios::in);

	if(!fin.is_open())
	{
		std::cout << "can't open the file " << filename.substr(filename.find_last_of("/")+1) << "." << std::endl;
		return false;
	}

	float px, py, pz, pr;
	while(fin.read((char *)&px, sizeof(px)) &&
		  fin.read((char *)&py, sizeof(py)) &&
		  fin.read((char *)&pz, sizeof(pz)) &&
		  fin.read((char *)&pr, sizeof(pr)))
	{
		pcl::PointXYZI point;
		point.x = px;
		point.y = py;
		point.z = pz;
		point.intensity = pr;
		pointcloud.points.push_back(point);
	}
	pointcloud.width = pointcloud.points.size();
	pointcloud.height = 1;
	
	fin.close();

	ROS_INFO_STREAM("Read " << pointcloud.points.size() << " points from " << filename.substr(filename.find_last_of("/")+1) << " file.");

	return true;
}


bool KITTIDatasetPlayer::readGroundTruth(const std::string & filename, pcl::PointCloud<pcl::PointXYZI> & pointcloud)
{
	// TODO
}

} // end namespace dataset_player