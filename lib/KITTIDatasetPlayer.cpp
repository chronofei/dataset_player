#include "dataset_player/KITTIDatasetPlayer.h"

namespace dataset_player
{

KITTIDatasetPlayer::KITTIDatasetPlayer(ros::NodeHandle & node, ros::NodeHandle & privateNode) 
: BasicDatasetPlayer(node)
{
	_privateNode = privateNode;
}

bool KITTIDatasetPlayer::setup()
{
	std::string sParam;
	bool        bParam;
	float       fParam;

	if (_privateNode.getParam("pathOfDataset", sParam))
	{
		ROS_INFO_STREAM("Set pathOfDataset: " << sParam);
		getConfigureParam()._pathOfDataset = sParam;
	}
	else
		ROS_INFO_STREAM("Set pathOfDataset: morenzhi");

	if (_privateNode.getParam("subDirectory", sParam))
	{
		ROS_INFO_STREAM("Set subDirectory: " << sParam);
		getConfigureParam()._subDirectory = sParam;
	}
	else
		ROS_INFO_STREAM("Set subDirectory: morenzhi");

	if (_privateNode.getParam("pubPointCloud", bParam) && bParam)
	{
		ROS_INFO_STREAM("Set pubPointCloud: TRUE");
		getConfigureParam()._pubPointCloud = true;
		if (_privateNode.getParam("pointCloudTopic", sParam))
		{
			ROS_INFO_STREAM("Set pointCloudTopic: " << sParam);
			getConfigureParam()._pointCloudTopic = sParam;
		}
		else
			ROS_INFO_STREAM("Set pointCloudTopic: morenzhi");
		if (_privateNode.getParam("pointCloudRate", fParam))
		{
			ROS_INFO_STREAM("Set pointCloudRate: " << fParam);
			getConfigureParam()._pointCloudRate = fParam;
		}
		else
			ROS_INFO_STREAM("Set pointCloudRate: morenzhi");
		if (_privateNode.getParam("pointCloudFrameID", sParam))
		{
			ROS_INFO_STREAM("Set pointCloudFrameID: " << sParam);
			getConfigureParam()._pointCloudFrameID = sParam;
		}
		else
			ROS_INFO_STREAM("Set pointCloudFrameID: morenzhi");
	}
	else
	{
		ROS_INFO_STREAM("Set pubPointCloud: FALSE");
		getConfigureParam()._pubPointCloud = false;
	}

	if (_privateNode.getParam("pubImageColor", bParam) && bParam)
	{
		ROS_INFO_STREAM("Set pubImageColor: TRUE");
		getConfigureParam()._pubImageColor = true;
	}
	else
	{
		getConfigureParam()._pubImageColor = false;
		ROS_INFO_STREAM("Set pubImageColor: FALSE");
	}

	if (_privateNode.getParam("pubImageGry", bParam) && bParam)
	{
		ROS_INFO_STREAM("Set pubImageGry: TRUE");
		getConfigureParam()._pubImageGry = true;
	}
	else
	{
		getConfigureParam()._pubImageGry = false;
		ROS_INFO_STREAM("Set pubImageGry: FALSE");
	}

	if (getConfigureParam()._pubImageGry || getConfigureParam()._pubImageColor)
	{
		if (_privateNode.getParam("isStereo", bParam))
		{
			ROS_INFO_STREAM("Set isStereo: TRUE");
			getConfigureParam()._isStereo = true;
		}
		else
		{
			getConfigureParam()._isStereo = false;
			ROS_INFO_STREAM("Set isStereo: FALSE");
		}
		if (_privateNode.getParam("imageTopic", sParam))
		{
			ROS_INFO_STREAM("Set imageTopic: " << sParam);
			getConfigureParam()._imageTopic = sParam;
		}
		else
			ROS_INFO_STREAM("Set imageTopic: morenzhi");
		if (_privateNode.getParam("imageRate", fParam))
		{
			ROS_INFO_STREAM("Set imageRate: " << fParam);
			getConfigureParam()._imageRate = fParam;
		}
		else
			ROS_INFO_STREAM("Set imageRate: morenzhi");
		if (_privateNode.getParam("imageFrameID", sParam))
		{
			ROS_INFO_STREAM("Set imageFrameID: " << sParam);
			getConfigureParam()._imageFrameID = sParam;
		}
		else
			ROS_INFO_STREAM("Set imageFrameID: morenzhi");
	}
	
	if (_privateNode.getParam("pubGroundTruth", bParam) && bParam)
	{
		ROS_INFO_STREAM("Set pubGroundTruth: TRUE");
		getConfigureParam()._pubGroundTruth = true;
		if (_privateNode.getParam("groundTruthTopic", sParam))
		{
			ROS_INFO_STREAM("Set groundTruthTopic: " << sParam);
			getConfigureParam()._groundTruthTopic = sParam;
		}
		else
			ROS_INFO_STREAM("Set groundTruthTopic: morenzhi");
		if (_privateNode.getParam("groundTruthRate", fParam))
		{
			ROS_INFO_STREAM("Set groundTruthRate: " << fParam);
			getConfigureParam()._groundTruthRate = fParam;
		}
		else
			ROS_INFO_STREAM("Set groundTruthRate: morenzhi");
		if (_privateNode.getParam("groundTruthFrameID", sParam))
		{
			ROS_INFO_STREAM("Set groundTruthFrameID: " << sParam);
			getConfigureParam()._groundTruthFrameID = sParam;
		}
		else
			ROS_INFO_STREAM("Set groundTruthFrameID: morenzhi");
	}
	else
	{
		getConfigureParam()._pubGroundTruth = false;
		ROS_INFO_STREAM("Set pubGroundTruth: FALSE");
	}
	
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