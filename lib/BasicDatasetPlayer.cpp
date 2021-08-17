#include "dataset_player/BasicDatasetPlayer.h"

namespace dataset_player
{

ConfigureParam::ConfigureParam()
{
	_pubPointCloud  = true;
	_pubImageColor  = false;
	_pubImageGry    = false;
	_pubImageDepth  = false;
	_pubGroundTruth = false;
}

ConfigureParam::ConfigureParam(const std::string pathOfDataset,    const std::string subDirectory,
	const bool pubPointCloud,  const std::string pointCloudTopic,  const float pointCloudRate,  const std::string pointCloudFrameID,
	const bool pubImageColor,  const std::string imageColorTopic,  const float imageColorRate,  const std::string imageColorFrameID,
	const bool pubImageGry,    const std::string imageGryTopic,    const float imageGryRate,    const std::string imageGryFrameID,
	const bool pubImageDepth,  const std::string imageDepthTopic,  const float imageDepthRate,  const std::string imageDepthFrameID,
	const bool pubGroundTruth, const std::string groundTruthTopic, const float groundTruthRate, const std::string groundTruthFrameID)
{
	_pathOfDataset      = pathOfDataset;
	_subDirectory       = subDirectory;

	_pubPointCloud      = pubPointCloud;
	_pointCloudTopic    = pointCloudTopic;
	_pointCloudRate     = pointCloudRate;
	_pointCloudFrameID  = pointCloudFrameID;

	_pubImageColor      = pubImageColor;
	_imageColorTopic    = imageColorTopic;
	_imageColorRate     = imageColorRate;
	_imageColorFrameID  = imageColorFrameID;

	_pubImageGry        = pubImageGry;
	_imageGryTopic      = imageGryTopic;
	_imageGryRate       = imageGryRate;
	_imageGryFrameID    = imageGryFrameID;

	_pubImageDepth      = pubImageDepth;
	_imageDepthTopic    = imageDepthTopic;
	_imageDepthRate     = imageDepthRate;
	_imageDepthFrameID  = imageDepthFrameID;

	_pubGroundTruth     = pubGroundTruth;
	_groundTruthTopic   = groundTruthTopic;
	_groundTruthRate    = groundTruthRate;
	_groundTruthFrameID = groundTruthFrameID;
}

ConfigureParam::ConfigureParam(const ConfigureParam & other)
{
	_pathOfDataset      = other._pathOfDataset;
	_subDirectory       = other._subDirectory;

	_pubPointCloud      = other._pubPointCloud;
	_pointCloudTopic    = other._pointCloudTopic;
	_pointCloudRate     = other._pointCloudRate;
	_pointCloudFrameID  = other._pointCloudFrameID;

	_pubImageColor      = other._pubImageColor;
	_imageColorTopic    = other._imageColorTopic;
	_imageColorRate     = other._imageColorRate;
	_imageColorFrameID  = other._imageColorFrameID;

	_pubImageGry        = other._pubImageGry;
	_imageGryTopic      = other._imageGryTopic;
	_imageGryRate       = other._imageGryRate;
	_imageGryFrameID    = other._imageGryFrameID;

	_pubImageDepth      = other._pubImageDepth;
	_imageDepthTopic    = other._imageDepthTopic;
	_imageDepthRate     = other._imageDepthRate;
	_imageDepthFrameID  = other._imageDepthFrameID;

	_pubGroundTruth     = other._pubGroundTruth;
	_groundTruthTopic   = other._groundTruthTopic;
	_groundTruthRate    = other._groundTruthRate;
	_groundTruthFrameID = other._groundTruthFrameID;
}

ConfigureParam ConfigureParam::operator=(const ConfigureParam & other)
{
	return ConfigureParam(other);
}

BasicDatasetPlayer::BasicDatasetPlayer(ros::NodeHandle node)
{
	_node        = node;
}

bool BasicDatasetPlayer::process()
{
	// TODO
}

void BasicDatasetPlayer::setPubPointCloud(bool flag)
{
	if (flag)
	{
		_configureParam._pubPointCloud = true;
		_pubPointCloud = _node.advertise<sensor_msgs::PointCloud2>(_configureParam._pointCloudTopic, 1);
	}
	else
		_configureParam._pubPointCloud = false;
}

void BasicDatasetPlayer::setPointCloudTopic(std::string pointCloudTopic)
{
	_configureParam._pointCloudTopic = pointCloudTopic;
}

void BasicDatasetPlayer::setPointCloudRate(float pointCloudRate)
{
	_configureParam._pointCloudRate = pointCloudRate;
}

void BasicDatasetPlayer::setPointCloudFrameID(std::string pointCloudFrameID)
{
	_configureParam._pointCloudFrameID = pointCloudFrameID;
}

void BasicDatasetPlayer::processPointCloud()
{
	ros::Rate loop_rate(_configureParam._pointCloudRate);
	
	int count = 0;
	while(ros::ok())
	{
		sensor_msgs::PointCloud2 pointcloud2;
		pcl::PointCloud<pcl::PointXYZI> pointcloud;

		std::stringstream ss;
		ss << std::setfill('0') << std::setw(6) << count++;
		std::string filename = _configureParam._pathOfDataset + "/data_odometry_velodyne/dataset/sequences/" + _configureParam._subDirectory + "/velodyne/" + ss.str() + ".bin";
		std::ifstream fin(filename, std::ios::binary | std::ios::in);

		if(!fin.is_open())
		{
			std::cout << "can't open the file " + _configureParam._pathOfDataset + "/sequences/" + _configureParam._subDirectory + "/velodyne/" + ss.str() + ".bin" << std::endl;
			break;
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

		ROS_INFO_STREAM("Read " << pointcloud.points.size() << " points from " << ss.str() + ".bin" << " file.");

		fin.close();

		pcl::toROSMsg(pointcloud, pointcloud2);

		pointcloud2.header.stamp    = ros::Time::now();
		pointcloud2.header.frame_id = _configureParam._pointCloudFrameID;
		pointcloud2.header.seq      = count;

		_pubPointCloud.publish(pointcloud2);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return;
}

void BasicDatasetPlayer::setPubImageColor(bool flag)
{
	_configureParam._pubImageColor = flag;
}

void BasicDatasetPlayer::setImageColorTopic(std::string imageColorTopic)
{
	_configureParam._imageColorTopic = imageColorTopic;
}

void BasicDatasetPlayer::setImageColorRate(float imageColorRate)
{
	_configureParam._imageColorRate = imageColorRate;
}

void BasicDatasetPlayer::setImageColorFrameID(std::string imageColorFrameID)
{
	_configureParam._imageColorFrameID = imageColorFrameID;
}

void BasicDatasetPlayer::processImageColor()
{
	// TODO
}

void BasicDatasetPlayer::setPubImageGry(bool flag)
{
	_configureParam._pubImageGry = flag;
}

void BasicDatasetPlayer::setImageGryTopic(std::string imageGryTopic)
{
	_configureParam._imageGryTopic = imageGryTopic;
}

void BasicDatasetPlayer::setImageGryRate(float imageGryRate)
{
	_configureParam._imageGryRate = imageGryRate;
}

void BasicDatasetPlayer::setImageGryFrameID(std::string imageColorFrameID)
{
	_configureParam._imageGryFrameID = _configureParam._imageGryFrameID;
}

void BasicDatasetPlayer::processImageGry()
{
	// TODO
}

void BasicDatasetPlayer::setPubImageDepth(bool flag)
{
	_configureParam._pubImageDepth = flag;
}

void BasicDatasetPlayer::setImageDepthTopic(std::string imageDepthTopic)
{
	_configureParam._imageDepthTopic = imageDepthTopic;
}

void BasicDatasetPlayer::setImageDepthRate(float imageDepthRate)
{
	_configureParam._imageDepthTopic = imageDepthRate;
}

void BasicDatasetPlayer::setImageDepthFrameID(std::string imageDepthFrameID)
{
	_configureParam._imageDepthFrameID = imageDepthFrameID;
}

void BasicDatasetPlayer::processImageDepth()
{
	// TODO
}

void BasicDatasetPlayer::setPubGroundTruth(bool flag)
{
	_configureParam._pubGroundTruth = flag;
}

void BasicDatasetPlayer::setGroundTruthTopic(std::string groundTruthTopic)
{
	_configureParam._groundTruthTopic = groundTruthTopic;
}

void BasicDatasetPlayer::setGroundTruthRate(float groundTruthRate)
{
	_configureParam._groundTruthTopic = groundTruthRate;
}

void BasicDatasetPlayer::setGroundTruthFrameID(std::string groundTruthFrameID)
{
	_configureParam._groundTruthFrameID = groundTruthFrameID;
}

void BasicDatasetPlayer::processGroundTruth()
{
	// TODO
}

const ConfigureParam & BasicDatasetPlayer::getConfigureParam()
{
	return _configureParam;
}

void BasicDatasetPlayer::setConfigureParam(const ConfigureParam & configureParam)
{
	_configureParam = configureParam;
}

void BasicDatasetPlayer::setPathOfDataset(std::string pathOfDataset)
{
	_configureParam._pathOfDataset = pathOfDataset;
}

void BasicDatasetPlayer::setSubDirectory(std::string subDirectory)
{
	_configureParam._subDirectory = subDirectory;
}


} // end namespace dataset_player