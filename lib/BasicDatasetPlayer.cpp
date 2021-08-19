#include "dataset_player/BasicDatasetPlayer.h"

namespace dataset_player
{

ConfigureParam::ConfigureParam()
{
	_pubPointCloud  = false;
	_pubImageColor  = false;
	_pubImageGry    = false;
	_pubImageDepth  = false;
	_pubGroundTruth = false;
}

BasicDatasetPlayer::BasicDatasetPlayer(ros::NodeHandle & node)
{
	_node = node;	
}

bool _endOfPointCloud   = true;
bool _endOfImageGry     = true;
bool _endOfImageColor   = true;
bool _endOfImageDepth   = true;
bool _endOfGroundTruth  = true;

bool BasicDatasetPlayer::process()
{
	if (getConfigureParam()._pubPointCloud)
	{
		ROS_INFO_STREAM("Begin point cloud thread!");
		_endOfPointCloud = false;
		std::thread threadPointCloud(&BasicDatasetPlayer::processPointCloud, this);
		threadPointCloud.detach();
	}

	if (getConfigureParam()._pubImageColor)
	{
		ROS_INFO_STREAM("Begin image color thread!");
		_endOfImageColor = false;
		std::thread threadImageColor(&BasicDatasetPlayer::processImage, this, TopicType::ImageColor, _configureParam._isStereo);
		threadImageColor.detach();
	}

	if (getConfigureParam()._pubImageGry)
	{
		ROS_INFO_STREAM("Begin image gry thread!");
		_endOfImageGry = false;
		std::thread threadImageGry(&BasicDatasetPlayer::processImage, this, TopicType::ImageGry, _configureParam._isStereo);
		threadImageGry.detach();
	}

	if (getConfigureParam()._pubImageDepth)
	{
		ROS_INFO_STREAM("Begin image gry thread!");
		_endOfImageDepth = false;
		std::thread threadImageDepth(&BasicDatasetPlayer::processImage, this, TopicType::ImageDepth, false);
		threadImageDepth.detach();
	}

	if (getConfigureParam()._pubGroundTruth)
	{
		ROS_INFO_STREAM("Begin ground truth thread!");
		_endOfGroundTruth = false;
		std::thread threadGroundTruth(&BasicDatasetPlayer::processGroundTruth, this);
		threadGroundTruth.detach();
	}

	while (ros::ok() && (!_endOfPointCloud || !_endOfImageColor || !_endOfImageGry || !_endOfImageDepth || !_endOfGroundTruth))
		ros::spinOnce();
	
	return true;
}

bool BasicDatasetPlayer::initPointCloud(sensor_msgs::PointCloud2 & pointcloud2, uint seq)
{
	pcl::PointCloud<pcl::PointXYZI> pointcloud;

	std::string filename;

	initFilename(filename, TopicType::PointCloud, _configureParam._pathOfDataset, _configureParam._subDirectory, seq);

	if (!readPointCloud(filename, pointcloud))
		return false;

	pcl::toROSMsg(pointcloud, pointcloud2);

	initHeader(pointcloud2, ros::Time::now(), _configureParam._pointCloudFrameID, seq);

	ROS_INFO_STREAM("Read " << seq << "th pointcloud from " << filename.substr(0, filename.find_last_of("/")) << " directory.");

	return true;
}

void BasicDatasetPlayer::processPointCloud()
{
	ros::Publisher _pubPointCloud = _node.advertise<sensor_msgs::PointCloud2>(_configureParam._pointCloudTopic, 1);

	ros::Rate loop_rate(_configureParam._pointCloudRate);
	
	uint seq = 0;
	while(ros::ok())
	{
		sensor_msgs::PointCloud2 pointcloud2;
		if (!initPointCloud(pointcloud2, seq))
		{
			_endOfPointCloud   = true;
			break;
		}
		_pubPointCloud.publish(pointcloud2);
		seq++;
		loop_rate.sleep();
	}

	return;
}

bool BasicDatasetPlayer::readImage(const std::string & filename, cv::Mat image, TopicType topicType)
{
	if (topicType == TopicType::ImageColor)
		image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
	else if (topicType == TopicType::ImageGry)
		image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
	else if (topicType == TopicType::ImageDepth)
		return false;

	if (image.empty())
	{
		ROS_INFO_STREAM("can't read image data from " << filename.substr(filename.find_last_of("/")+1) << " file.");
		return false;
	}

	ROS_INFO_STREAM("Read a " << image.rows << "x" << image.cols << ":" << image.channels() << " image from " << filename.substr(filename.find_last_of("/")+1) << " file.");
	return true;
}

bool BasicDatasetPlayer::initImage(sensor_msgs::ImagePtr imagePtr, uint seq, TopicType topicType, bool left)
{
	std::string filename;

	initFilename(filename, topicType, _configureParam._pathOfDataset, _configureParam._subDirectory, seq, left);

	cv::Mat image;

	if (!readImage(filename, image, topicType))
		return false;

	imagePtr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

	initHeader(*imagePtr, ros::Time::now(), _configureParam._imageFrameID, seq);

	ROS_INFO_STREAM("Read " << seq << "th image from " << filename.substr(0, filename.find_last_of("/")) << " directory.");

	return true;
}

void BasicDatasetPlayer::processImage(TopicType topicType, bool isStereo)
{
	image_transport::ImageTransport it(_node);
	image_transport::Publisher _pubImageLeft;
	image_transport::Publisher _pubImageRight;

	if (topicType == TopicType::ImageColor)
	{
		_pubImageLeft  = it.advertise(_configureParam._imageTopic + "/color/left", 1);
		_pubImageRight = it.advertise(_configureParam._imageTopic + "/color/right", 1);
	}
	else if (topicType == TopicType::ImageGry)
	{
		_pubImageLeft  = it.advertise(_configureParam._imageTopic + "/gry/left", 1);
		_pubImageRight = it.advertise(_configureParam._imageTopic + "/gry/right", 1);
	}
	else if (topicType == TopicType::ImageDepth)
	{
		return;
	}

	ros::Rate loop_rate(_configureParam._imageRate);

	uint seq = 0;
	while(ros::ok())
	{
		sensor_msgs::ImagePtr imageLeft;
		sensor_msgs::ImagePtr imageRight;

		if (!initImage(imageLeft, seq, topicType, true))
		{
			if (topicType == TopicType::ImageColor)
				_endOfImageColor = true;
			else if (topicType == TopicType::ImageGry)
				_endOfImageGry = true;
			else if (topicType == TopicType::ImageDepth)
				_endOfImageDepth = true;
			break;
		}

		if (isStereo)
			initImage(imageRight, seq, topicType, false);

		_pubImageLeft.publish(imageLeft);
		if (isStereo)
			_pubImageRight.publish(imageRight);
			
		seq++;
		loop_rate.sleep();
	}
}

void BasicDatasetPlayer::processGroundTruth()
{
	// TODO
}


} // end namespace dataset_player