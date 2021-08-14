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

ConfigureParam::ConfigureParam(const bool pubPointCloud,  const std::string pointCloudTopic,  const float pointCloudRate,
	           const bool pubImageColor,  const std::string imageColorTopic,  const float imageColorRate,
	           const bool pubImageGry,    const std::string imageGryTopic,    const float imageGryRate,
			   const bool pubImageDepth,  const std::string imageDepthTopic,  const float imageDepthRate,
			   const bool pubGroundTruth, const std::string groundTruthTopic, const float groundTruthRate)
{
	_pubPointCloud    = pubPointCloud;
	_pointCloudTopic  = pointCloudTopic;
	_pointCloudRate   = pointCloudRate;

	_pubImageColor    = pubImageColor;
	_imageColorTopic  = imageColorTopic;
	_imageColorRate   = imageColorRate;

	_pubImageGry      = pubImageGry;
	_imageGryTopic    = imageGryTopic;
	_imageGryRate     = imageGryRate;

	_pubImageDepth    = pubImageDepth;
	_imageDepthTopic  = imageDepthTopic;
	_imageDepthRate   = imageDepthRate;

	_pubGroundTruth   = pubGroundTruth;
	_groundTruthTopic = groundTruthTopic;
	_groundTruthRate  = groundTruthRate;
}

ConfigureParam::ConfigureParam(const ConfigureParam & other)
{
	_pubPointCloud    = other._pubPointCloud;
	_pointCloudTopic  = other._pointCloudTopic;
	_pointCloudRate   = other._pointCloudRate;

	_pubImageColor    = other._pubImageColor;
	_imageColorTopic  = other._imageColorTopic;
	_imageColorRate   = other._imageColorRate;

	_pubImageGry      = other._pubImageGry;
	_imageGryTopic    = other._imageGryTopic;
	_imageGryRate     = other._imageGryRate;

	_pubImageDepth    = other._pubImageDepth;
	_imageDepthTopic  = other._imageDepthTopic;
	_imageDepthRate   = other._imageDepthRate;

	_pubGroundTruth   = other._pubGroundTruth;
	_groundTruthTopic = other._groundTruthTopic;
	_groundTruthRate  = other._groundTruthRate;
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
		_pubPointCloud = _node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1);
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
} // end namespace dataset_player