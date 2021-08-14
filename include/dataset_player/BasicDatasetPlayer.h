#pragma once

// C/C++
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// dataset_player
#include "dataset_player/common.h"

namespace dataset_player
{

class ConfigureParam
{
public:
	ConfigureParam();
	ConfigureParam(const bool pubPointCloud,  const std::string pointCloudTopic,  const float pointCloudRate,
		           const bool pubImageColor,  const std::string imageColorTopic,  const float imageColorRate,
		           const bool pubImageGry,    const std::string imageGryTopic,    const float imageGryRate,
				   const bool pubImageDepth,  const std::string imageDepthTopic,  const float imageDepthRate,
				   const bool pubGroundTruth, const std::string groundTruthTopic, const float groundTruthRate);
	ConfigureParam(const ConfigureParam & other);

	bool        _pubPointCloud;
	std::string _pointCloudTopic;
	float       _pointCloudRate;

	bool        _pubImageColor;
	std::string _imageColorTopic;
	float       _imageColorRate;

	bool        _pubImageGry;
	std::string _imageGryTopic;
	float       _imageGryRate;

	bool        _pubImageDepth;
	std::string _imageDepthTopic;
	float       _imageDepthRate;

	bool        _pubGroundTruth;
	std::string _groundTruthTopic;
	float       _groundTruthRate;


};

class BasicDatasetPlayer
{
public:
	BasicDatasetPlayer(ros::NodeHandle node);
	virtual bool process();

	void setPubPointCloud(bool flag);
	void setPointCloudTopic(std::string pointCloudTopic);
	void setPointCloudRate(float pointCloudRate);

	void setPubImageColor(bool flag);
	void setImageColorTopic(std::string imageColorTopic);
	void setImageColorRate(float imageColorRate);

	void setPubImageGry(bool flag);
	void setImageGryTopic(std::string imageGryTopic);
	void setImageGryRate(float imageGryRate);

	void setPubImageDepth(bool flag);
	void setImageDepthTopic(std::string imageDepthTopic);
	void setImageDepthRate(float imageDepthRate);

	void setPubGroundTruth(bool flag);
	void setGroundTruthTopic(std::string groundTruthTopic);
	void setGroundTruthRate(float groundTruthRate);

private:
	ConfigureParam _configureParam;

	ros::NodeHandle _node;

	ros::Publisher _pubPointCloud;
	ros::Publisher _pubImageColor;
	ros::Publisher _pubImageGry;
	ros::Publisher _pubImageDepth;
	ros::Publisher _pubGroundTruth;
}; // end class BasicDatasetPlayer

} // end namespace dataset_player