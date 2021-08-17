#pragma once

// C/C++
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>

// pcl
#include<pcl/point_cloud.h>

// dataset_player
#include "dataset_player/common.h"

namespace dataset_player
{

class ConfigureParam
{
public:
	ConfigureParam();
	ConfigureParam(const std::string pathOfDataset,    const std::string subDirectory,
				   const bool pubPointCloud,  const std::string pointCloudTopic,  const float pointCloudRate,  const std::string pointCloudFrameID,
		           const bool pubImageColor,  const std::string imageColorTopic,  const float imageColorRate,  const std::string imageColorFrameID,
		           const bool pubImageGry,    const std::string imageGryTopic,    const float imageGryRate,    const std::string imageGryFrameID,
				   const bool pubImageDepth,  const std::string imageDepthTopic,  const float imageDepthRate,  const std::string imageDepthFrameID,
				   const bool pubGroundTruth, const std::string groundTruthTopic, const float groundTruthRate, const std::string groundTruthFrameID);
	ConfigureParam(const ConfigureParam & other);
	ConfigureParam operator=(const ConfigureParam & other);

	bool        _pubPointCloud;
	std::string _pointCloudTopic;
	float       _pointCloudRate;
	std::string _pointCloudFrameID;

	bool        _pubImageColor;
	std::string _imageColorTopic;
	float       _imageColorRate;
	std::string _imageColorFrameID;

	bool        _pubImageGry;
	std::string _imageGryTopic;
	float       _imageGryRate;
	std::string _imageGryFrameID;

	bool        _pubImageDepth;
	std::string _imageDepthTopic;
	float       _imageDepthRate;
	std::string _imageDepthFrameID;

	bool        _pubGroundTruth;
	std::string _groundTruthTopic;
	float       _groundTruthRate;
	std::string _groundTruthFrameID;

	std::string _pathOfDataset;
	std::string _subDirectory;


}; // end class ConfigureParam

class BasicDatasetPlayer
{
public:
	BasicDatasetPlayer(ros::NodeHandle node);
	virtual bool process();

	void setPubPointCloud(bool flag);
	void setPointCloudTopic(std::string pointCloudTopic);
	void setPointCloudRate(float pointCloudRate);
	void setPointCloudFrameID(std::string pointCloudFrameID);
	void processPointCloud();

	void setPubImageColor(bool flag);
	void setImageColorTopic(std::string imageColorTopic);
	void setImageColorRate(float imageColorRate);
	void setImageColorFrameID(std::string imageColorFrameID);
	void processImageColor();

	void setPubImageGry(bool flag);
	void setImageGryTopic(std::string imageGryTopic);
	void setImageGryRate(float imageGryRate);
	void setImageGryFrameID(std::string imageColorFrameID);
	void processImageGry();

	void setPubImageDepth(bool flag);
	void setImageDepthTopic(std::string imageDepthTopic);
	void setImageDepthRate(float imageDepthRate);
	void setImageDepthFrameID(std::string imageDepthFrameID);
	void processImageDepth();

	void setPubGroundTruth(bool flag);
	void setGroundTruthTopic(std::string groundTruthTopic);
	void setGroundTruthRate(float groundTruthRate);
	void setGroundTruthFrameID(std::string groundTruthFrameID);
	void processGroundTruth();

	const ConfigureParam & getConfigureParam();
	void setConfigureParam(const ConfigureParam & configureParam);

	void setPathOfDataset(std::string pathOfDataset);
	void setSubDirectory(std::string subDirectory);

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