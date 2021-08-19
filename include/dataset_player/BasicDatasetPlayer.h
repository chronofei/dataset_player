#pragma once

// C/C++
#include <iostream>
#include <string>
#include <thread>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// pcl
#include<pcl/point_cloud.h>

// opencv
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// dataset_player
#include "dataset_player/common.h"

namespace dataset_player
{

enum TopicType
{
	PointCloud  = 0,
	ImageColor  = 1,
	ImageGry    = 2,
	ImageDepth  = 3,
	GroundTruth = 4
};

class ConfigureParam
{
public:
	ConfigureParam();

	bool        _pubPointCloud;
	std::string _pointCloudTopic;
	float       _pointCloudRate;
	std::string _pointCloudFrameID;

	bool        _pubImageColor;
	bool        _pubImageGry;
	bool        _pubImageDepth;
	bool        _isStereo;
	std::string _imageTopic;
	float       _imageRate;
	std::string _imageFrameID;


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
	BasicDatasetPlayer(ros::NodeHandle & node);
	bool process();
	ConfigureParam & getConfigureParam(){return _configureParam;};

	virtual bool setup(){}
	virtual bool initFilename(std::string & filename, const TopicType topicType, 
							  const std::string & pathOfDataset, const std::string & subDirectory, uint seq, bool left = true){};

	void processPointCloud();
	bool initPointCloud(sensor_msgs::PointCloud2 & pointcloud2, uint seq);
	virtual bool readPointCloud(const std::string & filename, pcl::PointCloud<pcl::PointXYZI> & pointcloud){}

	void processImage(TopicType topicType, bool isStereo);
	bool initImage(sensor_msgs::ImagePtr imagePtr, uint seq, TopicType topicType, bool left);
	bool readImage(const std::string & filename, cv::Mat image, TopicType topicType);

	void processGroundTruth();
	bool initGroundTruth();
	virtual bool readGroundTruth(const std::string & filename, pcl::PointCloud<pcl::PointXYZI> & pointcloud){}

private:
	ConfigureParam _configureParam;

	ros::NodeHandle _node;
}; // end class BasicDatasetPlayer

} // end namespace dataset_player