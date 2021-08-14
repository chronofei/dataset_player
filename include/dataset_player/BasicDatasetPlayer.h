#pragma once

// C/C++
#include<iostream>
#include<string>

// dataset_player
#include "dataset_player/common.h"

namespace dataset_player
{

class ConfigureParam
{
public:
	ConfigureParam();
	ConfigureParam(const bool pubPointCloud, const bool pubImageColor, const bool pubImageGry, const bool pubImageDepth, const bool pubGroundTruth);
	ConfigureParam(const ConfigureParam & other);

	bool _pubPointCloud;
	bool _pubImageColor;
	bool _pubImageGry;
	bool _pubImageDepth;
	bool _pubGroundTruth;
};

class BasicDatasetPlayer
{
public:
	BasicDatasetPlayer();

	void setPubPointCloud(bool flag);
	void setPubImageColor(bool flag);
	void setPubImageGry(bool flag);
	void setPubImageDepth(bool flag);
	void setPubGroundTruth(bool flag);

private:
	ConfigureParam _configureParam;
}; // end class BasicDatasetPlayer

} // end namespace dataset_player