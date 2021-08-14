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

ConfigureParam::ConfigureParam(const bool pubPointCloud, const bool pubImageColor, const bool pubImageGry, const bool pubImageDepth, const bool pubGroundTruth)
{
	_pubPointCloud  = pubPointCloud;
	_pubImageColor  = pubImageColor;
	_pubImageGry    = pubImageColor;
	_pubImageDepth  = pubImageDepth;
	_pubGroundTruth = pubGroundTruth;
}

ConfigureParam::ConfigureParam(const ConfigureParam & other)
{
	_pubPointCloud  = other._pubPointCloud;
	_pubImageColor  = other._pubImageColor;
	_pubImageGry    = other._pubImageColor;
	_pubImageDepth  = other._pubImageDepth;
	_pubGroundTruth = other._pubGroundTruth;
}

BasicDatasetPlayer::BasicDatasetPlayer()
{
	// TODO
}

void BasicDatasetPlayer::setPubPointCloud(bool flag)
{
	_configureParam._pubPointCloud = flag;
}

void BasicDatasetPlayer::setPubImageColor(bool flag)
{
	_configureParam._pubImageColor = flag;
}

void BasicDatasetPlayer::setPubImageGry(bool flag)
{
	_configureParam._pubImageGry = flag;
}

void BasicDatasetPlayer::setPubImageDepth(bool flag)
{
	_configureParam._pubImageDepth = flag;
}

void BasicDatasetPlayer::setPubGroundTruth(bool flag)
{
	_configureParam._pubGroundTruth = flag;
}

} // end namespace dataset_player