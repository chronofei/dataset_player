#include "dataset_player/DatasetPlayer.h"

namespace dataset_player
{

DatasetPlayer::DatasetPlayer()
{
	// TODO
}

bool DatasetPlayer::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
	std::string sParam;
	bool        bParam;
	float       fParam;

	if (privateNode.getParam("datasetType", sParam))
	{
		if (sParam == "KITTI")
		{
			ROS_INFO_STREAM("Set datasetType: KITTI");
			_datasetType = DatasetType::KITTI;
			_basicDatasetPlayer = new KITTIDatasetPlayer(node);
		}
		else if (sParam == "TUM")
		{
			ROS_INFO_STREAM("Set datasetType: TUM");
			_datasetType = DatasetType::TUM;
			_basicDatasetPlayer = new TUMDatasetPlayer(node);
		}
		else
		{
			ROS_ERROR_STREAM("Invalid datasetType parameter: " << sParam);
			return false;
		}
	}
	else
	{
		ROS_ERROR_STREAM("Unable to get the type of dataset");
		return false;
	}

	if(_datasetType == DatasetType::TUM)
	{
		if (privateNode.getParam("pubImageDepth", bParam) && bParam)
		{
			ROS_INFO_STREAM("Set pubImageDepth: TRUE");
			if (privateNode.getParam("imageDepthTopic", sParam))
			{
				ROS_INFO_STREAM("Set imageDepthTopic: " << sParam);
				_basicDatasetPlayer->setImageDepthTopic(sParam);
			}
			else
				ROS_INFO_STREAM("Set imageDepthTopic: morenzhi");
			if (privateNode.getParam("imageDepthRate", fParam))
			{
				ROS_INFO_STREAM("Set imageDepthRate: " << fParam);
				_basicDatasetPlayer->setImageDepthRate(fParam);
			}
			else
				ROS_INFO_STREAM("Set imageDepthRate: morenzhi");
			if (privateNode.getParam("imageDepthFrameID", sParam))
			{
				ROS_INFO_STREAM("Set imageDepthFrameID: " << sParam);
				_basicDatasetPlayer->setImageDepthFrameID(sParam);
			}
			else
				ROS_INFO_STREAM("Set imageDepthFrameID: morenzhi");
			_basicDatasetPlayer->setPubImageDepth(true);
		}
		else
		{
			_basicDatasetPlayer->setPubImageDepth(false);
			ROS_INFO_STREAM("Set imageDepthTopic: FALSE");
		}
	}
	else if (_datasetType == DatasetType::KITTI)
	{
		if (privateNode.getParam("pubPointCloud", bParam) && bParam)
		{
			ROS_INFO_STREAM("Set pubPointCloud: TRUE");
			if (privateNode.getParam("pointCloudTopic", sParam))
			{
				ROS_INFO_STREAM("Set pointCloudTopic: " << sParam);
				_basicDatasetPlayer->setPointCloudTopic(sParam);
			}
			else
				ROS_INFO_STREAM("Set pointCloudTopic: morenzhi");
			if (privateNode.getParam("pointCloudRate", fParam))
			{
				ROS_INFO_STREAM("Set pointCloudRate: " << fParam);
				_basicDatasetPlayer->setPointCloudRate(fParam);
			}
			else
				ROS_INFO_STREAM("Set pointCloudRate: morenzhi");
			if (privateNode.getParam("pointCloudFrameID", sParam))
			{
				ROS_INFO_STREAM("Set pointCloudFramdID: " << sParam);
				_basicDatasetPlayer->setPointCloudFrameID(sParam);
			}
			else
				ROS_INFO_STREAM("Set pointCloudFrameID: morenzhi");
			_basicDatasetPlayer->setPubPointCloud(true);
		}
		else
		{
			_basicDatasetPlayer->setPubPointCloud(false);
			ROS_INFO_STREAM("Set pubPointCloud: FALSE");
		}

		if (privateNode.getParam("pubImageGry", bParam) && bParam)
		{
			ROS_INFO_STREAM("Set pubImageGry: TRUE");
			if (privateNode.getParam("imageGryTopic", sParam))
			{
				ROS_INFO_STREAM("Set imageGryTopic: " << sParam);
				_basicDatasetPlayer->setImageGryTopic(sParam);
			}
			else
				ROS_INFO_STREAM("Set imageGryTopic: morenzhi");
			if (privateNode.getParam("imageGryRate", fParam))
			{
				ROS_INFO_STREAM("Set imageGryRate: " << fParam);
				_basicDatasetPlayer->setImageGryRate(fParam);
			}
			else
				ROS_INFO_STREAM("Set imageGryRate: morenzhi");
			if (privateNode.getParam("imageGryFrameID", sParam))
			{
				ROS_INFO_STREAM("Set imageGryFrameID: " << sParam);
				_basicDatasetPlayer->setImageGryFrameID(sParam);
			}
			else
				ROS_INFO_STREAM("Set imageGryFrameID: morenzhi");
			_basicDatasetPlayer->setPubImageGry(true);
		}
		else
		{
			_basicDatasetPlayer->setPubImageGry(false);
			ROS_INFO_STREAM("Set pubImageGry: FALSE");
		}
	}

	if (privateNode.getParam("pubImageColor", bParam) && bParam)
	{
		ROS_INFO_STREAM("Set pubImageColor: TRUE");
		if (privateNode.getParam("imageColorTopic", sParam))
		{
			ROS_INFO_STREAM("Set imageColorTopic: " << sParam);
			_basicDatasetPlayer->setImageColorTopic(sParam);
		}
		else
			ROS_INFO_STREAM("Set imageColorTopic: morenzhi");
		if (privateNode.getParam("imageColorRate", fParam))
		{
			ROS_INFO_STREAM("Set imageColorRate: " << fParam);
			_basicDatasetPlayer->setImageColorRate(fParam);
		}
		else
			ROS_INFO_STREAM("Set imageColorRate: morenzhi");
		if (privateNode.getParam("imageColorFrameID", sParam))
		{
			ROS_INFO_STREAM("Set imageColorFrameID: " << sParam);
			_basicDatasetPlayer->setImageColorFrameID(sParam);
		}
		else
			ROS_INFO_STREAM("Set imageColorFrameID: morenzhi");
		_basicDatasetPlayer->setPubImageColor(true);
	}
	else
	{
		_basicDatasetPlayer->setPubImageColor(false);
		ROS_INFO_STREAM("Set pubImageColor: FALSE");
	}

	if (privateNode.getParam("pubGroundTruth", bParam) && bParam)
	{
		ROS_INFO_STREAM("Set pubGroundTruth: TRUE");
		if (privateNode.getParam("groundTruthTopic", sParam))
		{
			ROS_INFO_STREAM("Set groundTruthTopic: " << sParam);
			_basicDatasetPlayer->setGroundTruthTopic(sParam);
		}
		else
			ROS_INFO_STREAM("Set groundTruthTopic: morenzhi");
		if (privateNode.getParam("groundTruthRate", fParam))
		{
			ROS_INFO_STREAM("Set groundTruthRate: " << fParam);
			_basicDatasetPlayer->setGroundTruthRate(fParam);
		}
		else
			ROS_INFO_STREAM("Set groundTruthRate: morenzhi");
		if (privateNode.getParam("groundTruthFrameID", sParam))
		{
			ROS_INFO_STREAM("Set groundTruthFrameID: " << sParam);
			_basicDatasetPlayer->setGroundTruthFrameID(sParam);
		}
		else
			ROS_INFO_STREAM("Set groundTruthFrameID: morenzhi");
		_basicDatasetPlayer->setPubGroundTruth(true);
	}
	else
	{
		_basicDatasetPlayer->setPubGroundTruth(false);
		ROS_INFO_STREAM("Set pubGroundTruth: FALSE");
	}

	if (privateNode.getParam("pathOfDataset", sParam))
	{
		ROS_INFO_STREAM("Set pathOfDataset: " << sParam);
		_basicDatasetPlayer->setPathOfDataset(sParam);
	}
	else
		ROS_INFO_STREAM("Set pathOfDataset: morenzhi");

	if (privateNode.getParam("subDirectory", sParam))
	{
		ROS_INFO_STREAM("Set subDirectory: " << sParam);
		_basicDatasetPlayer->setSubDirectory(sParam);
	}
	else
		ROS_INFO_STREAM("Set subDirectory: morenzhi");
	
	return true;
}

void DatasetPlayer::spin()
{
	_basicDatasetPlayer->process();

	ros::spin();
}

} // end namespace dataset_player